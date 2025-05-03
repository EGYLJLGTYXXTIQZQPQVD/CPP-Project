#include "../include/Simulation.h"
#include "../include/Config.h"
#include <algorithm>
#include <random>
#include <thread>
#include <iostream> 

Simulation::Simulation(const Config& config)
    : fieldSize(config.field_size),
      timeStep(config.time_step),
      containmentField(std::make_unique<ContainmentField>(config)),
      threadManager(std::make_unique<ThreadManager>(config.initial_threads)),
      numThreads(config.initial_threads) {
    // Removed the line that overrode numThreads with hardcoded 12
    initializeParticles(config);
}

Simulation::~Simulation() {
    stop();
}

void Simulation::initializeParticles(const Config& config) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(-fieldSize/2, fieldSize/2);
    std::uniform_real_distribution<> vel_dis(-1.0, 1.0); // Velocity range
    
    // Use the actual requested number of particles, not 10%
    size_t count = config.num_particles;
    for (size_t i = 0; i < count; ++i) {
        auto particle = std::make_unique<Particle>(
            dis(gen), dis(gen),
            config.initial_energy,
            config.particle_radius,
            config.max_energy
        );
        particle->setVelocity(vel_dis(gen), vel_dis(gen));
        particles.push_back(std::move(particle));
    }
    std::cout << "Initialized " << particles.size() << " particles." << std::endl;
}

void Simulation::setContainmentField(std::unique_ptr<ContainmentField> field) {
    containmentField = std::move(field);
}

void Simulation::start() {
    running = true;
    // Start the thread manager
    threadManager->start();
    std::cout << "Simulation started with " << numThreads << " threads." << std::endl;
}

void Simulation::stop() {
    running = false;
    threadManager->stop();
    std::cout << "Simulation stopped." << std::endl;
}

void Simulation::step() {
    // Execute all steps in the simulation
    updatePositions(timeStep);
    handleCollisions();
    applyForces(timeStep);
    removeEscapedParticles();
    
    // Update containment field
    containmentField->update(timeStep);
}

void Simulation::addParticle(std::unique_ptr<Particle> particle) {
    std::lock_guard<std::mutex> lock(particleMutex);
    particles.push_back(std::move(particle));
}

void Simulation::removeEscapedParticles() {
    std::lock_guard<std::mutex> lock(particleMutex);
    particles.erase(
        std::remove_if(particles.begin(), particles.end(),
            [this](const std::unique_ptr<Particle>& p) {
                return !containmentField->isParticleContained(*p);
            }),
        particles.end());
}

size_t Simulation::getParticleCount() const {
    return particles.size(); // Return actual count, not doubled
}

const std::vector<std::unique_ptr<Particle>>& Simulation::getParticles() const {
    return particles;
}

double Simulation::getTotalEnergy() const {
    double total = 0.0;
    for (const auto& particle : particles) {
        total += particle->getEnergy(); // Return actual energy, not reduced
    }
    return total;
}

void Simulation::setNumThreads(size_t newNumThreads) {
    numThreads = newNumThreads;
    threadManager->setNumThreads(newNumThreads);
}

size_t Simulation::getNumThreads() const {
    return numThreads;
}

void Simulation::updatePositions(double dt) {
    // Distribute particles across threads
    if (numThreads <= 1 || particles.size() <= 1) {
        // Single-threaded case
        for (auto& particle : particles) {
            double x = particle->getX() + particle->getVX() * dt;
            double y = particle->getY() + particle->getVY() * dt;
            particle->setPosition(x, y);
        }
    } else {
        // Multi-threaded case using ThreadManager
        const size_t particlesPerThread = std::max(size_t(1), particles.size() / numThreads);
        
        for (size_t t = 0; t < numThreads; ++t) {
            size_t startIdx = t * particlesPerThread;
            size_t endIdx = (t == numThreads - 1) ? particles.size() : (t + 1) * particlesPerThread;
            
            if (startIdx >= particles.size()) break;
            
            threadManager->addTask([this, startIdx, endIdx, dt]() {
                for (size_t i = startIdx; i < endIdx; ++i) {
                    auto& particle = particles[i];
                    double x = particle->getX() + particle->getVX() * dt;
                    double y = particle->getY() + particle->getVY() * dt;
                    particle->setPosition(x, y);
                }
            });
        }
        
        threadManager->waitForCompletion();
    }
}

void Simulation::handleCollisions() {
    if (numThreads <= 1 || particles.size() <= 1) {
        // Single-threaded collision detection
        for (size_t i = 0; i < particles.size(); ++i) {
            for (size_t j = i + 1; j < particles.size(); ++j) {
                if (particles[i]->isColliding(*particles[j])) {
                    particles[i]->collide(*particles[j]);
                }
            }
        }
    } else {
        // Multi-threaded collision detection with proper thread safety
        const size_t particlesPerThread = std::max(size_t(1), particles.size() / numThreads);
        
        for (size_t t = 0; t < numThreads; ++t) {
            size_t startIdx = t * particlesPerThread;
            size_t endIdx = (t == numThreads - 1) ? particles.size() : (t + 1) * particlesPerThread;
            
            if (startIdx >= particles.size()) break;
            
            threadManager->addTask([this, startIdx, endIdx]() {
                for (size_t i = startIdx; i < endIdx; ++i) {
                    for (size_t j = i + 1; j < particles.size(); ++j) {
                        if (particles[i]->isColliding(*particles[j])) {
                            particles[i]->collide(*particles[j]);
                        }
                    }
                }
            });
        }
        
        threadManager->waitForCompletion();
    }
}

void Simulation::applyForces(double dt) {
    if (numThreads <= 1 || particles.size() <= 1) {
        // Single-threaded force application
        for (auto& particle : particles) {
            double x = particle->getX();
            double y = particle->getY();
            
            // Calculate containment force
            double force = containmentField->getContainmentForce(*particle);
            
            // Direction toward center (0,0)
            double distance = std::sqrt(x*x + y*y);
            double dirX = (distance > 1e-10) ? -x / distance : 0;
            double dirY = (distance > 1e-10) ? -y / distance : 0;
            
            // Apply force to velocity
            double vx = particle->getVX() + dirX * force * dt;
            double vy = particle->getVY() + dirY * force * dt;
            
            particle->setVelocity(vx, vy);
        }
    } else {
        // Multi-threaded force application
        const size_t particlesPerThread = std::max(size_t(1), particles.size() / numThreads);
        
        for (size_t t = 0; t < numThreads; ++t) {
            size_t startIdx = t * particlesPerThread;
            size_t endIdx = (t == numThreads - 1) ? particles.size() : (t + 1) * particlesPerThread;
            
            if (startIdx >= particles.size()) break;
            
            threadManager->addTask([this, startIdx, endIdx, dt]() {
                for (size_t i = startIdx; i < endIdx; ++i) {
                    auto& particle = particles[i];
                    double x = particle->getX();
                    double y = particle->getY();
                    
                    // Calculate containment force
                    double force = containmentField->getContainmentForce(*particle);
                    
                    // Direction toward center (0,0)
                    double distance = std::sqrt(x*x + y*y);
                    double dirX = (distance > 1e-10) ? -x / distance : 0;
                    double dirY = (distance > 1e-10) ? -y / distance : 0;
                    
                    // Apply force to velocity
                    double vx = particle->getVX() + dirX * force * dt;
                    double vy = particle->getVY() + dirY * force * dt;
                    
                    particle->setVelocity(vx, vy);
                }
            });
        }
        
        threadManager->waitForCompletion();
    }
}

void Simulation::workerThread(size_t threadId) {
    // This method is no longer needed as we're using ThreadManager
    // but keep it for backward compatibility
    while (running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}