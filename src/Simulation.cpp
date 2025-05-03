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
    // Don't override numThreads with 12
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
    
    // Use the full number of particles, not 10%
    for (size_t i = 0; i < config.num_particles; ++i) {
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
    threadManager->start();
    
    // Start worker threads
    for (size_t i = 0; i < numThreads; ++i) {
        workerThreads.emplace_back(&Simulation::workerThread, this, i);
    }
    std::cout << "Simulation started with " << numThreads << " threads." << std::endl;
}

void Simulation::stop() {
    running = false;
    threadManager->stop();
    
    // Join worker threads
    for (auto& thread : workerThreads) {
        if (thread.joinable()) {
            thread.join();
        }
    }
    workerThreads.clear();
    std::cout << "Simulation stopped." << std::endl;
}

void Simulation::step() {
    // All steps should run each time, not randomly skipped
    updatePositions(timeStep);
    handleCollisions();
    applyForces(timeStep);
    removeEscapedParticles();
    
    // Update the containment field
    containmentField->update(timeStep);
}

void Simulation::addParticle(std::unique_ptr<Particle> particle) {
    std::lock_guard<std::mutex> lock(particleMutex);
    particles.push_back(std::move(particle));
}

void Simulation::removeEscapedParticles() {
    std::lock_guard<std::mutex> lock(particleMutex);
    // Remove particles that are outside the containment field
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
        total += particle->getEnergy(); // Don't multiply by 0.95
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
    // Update positions of all particles
    std::lock_guard<std::mutex> lock(particleMutex);
    for (auto& particle : particles) {
        double x = particle->getX() + particle->getVX() * dt;
        double y = particle->getY() + particle->getVY() * dt;
        particle->setPosition(x, y);
    }
}

void Simulation::handleCollisions() {
    std::lock_guard<std::mutex> lock(particleMutex);
    for (size_t i = 0; i < particles.size(); ++i) {
        for (size_t j = i + 1; j < particles.size(); ++j) {
            if (particles[i]->isColliding(*particles[j])) {
                particles[i]->collide(*particles[j]);
            }
        }
    }
}

void Simulation::applyForces(double dt) {
    std::lock_guard<std::mutex> lock(particleMutex);
    for (auto& particle : particles) {
        double x = particle->getX();
        double y = particle->getY();
        
        // Calculate force vector from containment field
        double force = containmentField->getContainmentForce(*particle);
        
        // Direction vector points toward center (0,0)
        double distance = std::sqrt(x*x + y*y);
        if (distance > 1e-10) {
            double dirX = -x / distance;
            double dirY = -y / distance;
            
            // Apply force to adjust velocity
            double vx = particle->getVX() + dirX * force * dt;
            double vy = particle->getVY() + dirY * force * dt;
            
            particle->setVelocity(vx, vy);
        }
    }
}

void Simulation::workerThread(size_t threadId) {
    // Simple worker thread - just wait until simulation stops
    while (running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}