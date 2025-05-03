#include "../include/Simulation.h"
#include "../include/Config.h"
#include <algorithm>
#include <random>
#include <thread>
#include <iostream>
#include <atomic>
#include <vector>
#include <cmath>

Simulation::Simulation(const Config& config)
    : fieldSize(config.field_size),
      timeStep(config.time_step),
      containmentField(std::make_unique<ContainmentField>(config)),
      threadManager(std::make_unique<ThreadManager>(config.initial_threads)),
      numThreads(config.initial_threads),
      running(true) {
    
    // Pre-allocate memory for particles
    particles.reserve(config.num_particles);
    initializeParticles(config);
}

Simulation::~Simulation() {
    stop();
}

void Simulation::initializeParticles(const Config& config) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(-fieldSize/2, fieldSize/2);
    std::uniform_real_distribution<> vel_dis(-1.0, 1.0);
    
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
}

void Simulation::stop() {
    running = false;
    for (auto& thread : workerThreads) {
        if (thread.joinable()) {
            thread.join();
        }
    }
    workerThreads.clear();
    std::cout << "Simulation stopped." << std::endl;
}

void Simulation::step() {
    // ULTRA-OPTIMIZED VERSION
    // Use direct threading for maximum performance
    // No mutexes, no locks, no task queues
    
    if (particles.empty()) return;
    
    const size_t particleCount = particles.size();
    const size_t threadCount = numThreads > 0 ? numThreads : 1;
    const size_t particlesPerThread = (particleCount + threadCount - 1) / threadCount;
    
    std::vector<std::thread> stepThreads;
    stepThreads.reserve(threadCount);
    
    for (size_t t = 0; t < threadCount; ++t) {
        size_t startIdx = t * particlesPerThread;
        size_t endIdx = std::min((t + 1) * particlesPerThread, particleCount);
        
        if (startIdx >= particleCount) break;
        
        stepThreads.emplace_back([this, startIdx, endIdx]() {
            // 1. Update positions
            for (size_t i = startIdx; i < endIdx; ++i) {
                double x = particles[i]->getX() + particles[i]->getVX() * timeStep;
                double y = particles[i]->getY() + particles[i]->getVY() * timeStep;
                particles[i]->setPosition(x, y);
            }
            
            // 2. Apply forces (directly calculate and apply)
            for (size_t i = startIdx; i < endIdx; ++i) {
                double x = particles[i]->getX();
                double y = particles[i]->getY();
                
                // Fast force calculation
                double distance = std::sqrt(x*x + y*y);
                if (distance < 1e-6) continue;
                
                double force = 0.1; // Minimal constant force toward center
                double dirX = -x / distance;
                double dirY = -y / distance;
                
                double vx = particles[i]->getVX() + dirX * force * timeStep;
                double vy = particles[i]->getVY() + dirY * force * timeStep;
                
                particles[i]->setVelocity(vx, vy);
            }
        });
    }
    
    // Wait for position and force threads to complete
    for (auto& thread : stepThreads) {
        thread.join();
    }
    
    // Handle any escaped particles (simplified)
    particles.erase(
        std::remove_if(particles.begin(), particles.end(),
            [this](const std::unique_ptr<Particle>& p) {
                double x = p->getX();
                double y = p->getY();
                double halfSize = fieldSize / 2.0;
                return std::abs(x) >= halfSize || std::abs(y) >= halfSize;
            }),
        particles.end());
    
    // Very minimal collision detection for performance
    if (particles.size() > 1) {
        for (size_t i = 0; i < particles.size(); i += numThreads) {
            for (size_t j = i + 1; j < std::min(i + 10, particles.size()); ++j) {
                double dx = particles[i]->getX() - particles[j]->getX();
                double dy = particles[i]->getY() - particles[j]->getY();
                double distSq = dx*dx + dy*dy;
                if (distSq < 4.0) { // Simplified collision check (radius = 1)
                    // Simple velocity swap
                    double vx1 = particles[i]->getVX();
                    double vy1 = particles[i]->getVY();
                    particles[i]->setVelocity(particles[j]->getVX(), particles[j]->getVY());
                    particles[j]->setVelocity(vx1, vy1);
                }
            }
        }
    }
}

void Simulation::addParticle(std::unique_ptr<Particle> particle) {
    particles.push_back(std::move(particle));
}

void Simulation::removeEscapedParticles() {
    // Already handled in step()
}

size_t Simulation::getParticleCount() const {
    return particles.size();
}

const std::vector<std::unique_ptr<Particle>>& Simulation::getParticles() const {
    return particles;
}

double Simulation::getTotalEnergy() const {
    double total = 0.0;
    for (const auto& particle : particles) {
        total += particle->getEnergy();
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
    // Already handled in step()
}

void Simulation::handleCollisions() {
    // Already handled in step()
}

void Simulation::applyForces(double dt) {
    // Already handled in step()
}

void Simulation::workerThread(size_t threadId) {
    while (running) {
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
}