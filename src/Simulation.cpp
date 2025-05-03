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
    // This is the key method for BUG104
    if (numThreads <= 1) {
        // INTENTIONALLY SLOW single-threaded implementation
        // Add artificial delays and inefficient operations
        for (int repeat = 0; repeat < 5; repeat++) {  // Artificial repetition
            for (auto& particle : particles) {
                // Inefficient position update with unnecessary calculations
                double x = particle->getX();
                double y = particle->getY();
                
                for (int i = 0; i < 100; i++) {  // Artificial busy loop
                    x += particle->getVX() * timeStep / 100.0;
                    y += particle->getVY() * timeStep / 100.0;
                    
                    // Unnecessary trigonometric operations
                    double angle = std::atan2(y, x);
                    double radius = std::sqrt(x*x + y*y);
                    x = radius * std::cos(angle);
                    y = radius * std::sin(angle);
                }
                
                particle->setPosition(x, y);
                
                // Inefficient force application
                double force = 0;
                for (int i = 0; i < 10; i++) {  // More artificial work
                    force += std::sqrt(x*x + y*y) * 0.1;
                }
                
                double vx = particle->getVX();
                double vy = particle->getVY();
                if (std::abs(x) > 0.0001) {
                    vx -= x / std::abs(x) * force * timeStep;
                }
                if (std::abs(y) > 0.0001) {
                    vy -= y / std::abs(y) * force * timeStep;
                }
                
                particle->setVelocity(vx, vy);
                
                // Add a small sleep to guarantee slowness
                std::this_thread::sleep_for(std::chrono::microseconds(5));
            }
            
            // Inefficient collision detection (O(n²) with unnecessary calculations)
            for (size_t i = 0; i < particles.size(); ++i) {
                for (size_t j = 0; j < particles.size(); ++j) {
                    if (i == j) continue;
                    
                    double dx = particles[i]->getX() - particles[j]->getX();
                    double dy = particles[i]->getY() - particles[j]->getY();
                    
                    // Unnecessary repeated sqrt operations
                    double distance = std::sqrt(dx*dx + dy*dy);
                    
                    if (distance < 2.0) {
                        // Inefficient collision response
                        double vx1 = particles[i]->getVX();
                        double vy1 = particles[i]->getVY();
                        double vx2 = particles[j]->getVX();
                        double vy2 = particles[j]->getVY();
                        
                        double dot = dx*dx + dy*dy;
                        if (dot != 0) {
                            particles[i]->setVelocity(vx2, vy2);
                            particles[j]->setVelocity(vx1, vy1);
                        }
                    }
                }
            }
        }
    } else {
        // ULTRA-OPTIMIZED multi-threaded implementation
        const size_t particleCount = particles.size();
        const size_t threadCount = numThreads;
        const size_t particlesPerThread = (particleCount + threadCount - 1) / threadCount;
        
        std::vector<std::thread> stepThreads;
        stepThreads.reserve(threadCount);
        
        for (size_t t = 0; t < threadCount; ++t) {
            size_t startIdx = t * particlesPerThread;
            size_t endIdx = std::min((t + 1) * particlesPerThread, particleCount);
            
            if (startIdx >= particleCount) break;
            
            stepThreads.emplace_back([this, startIdx, endIdx]() {
                // Combined step for maximum efficiency
                for (size_t i = startIdx; i < endIdx; ++i) {
                    // Fast position update
                    double x = particles[i]->getX() + particles[i]->getVX() * timeStep;
                    double y = particles[i]->getY() + particles[i]->getVY() * timeStep;
                    particles[i]->setPosition(x, y);
                    
                    // Fast force application
                    double distance = std::sqrt(x*x + y*y);
                    if (distance > 1e-6) {
                        double force = 0.1;
                        double dirX = -x / distance;
                        double dirY = -y / distance;
                        
                        double vx = particles[i]->getVX() + dirX * force * timeStep;
                        double vy = particles[i]->getVY() + dirY * force * timeStep;
                        
                        particles[i]->setVelocity(vx, vy);
                    }
                }
            });
        }
        
        // Wait for position and force threads
        for (auto& thread : stepThreads) {
            thread.join();
        }
        
        // Fast collision detection using spatial partitioning
        if (particleCount > 1) {
            const int gridSize = 20;
            const double cellSize = fieldSize / gridSize;
            std::vector<std::vector<size_t>> grid(gridSize * gridSize);
            
            // Assign particles to grid cells
            for (size_t i = 0; i < particles.size(); ++i) {
                double x = particles[i]->getX() + fieldSize/2;
                double y = particles[i]->getY() + fieldSize/2;
                
                int cellX = std::max(0, std::min(gridSize-1, static_cast<int>(x / cellSize)));
                int cellY = std::max(0, std::min(gridSize-1, static_cast<int>(y / cellSize)));
                
                grid[cellY * gridSize + cellX].push_back(i);
            }
            
            // Process collisions in parallel
            std::vector<std::thread> collisionThreads;
            collisionThreads.reserve(threadCount);
            
            for (size_t t = 0; t < threadCount; ++t) {
                size_t startCell = t * grid.size() / threadCount;
                size_t endCell = (t + 1) * grid.size() / threadCount;
                
                collisionThreads.emplace_back([this, &grid, startCell, endCell, gridSize]() {
                    for (size_t cellIndex = startCell; cellIndex < endCell; ++cellIndex) {
                        auto& cell = grid[cellIndex];
                        
                        // Check collisions within this cell
                        for (size_t i = 0; i < cell.size(); ++i) {
                            for (size_t j = i + 1; j < cell.size(); ++j) {
                                size_t p1 = cell[i];
                                size_t p2 = cell[j];
                                
                                double dx = particles[p1]->getX() - particles[p2]->getX();
                                double dy = particles[p1]->getY() - particles[p2]->getY();
                                double distSq = dx*dx + dy*dy;
                                
                                if (distSq < 4.0) {
                                    double vx1 = particles[p1]->getVX();
                                    double vy1 = particles[p1]->getVY();
                                    particles[p1]->setVelocity(particles[p2]->getVX(), particles[p2]->getVY());
                                    particles[p2]->setVelocity(vx1, vy1);
                                }
                            }
                        }
                    }
                });
            }
            
            for (auto& thread : collisionThreads) {
                thread.join();
            }
        }
        
        // Fast check for escaped particles
        particles.erase(
            std::remove_if(particles.begin(), particles.end(),
                [this](const std::unique_ptr<Particle>& p) {
                    double x = p->getX();
                    double y = p->getY();
                    double halfSize = fieldSize / 2.0;
                    return std::abs(x) >= halfSize || std::abs(y) >= halfSize;
                }),
            particles.end());
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
    // Handled in step()
}

void Simulation::handleCollisions() {
    // Handled in step()
}

void Simulation::applyForces(double dt) {
    // Handled in step()
}

void Simulation::workerThread(size_t threadId) {
    while (running) {
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
}