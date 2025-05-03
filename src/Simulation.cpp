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
    
    // Initialize particles
    initializeParticles(config);
    
    // No explicit call to start() needed as ThreadManager now starts on construction
}

Simulation::~Simulation() {
    stop();
}

void Simulation::initializeParticles(const Config& config) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(-fieldSize/2, fieldSize/2);
    std::uniform_real_distribution<> vel_dis(-1.0, 1.0);
    
    particles.reserve(config.num_particles);
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
    // No need to start threadManager, it's already running
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
    updatePositions(timeStep);
    handleCollisions();
    applyForces(timeStep);
    removeEscapedParticles();
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
    return particles.size();
}

const std::vector<std::unique_ptr<Particle>>& Simulation::getParticles() const {
    return particles;
}

double Simulation::getTotalEnergy() const {
    // Optimize the energy calculation (for BUG107)
    if (numThreads <= 1 || particles.size() <= 100) {
        // Fast single-threaded calculation for small particle counts
        double total = 0.0;
        for (const auto& particle : particles) {
            total += particle->getEnergy();
        }
        return total;
    }
    
    // Multi-threaded calculation for better performance
    const size_t particlesPerThread = std::max(size_t(1), particles.size() / numThreads);
    std::vector<double> partialSums(numThreads, 0.0);
    
    std::vector<std::thread> threads;
    for (size_t t = 0; t < numThreads; ++t) {
        size_t startIdx = t * particlesPerThread;
        size_t endIdx = (t == numThreads - 1) ? particles.size() : (t + 1) * particlesPerThread;
        
        if (startIdx >= particles.size()) break;
        
        threads.emplace_back([this, startIdx, endIdx, &partialSums, t]() {
            double sum = 0.0;
            for (size_t i = startIdx; i < endIdx; ++i) {
                sum += particles[i]->getEnergy();
            }
            partialSums[t] = sum;
        });
    }
    
    for (auto& thread : threads) {
        thread.join();
    }
    
    double total = 0.0;
    for (double sum : partialSums) {
        total += sum;
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
    if (numThreads <= 1 || particles.size() <= 100) {
        // Single-threaded update for small particle counts
        for (auto& particle : particles) {
            double x = particle->getX() + particle->getVX() * dt;
            double y = particle->getY() + particle->getVY() * dt;
            particle->setPosition(x, y);
        }
    } else {
        // Highly optimized multi-threaded update
        const size_t particlesPerThread = (particles.size() + numThreads - 1) / numThreads;
        std::vector<std::thread> threads;
        
        for (size_t t = 0; t < numThreads; ++t) {
            size_t startIdx = t * particlesPerThread;
            size_t endIdx = std::min(startIdx + particlesPerThread, particles.size());
            
            if (startIdx >= particles.size()) break;
            
            threads.emplace_back([this, startIdx, endIdx, dt]() {
                for (size_t i = startIdx; i < endIdx; ++i) {
                    double x = particles[i]->getX() + particles[i]->getVX() * dt;
                    double y = particles[i]->getY() + particles[i]->getVY() * dt;
                    particles[i]->setPosition(x, y);
                }
            });
        }
        
        for (auto& thread : threads) {
            thread.join();
        }
    }
}

void Simulation::handleCollisions() {
    if (numThreads <= 1 || particles.size() <= 500) {
        // Single-threaded collision detection for small particle counts
        for (size_t i = 0; i < particles.size(); ++i) {
            for (size_t j = i + 1; j < particles.size(); ++j) {
                if (particles[i]->isColliding(*particles[j])) {
                    particles[i]->collide(*particles[j]);
                }
            }
        }
    } else {
        // Super optimized spatial partitioning for collision detection
        // Division into grid cells improves performance dramatically
        const double cellSize = 2.0; // Assuming particle radius is around 1.0
        const int gridSize = static_cast<int>(fieldSize / cellSize) + 1;
        
        // Grid-based spatial partitioning
        std::vector<std::vector<size_t>> grid(gridSize * gridSize);
        
        // Assign particles to grid cells
        for (size_t i = 0; i < particles.size(); ++i) {
            double x = particles[i]->getX();
            double y = particles[i]->getY();
            
            int cellX = static_cast<int>((x + fieldSize/2) / cellSize);
            int cellY = static_cast<int>((y + fieldSize/2) / cellSize);
            
            cellX = std::max(0, std::min(cellX, gridSize-1));
            cellY = std::max(0, std::min(cellY, gridSize-1));
            
            grid[cellY * gridSize + cellX].push_back(i);
        }
        
        // Process collisions in parallel
        std::vector<std::thread> threads;
        for (size_t t = 0; t < numThreads; ++t) {
            size_t startCell = t * grid.size() / numThreads;
            size_t endCell = (t + 1) * grid.size() / numThreads;
            
            threads.emplace_back([this, &grid, startCell, endCell, gridSize]() {
                for (size_t cellIndex = startCell; cellIndex < endCell; ++cellIndex) {
                    int cellX = cellIndex % gridSize;
                    int cellY = cellIndex / gridSize;
                    
                    // Check collisions within this cell
                    for (size_t i = 0; i < grid[cellIndex].size(); ++i) {
                        for (size_t j = i + 1; j < grid[cellIndex].size(); ++j) {
                            size_t pIndex1 = grid[cellIndex][i];
                            size_t pIndex2 = grid[cellIndex][j];
                            
                            if (particles[pIndex1]->isColliding(*particles[pIndex2])) {
                                particles[pIndex1]->collide(*particles[pIndex2]);
                            }
                        }
                    }
                    
                    // Check collisions with adjacent cells
                    for (int dy = -1; dy <= 1; ++dy) {
                        for (int dx = -1; dx <= 1; ++dx) {
                            if (dx == 0 && dy == 0) continue; // Skip self
                            
                            int neighborX = cellX + dx;
                            int neighborY = cellY + dy;
                            
                            if (neighborX < 0 || neighborX >= gridSize || 
                                neighborY < 0 || neighborY >= gridSize) continue;
                            
                            size_t neighborIndex = neighborY * gridSize + neighborX;
                            if (neighborIndex >= endCell) continue; // Skip cells handled by other threads
                            
                            for (size_t i = 0; i < grid[cellIndex].size(); ++i) {
                                for (size_t j = 0; j < grid[neighborIndex].size(); ++j) {
                                    size_t pIndex1 = grid[cellIndex][i];
                                    size_t pIndex2 = grid[neighborIndex][j];
                                    
                                    if (particles[pIndex1]->isColliding(*particles[pIndex2])) {
                                        particles[pIndex1]->collide(*particles[pIndex2]);
                                    }
                                }
                            }
                        }
                    }
                }
            });
        }
        
        for (auto& thread : threads) {
            thread.join();
        }
    }
}

void Simulation::applyForces(double dt) {
    if (numThreads <= 1 || particles.size() <= 100) {
        // Single-threaded force application for small particle counts
        for (auto& particle : particles) {
            double x = particle->getX();
            double y = particle->getY();
            
            double force = containmentField->getContainmentForce(*particle);
            double distance = std::sqrt(x*x + y*y);
            
            if (distance > 1e-10) {
                double dirX = -x / distance;
                double dirY = -y / distance;
                
                double vx = particle->getVX() + dirX * force * dt;
                double vy = particle->getVY() + dirY * force * dt;
                
                particle->setVelocity(vx, vy);
            }
        }
    } else {
        // Optimized multi-threaded force application
        std::vector<std::thread> threads;
        
        const size_t particlesPerThread = (particles.size() + numThreads - 1) / numThreads;
        
        for (size_t t = 0; t < numThreads; ++t) {
            size_t startIdx = t * particlesPerThread;
            size_t endIdx = std::min(startIdx + particlesPerThread, particles.size());
            
            if (startIdx >= particles.size()) break;
            
            threads.emplace_back([this, startIdx, endIdx, dt]() {
                for (size_t i = startIdx; i < endIdx; ++i) {
                    double x = particles[i]->getX();
                    double y = particles[i]->getY();
                    
                    double force = containmentField->getContainmentForce(*particles[i]);
                    double distance = std::sqrt(x*x + y*y);
                    
                    if (distance > 1e-10) {
                        double dirX = -x / distance;
                        double dirY = -y / distance;
                        
                        double vx = particles[i]->getVX() + dirX * force * dt;
                        double vy = particles[i]->getVY() + dirY * force * dt;
                        
                        particles[i]->setVelocity(vx, vy);
                    }
                }
            });
        }
        
        for (auto& thread : threads) {
            thread.join();
        }
    }
}

void Simulation::workerThread(size_t threadId) {
    while (running) {
        // Just a minimal worker loop to keep the thread alive
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}