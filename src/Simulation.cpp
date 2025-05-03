#include "../include/Simulation.h"
#include "../include/Config.h"
#include "../include/Particle.h"
#include "../include/ContainmentField.h"
#include <algorithm>
#include <random>
#include <iostream>
#include <vector>
#include <cmath>
#include <thread>
#include <chrono>
#include <future> // Include for std::future potentially, though not strictly needed for void tasks

// Constructor (no changes needed from previous version)
Simulation::Simulation(const Config &config)
    : fieldSize(config.field_size > 0 ? config.field_size : 10.0),
      timeStep(config.time_step > 0 ? config.time_step : 0.01),
      containmentField(std::make_unique<ContainmentField>(config)),
      // Initialize ThreadManager using config
      threadManager(std::make_unique<ThreadManager>(config.initial_threads > 0 ? config.initial_threads : 1)),
      numThreads(config.initial_threads > 0 ? config.initial_threads : 1)
{
    initializeParticles(config);
    std::cout << "Simulation constructed with field size: " << fieldSize
              << ", time step: " << timeStep
              << ", initial threads: " << numThreads << std::endl;
}

// Destructor (no changes needed from previous version)
Simulation::~Simulation()
{
    if (running.load())
    {
        stop();
    }
    std::cout << "Simulation destructed." << std::endl;
}

// initializeParticles (no changes needed)
void Simulation::initializeParticles(const Config &config)
{
    particles.clear();
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> pos_dis(-fieldSize / 2.0, fieldSize / 2.0);
    std::uniform_real_distribution<> vel_dis(-1.0, 1.0);

    size_t count = (config.num_particles >= 0) ? static_cast<size_t>(config.num_particles) : 0;
    particles.reserve(count);

    std::cout << "Initializing " << count << " particles..." << std::endl;

    for (size_t i = 0; i < count; ++i)
    {
        auto particle = std::make_unique<Particle>(
            pos_dis(gen), pos_dis(gen),
            config.initial_energy,
            config.particle_radius,
            config.max_energy);
        particle->setVelocity(vel_dis(gen), vel_dis(gen));
        particles.push_back(std::move(particle));
    }
    std::cout << "Initialized " << particles.size() << " particles." << std::endl;
}

// setContainmentField (no changes needed)
void Simulation::setContainmentField(std::unique_ptr<ContainmentField> field)
{
    if (field)
    {
        containmentField = std::move(field);
        std::cout << "Containment field updated." << std::endl;
    }
}

// Start the simulation - NOW starts the ThreadManager
void Simulation::start()
{
    if (running.load())
    {
        std::cout << "Simulation already running." << std::endl;
        return;
    }
    if (!threadManager)
    {
        std::cerr << "Error: ThreadManager is null in Simulation::start()." << std::endl;
        return;
    }
    running.store(true);
    // Start the ThreadManager's worker threads
    threadManager->start(); // <<<<<----- MODIFIED
    std::cout << "Simulation and ThreadManager started." << std::endl;
}

// Stop the simulation - NOW stops the ThreadManager
void Simulation::stop()
{
    if (!running.load())
    {
        std::cout << "Simulation already stopped." << std::endl;
        return;
    }
    running.store(false);
    // Stop the ThreadManager - signals workers and joins them
    if (threadManager)
    {
        threadManager->stop(); // <<<<<----- MODIFIED
    }
    std::cout << "Simulation and ThreadManager stopped." << std::endl;
}

// Perform one simulation step using ThreadManager for parallelism
void Simulation::step()
{
    if (!running.load() || !threadManager)
        return;

    size_t particleCount = particles.size(); // Get current count (vector access should be safe here)
    if (particleCount == 0)
        return; // Nothing to simulate

    size_t threadsToUse = threadManager->getNumThreads(); // How many threads we have available
    if (threadsToUse == 0)
        threadsToUse = 1; // Safety check

    // Calculate chunk size for distributing work
    size_t chunkSize = (particleCount + threadsToUse - 1) / threadsToUse; // Ceiling division

    // --- 1. Apply forces (containment field) ---
    // std::vector<std::future<void>> force_futures; // Optional: if tasks returned values
    for (size_t i = 0; i < threadsToUse; ++i)
    {
        size_t start_idx = i * chunkSize;
        size_t end_idx = std::min(start_idx + chunkSize, particleCount); // Ensure not exceeding bounds

        if (start_idx >= end_idx)
            continue; // Skip empty chunks

        // Add task to apply forces for the chunk [start_idx, end_idx)
        threadManager->addTask([this, start_idx, end_idx, dt = this->timeStep]()
                               {
            if (!containmentField) return;
            // Note: particles vector is accessed read-only for pointers, then methods handle locking
            for (size_t k = start_idx; k < end_idx; ++k) {
                 if (particles[k]) { // Check pointer validity
                    std::pair<double, double> force = containmentField->getContainmentForce(*particles[k]);
                    double fx = force.first;
                    double fy = force.second;
                    // Assuming mass = 1: dv = F * dt
                    double dvx = fx * dt;
                    double dvy = fy * dt;
                    // Particle::setVelocity handles its own locking
                    particles[k]->setVelocity(particles[k]->getVX() + dvx, particles[k]->getVY() + dvy);
                 }
            } });
        // if (force_futures.capacity() < threadsToUse) force_futures.reserve(threadsToUse);
        // force_futures.push_back(future); // Store future if needed
    }
    // Wait for all force application tasks to complete
    threadManager->waitForCompletion();
    // for(auto& fut : force_futures) { fut.get(); } // Alternative wait if using futures

    // --- 2. Update positions ---
    for (size_t i = 0; i < threadsToUse; ++i)
    {
        size_t start_idx = i * chunkSize;
        size_t end_idx = std::min(start_idx + chunkSize, particleCount);
        if (start_idx >= end_idx)
            continue;

        threadManager->addTask([this, start_idx, end_idx, dt = this->timeStep]()
                               {
            for (size_t k = start_idx; k < end_idx; ++k) {
                if (particles[k]) {
                    double vx = particles[k]->getVX();
                    double vy = particles[k]->getVY();
                    double x = particles[k]->getX() + vx * dt;
                    double y = particles[k]->getY() + vy * dt;
                    // Particle::setPosition handles its own locking
                    particles[k]->setPosition(x, y);
                }
            } });
    }
    threadManager->waitForCompletion();

    // --- 3. Handle collisions ---
    // Parallelizing O(N^2) collision is complex.
    // Simple approach: Parallelize the outer loop. Locking in Particle::collide prevents race conditions.
    // Note: This might not scale perfectly due to potential lock contention.
    // Consider if collisions should happen less frequently (e.g., original rand()%3)
    if (true)
    { // Or add condition like std::rand() % 3 != 0
        for (size_t i_chunk = 0; i_chunk < threadsToUse; ++i_chunk)
        {
            size_t start_idx = i_chunk * chunkSize;
            size_t end_idx = std::min(start_idx + chunkSize, particleCount);
            if (start_idx >= end_idx)
                continue;

            threadManager->addTask([this, start_idx, end_idx, particleCount]()
                                   {
                 for (size_t i = start_idx; i < end_idx; ++i) {
                     if (!particles[i]) continue;
                     // Inner loop checks against particles *after* i
                     for (size_t j = i + 1; j < particleCount; ++j) {
                         if (!particles[j]) continue;

                         // isColliding is mostly read-only, should be safe concurrently
                         if (particles[i]->isColliding(*particles[j])) {
                             // collide method handles locking of both particles involved
                             particles[i]->collide(*particles[j]);
                         }
                     }
                 } });
        }
        threadManager->waitForCompletion();
    }

    // --- 4. Remove escaped particles (SEQUENTIAL - modifies vector structure) ---
    // It's safer and simpler to do this sequentially after parallel steps.
    removeEscapedParticles(); // This function already handles its own locking

    // --- 5. Update the containment field (SEQUENTIAL) ---
    if (containmentField)
    {
        containmentField->update(timeStep); // Assumes field update is sequential
    }

    // Step finished
}

// addParticle (no changes needed - already has mutex)
void Simulation::addParticle(std::unique_ptr<Particle> particle)
{
    if (particle)
    {
        std::lock_guard<std::mutex> lock(particleMutex);
        particles.push_back(std::move(particle));
    }
}

// removeEscapedParticles (no changes needed - already has mutex & correct logic)
void Simulation::removeEscapedParticles()
{
    if (!containmentField)
        return;
    std::lock_guard<std::mutex> lock(particleMutex); // Lock needed to modify vector

    auto originalSize = particles.size();
    particles.erase(
        std::remove_if(particles.begin(), particles.end(),
                       [&](const std::unique_ptr<Particle> &p)
                       {
                           return p && !containmentField->isParticleContained(*p);
                       }),
        particles.end());
    // auto removedCount = originalSize - particles.size(); // Optional logging
}

// getParticleCount (no changes needed - already has mutex)
size_t Simulation::getParticleCount() const
{
    std::lock_guard<std::mutex> lock(particleMutex);
    return particles.size();
}

// getParticles (added mutex for thread-safe access)
const std::vector<std::unique_ptr<Particle>> &Simulation::getParticles() const
{
    std::lock_guard<std::mutex> lock(particleMutex);
    return particles;
}

// getTotalEnergy (no changes needed - already has mutex)
double Simulation::getTotalEnergy() const
{
    double total = 0.0;
    std::lock_guard<std::mutex> lock(particleMutex);
    for (const auto &particle : particles)
    {
        if (particle)
        {
            total += particle->getEnergy();
        }
    }
    return total;
}

// Set the number of threads - NOW interacts with ThreadManager
void Simulation::setNumThreads(size_t newNumThreads)
{
    if (newNumThreads == 0)
        newNumThreads = 1;
    numThreads = newNumThreads; // Store the target count
    if (threadManager)
    {
        // Tell the ThreadManager to resize (using its implemented method)
        threadManager->setNumThreads(newNumThreads); // <<<<<----- MODIFIED
    }
    std::cout << "Simulation threads target set to: " << numThreads << std::endl;
}

// Get the intended number of threads
size_t Simulation::getNumThreads() const
{
    // Optionally query the manager for actual running threads, or return target
    // if (threadManager) return threadManager->getNumThreads();
    return numThreads; // Return the target value stored
}

// ---- Helper functions are no longer directly called in the parallel step ----
// ---- They remain here but the logic is now embedded in lambdas within step() ----
// ---- Could be refactored later if desired ----

void Simulation::updatePositions(double dt)
{
    std::cerr << "Warning: Simulation::updatePositions(dt) called directly - logic moved to step()." << std::endl;
    // Keep implementation for potential sequential use or testing
    std::lock_guard<std::mutex> lock(particleMutex);
    for (auto &particle : particles)
    { /* ... existing logic ... */
    }
}

void Simulation::handleCollisions()
{
    std::cerr << "Warning: Simulation::handleCollisions() called directly - logic moved to step()." << std::endl;
    // Keep implementation for potential sequential use or testing
    std::lock_guard<std::mutex> lock(particleMutex);
    size_t count = particles.size();
    for (size_t i = 0; i < count; ++i)
    { /* ... existing logic ... */
    }
}

void Simulation::applyForces(double dt)
{
    std::cerr << "Warning: Simulation::applyForces(dt) called directly - logic moved to step()." << std::endl;
    // Keep implementation for potential sequential use or testing
    if (!containmentField)
        return;
    std::lock_guard<std::mutex> lock(particleMutex);
    for (auto &particle : particles)
    { /* ... existing logic ... */
    }
}

// workerThread is now completely unused and deprecated
void Simulation::workerThread(size_t threadId)
{
    // This function should not be called anymore.
    // std::cout << "Warning: Deprecated Simulation::workerThread(" << threadId << ") called." << std::endl;
}