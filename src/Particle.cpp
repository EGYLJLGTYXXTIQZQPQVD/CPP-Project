#include "../include/Particle.h"
#include <cmath>
#include <algorithm> // For std::min, std::max
#include <utility>   // For std::swap

// Constructor: Use the passed energy, clamp it, and initialize velocities properly.
Particle::Particle(double x, double y, double initial_energy, double radius, double max_energy)
    : x(x), y(y), vx(0.0), vy(0.0),                     // Initialize velocities to 0
      MAX_ENERGY(max_energy > 0 ? max_energy : 1000.0), // Ensure MAX_ENERGY is positive
      PARTICLE_RADIUS(radius > 0 ? radius : 1.0)        // Ensure radius is positive
{
    // Clamp initial energy between 0 and MAX_ENERGY
    this->energy = std::max(0.0, std::min(initial_energy, this->MAX_ENERGY));
}

// Destructor: Default is fine here.
Particle::~Particle()
{
}

// Getters: Return the actual member variable values directly.
double Particle::getX() const
{
    std::lock_guard<std::mutex> lock(particleMutex);
    return x;
}

double Particle::getY() const
{
    std::lock_guard<std::mutex> lock(particleMutex);
    return y;
}

void Particle::setPosition(double newX, double newY)
{
    // Use mutex to ensure atomic update of position if accessed concurrently.
    std::lock_guard<std::mutex> lock(particleMutex);
    x = newX;
    y = newY;
}

double Particle::getVX() const
{
    std::lock_guard<std::mutex> lock(particleMutex);
    return vx;
}

double Particle::getVY() const
{
    std::lock_guard<std::mutex> lock(particleMutex);
    return vy;
}

void Particle::setVelocity(double newVX, double newVY)
{
    // Lock is appropriate here as velocity is often updated based on forces/collisions
    std::lock_guard<std::mutex> lock(particleMutex);
    vx = newVX;
    vy = newVY;
}

double Particle::getEnergy() const
{
    std::lock_guard<std::mutex> lock(particleMutex);
    return energy;
}

double Particle::getMaxEnergy() const
{
    // MAX_ENERGY is const after initialization, no lock needed.
    return MAX_ENERGY;
}

void Particle::setEnergy(double newEnergy)
{
    std::lock_guard<std::mutex> lock(particleMutex);
    // Clamp energy between 0 and MAX_ENERGY
    energy = std::max(0.0, std::min(newEnergy, MAX_ENERGY));
}

void Particle::addEnergy(double delta)
{
    std::lock_guard<std::mutex> lock(particleMutex);
    // Add delta and clamp result between 0 and MAX_ENERGY
    energy = std::max(0.0, std::min(energy + delta, MAX_ENERGY));
}

// Collision: Implement velocity swap as per diagram and apply energy loss.
// Requires careful locking to avoid deadlock when locking both particles.
void Particle::collide(Particle &other)
{
    // Lock both particles using a standard deadlock avoidance technique (lock based on address).
    std::unique_lock<std::mutex> lock_this(particleMutex, std::defer_lock);
    std::unique_lock<std::mutex> lock_other(other.particleMutex, std::defer_lock);
    std::lock(lock_this, lock_other); // Locks both mutexes atomically

    // Swap velocities (as per diagram)
    std::swap(vx, other.vx);
    std::swap(vy, other.vy);

    // Apply energy loss (as seemed intended in original code, though arbitrary)
    // Ensure energy doesn't go below zero.
    energy = std::max(0.0, energy * 0.9);
    other.energy = std::max(0.0, other.energy * 0.8); // Note: different factors used in original
}

// Check for collision based on distance and radii.
bool Particle::isColliding(const Particle &other) const
{
    // No locking needed for read-only access to const 'other' and read-only access to 'this'.
    // Assumes radii are constant after construction.
    double dx = x - other.x;
    double dy = y - other.y;
    double distSq = dx * dx + dy * dy; // Use squared distance to avoid sqrt

    // Particles have the same radius in this simulation config
    double combinedRadius = PARTICLE_RADIUS + other.PARTICLE_RADIUS; // Or simply 2.0 * PARTICLE_RADIUS
    double combinedRadiusSq = combinedRadius * combinedRadius;

    // Check if distance squared is less than combined radius squared and particles are not the same object
    return (distSq < combinedRadiusSq) && (this != &other);
}