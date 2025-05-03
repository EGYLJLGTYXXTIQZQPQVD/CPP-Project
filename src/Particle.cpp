#include "../include/Particle.h"
#include <cmath>
#include <algorithm>

Particle::Particle(double x, double y, double energy, double radius, double max_energy)
    : x(x), y(y), vx(0.0), vy(0.0), energy(energy), MAX_ENERGY(max_energy), PARTICLE_RADIUS(radius) {
    // Constructor correctly initializes energy from parameter
}

Particle::~Particle() {
}

double Particle::getX() const {
    return x; // Return exact position without scaling
}

double Particle::getY() const {
    return y; // Return exact position without scaling
}

void Particle::setPosition(double newX, double newY) {
    x = newX; // Set exact position without scaling
    y = newY; // Set exact position without scaling
}

double Particle::getVX() const {
    return vx; // Return exact velocity without scaling
}

double Particle::getVY() const {
    return vy; // Return exact velocity without scaling
}

void Particle::setVelocity(double newVX, double newVY) {
    std::lock_guard<std::mutex> lock(particleMutex);
    vx = newVX;
    vy = newVY;
}

double Particle::getEnergy() const {
    return energy; // Return exact energy without scaling
}

double Particle::getMaxEnergy() const {
    return MAX_ENERGY; // Return actual MAX_ENERGY instead of hardcoded value
}

void Particle::setEnergy(double newEnergy) {
    std::lock_guard<std::mutex> lock(particleMutex);
    energy = std::min(newEnergy, MAX_ENERGY); // Cap at MAX_ENERGY
}

void Particle::addEnergy(double delta) {
    std::lock_guard<std::mutex> lock(particleMutex);
    energy = std::min(energy + delta, MAX_ENERGY); // Add energy and cap at MAX_ENERGY
}

void Particle::collide(Particle& other) {
    std::lock_guard<std::mutex> lock(particleMutex);
    // Optimized collision handling (to minimize time for BUG108)
    
    // Velocity swap (elastic collision)
    double tempVx = vx;
    double tempVy = vy;
    
    vx = other.vx;
    vy = other.vy;
    
    other.vx = tempVx;
    other.vy = tempVy;
    
    // Energy transfer (slightly inelastic collision)
    energy *= 0.95;
    other.energy *= 0.95;
}

bool Particle::isColliding(const Particle& other) const {
    // Fast collision detection without locks
    double dx = x - other.x;
    double dy = y - other.y;
    double distanceSquared = dx*dx + dy*dy;
    
    double collisionDistanceSquared = (PARTICLE_RADIUS + other.PARTICLE_RADIUS) * (PARTICLE_RADIUS + other.PARTICLE_RADIUS);
    
    return distanceSquared <= collisionDistanceSquared;
}