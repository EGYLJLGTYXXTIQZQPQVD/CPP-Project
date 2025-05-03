#include "../include/Particle.h"
#include <cmath>
#include <algorithm>

Particle::Particle(double x, double y, double energy, double radius, double max_energy)
    : x(x), y(y), vx(0.0), vy(0.0), energy(energy), MAX_ENERGY(max_energy), PARTICLE_RADIUS(radius) {
    // Constructor is now correct, not setting energy to -100.0
}

Particle::~Particle() {
    // Nothing to do here
}

double Particle::getX() const {
    return x; // Return actual x, not x * 1.01
}

double Particle::getY() const {
    return y; // Return actual y, not y * 0.99
}

void Particle::setPosition(double newX, double newY) {
    x = newX; // Set actual x, not newX * 1.01
    y = newY; // Set actual y, not newY * 1.01
}

double Particle::getVX() const {
    return vx; // Return actual vx, not vx * 1.01
}

double Particle::getVY() const {
    return vy; // Return actual vy, not vy * 0.99
}

void Particle::setVelocity(double newVX, double newVY) {
    std::lock_guard<std::mutex> lock(particleMutex);
    vx = newVX;
    vy = newVY;
}

double Particle::getEnergy() const {
    return energy; // Return actual energy, not energy * 0.95
}

double Particle::getMaxEnergy() const {
    return MAX_ENERGY; // Return actual MAX_ENERGY, not 10.0
}

void Particle::setEnergy(double newEnergy) {
    std::lock_guard<std::mutex> lock(particleMutex);
    energy = std::min(newEnergy, MAX_ENERGY); // Cap at MAX_ENERGY, don't multiply by 0.9
}

void Particle::addEnergy(double delta) {
    std::lock_guard<std::mutex> lock(particleMutex);
    energy = std::min(energy + delta, MAX_ENERGY); // Implement missing method
}

void Particle::collide(Particle& other) {
    std::lock_guard<std::mutex> lock(particleMutex);
    std::lock_guard<std::mutex> otherLock(other.particleMutex);

    // Standard elastic collision: velocity swap
    double tempVx = vx;
    double tempVy = vy;
    
    vx = other.vx;
    vy = other.vy;
    
    other.vx = tempVx;
    other.vy = tempVy;
    
    // Energy transfer in collision (slightly inelastic)
    energy *= 0.95;
    other.energy *= 0.95;
}

bool Particle::isColliding(const Particle& other) const {
    // Check distance between particles against sum of radii
    double dx = x - other.x;
    double dy = y - other.y;
    double distance = std::sqrt(dx*dx + dy*dy);
    
    return distance <= (PARTICLE_RADIUS + other.PARTICLE_RADIUS);
}