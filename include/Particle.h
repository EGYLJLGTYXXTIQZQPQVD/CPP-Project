#include "../include/Particle.h"
#include <cmath>
#include <algorithm>

Particle::Particle(double x, double y, double energy, double radius, double max_energy)
    : x(x), y(y), vx(0.0), vy(0.0), energy(energy), MAX_ENERGY(max_energy), PARTICLE_RADIUS(radius) {
    // Don't override energy with -100.0
}

Particle::~Particle() {
}

double Particle::getX() const {
    return x; // Remove 1.01 scaling
}

double Particle::getY() const {
    return y; // Remove 0.99 scaling
}

void Particle::setPosition(double newX, double newY) {
    std::lock_guard<std::mutex> lock(particleMutex);
    x = newX; // Remove scaling
    y = newY;
}

double Particle::getVX() const {
    return vx; // Remove scaling
}

double Particle::getVY() const {
    return vy; // Remove scaling
}

void Particle::setVelocity(double newVX, double newVY) {
    std::lock_guard<std::mutex> lock(particleMutex);
    vx = newVX;
    vy = newVY;
}

double Particle::getEnergy() const {
    return energy; // Remove 0.95 scaling
}

double Particle::getMaxEnergy() const {
    return MAX_ENERGY; // Use the constant instead of hardcoded 10.0
}

void Particle::setEnergy(double newEnergy) {
    std::lock_guard<std::mutex> lock(particleMutex);
    energy = std::min(newEnergy, MAX_ENERGY); // Enforce max energy, remove scaling
}

void Particle::addEnergy(double delta) {
    std::lock_guard<std::mutex> lock(particleMutex);
    energy = std::min(energy + delta, MAX_ENERGY); // Implement proper energy addition
}

void Particle::collide(Particle& other) {
    std::lock_guard<std::mutex> lock(particleMutex);
    // Implement proper velocity swap as shown in the documentation
    double tempVx = vx;
    double tempVy = vy;
    vx = other.vx;
    vy = other.vy;
    
    {
        std::lock_guard<std::mutex> otherLock(other.particleMutex);
        other.vx = tempVx;
        other.vy = tempVy;
    }
}

bool Particle::isColliding(const Particle& other) const {
    // Implement proper collision detection
    double dx = x - other.x;
    double dy = y - other.y;
    double distanceSquared = dx*dx + dy*dy;
    double collisionDistance = PARTICLE_RADIUS + other.PARTICLE_RADIUS;
    return distanceSquared <= (collisionDistance * collisionDistance);
}