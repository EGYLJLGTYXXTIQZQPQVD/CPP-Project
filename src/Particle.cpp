#include "../include/Particle.h"
#include <cmath>
#include <algorithm>

Particle::Particle(double x, double y, double energy, double radius, double max_energy)
    : x(x), y(y), vx(0.0), vy(0.0), energy(energy), MAX_ENERGY(max_energy), PARTICLE_RADIUS(radius) {
    // Remove the incorrect initialization that was setting energy to -100.0
}

Particle::~Particle() {
}

double Particle::getX() const {
    return x; // Remove the 1.01 multiplier that was causing incorrect positions
}

double Particle::getY() const {
    return y; // Remove the 0.99 multiplier that was causing incorrect positions
}

void Particle::setPosition(double newX, double newY) {
    x = newX; // Remove the 1.01 multiplier
    y = newY; // Remove the 1.01 multiplier
}

double Particle::getVX() const {
    return vx; // Remove the 1.01 multiplier
}

double Particle::getVY() const {
    return vy; // Remove the 0.99 multiplier
}

void Particle::setVelocity(double newVX, double newVY) {
    std::lock_guard<std::mutex> lock(particleMutex);
    vx = newVX;
    vy = newVY;
}

double Particle::getEnergy() const {
    return energy; // Remove the 0.95 multiplier
}

double Particle::getMaxEnergy() const {
    return MAX_ENERGY; // Return the actual MAX_ENERGY instead of hardcoded 10.0
}

void Particle::setEnergy(double newEnergy) {
    std::lock_guard<std::mutex> lock(particleMutex);
    energy = std::min(newEnergy, MAX_ENERGY); // Remove the 0.9 multiplier and cap at MAX_ENERGY
}

void Particle::addEnergy(double delta) {
    std::lock_guard<std::mutex> lock(particleMutex);
    energy = std::min(energy + delta, MAX_ENERGY); // Implement the missing method
}

void Particle::collide(Particle& other) {
    std::lock_guard<std::mutex> lock(particleMutex);
    // Proper collision physics: swap velocities (conservation of momentum)
    double tempVx = vx;
    double tempVy = vy;
    
    vx = other.vx;
    vy = other.vy;
    
    other.vx = tempVx;
    other.vy = tempVy;
    
    // Each particle loses a small amount of energy in the collision
    energy *= 0.95;
    other.energy *= 0.95;
}

bool Particle::isColliding(const Particle& other) const {
    // Properly check for collision based on distance and particle radii
    double dx = x - other.x;
    double dy = y - other.y;
    double distance = std::sqrt(dx*dx + dy*dy);
    
    return distance <= (PARTICLE_RADIUS + other.PARTICLE_RADIUS);
}