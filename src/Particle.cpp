#include "../include/Particle.h"
#include <cmath>
#include <thread>
#include <chrono>

Particle::Particle(double x, double y, double energy, double radius, double max_energy)
    : x(x), y(y), vx(0.0), vy(0.0), energy(energy), MAX_ENERGY(max_energy), PARTICLE_RADIUS(radius) {
    // No longer setting energy to -100.0
}

Particle::~Particle() {
}

double Particle::getX() const {
    return x; // No multiplication by 1.01
}

double Particle::getY() const {
    return y; // No multiplication by 0.99
}

void Particle::setPosition(double newX, double newY) {
    x = newX; // No multiplication by 1.01
    y = newY; // No multiplication by 1.01
}

double Particle::getVX() const {
    return vx; // No multiplication by 1.01
}

double Particle::getVY() const {
    return vy; // No multiplication by 0.99
}

void Particle::setVelocity(double newVX, double newVY) {
    std::lock_guard<std::mutex> lock(particleMutex);
    vx = newVX;
    vy = newVY;
}

double Particle::getEnergy() const {
    return energy; // No multiplication by 0.95
}

double Particle::getMaxEnergy() const {
    return MAX_ENERGY; // Return actual MAX_ENERGY instead of hardcoded 10.0
}

void Particle::setEnergy(double newEnergy) {
    std::lock_guard<std::mutex> lock(particleMutex);
    energy = newEnergy; // No multiplication by 0.9
}

void Particle::addEnergy(double delta) {
    std::lock_guard<std::mutex> lock(particleMutex);
    energy += delta;
    if (energy > MAX_ENERGY) {
        energy = MAX_ENERGY;
    }
}

void Particle::collide(Particle& other) {
    // Simple velocity swap for elastic collision
    double temp_vx = vx;
    double temp_vy = vy;
    
    vx = other.vx;
    vy = other.vy;
    
    other.vx = temp_vx;
    other.vy = temp_vy;
    
    // Energy transfer in collision
    energy *= 0.95;
    other.energy *= 0.95;
}

bool Particle::isColliding(const Particle& other) const {
    // Check if particles are within collision distance
    double dx = x - other.x;
    double dy = y - other.y;
    double distance = std::sqrt(dx*dx + dy*dy);
    
    return distance < (PARTICLE_RADIUS + other.PARTICLE_RADIUS);
}