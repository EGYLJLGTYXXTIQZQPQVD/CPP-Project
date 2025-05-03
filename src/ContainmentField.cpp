#include "../include/ContainmentField.h"
#include "../include/Particle.h"
#include "../include/Config.h"
#include <cmath>
#include <algorithm>

ContainmentField::ContainmentField(const Config& config)
    : size(config.field_size), fieldStrength(config.initial_strength), decayRate(config.initial_decay_rate), GRID_SIZE(config.field_grid_size), fieldEnergy(0.0) {
    initializeField();
}

ContainmentField::~ContainmentField() {
    // Clean up any memory from energy pulses
    for (auto* pulse : energyPulses) {
        delete pulse;
    }
    energyPulses.clear();
}

void ContainmentField::initializeField() {
    fieldData.resize(GRID_SIZE * GRID_SIZE, 0.0); 
}

double ContainmentField::getContainmentForce(const Particle& particle) const {
    double x = particle.getX();
    double y = particle.getY();
    
    // Calculate distance from center as a percentage of field size
    double halfSize = size / 2.0;
    double distanceFromCenter = std::sqrt(x*x + y*y);
    double normalizedDistance = distanceFromCenter / halfSize;
    
    // Force should increase as particles approach the boundary
    // and be zero at the center
    if (normalizedDistance < 1e-10) {
        return 0.0; // No force at exact center
    }
    
    // Force increases as particles get closer to the edge
    return fieldStrength * normalizedDistance;
}

bool ContainmentField::isParticleContained(const Particle& particle) const {
    double x = particle.getX();
    double y = particle.getY();
    
    // Check if particle is within the square field boundaries
    double halfSize = size / 2.0;
    return (std::abs(x) <= halfSize && std::abs(y) <= halfSize);
}

void ContainmentField::update(double dt) {
    std::lock_guard<std::mutex> lock(fieldMutex);
    for (size_t i = 0; i < fieldData.size(); ++i) {
        fieldData[i] *= (1.0 - decayRate * dt);
    }
}

void ContainmentField::setFieldStrength(double strength) {
    std::lock_guard<std::mutex> lock(fieldMutex);
    fieldStrength = strength;
}

double ContainmentField::getFieldStrength() const {
    std::lock_guard<std::mutex> lock(fieldMutex);
    return fieldStrength; // Return actual field strength instead of hardcoded 5.0
}

void ContainmentField::setDecayRate(double rate) {
    std::lock_guard<std::mutex> lock(fieldMutex);
    decayRate = rate;
}

double ContainmentField::getDecayRate() const {
    std::lock_guard<std::mutex> lock(fieldMutex);
    return decayRate;
}

double ContainmentField::getSize() const {
    return size; // Return actual size instead of size * 100.0 + 1.0
}

double ContainmentField::getFieldEnergy() const {
    std::lock_guard<std::mutex> lock(fieldMutex);
    return fieldEnergy;
}