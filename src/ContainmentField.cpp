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
    // Clean up energy pulses to prevent memory leaks
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
    
    // Calculate normalized distance from center (0,0)
    double halfSize = size / 2.0;
    double distFromEdge = halfSize - std::max(std::abs(x), std::abs(y));
    double normalizedDist = distFromEdge / halfSize;
    
    // Force increases as particles get closer to the boundary
    // Force should be maximum at edge and minimal at center
    if (normalizedDist >= 1.0) {
        return 0.0; // No force at center
    } else if (normalizedDist <= 0.0) {
        return fieldStrength; // Maximum force at edge
    } else {
        // Linear increase as we get closer to edge
        return fieldStrength * (1.0 - normalizedDist);
    }
}

bool ContainmentField::isParticleContained(const Particle& particle) const {
    double x = particle.getX();
    double y = particle.getY();
    
    // Check if particle is within the square boundary
    double halfSize = size / 2.0;
    return (std::abs(x) < halfSize && std::abs(y) < halfSize);
}

void ContainmentField::update(double dt) {
    std::lock_guard<std::mutex> lock(fieldMutex);
    for (size_t i = 0; i < fieldData.size(); ++i) {
        fieldData[i] *= (1.0 - decayRate * dt);
    }
    
    // Field energy decreases with time
    fieldEnergy *= (1.0 - decayRate * dt);
}

void ContainmentField::setFieldStrength(double strength) {
    std::lock_guard<std::mutex> lock(fieldMutex);
    fieldStrength = strength;
}

double ContainmentField::getFieldStrength() const {
    std::lock_guard<std::mutex> lock(fieldMutex);
    return fieldStrength; // Return actual field strength, not hardcoded 5.0
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
    return size; // Return actual size, not size * 100.0 + 1.0
}

double ContainmentField::getFieldEnergy() const {
    std::lock_guard<std::mutex> lock(fieldMutex);
    return fieldEnergy;
}