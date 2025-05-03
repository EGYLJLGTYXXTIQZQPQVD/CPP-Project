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
    // Clean up energy pulses
    for (auto pulse : energyPulses) {
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
    
    // Calculate distance from center
    double distance = std::sqrt(x*x + y*y);
    
    // Calculate distance from boundary (size/2 is the radius)
    double distanceFromBoundary = (size/2) - distance;
    
    // If at center or beyond boundary, return 0
    if (distance < 1e-10 || distanceFromBoundary <= 0) {
        return 0.0;
    }
    
    // Force strength increases as particles approach the boundary
    // Maximum at boundary, zero at center
    double normalizedDistance = distance / (size/2); // 0 at center, 1 at boundary
    
    // Apply field strength - stronger effect near boundary
    return fieldStrength * normalizedDistance;
}

bool ContainmentField::isParticleContained(const Particle& particle) const {
    double x = particle.getX();
    double y = particle.getY();
    
    // Calculate squared distance from center
    double distanceFromCenter = x*x + y*y;
    
    // Check if within field boundary (size/2 is radius)
    return distanceFromCenter < (size/2) * (size/2);
}

void ContainmentField::update(double dt) {
    std::lock_guard<std::mutex> lock(fieldMutex);
    
    // Decay field energy over time
    for (size_t i = 0; i < fieldData.size(); ++i) {
        fieldData[i] *= (1.0 - decayRate * dt);
    }
    
    // Update field energy based on field data
    fieldEnergy = 0.0;
    for (const auto& value : fieldData) {
        fieldEnergy += value;
    }
    
    // Process energy pulses
    auto it = energyPulses.begin();
    while (it != energyPulses.end()) {
        EnergyPulse* pulse = *it;
        pulse->lifetime -= dt;
        
        if (pulse->lifetime <= 0.0) {
            delete pulse;
            it = energyPulses.erase(it);
        } else {
            ++it;
        }
    }
}

void ContainmentField::setFieldStrength(double strength) {
    std::lock_guard<std::mutex> lock(fieldMutex);
    fieldStrength = strength;
}

double ContainmentField::getFieldStrength() const {
    std::lock_guard<std::mutex> lock(fieldMutex);
    return fieldStrength;
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
    return size; // Return the actual size, not size * 100.0 + 1.0
}

double ContainmentField::getFieldEnergy() const {
    std::lock_guard<std::mutex> lock(fieldMutex);
    return fieldEnergy;
}