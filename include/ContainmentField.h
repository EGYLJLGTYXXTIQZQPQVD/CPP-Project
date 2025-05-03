#pragma once

#include <vector>
#include <memory>
#include <mutex>
#include <utility> // Include for std::pair

struct Config;  // Forward declaration
class Particle; // Forward declaration

class ContainmentField
{
public:
    ContainmentField(const Config &config);
    ~ContainmentField();

    double getSize() const;

    // Check if particle is within the square field boundaries
    bool isParticleContained(const Particle &particle) const;

    // Calculate the containment force vector (fx, fy) acting on the particle
    // Force points inwards towards the center (0,0).
    // Magnitude increases linearly with distance from the center, proportional to fieldStrength.
    std::pair<double, double> getContainmentForce(const Particle &particle) const;

    void setFieldStrength(double strength);
    double getFieldStrength() const;

    // Field energy concept seems unused/unclear from README, return 0 for now.
    double getFieldEnergy() const;

    // Update field properties over time (e.g., decay strength)
    void update(double dt);

    void setDecayRate(double rate);
    double getDecayRate() const;

private:
    double size;          // The side length of the square containment field
    double fieldStrength; // Current strength of the containment field
    double decayRate;     // Rate at which fieldStrength decreases over time

    // These seem unused based on README force description, kept for potential future use/completeness
    const size_t GRID_SIZE;
    std::vector<double> fieldData;
    double fieldEnergy; // This also seems unused/unclear

    mutable std::mutex fieldMutex; // Mutex to protect shared field properties

    void initializeField(); // Private helper if needed for fieldData initialization

    // Removed EnergyPulse struct and vector as they were unused
};