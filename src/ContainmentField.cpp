#include "../include/ContainmentField.h"
#include "../include/Particle.h"
#include "../include/Config.h"
#include <cmath>
#include <algorithm> // For std::max
#include <limits>    // For epsilon comparison

ContainmentField::ContainmentField(const Config &config)
    : size(config.field_size > 0 ? config.field_size : 10.0),                      // Ensure size is positive
      fieldStrength(config.initial_strength >= 0 ? config.initial_strength : 1.0), // Ensure strength >= 0
      decayRate(config.initial_decay_rate >= 0 ? config.initial_decay_rate : 0.1), // Ensure decay rate >= 0
      GRID_SIZE(config.field_grid_size > 0 ? config.field_grid_size : 100),        // Ensure grid size > 0
      fieldEnergy(0.0)                                                             // Initialize unused energy to 0
{
    // Initialize the fieldData vector, even if currently unused by force logic
    initializeField();
}

ContainmentField::~ContainmentField()
{
    // Destructor usually empty unless managing raw pointers or specific resources
}

void ContainmentField::initializeField()
{
    // Resize fieldData based on GRID_SIZE, initialize elements to 0.0
    fieldData.resize(GRID_SIZE * GRID_SIZE, 0.0);
    // fieldEnergy might be initialized here too if it had a purpose
    fieldEnergy = 0.0;
}

// Returns the containment force vector (fx, fy)
// Force is directed towards the center (0,0)
// Magnitude increases linearly from 0 at center to max at distance size/2
std::pair<double, double> ContainmentField::getContainmentForce(const Particle &particle) const
{
    std::lock_guard<std::mutex> lock(fieldMutex); // Lock if reading potentially changing fieldStrength

    double x = particle.getX();
    double y = particle.getY();
    double distSq = x * x + y * y;

    // If particle is very close to the center, force is negligible (avoids division by zero)
    if (distSq < std::numeric_limits<double>::epsilon())
    {
        return {0.0, 0.0};
    }

    double dist = std::sqrt(distSq);
    double halfSize = size / 2.0;

    // If particle is outside the nominal boundary (dist > halfSize),
    // we can still apply force towards center, maybe even stronger?
    // Let's cap the distance used for force calculation at halfSize
    // Or let it grow linearly? README implies force is *inside*.
    // Let's apply force based on position, increasing linearly outwards.
    // Force = Strength * (distance / halfSize) * (unit vector towards center)
    double magnitudeFactor = fieldStrength * (dist / halfSize);

    // Unit vector towards center is (-x/dist, -y/dist)
    double fx = magnitudeFactor * (-x / dist);
    double fy = magnitudeFactor * (-y / dist);

    // Simplified: fx = -fieldStrength * (1.0 / halfSize) * x
    //             fy = -fieldStrength * (1.0 / halfSize) * y
    // Let's use the simplified version assuming force is proportional to displacement from center.
    fx = -fieldStrength * (2.0 / size) * x;
    fy = -fieldStrength * (2.0 / size) * y;

    return {fx, fy};
}

// Check if the particle is within the square boundaries [-size/2, +size/2]
bool ContainmentField::isParticleContained(const Particle &particle) const
{
    double halfSize = size / 2.0;
    // No lock needed here as 'size' is effectively constant or managed externally
    // Get particle position (assuming Particle getters are safe/correct)
    double x = particle.getX();
    double y = particle.getY();

    return std::abs(x) < halfSize && std::abs(y) < halfSize;
}

// Update: Decay the field strength over time
void ContainmentField::update(double dt)
{
    std::lock_guard<std::mutex> lock(fieldMutex);
    if (decayRate > 0 && dt > 0)
    {
        fieldStrength *= (1.0 - decayRate * dt);
        // Ensure field strength doesn't become negative
        fieldStrength = std::max(0.0, fieldStrength);
    }
    // Note: fieldData and fieldEnergy are not updated here as their purpose is unclear
    // from the README's core force description.
}

// Set field strength, ensuring it's non-negative
void ContainmentField::setFieldStrength(double strength)
{
    std::lock_guard<std::mutex> lock(fieldMutex);
    fieldStrength = std::max(0.0, strength); // Ensure strength is not negative
}

// Get current field strength
double ContainmentField::getFieldStrength() const
{
    std::lock_guard<std::mutex> lock(fieldMutex);
    return fieldStrength;
}

// Set decay rate, ensuring it's non-negative
void ContainmentField::setDecayRate(double rate)
{
    std::lock_guard<std::mutex> lock(fieldMutex);
    decayRate = std::max(0.0, rate); // Ensure rate is not negative
}

// Get current decay rate
double ContainmentField::getDecayRate() const
{
    std::lock_guard<std::mutex> lock(fieldMutex);
    return decayRate;
}

// Get the side length of the square field
double ContainmentField::getSize() const
{
    // size is generally constant after construction, mutex likely not needed,
    // but included for strict consistency if size could somehow be changed later.
    // std::lock_guard<std::mutex> lock(fieldMutex);
    return size;
}

// Get field energy (currently unused concept)
double ContainmentField::getFieldEnergy() const
{
    // std::lock_guard<std::mutex> lock(fieldMutex); // If fieldEnergy were mutable
    return fieldEnergy; // Returns 0.0 as initialized
}