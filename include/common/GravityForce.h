#ifndef GRAVITY_FORCE_H
#define GRAVITY_FORCE_H

#include <vector>
#include "Particle.h"
#include "GeneralizedForce.h"

class GravityForce : public Force {
private:
    Vec2f m_gravity; // Gravity vector
    
public:
    // Constructor to initialize gravity
    GravityForce(const Vec2f& gravity) : m_gravity(gravity) {};

    // Apply gravityForce to all particles (overriding applyForce method)
    void applyForce(std::vector<Particle*>& particles) override {
        for (auto& particle : particles) {
            particle->addForce(m_gravity); // Add gravity as a force to the particle
        }
    }
};

#endif // GRAVITY_FORCE_H