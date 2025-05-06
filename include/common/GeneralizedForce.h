#ifndef GENERALIZED_FORCE_H
#define GENERALIZED_FORCE_H

#include <vector>
#include <memory> // For std::shared_ptr
#include "Particle.h"

class Force
{
public:
    virtual void applyForce(std::vector<Particle*>& particles) = 0;
    virtual void draw() const = 0;
    virtual ~Force() = default;
};

// Generalized force system to manage multiple forces

class GeneralizedForce
{
private:
std::vector<std::shared_ptr<Force>> forces;

public:
    // Add a force to the sytem
    void addForce(const std::shared_ptr<Force>& force) {
        forces.push_back(force);
    }

    // Clear all forces from the system
    void clearForces() {
        forces.clear();
    }

    // Apply all forces to the particles
    void applyForces(std::vector<Particle*>& particles) {
        for (const auto& force : forces) {
            force->applyForce(particles);
        }
    }

    // Draw all forces in the system
    void drawForces() const {
        for (const auto& force : forces) {
            force->draw();
        }
    }
};

#endif // GENERALIZED_FORCE_H