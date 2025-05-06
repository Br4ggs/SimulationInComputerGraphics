#ifndef GENERALIZED_CONSTRAINT_H
#define GENERALIZED_CONSTRAINT_H

#include <vector>
#include <memory> // For std::shared_ptr
#include "Particle.h"

class Constraint
{
public:
    virtual void applyConstraint(std::vector<Particle*>& particles) = 0;
    virtual void draw() const = 0;
    virtual ~Constraint() = default;
};

// Generalized constraint system to manage multiple constraints

class GeneralizedConstraint
{
private:
std::vector<std::shared_ptr<Constraint>> constraints;

public:
    // Add a constraint to the system
    void addConstraint(const std::shared_ptr<Constraint>& constraint) {
        constraints.push_back(constraint);
    }

    // Clear all constraints from the system
    void clearConstraints() {
        constraints.clear();
    }

    // Apply all constraints to the particles
    void applyConstraints(std::vector<Particle*>& particles) {
        for (const auto& constraint : constraints) {
            constraint->applyConstraint(particles);
        }
    }

    // Draw all constraints in the system
    void drawConstraints() const {
        for (const auto& constraint : constraints) {
            constraint->draw();
        }
    }   
};

#endif // GENERALIZED_CONSTRAINT_H