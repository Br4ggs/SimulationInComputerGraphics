#pragma once

#include "Force.h"
#include "Particle.h"
#include <vector>
#include <gfx/vec2.h>

class GravityForce : public Force 
{
public:
    GravityForce(std::vector<Particle*> pVector, Vec2f gravitational_acceleration);

    void apply_force() override;

private:
    std::vector<Particle*> pVector; //the vector of particles that gravity applies to
    Vec2f gravitational_acceleration; //the direction and magnitude of gravity encoded in a vector
};