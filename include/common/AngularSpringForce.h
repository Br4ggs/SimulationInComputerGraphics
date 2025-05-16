#pragma once

#include "Particle.h"
#include "Force.h"
#include <vector>
#include <gfx/vec2.h>

class AngularSpringForce : public Force 
{
public:
    AngularSpringForce(Particle *p1, Particle * p2, Particle * p3, double rest_angle, double stiffness_constant);

    void draw() override;

    void apply_force() override;

 private:
    Particle * const m_p1;   // particle 1
    Particle * const m_p2;   // particle 2 
    Particle * const m_p3;   // particle 3
    double const m_rest_angle;     // rest length
    double const m_stiffness_constant;
};
