#include "GravityForce.h"

#include <cstdio>

GravityForce::GravityForce(std::vector<Particle*> pVector, Vec2f gravitational_acceleration)
    :pVector(pVector), gravitational_acceleration(gravitational_acceleration)
{}

void GravityForce::apply_force()
{
    // Force = mass * gravitation constant
    // Loop over all particles and add to force accumulator
    for (int i = 0; i < pVector.size(); ++i)
    {
        pVector[i]->m_Force += pVector[i]->f_Mass * gravitational_acceleration;
    }
}