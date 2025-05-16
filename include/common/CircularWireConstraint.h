#pragma once

#include "Particle.h"
#include "Constraint.h"

class CircularWireConstraint : public Constraint
{
public:
    CircularWireConstraint(Particle *p, const Vec2f & center, const double radius, unsigned int index);

    void draw();

    float C() override;

    float C_prim() override;

    void jacob(Matrix* J) override;
    
    void jacob_prim(Matrix* J_prim) override;

private:
    Particle * const m_p;
    Vec2f const m_center;
    double const m_radius;
    unsigned int index;
};
