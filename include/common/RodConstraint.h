#pragma once

#include "Particle.h"
#include "Constraint.h"

class RodConstraint : public Constraint
{ //Constraint object
public:
    RodConstraint(Particle *p1, Particle * p2, double dist);

    void draw() override;

    float C() override;

    float C_prim() override;

    void jacob(Matrix* J) override;

    //will populate a 2x2 block in the provided matrix
    void Jacob_p1();

    void Jacob_p2();

    void Jacob_prime_p1();

    void Jacob_prime_p2();

    //TODO: add functions for:
    //-jacobian particles p1 and p2
    //-time derivatives
    //-partial derivatives w.r.t x,y

 private:
    Particle * const m_p1;
    Particle * const m_p2;
    double const m_dist;
};