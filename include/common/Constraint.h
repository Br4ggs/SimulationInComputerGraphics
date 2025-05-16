#pragma once

#include "Matrix.h"

class Constraint
{
public:
    virtual void draw() = 0;
    virtual float C() = 0;
    virtual float C_prim() = 0;
    virtual void jacob(Matrix* J) = 0;
    virtual void jacob_prim(Matrix* J_prim) = 0;
};