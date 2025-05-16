#include "ExplicitEulerSolver.h"
#include "System.h"

#define DAMP 0.98f

void explicit_euler_solver(float dt)
{
    //TODO: solve system
    int dimensions = get_dim();

    std::vector<float> sVector = get_state();
    std::vector<float> dVector = deriv_eval();

    for (int i = 0; i < dimensions; i++)
    {
        sVector[i] += dt*dVector[i] * DAMP;
    }

    set_state(sVector);
}