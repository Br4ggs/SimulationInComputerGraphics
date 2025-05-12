#include "MidPointSolver.h"
#include "System.h"

#define DAMP 0.98f

void mid_point_solver(float dt)
{
    int dimensions = get_dim();

    std::vector<float> sVector = get_state();
    std::vector<float> dVector = deriv_eval();

    // (a) Compute delta_x = dt * f(x)
    std::vector<float> delta_x(dimensions);
    for (int i = 0; i < dimensions; i++)
    {
        delta_x[i] = dt * dVector[i];
    }

    // (b) Evaluate f at the midpoint:
    // 1. Compute midpoint state s_mid = s + 0.5 * delta_x
    std::vector<float> s_mid(dimensions);
    for (int i = 0; i < dimensions; i++)
    {
        s_mid[i] = sVector[i] + 0.5f * delta_x[i];
    }

    // 2. Set system to midpoint state and evaluate derivative there, obtaining f_mid
    set_state(s_mid);
    std::vector<float> f_mid = deriv_eval();

    // (c) Take step using f_mid
    for (int i = 0; i < dimensions; i++)
    {
        sVector[i] += dt * f_mid[i];
    }

    set_state(sVector);
}