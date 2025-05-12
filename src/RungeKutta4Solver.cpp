#include "RungeKutta4Solver.h"
#include "System.h"

#define DAMP 0.98f

void runge_kutta_4_solver(float dt)
{
    int dimensions = get_dim();

    std::vector<float> sVector = get_state();
    // k1 = f(s, t)
    std::vector<float> k1 = deriv_eval();

    // k2 = f(s + dt/2 * k1, t + dt/2)
    std::vector<float> s_temp(dimensions);
    for (int i = 0; i < dimensions; i++)
    {
        s_temp[i] = sVector[i] + 0.5f * dt * k1[i];
    }
    set_state(s_temp);
    std::vector<float> k2 = deriv_eval();

    // k3 = f(s + dt/2 * k2, t + dt/2)
    for (int i = 0; i < dimensions; i++)
    {
        s_temp[i] = sVector[i] + 0.5f * dt * k2[i];
    }
    set_state(s_temp);
    std::vector<float> k3 = deriv_eval();

    // k4 = f(s + dt * k3, t + dt)
    for (int i = 0; i < dimensions; i++)
    {
        s_temp[i] = sVector[i] + dt * k3[i];
    }
    set_state(s_temp);
    std::vector<float> k4 = deriv_eval();

    // Final update: s_new = s + dt/6 * (k1 + 2*k2 + 2*k3 + k4)
    for (int i = 0; i < dimensions; i++)
    {
        sVector[i] += dt * (k1[i]/6.0f + k2[i]/3.0f + k3[i]/3.0f + k4[i]/6.0f);
    }

    set_state(sVector);
}