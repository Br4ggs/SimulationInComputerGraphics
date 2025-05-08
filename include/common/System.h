#ifndef  SYSTEM_H
#define SYSTEM_H

#include <vector>

int get_dim();
std::vector<float> get_state();
void set_state(std::vector<float> sVector);
std::vector<float> deriv_eval();

#endif // ! SYSTEM_H
