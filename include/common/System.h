#pragma once

#include <vector>

int get_dim();
std::vector<float> get_state();
void set_state(std::vector<float> sVector);
std::vector<float> deriv_eval();
