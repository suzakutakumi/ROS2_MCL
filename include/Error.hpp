#pragma once
#include <vector>
double calc_error_ellipse(const std::vector<std::pair<double, double>> &points, double ganma = 0.4, double vc = 0.05, double resolution = 0.05);