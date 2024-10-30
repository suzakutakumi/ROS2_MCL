#include "Error.hpp"
#include <iostream>

using std::cout;
using std::endl;

inline double calc_eigenvalue1(double &x, double &y, double &xy)
{
    return (x + y) / 2 + sqrt((x - y) * (x - y) / 4 + xy * xy);
}
inline double calc_eigenvalue2(double &x, double &y, double &xy)
{
    return (x + y) / 2 - sqrt((x - y) * (x - y) / 4 + xy * xy);
}

double calc_error_ellipse(const std::vector<std::pair<double, double>> &points, double ganma, double vc)
{
    double mean_x = 0, mean_y = 0;
    for (auto &p : points)
    {
        mean_x += p.first;
        mean_y += p.second;
    }
    mean_x /= points.size();
    mean_y /= points.size();

    double variance_x = 0, variance_y = 0, variance_xy = 0;
    for (auto &p : points)
    {
        variance_x += (p.first - mean_x) * (p.first - mean_x);
        variance_y += (p.second - mean_y) * (p.second - mean_y);

        variance_xy += (p.first - mean_x) * (p.second - mean_y);
    }
    variance_x /= points.size();
    variance_y /= points.size();
    variance_xy /= points.size();

    auto lambda1 = calc_eigenvalue1(variance_x, variance_y, variance_xy);
    auto lambda2 = calc_eigenvalue2(variance_x, variance_y, variance_xy);

    auto ra = sqrt(lambda1);
    auto rb = sqrt(lambda2);

    return exp(-ganma * (ra * ra + rb * rb)) + vc;
}