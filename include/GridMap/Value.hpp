#pragma once

#include <utility>

namespace Grid
{
    struct Value : public std::pair<double, double>
    {
        using pair::pair;

        double &prob() { return this->first; };
        double &distance() { return this->second; };
        const double& prob() const { return this->first; };
        const double& distance() const { return this->second; };
    };
}