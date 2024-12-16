#pragma once

#include <utility>

namespace Grid
{
    struct Value : public std::pair<double, double>
    {
        using pair::pair;

        double &prob = this->first;
        double &distance = this->second;
    };
}