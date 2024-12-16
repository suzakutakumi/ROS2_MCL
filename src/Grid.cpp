#include "MCL2D.hpp"
#include <memory>
#include <utility>

using namespace Grid;

template <>
Pos::Pos(const SensorOne &p) : pair((int)p.x, (int)p.z)
{
}

Pos operator+(const Pos &x, const Pos &y)
{
    return Pos(x.first + y.first, x.second + y.second);
}

Pos operator-(const Pos &x, const Pos &y)
{
    return Pos(x.first - y.first, x.second - y.second);
}