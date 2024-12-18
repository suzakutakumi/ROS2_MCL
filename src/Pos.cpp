#include "Sensor.hpp"

template <>
template <>
Common::RealPos::Pos(const Sensor::One &p)
{
    x() = p.x;
    y() = p.z;
}

template <>
template <>
Common::IntegerPos::Pos(const Sensor::One &p) : Pos(Common::RealPos(p))
{
}