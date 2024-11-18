#include "MCL2D.hpp"

GridType GridPos(const SensorOne &p)
{
    return GridType((int)p.x, (int)p.z);
}

GridType operator+(const GridType x, const GridType y)
{
    return GridType(x.first + y.first, x.second + y.second);
}

GridType operator-(const GridType x, const GridType y)
{
    return GridType(x.first - y.first, x.second - y.second);
}