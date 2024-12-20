#include <iostream>
#include "Utility/Angle.hpp"

std::ostream &operator<<(std::ostream &os, const Utility::Angle &r)
{
    os << r.get_degree();
    return os;
}
