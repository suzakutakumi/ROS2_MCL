#pragma once

#include <iostream>
#include <cmath>
#include <type_traits>

namespace Utility
{
    class Angle
    {
    public:
        Angle() : angle(0.0) {}

        static Angle FromRadian(double rad)
        {
            auto cls = Angle();
            cls.set_radian(rad);
            return cls;
        }

        static Angle FromDegree(double deg)
        {
            auto cls = Angle();
            cls.set_degree(deg);
            return cls;
        }

        double get_radian() const { return angle; }
        void set_radian(double radian)
        {
            angle = radian;
            norm_angle();
        }

        double get_degree() const { return angle * 180 / M_PI; }
        void set_degree(double degree)
        {
            angle = degree * M_PI / 180;
            norm_angle();
        }

        double cos() const
        {
            return std::cos(angle);
        }

        double sin() const
        {
            return std::sin(angle);
        }

        double tan() const
        {
            return std::tan(angle);
        }

        Angle operator+(const Angle &r)
        {
            return Angle::FromRadian(get_radian() + r.get_radian());
        }

        Angle operator-(const Angle &r)
        {
            return Angle::FromRadian(get_radian() - r.get_radian());
        }

        Angle operator+=(const Angle &r)
        {
            auto cls = *this + r;
            set_radian(cls.get_radian());
            return cls;
        }

        Angle operator-=(const Angle &r)
        {
            auto cls = *this - r;
            set_radian(cls.get_radian());
            return cls;
        }

    private:
        double angle;

        void norm_angle()
        {
            if (angle >= 0.0)
                angle = std::fmod(angle, 2 * M_PI);
            else
                angle = 2 * M_PI + std::fmod(angle, 2 * M_PI);
        }
    };
}

std::ostream &operator<<(std::ostream &os, const Utility::Angle &r);