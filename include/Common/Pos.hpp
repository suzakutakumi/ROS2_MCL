#pragma once

#include <utility>

namespace Common
{
    template <typename T>
    struct Pos : public std::pair<T, T>
    {
        using std::pair<T, T>::pair;

        template <typename V>
        Pos(const Pos<V> &v) : Pos(static_cast<T>(v.x), static_cast<T>(v.y))
        {
        }

        template <typename V>
        Pos(const V &);

        T &x = this->first;
        T &y = this->second;
    };

    using RealPos = Pos<double>;
    using IntegerPos = Pos<int>;
}

template <typename T>
Common::Pos<T> operator+(const Common::Pos<T> &x, const Common::Pos<T> &y)
{
    return Common::Pos<T>(x.first + y.first, x.second + y.second);
}

template <typename T>
Common::Pos<T> operator-(const Common::Pos<T> &x, const Common::Pos<T> &y)
{
    return Common::Pos<T>(x.first - y.first, x.second - y.second);
}
