#pragma once

#include <utility>
#include <functional>

namespace Grid
{
    struct Pos : public std::pair<int, int>
    {
        using pair::pair;

        template <typename T>
        Pos(const T &);

        int &x = this->first;
        int &y = this->second;
    };
}

Grid::Pos operator+(const Grid::Pos &x, const Grid::Pos &y);
Grid::Pos operator-(const Grid::Pos &x, const Grid::Pos &y);

template <class T, class S>
struct std::hash<std::pair<T, S>>
{
    size_t operator()(const std::pair<T, S> &keyval) const noexcept
    {
        auto seed = std::hash<T>()(keyval.first);
        return seed ^ (std::hash<S>()(keyval.second) + 0x9e3779b9 + (seed << 6) + (seed >> 2));
    }
};

template <>
struct std::hash<Grid::Pos>
{
    size_t operator()(const Grid::Pos &keyval) const noexcept
    {
        return std::hash<Grid::Pos::pair>()(keyval);
    }
};
