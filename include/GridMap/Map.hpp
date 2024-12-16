#pragma once

#include <unordered_map>
#include "GridMap/Pos.hpp"
#include "GridMap/Value.hpp"

namespace Grid
{
    class Map : public std::unordered_map<Pos, Value>
    {
    public:
        Pos min_corner{0, 0};
        Pos max_corner{0, 0};

        template <typename... Args>
        std::pair<Map::iterator, bool> emplace(Args &&...args)
        {
            auto res = ParentType::emplace(std::forward<Args>(args)...);
            if (res.second)
            {
                auto pos = res.first->first;
                min_corner.first = std::min(pos.first, min_corner.first);
                min_corner.second = std::min(pos.second, min_corner.second);
                max_corner.first = std::max(pos.first, max_corner.first);
                max_corner.second = std::max(pos.second, max_corner.second);
            }
            return res;
        };

    private:
        using ParentType = unordered_map;
        using ParentType::operator[];
    };
}