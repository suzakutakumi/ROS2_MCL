#pragma once

#include "MCL_Abstruct.hpp"
#include <unordered_map>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h>

struct Pose
{
    double x, y;
    double deg;
};
struct MotionModel
{
    double delta_distance;
    double delta_degree;
};

using SensorOne = pcl::PointXYZRGB;
using SensorData = pcl::PointCloud<SensorOne>;

struct SensorModel
{
    SensorData data;
    double max_range;
};

template <class T>
size_t HashCombine(const size_t seed, const T &v)
{
    return seed ^ (std::hash<T>()(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2));
}
/* pair用 */
template <class T, class S>
struct std::hash<std::pair<T, S>>
{
    size_t operator()(const std::pair<T, S> &keyval) const noexcept
    {
        return HashCombine(std::hash<T>()(keyval.first), keyval.second);
    }
};

using GridType = std::pair<int, int>;
struct GridValue
{
    double prob;
    double distance;
};

class GridMap : public std::unordered_map<GridType, GridValue>
{
public:
    GridType min_corner{0, 0};
    GridType max_corner{0, 0};

    template <typename... Args>
    std::pair<GridMap::iterator, bool> emplace(Args &&...args)
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
    using ParentType = std::unordered_map<GridType, GridValue>;
    using ParentType::operator[];
};

GridType GridPos(const SensorOne &p);
GridType operator+(const GridType &x, const GridType &y);
GridType operator-(const GridType &x, const GridType &y);

struct MCLConfig
{
    int particle_num;              // パーティクルの数
    double sigma;                  // motion_update時の誤差
    double map_threshold;          // 障害物の有無の閾値
    double distance_map_max_value; // 距離マップの最大値
    double variance;               // 測定値の誤差の分散
    double hit_prob;               // 地図情報と測定値が一致する場合の信頼度
    double rand_prob;              // 測定のノイズがのっている場合の信頼度

    double resmapling_prob; // 統計的なリサンプリングになる確率
};

class MCL2D : public AbstructMCL<Pose, MotionModel, SensorModel, GridMap>
{
public:
    MCLConfig config;

    std::vector<Particle> current_particles;

    MCL2D() = default;
    MCL2D(GridMap map, MCLConfig config, bool (*comp)(double, double) = [](double value, double threshould)
                                         { return value <= threshould; });
    void debug() override;

    double LikelihoodFieldModelOnce(const GridType &point, const double &max_range, const GridMap &map);
    double LikelihoodFieldModelOnce(const GridType &point, const double &max_range, const GridMap &map, const double &map_max_distance, const double &variance, const double &hit_prob, const double &rand_prob);

private:
    void motion_update(const MotionModel &motion) override;

    double calculate_weight(const Particle &particle, const SensorModel &sensor_data, const GridMap &map) override;

    Pose prediction_pose() override;

    void resampling(const GridMap &map) override;

    bool (*check_blank_func)(double, double); // グリッドに障害物がないかの判定
};