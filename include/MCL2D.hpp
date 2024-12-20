#pragma once

#include "MCL_Abstruct.hpp"
#include "GridMap/Map.hpp"
#include "Sensor.hpp"
#include "Utility/Angle.hpp"

struct Pose
{
    double x, y;
    Utility::Angle angle;
};
struct MotionModel
{
    double delta_distance;
    Utility::Angle delta_angle;
};

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

class MCL2D : public AbstructMCL<Pose, MotionModel, Sensor::Model, Grid::Map>
{
public:
    MCLConfig config;

    std::vector<Particle> current_particles;

    MCL2D() = default;
    MCL2D(Grid::Map map, MCLConfig config, bool (*comp)(double, double) = [](double value, double threshould)
                                           { return value <= threshould; });
    void debug() override;

    double LikelihoodFieldModelOnce(const Grid::Pos &point, const double &max_range, const Grid::Map &map);
    double LikelihoodFieldModelOnce(const Grid::Pos &point, const double &max_range, const Grid::Map &map, const double &map_max_distance, const double &variance, const double &hit_prob, const double &rand_prob);

private:
    void motion_update(const MotionModel &motion) override;

    double calculate_weight(const Particle &particle, const Sensor::Model &sensor_data, const Grid::Map &map) override;

    Pose prediction_pose() override;

    void resampling(const Grid::Map &map) override;

    bool (*check_blank_func)(double, double); // グリッドに障害物がないかの判定
};