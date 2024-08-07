#pragma once

#include "MCL_Abstruct.hpp"

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

struct SensorData
{
    double distance;
    double degree;
};

struct SensorModel
{
    std::vector<SensorData> data;
    double max_range;
};

using Map = std::vector<std::vector<int>>;

class MCL2D : public AbstructMCL<Pose, MotionModel, SensorModel, Map>
{
public:
    double sigma;
    double map_threshold;

    std::vector<Particle> current_particles;

    MCL2D() = default;
    MCL2D(int num, Map map, double sig, double threshold);
    void debug() override;

private:
    void motion_update(const MotionModel &motion) override;

    double calculate_weight(const Particle &particle, const SensorModel &sensor_data, const Map &map) override;

    Pose prediction_pose() override;

    void resampling(const Map &map) override;

    int map_size_y, map_size_x;
};