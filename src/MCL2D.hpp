#pragma once

#include "MCL_Abstruct.hpp"
#include <unordered_map>

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

template<class T> size_t HashCombine(const size_t seed,const T &v){
    return seed^(std::hash<T>()(v)+0x9e3779b9+(seed<<6)+(seed>>2));
}
/* pair用 */
template<class T,class S> struct std::hash<std::pair<T,S>>{
    size_t operator()(const std::pair<T,S> &keyval) const noexcept {
        return HashCombine(std::hash<T>()(keyval.first), keyval.second);
    }
};
using GridType = std::pair<int, int>;
using GridMap = std::unordered_map<GridType, double>;

class MCL2D : public AbstructMCL<Pose, MotionModel, SensorModel, GridMap>
{
public:
    double sigma;
    double map_threshold;

    std::vector<Particle> current_particles;

    MCL2D() = default;
    MCL2D(int num, GridMap map, double sig, double threshold, bool (*comp)(double, double) = [](double value, double threshould)
                                                              { return value <= threshould; });
    void debug() override;

private:
    void motion_update(const MotionModel &motion) override;

    double calculate_weight(const Particle &particle, const SensorModel &sensor_data, const GridMap &map) override;

    Pose prediction_pose() override;

    void resampling(const GridMap &map) override;

    bool (*check_blank_func)(double, double); // グリッドに障害物がないかの判定
};