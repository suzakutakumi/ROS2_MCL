#pragma once

#include <vector>
#include <iostream>
#include <thread>
#include <cmath>
#include <omp.h>

template <typename PoseType, typename MotionModelType, typename SensorModelType, typename MapType>
class AbstructMCL
{
public:
    AbstructMCL(){};

    void update(const MotionModelType &motion, const SensorModelType &sensor_data, const MapType &map)
    {
        motion_update(motion);

        sensor_update(sensor_data, map);

        pose = prediction_pose();

        resampling(map);
    }

    virtual void debug_update(const MotionModelType &motion, const SensorModelType &sensor_data, const MapType &map)
    {
        std::cout << "motion" << std::endl;
        motion_update(motion);
        std::cout << "sensor" << std::endl;
        sensor_update(sensor_data, map);
        std::cout << "prediction" << std::endl;
        pose = prediction_pose();
        std::cout << "resampling" << std::endl;
        resampling(map);
    };

    virtual void debug() {};

    PoseType pose;

protected:
    struct Particle : PoseType
    {
        double weight;
    };
    std::vector<Particle> particles;

    virtual void motion_update(const MotionModelType &motion) = 0;
    virtual double calculate_weight(const Particle &particle, const SensorModelType &sensor_data, const MapType &map) = 0;
    virtual PoseType prediction_pose() = 0;
    virtual void resampling(const MapType &map) = 0;

private:
    void sensor_update(const SensorModelType &sensor_data, const MapType &map)
    {
        // calculate weights of particles
        double weight_sum = 0;

        #pragma omp parallel for
        for (auto &p : particles)
        {
            p.weight = calculate_weight(p, sensor_data, map);
            weight_sum += p.weight;
        }

        for (auto &p : particles)
        {
            p.weight /= weight_sum;
        }
    }
};