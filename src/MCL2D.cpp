#include "MCL2D.hpp"
#include "Utility/Random.hpp"
#include "Utility/Angle.hpp"
#include <iostream>
#include <fstream>
#include <cmath>
#include <omp.h>

MCL2D::MCL2D(Grid::Map map, MCLConfig mcl_config, bool (*comp)(double, double))
{
    config = mcl_config;
    check_blank_func = comp;

    if (config.init_pos != nullptr)
    {
        // Localization
        particles.clear();
        for (int i = 0; i < config.particle_num; i++)
        {
            Particle p;
            p.x = Utility::Random::NormalDistribution(config.init_pos->x(), 2 * config.sigma);
            p.y = Utility::Random::NormalDistribution(config.init_pos->y(), 2 * config.sigma);

            p.angle.set_degree(Utility::Random::Real(0, 359));
            p.weight = 1.0 / config.particle_num;
            particles.push_back(p);
        }
    }
    else
    {
        // Global Localization
        particles.clear();
        for (int i = 0; i < config.particle_num; i++)
        {
            Particle p;
            int x = Utility::Random::Integer(map.min_corner.x(), map.max_corner.x());
            int y = Utility::Random::Integer(map.min_corner.y(), map.max_corner.y());

            p.x = (double)x;
            p.y = (double)y;
            p.angle.set_degree(Utility::Random::Real(0, 359));
            p.weight = 1.0 / config.particle_num;
            particles.push_back(p);
        }
    }

    current_particles = particles;
    pose = Pose{};
}

void MCL2D::motion_update(const MotionModel &motion)
{
    for (auto &p : particles)
    {
        auto dx = Utility::Random::NormalDistribution(motion.delta_distance, config.sigma);
        auto dy = Utility::Random::NormalDistribution(motion.delta_distance, config.sigma);
        auto angle = Utility::Random::NormalDistribution(motion.delta_angle.get_degree(), 2.0);

        p.x += dx;
        p.y += dy;
        p.angle += Utility::Angle::FromDegree(angle);
    }
}

double MCL2D::LikelihoodFieldModelOnce(const Grid::Pos &point, const double &max_range, const Grid::Map &map)
{
    const auto &map_max_distance = config.distance_map_max_value;
    const auto &variance = config.variance;
    const auto &hit_prob = config.hit_prob;
    const auto &rand_prob = config.rand_prob;
    return LikelihoodFieldModelOnce(point, max_range, map, map_max_distance, variance, hit_prob, rand_prob);
}

double MCL2D::LikelihoodFieldModelOnce(const Grid::Pos &point, const double &max_range, const Grid::Map &map, const double &map_max_distance, const double &variance, const double &hit_prob, const double &rand_prob)
{
    auto map_itr = map.find(point);
    double distance = map_itr != map.end() ? map_itr->second.distance() : map_max_distance;

    return hit_prob * exp(-distance * distance / (2 * variance * variance)) + rand_prob / max_range;
}

double MCL2D::distance_duration(const Grid::Pos &pos, const Utility::Angle &angle, const Grid::Map &map, double sensor_distance, const double &max_sensor_distance)
{
    auto map_itr = map.find(pos);
    double x = 0;
    double y = 0;

    while ((map_itr == map.end() or map_itr->second.distance() > 0) and x * x + y * y < max_sensor_distance * max_sensor_distance)
    {
        x += angle.cos();
        y += angle.sin();
        auto p = pos + Grid::Pos(x, y);
        map_itr = map.find(p);
    }
    auto dis = std::min(std::sqrt(x * x + y * y), max_sensor_distance);
    return sensor_distance - dis;
}

double MCL2D::calculate_weight(const Particle &particle, const Sensor::Model &sensor_data, const Grid::Map &map)
{
    // パーティクルごとに各点群のマッチングをする
    double weight = 1e-10;
    double weight_log = 0;
    const auto pos = Common::RealPos(particle.x, particle.y);
    const auto integer_pos = Grid::Pos(pos);

    for (double degree_ = 0; degree_ < 360.0; degree_ += sensor_data.increment_angle)
    {
        auto angle = Utility::Angle::FromDegree(degree_);
        auto iter = sensor_data.data.find(angle);
        double current_dis;
        if (iter == sensor_data.data.end())
        {
            continue;
            current_dis = sensor_data.max_range;
        }
        else
        {
            current_dis = iter->second;
        }

        angle += particle.angle;
        auto dis = distance_duration(integer_pos, angle, map, current_dis, sensor_data.max_range);

        auto v = config.hit_prob * exp(-dis * dis / (2 * config.variance * config.variance)) + config.rand_prob / sensor_data.max_range;
        weight += v * v * v;
        weight_log += log(v);
    }

    return particle.weight * weight;
}

Pose MCL2D::prediction_pose()
{
    current_particles = particles;

    Pose new_pose{};
    double deg_x = 0, deg_y = 0;
    for (auto p : particles)
    {
        new_pose.x += p.x * p.weight;
        new_pose.y += p.y * p.weight;

        deg_x += p.angle.cos() * p.weight;
        deg_y += p.angle.sin() * p.weight;
    }
    new_pose.angle = Utility::Angle::FromRadian(atan2(deg_y, deg_x));
    return new_pose;
}

void MCL2D::resampling(const Grid::Map &map)
{
    std::vector<Particle> new_particles;
    new_particles.clear();

    double ess = 0.0;
    for (auto &p : particles)
    {
        ess += p.weight * p.weight;
    }
    ess = 1.0 / ess;

    if (ess > 0.5 * particles.size())
    {
        return;
    }

    {
        // low-variance resampling
        double step = 1.0 / config.particle_num;
        double start = Utility::Random::Real(0.0, step);
        double target_weight = start;
        double sum_weight = 0.0;
        int index = -1;
        for (int i = 0; i < config.particle_num; i++)
        {
            while (target_weight > sum_weight)
            {
                index++;
                sum_weight += particles[index].weight;
            }
            new_particles.push_back(particles[index]);
            target_weight += step;
        }

        // random resampling方法もある
        // std::vector<bool> check_exist(particles.size(), false);
        // for (std::size_t i = 0; i < config.particle_num; i++)
        // {
        //     if (Utility::Random::Real(0.0, 1.0) < config.resmapling_prob)
        //     {
        //         // random_sampliing
        //         Particle p;
        //         int x = Utility::Random::Integer(map.min_corner.x(), map.max_corner.x());
        //         int y = Utility::Random::Integer(map.min_corner.y(), map.max_corner.y());

        //         p.x = (double)x;
        //         p.y = (double)y;
        //         p.angle.set_degree(Utility::Random::Real(0, 359));
        //         p.weight = 1.0 / config.particle_num;

        //         new_particles.push_back(p);
        //     }
        //     else
        //     {
        //         double var = Utility::Random::Real(0.0, 1.0);
        //         double sum = 0;
        //         std::size_t index = 0;
        //         for (std::size_t j = 0; j < particles.size(); j++)
        //         {
        //             sum += particles[j].weight;
        //             if (var <= sum)
        //             {
        //                 index = j;
        //                 break;
        //             }
        //         }

        //         check_exist[index] = true;
        //         new_particles.push_back(particles[index]);
        //     }
        // }
    }

    particles = std::move(new_particles);
}

void MCL2D::debug()
{
    using std::cout;
    using std::endl;
    cout << "(" << pose.x << "," << pose.y << ")" << " angle:" << pose.angle << endl;

    std::ofstream file("test.csv");
    for (auto p : particles)
    {
        if (p.weight > 1.0 / particles.size() / 2)
            file << p.x << "," << p.y << "," << p.weight << "," << p.angle << "," << endl;
    }

    file.close();
}