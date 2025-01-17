#include "MCL2D.hpp"
#include "Utility/Random.hpp"
#include "Utility/Angle.hpp"
#include <iostream>
#include <fstream>
#include <omp.h>

MCL2D::MCL2D(Grid::Map map, MCLConfig mcl_config, bool (*comp)(double, double))
{
    config = mcl_config;
    check_blank_func = comp;

    // Global Localization
    particles.clear();
    for (int i = 0; i < config.particle_num; i++)
    {
        Particle p;
        while (true)
        {
            int x = Utility::Random::Integer(map.min_corner.x(), map.max_corner.x());
            int y = Utility::Random::Integer(map.min_corner.y(), map.max_corner.y());
            auto iter = map.find(Grid::Pos(x, y));
            if (iter == map.end() or check_blank_func(iter->second.prob(), config.map_threshold))
            {
                p.x = (double)x;
                p.y = (double)y;
                break;
            }
        }
        p.angle.set_degree(Utility::Random::Real(0, 359));
        p.weight = 1.0 / config.particle_num;
        particles.push_back(p);
    }

    current_particles = particles;
    pose = Pose{};
}

void MCL2D::motion_update(const MotionModel &motion)
{
    for (auto &p : particles)
    {
        auto dis = Utility::Random::NormalDistribution(motion.delta_distance, config.sigma);
        auto angle = Utility::Random::NormalDistribution(motion.delta_angle.get_radian(), config.sigma);

        p.x += dis * p.angle.cos();
        p.y += dis * p.angle.sin();
        p.angle += Utility::Angle::FromRadian(angle);
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

double MCL2D::calculate_weight(const Particle &particle, const Sensor::Model &sensor_data, const Grid::Map &map)
{
    // パーティクルごとに各点群のマッチングをする
    double weight = 0;
    const auto pos = Common::RealPos(particle.x, particle.y);
    const auto cos_ = particle.angle.cos(), sin_ = particle.angle.sin();

#pragma omp parallel for reduction(+ : weight)
    for (const auto &s : sensor_data.data)
    {
        auto data = Common::RealPos(s);
        auto point = pos + Common::RealPos(data.x() * cos_ - data.y() * sin_, data.x() * sin_ + data.y() * cos_);
        auto p = LikelihoodFieldModelOnce(Grid::Pos(point), sensor_data.max_range, map);
        weight += p * p * p;
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
    for (std::size_t i = 0; i < particles.size(); i++)
    {
        if (Utility::Random::Real(0.0, 1.0) < config.resmapling_prob)
        {
            double var = Utility::Random::Real(0.0, 1.0);
            double sum = 0;
            Particle new_p;
            for (auto p : particles)
            {
                sum += p.weight;
                if (var <= sum)
                {
                    new_p = p;
                    break;
                }
            }
            new_p.weight = 1.0;
            new_particles.push_back(new_p);
        }
        else
        {
            Particle p;
            while (true)
            {
                int x = Utility::Random::Integer(map.min_corner.x(), map.max_corner.x());
                int y = Utility::Random::Integer(map.min_corner.y(), map.max_corner.y());
                auto iter = map.find(Grid::Pos(x, y));
                if (iter == map.end() or check_blank_func(iter->second.prob(), config.map_threshold))
                {
                    p.x = (double)x;
                    p.y = (double)y;
                    break;
                }
            }
            p.angle.set_degree(Utility::Random::Real(0, 359));
            p.weight = 1.0;
            new_particles.push_back(p);
        }
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