#include "MCL2D.hpp"
#include "utils.hpp"
#include <iostream>
#include <fstream>
#include <omp.h>

MCL2D::MCL2D(Grid::Map map, MCLConfig mcl_config, bool (*comp)(double, double))
{
    config = mcl_config;
    check_blank_func = comp;

    pose = Pose{0, 0, 0};

    // Global Localization
    particles.clear();
    for (int i = 0; i < config.particle_num; i++)
    {
        Particle p;
        while (true)
        {
            int x = utils::common_random::randint(map.min_corner.x, map.max_corner.x);
            int y = utils::common_random::randint(map.min_corner.y, map.max_corner.y);
            auto iter = map.find(Grid::Pos(x, y));
            if (iter == map.end() or check_blank_func(iter->second.prob, config.map_threshold))
            {
                p.x = (double)x;
                p.y = (double)y;
                break;
            }
        }
        p.deg = (double)utils::common_random::randint(0, 359);
        p.weight = 1.0 / config.particle_num;
        particles.push_back(p);
    }

    current_particles = particles;
    pose = Pose{0, 0, 0};
}

void MCL2D::motion_update(const MotionModel &motion)
{
    for (auto &p : particles)
    {
        auto dis = utils::common_random::random_normal_distribution(motion.delta_distance, config.sigma);
        auto deg = utils::common_random::random_normal_distribution(motion.delta_degree, config.sigma);

        p.x += dis * cos(p.deg * M_PI / 180);
        p.y += dis * sin(p.deg * M_PI / 180);
        p.deg = utils::deg_mod(p.deg + deg);
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
    double distance = map_itr != map.end() ? map_itr->second.distance : map_max_distance;

    return hit_prob * exp(-distance * distance / (2 * variance * variance)) + rand_prob / max_range;
}

double MCL2D::calculate_weight(const Particle &particle, const SensorModel &sensor_data, const Grid::Map &map)
{
    // パーティクルごとに各点群のマッチングをする
    double weight = 0;
    const auto pos = Grid::Pos(particle.x, particle.y);
    const auto cos_ = cos(particle.deg * M_PI / 180), sin_ = sin(particle.deg * M_PI / 180);

#pragma omp parallel for reduction(+ : weight)
    for (const auto &s : sensor_data.data)
    {
        auto data = Grid::Pos(s);
        auto point = pos + Grid::Pos(data.x * cos_ - data.y * sin_, data.x * sin_ + data.y * cos_);
        auto p = LikelihoodFieldModelOnce(point, sensor_data.max_range, map);
        weight += p;
    }

    return weight;
}

Pose MCL2D::prediction_pose()
{
    current_particles = particles;

    Pose new_pose{0, 0, 0};
    double deg_x = 0, deg_y = 0;
    for (auto p : particles)
    {
        new_pose.x += p.x * p.weight;
        new_pose.y += p.y * p.weight;

        deg_x += cos(p.deg * M_PI / 180) * p.weight;
        deg_y += sin(p.deg * M_PI / 180) * p.weight;
    }
    new_pose.deg = utils::deg_mod(atan2(deg_y, deg_x) * 180 / M_PI);
    return new_pose;
}

void MCL2D::resampling(const Grid::Map &map)
{
    std::vector<Particle> new_particles;
    new_particles.clear();
    for (std::size_t i = 0; i < particles.size(); i++)
    {
        if (utils::common_random::randint(0, 99) / 100.0 < config.resmapling_prob)
        {
            double var = utils::common_random::randdouble(0, 1);
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
            new_particles.push_back(new_p);
        }
        else
        {
            Particle p;
            while (true)
            {
                int x = utils::common_random::randint(map.min_corner.x, map.max_corner.x);
                int y = utils::common_random::randint(map.min_corner.y, map.max_corner.y);
                auto iter = map.find(Grid::Pos(x, y));
                if (iter == map.end() or check_blank_func(iter->second.prob, config.map_threshold))
                {
                    p.x = (double)x;
                    p.y = (double)y;
                    break;
                }
            }
            p.deg = (double)utils::common_random::randdouble(0, 359);
            p.weight = 0;
            new_particles.push_back(p);
        }
    }
    particles = std::move(new_particles);
}

void MCL2D::debug()
{
    using std::cout;
    using std::endl;
    cout << "(" << pose.x << "," << pose.y << ")" << " angle:" << pose.deg << endl;

    std::ofstream file("test.csv");
    for (auto p : particles)
    {
        if (p.weight > 1.0 / particles.size() / 2)
            file << p.x << "," << p.y << "," << p.weight << "," << p.deg << "," << endl;
    }

    file.close();
}