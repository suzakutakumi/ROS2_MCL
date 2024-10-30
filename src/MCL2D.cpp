#include "MCL2D.hpp"
#include "utils.hpp"
#include <iostream>
#include <fstream>
#include <omp.h>

MCL2D::MCL2D(int num, GridMap map, double sig, double threshold, bool (*comp)(double, double))
{
    check_blank_func = comp;
    map_threshold = threshold;

    particles.clear();
    for (int i = 0; i < num; i++)
    {
        Particle p;
        while (true)
        {
            int index = utils::common_random::randint(0, map.size() - 1);
            auto iter = std::next(map.begin(), index);
            if (check_blank_func(iter->second, map_threshold))
            {
                p.x = (double)iter->first.first;
                p.y = (double)iter->first.second;
                break;
            }
        }
        p.deg = (double)utils::common_random::randint(0, 359);
        p.weight = 1.0 / num;
        particles.push_back(p);
    }

    current_particles = particles;

    pose = Pose{0, 0, 0};
    sigma = sig;
}

void MCL2D::motion_update(const MotionModel &motion)
{
    for (auto &p : particles)
    {
        auto dis = utils::common_random::random_normal_distribution(motion.delta_distance, sigma);
        auto deg = utils::common_random::random_normal_distribution(motion.delta_degree, sigma);

        p.x += dis * cos(p.deg * M_PI / 180);
        p.y += dis * sin(p.deg * M_PI / 180);
        p.deg = utils::deg_mod(p.deg + deg);
    }
}

double MCL2D::calculate_weight(const Particle &particle, const SensorModel &sensor_data, const GridMap &map)
{
    // パーティクルごとに各点群のマッチングをする
    double weight = 0;
#pragma omp parallel for reduction(+ : weight)
    for (const auto &s : sensor_data.data)
    {
        auto deg = s.degree + particle.deg; // パーティクルの角度も足す

        // grid-map上にあるか
        auto x_step = cos(deg * M_PI / 180);
        auto y_step = sin(deg * M_PI / 180);

        double m_distance_y = 0;
        double m_distance_x = 0;

        int p_distance = 0;

        while (true)
        {
            int y = (int)(particle.y + m_distance_y);
            int x = (int)(particle.x + m_distance_x);

            auto map_itr = map.find(GridType(x, y));
            if (p_distance > sensor_data.max_range or
                (map_itr != map.end() and not check_blank_func(map_itr->second, map_threshold)))
            {
                break;
            }

            p_distance++;

            m_distance_y += y_step;
            m_distance_x += x_step;
        }

        if (std::isinf(s.distance))
        {
            // 観測範囲を正当に超えている
            if (p_distance > sensor_data.max_range)
            {
                weight += exp(-10);
            }
            else
            {
                weight += exp(-(sensor_data.max_range - p_distance) * (sensor_data.max_range - p_distance) / 2);
            }
        }
        else
        {
            weight += exp(-(s.distance - p_distance) * (s.distance - p_distance) / 2);
        }
    }

    return weight;
}

Pose MCL2D::prediction_pose()
{
    current_particles = particles;

    double current_angle = pose.deg;
    Pose new_pose{0, 0, 0};
    for (auto p : particles)
    {
        new_pose.x += p.x * p.weight;
        new_pose.y += p.y * p.weight;

        new_pose.deg += utils::deg_mod(current_angle - p.deg) * p.weight;
    }
    new_pose.deg = utils::deg_mod(current_angle - new_pose.deg);
    return new_pose;
}

void MCL2D::resampling(const GridMap &map)
{
    std::vector<Particle> new_particles;
    new_particles.clear();
    for (std::size_t i = 0; i < particles.size(); i++)
    {
        if (utils::common_random::randint(0, 99) < 70)
        {
            double var = utils::common_random::randdouble(0, 1);
            double sum = 0;
            for (auto p : particles)
            {
                sum += p.weight;
                if (var <= sum)
                {
                    new_particles.push_back(p);
                    break;
                }
            }
        }
        else
        {
            Particle p;
            while (true)
            {
                int index = utils::common_random::randint(0, map.size() - 1);
                auto iter = std::next(map.begin(), index);
                if (check_blank_func(iter->second, map_threshold))
                {
                    p.x = (double)iter->first.first;
                    p.y = (double)iter->first.second;
                    break;
                }
            }
            p.deg = (double)utils::common_random::randdouble(0, 359);
            p.weight = 0;
            new_particles.push_back(p);
        }
    }
    particles = new_particles;
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