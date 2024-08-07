#pragma once
#include <random>

namespace utils
{
    namespace common_random
    {
        std::random_device rd;
        std::mt19937 gen(rd());

        int randint(int low, int high)
        {

            std::uniform_int_distribution<> dist(low, high);
            return dist(gen);
        }

        double randdouble(double low, double high)
        {
            std::uniform_real_distribution<double> dist(low, high);
            return dist(gen);
        }

        double random_normal_distribution(double mean, double stddev)
        {
            std::normal_distribution<> dist(mean, stddev);
            return dist(gen);
        }
    }

    double deg_mod(double deg)
    {
        while (deg < 0.0f)
        {
            deg += 360.0f;
        }

        while (deg >= 360.0f)
        {
            deg -= 360.0f;
        }

        return deg;
    }
}