#pragma once
#include <random>

namespace Utility
{
    namespace Random
    {
        std::random_device rd;
        std::mt19937 gen(rd());

        int Integer(int low, int high)
        {
            std::uniform_int_distribution<> dist(low, high);
            return dist(gen);
        }

        double Real(double low, double high)
        {
            std::uniform_real_distribution<double> dist(low, high);
            return dist(gen);
        }

        double NormalDistribution(double mean, double stddev)
        {
            std::normal_distribution<> dist(mean, stddev);
            return dist(gen);
        }
    }
}