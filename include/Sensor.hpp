#pragma once

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h>

#include "Utility/Angle.hpp"
#include "Common/Pos.hpp"

namespace Sensor
{
    using One = pcl::PointXYZRGB;
    using Data=std::map<Utility::Angle,double>;

    struct Model
    {
        Data data;
        double max_range;
        double increment_angle;
    };
}

template <>
template <>
Common::RealPos::Pos(const Sensor::One &p);

template <>
template <>
Common::IntegerPos::Pos(const Sensor::One &p);
