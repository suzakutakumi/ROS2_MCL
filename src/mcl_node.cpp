#include <chrono>
#include <functional>
#include <algorithm>
#include <memory>
#include <string>
#include <vector>
#include <queue>

#include "MCL2D.hpp"
#include "Error.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <pcl_conversions/pcl_conversions.h>

using std::placeholders::_1;
using namespace std;

class MclNode : public rclcpp::Node
{
public:
  MclNode() : Node("mcl_node")
  {
    subscription1_ = create_subscription<sensor_msgs::msg::PointCloud2>("scan", 10, std::bind(&MclNode::scanCallback, this, _1));
    subscription2_ = create_subscription<sensor_msgs::msg::PointCloud2>("map", 10, std::bind(&MclNode::registerMap, this, _1));

    publisher_ = create_publisher<sensor_msgs::msg::Image>("map_image", 10);
    pose_and_likelihoods_publisher_ = create_publisher<std_msgs::msg::Float64MultiArray>("pose_and_likelihoods", 10);
    timer_ = create_wall_timer(500ms, std::bind(&MclNode::image_publisher, this));

    // MCLの設定値
    this->declare_parameter("num_of_particle", 10000);
    this->declare_parameter("sigma", 0.04);
    this->declare_parameter("map_threshold", 0.5);
    this->declare_parameter("map_max_distance", 4.0);
    this->declare_parameter("variance", 0.5);
    this->declare_parameter("hit_prob", 0.5);
    this->declare_parameter("rand_prob", 0.5);
    this->declare_parameter("resmapling_prob", 0.8);

    mcl_config.particle_num = get_parameter("num_of_particle").as_int();
    mcl_config.sigma = get_parameter("sigma").as_double();
    mcl_config.map_threshold = get_parameter("map_threshold").as_double();
    mcl_config.distance_map_max_value = get_parameter("map_max_distance").as_double();
    mcl_config.variance = get_parameter("variance").as_double();
    mcl_config.hit_prob = get_parameter("hit_prob").as_double();
    mcl_config.rand_prob = get_parameter("rand_prob").as_double();

    mcl_config.resmapling_prob = get_parameter("resmapling_prob").as_double();

    // MCL以外の値
    this->declare_parameter("map_resolution", 0.05);
    this->declare_parameter("error_ganma", 0.4);
    this->declare_parameter("error_vc", 0.05);
    this->declare_parameter("max_range", mcl_config.distance_map_max_value);

    resolution = get_parameter("map_resolution").as_double();
    ganma = get_parameter("error_ganma").as_double();
    vc = get_parameter("error_vc").as_double();
    sensor_max_range = get_parameter("max_range").as_double();

    adaptResolution(mcl_config.distance_map_max_value);
    adaptResolution(mcl_config.rand_prob);
    adaptResolution(mcl_config.variance);

    RCLCPP_INFO(get_logger(), "number of particles, resolution = %d, %f", mcl_config.particle_num, resolution);
  }

private:
  MCLConfig mcl_config;
  Grid::Map map;
  MCL2D mcl = MCL2D();
  vector<vector<vector<int>>> map_;

  double resolution;
  double ganma;
  double vc;
  double sensor_max_range;

  void adaptResolution(double &v)
  {
    v /= resolution;
  }

  void adaptResolution(Sensor::Data &ps)
  {
    for (auto &p : ps)
    {
      p.x /= resolution;
      p.y /= resolution;
      p.z /= resolution;
    }
  }

  void restoreResolution(double &v)
  {
    v *= resolution;
  }

  void restoreResolution(Sensor::Data &ps)
  {
    for (auto &p : ps)
    {
      p.x *= resolution;
      p.y *= resolution;
      p.z *= resolution;
    }
  }

  void updateGridMap(Sensor::Data &pointcloud)
  {
    double max_range = mcl_config.distance_map_max_value;

    std::vector<std::vector<double>> distance_map;
    for (int i = 0; i <= (int)max_range + 1; i++)
    {
      std::vector<double> row;
      for (int j = 0; j <= (int)max_range + 1; j++)
      {
        row.push_back(sqrt(i * i + j * j));
      }
      distance_map.push_back(row);
    }

    std::queue<std::pair<Grid::Pos, Grid::Pos>> q;
    for (auto &p : pointcloud)
    {
      const auto pos = Grid::Pos(p);
      if (map.find(pos) != map.end())
      {
        continue;
      }

      map.emplace(pos, Grid::Value{1.0, 0.0});

      q.emplace(pos, pos);
    }

    while (not q.empty())
    {
      auto v = q.front();
      q.pop();
      auto current_pos = v.first;
      auto root_pos = v.second;

      std::vector<Grid::Pos> round{Grid::Pos(1, 0), Grid::Pos(-1, 0), Grid::Pos(0, 1), Grid::Pos(0, -1)};
      for (auto &r : round)
      {
        auto new_pos = current_pos + r;
        auto distance_vec = new_pos - root_pos;
        auto distance = distance_map[abs(distance_vec.x())][abs(distance_vec.y())];
        if (distance > max_range)
          continue;

        auto map_itr = map.find(new_pos);
        if (map_itr == map.end())
        {
          map.emplace(new_pos, Grid::Value{0.0, distance});

          q.emplace(new_pos, root_pos);
        }
        else if (distance < map_itr->second.distance())
        {
          map_itr->second.distance() = distance;

          q.emplace(new_pos, root_pos);
        }
      }
    }
  }

  void registerMap(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    RCLCPP_INFO(get_logger(), "register map");

    Sensor::Data pointcloud;
    pcl::fromROSMsg(*msg, pointcloud);

    adaptResolution(pointcloud);

    updateGridMap(pointcloud);

    mcl = MCL2D(map, mcl_config);

    draw_image();
  }

  void scanCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    RCLCPP_INFO(get_logger(), "start scan");

    MotionModel motion_model{};
    Sensor::Model sensor_model;

    pcl::fromROSMsg(*msg, sensor_model.data);
    sensor_model.max_range = sensor_max_range;

    adaptResolution(sensor_model.data);
    adaptResolution(sensor_model.max_range);

    if (map.empty())
    {
      return;
      RCLCPP_INFO(get_logger(), "init");
      updateGridMap(sensor_model.data);

      mcl = MCL2D(map, mcl_config);
    }

    RCLCPP_INFO_STREAM(get_logger(), "number of sensor data: " << sensor_model.data.size());

    // RCLCPP_INFO(get_logger(), "mcl update");
    mcl.update(motion_model, sensor_model, map);
    RCLCPP_INFO(get_logger(), "position(%f, %f, %f deg)", mcl.pose.x * resolution, mcl.pose.y * resolution, mcl.pose.angle.get_degree());

    // RCLCPP_INFO(get_logger(), "calc likelihood");
    vector<pair<double, double>> points;
    for (auto &p : mcl.current_particles)
    {
      points.emplace_back(p.x, p.y);
    }
    auto likelihood1 = calc_error_ellipse(points, ganma, vc, resolution);
    RCLCPP_INFO_STREAM(get_logger(), "likelihood value1:\t" << likelihood1);

    auto likelihood2 = 1.0;
    auto likelihood3 = 0.0;
    auto likelihood4 = 0.0;
    const auto pos = Common::RealPos(mcl.pose.x, mcl.pose.y);
    const auto cos_ = mcl.pose.angle.cos(), sin_ = mcl.pose.angle.sin();
    const auto hit_prob = 1.0 - mcl_config.rand_prob / sensor_model.max_range;
    for (const auto &s : sensor_model.data)
    {
      auto data = Common::RealPos(s);
      auto point = pos + Common::RealPos(data.first * cos_ - data.second * sin_, data.first * sin_ + data.second * cos_);
      auto v = mcl.LikelihoodFieldModelOnce(Grid::Pos(point), sensor_model.max_range, map, mcl_config.distance_map_max_value, mcl_config.variance, hit_prob, mcl_config.rand_prob);
      likelihood2 *= v;
      likelihood3 += v;
      likelihood4 += log(v);
    }
    RCLCPP_INFO_STREAM(get_logger(), "likelihood value2-1(*=):\t" << likelihood2);
    RCLCPP_INFO_STREAM(get_logger(), "likelihood value2-2(+=):\t" << likelihood3 << ",\t" << likelihood3 / sensor_model.data.size());
    RCLCPP_INFO_STREAM(get_logger(), "likelihood value2-3(+=log):\t" << likelihood4 << ",\t" << exp(likelihood4));

    // 2D LiDARデータへ変換し、360度分の最短距離で尤度計算をする
    double sensor_likelihood = 0;
    int number_of_correct = 0;
    std::map<Utility::Angle, std::pair<Common::RealPos, double>> laser_data;
    for (const auto &s : sensor_model.data)
    {
      auto data = Common::RealPos(s);
      auto angle = Utility::Angle::FromRadian(std::atan2(data.y(), data.x()));
      auto distance = sqrt(data.x() * data.x() + data.y() * data.y());

      auto iter = laser_data.find(angle);
      if (iter != laser_data.end() and iter->second.second < distance)
      {
        continue;
      }

      laser_data.emplace(angle, std::make_pair(data, distance));
    }

    const double increment_unit = 1.0;
    double min_distance;
    bool check_exist_laser = false;
    Common::RealPos min_data;

    {
      auto iter = laser_data.rbegin();
      while (iter != laser_data.rend() and iter->first > Utility::Angle::FromDegree(360.0 - increment_unit / 2.0))
      {
        if (not check_exist_laser or iter->second.second < min_distance)
        {
          min_data = iter->second.first;
          min_distance = iter->second.second;
          check_exist_laser = true;
        }
        iter++;
      }
    }

    auto laser_iter = laser_data.begin();
    for (double angle = 0.0; angle < 360.0; angle += increment_unit)
    {
      while (laser_iter != laser_data.end() and laser_iter->first < Utility::Angle::FromDegree(angle + increment_unit / 2.0))
      {
        if (laser_iter->first < Utility::Angle::FromDegree(angle - increment_unit / 2.0))
        {
          laser_iter++;
          continue;
        }

        if (not check_exist_laser or laser_iter->second.second < min_distance)
        {
          min_data = laser_iter->second.first;
          min_distance = laser_iter->second.second;
          check_exist_laser = true;
        }
        laser_iter++;
      }

      double v;
      if (check_exist_laser)
      {
        auto point = pos + Common::RealPos(min_data.x() * cos_ - min_data.y() * sin_, min_data.x() * sin_ + min_data.y() * cos_);
        v = mcl.LikelihoodFieldModelOnce(Grid::Pos(point), sensor_model.max_range, map, mcl_config.distance_map_max_value, mcl_config.variance, hit_prob, mcl_config.rand_prob);
        number_of_correct++;
      }
      else
      {
        v = 0.2;
      }
      sensor_likelihood += log(v);

      min_data = Common::RealPos{};
      min_distance = __DBL_MAX__;
      check_exist_laser = false;
    }

    std_msgs::msg::Float64MultiArray pose_and_likelihoods;
    pose_and_likelihoods.data = std_msgs::msg::Float64MultiArray::_data_type{
        mcl.pose.x,
        mcl.pose.y,
        mcl.pose.angle.get_radian(),
        likelihood1,
        // likelihood2,
        // likelihood3 / sensor_model.data.size(),
        // likelihood3,
        likelihood4 / sensor_model.data.size(),
        likelihood4,
        sensor_likelihood,
        (double)number_of_correct,
        (double)number_of_correct / (360 / increment_unit),
    };
    restoreResolution(pose_and_likelihoods.data[0]);
    restoreResolution(pose_and_likelihoods.data[1]);

    pose_and_likelihoods_publisher_->publish(pose_and_likelihoods);

    draw_image();
    draw_image_now_state(mcl.pose, sensor_model);
  }

  void image_publisher()
  {
    if (map_.size() <= 0)
    {
      return;
    }
    RCLCPP_INFO(get_logger(), "publish image");

    sensor_msgs::msg::Image image;
    image.height = map_.size();
    image.width = map_[0].size();
    image.encoding = "rgb8";
    image.step = image.width * 3;

    for (auto itr = map_.rbegin(); itr != map_.rend(); ++itr)
    {
      for (auto v : *itr)
      {
        image.data.push_back(v[0]);
        image.data.push_back(v[1]);
        image.data.push_back(v[2]);
      }
    }

    publisher_->publish(image);
  }

  void draw_image()
  {
    if (map.size() <= 0)
    {
      return;
    }

    // 最初値・最大値の取得
    auto &min_range = map.min_corner;
    auto &max_range = map.max_corner;

    int width = (max_range - min_range).x() + 1;
    int height = (max_range - min_range).y() + 1;

    // 地図初期化
    map_ = vector<vector<vector<int>>>(height);
    for (int i = 0; i < height; i++)
    {
      vector<vector<int>> blank_row(width);
      for (int j = 0; j < width; j++)
      {
        blank_row[j] = vector<int>{0, 0, 0};
      }

      map_[i] = blank_row;
    }
    // 障害物の状態を表示
    for (auto &grid : map)
    {
      if (grid.second.prob() > 0.4)
      {
        auto pos = grid.first - min_range;
        map_[pos.y()][pos.x()] =
            vector<int>(3, (int)(255 * grid.second.prob()));
      }
    }

    // パーティクルの表示
    auto max_weight = std::max_element(mcl.current_particles.begin(), mcl.current_particles.end(), [](const auto &a, const auto &b)
                                       { return a.weight < b.weight; });
    for (auto &p : mcl.current_particles)
    {
      auto x = (int)p.x - min_range.x();
      auto y = (int)p.y - min_range.y();
      if (0 > y or y >= height or 0 > x or x >= width)
        continue;
      auto color = (int)(200 - p.weight * 200 / max_weight->weight);
      map_[y][x][0] = 255;
      map_[y][x][1] = color;
      map_[y][x][2] = color;
    }

    // 中央を表示
    for (int i = -1; i <= 1; i++)
    {
      for (int j = -1; j <= 1; j++)
      {
        auto y = -min_range.y() + i, x = -min_range.x() + j;
        if (0 > y or y >= height or 0 > x or x >= width)
          continue;
        map_[y][x][0] = 0;
        map_[y][x][1] = 0;
        map_[y][x][2] = 255;
      }
    }
    // 向きも表示
    {
      for (int i = 0; i < 10; i++)
      {
        auto x = (int)(-min_range.x() + i);
        auto y = (int)(-min_range.y());
        if (0 > y or y >= height or 0 > x or x >= width)
          continue;
        map_[y][x][0] = 0;
        map_[y][x][1] = 0;
        map_[y][x][2] = 255;
      }
    }

    // 予測自己位置を表示
    for (int i = -1; i <= 1; i++)
    {
      for (int j = -1; j <= 1; j++)
      {
        auto y = (int)mcl.pose.y - min_range.y() + i, x = (int)mcl.pose.x - min_range.x() + j;
        if (0 > y or y >= height or 0 > x or x >= width)
          continue;

        map_[y][x][0] = 0;
        map_[y][x][1] = 255;
        map_[y][x][2] = 0;
      }
    }
    // 向きも表示
    {
      auto x_step = mcl.pose.angle.cos(), y_step = mcl.pose.angle.sin();
      for (int i = 0; i < 10; i++)
      {
        auto x = (int)(mcl.pose.x - min_range.x() + x_step * i);
        auto y = (int)(mcl.pose.y - min_range.y() + y_step * i);
        if (0 > y or y >= height or 0 > x or x >= width)
          continue;

        map_[y][x][0] = 0;
        map_[y][x][1] = 255;
        map_[y][x][2] = 0;
      }
    }
  }

  void draw_image_now_state(Pose &pose, Sensor::Model &sensor_model)
  {
    if (map.size() <= 0)
    {
      return;
    }

    auto pos = Common::RealPos(pose.x, pose.y);
    auto cos_ = pose.angle.cos(), sin_ = pose.angle.sin();
#pragma omp parallel for
    for (const auto &s : sensor_model.data)
    {
      auto data = Common::RealPos(s);
      auto point = pos + Common::RealPos(data.x() * cos_ - data.y() * sin_, data.x() * sin_ + data.y() * cos_);

      auto y = (Grid::Pos(point) - map.min_corner).y();
      auto x = (Grid::Pos(point) - map.min_corner).x();

      int width = (map.max_corner - map.min_corner).x() + 1;
      int height = (map.max_corner - map.min_corner).y() + 1;
      if (0 > y or y >= height or 0 > x or x >= width)
        continue;

      map_[y][x][0] = 0;
      // map_[y][x][1] = 255;
      map_[y][x][2] = 255;
    }
  }
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription1_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription2_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pose_and_likelihoods_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(make_shared<MclNode>());
  rclcpp::shutdown();
  return 0;
}
