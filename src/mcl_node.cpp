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

// #include <opencv2/opencv.hpp>

using std::placeholders::_1;
using namespace std;

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

class MclNode : public rclcpp::Node
{
public:
  MclNode() : Node("mcl_node")
  {
    subscription1_ = create_subscription<sensor_msgs::msg::PointCloud2>("/scan", 10, std::bind(&MclNode::scanCallback, this, _1));
    subscription2_ = create_subscription<sensor_msgs::msg::PointCloud2>("/map", 10, std::bind(&MclNode::registerMap, this, _1));

    publisher_ = create_publisher<sensor_msgs::msg::Image>("/map_image", 10);
    timer_ = create_wall_timer(500ms, std::bind(&MclNode::image_publisher, this));

    // MCLの設定値
    this->declare_parameter("num_of_particle", 10000);
    this->declare_parameter("sigma", 0.04);
    this->declare_parameter("map_threshold", 0.5);
    this->declare_parameter("max_distance", 4.0);
    this->declare_parameter("variance", 0.5);

    mcl_config.particle_num = get_parameter("num_of_particle").as_int();
    mcl_config.sigma = get_parameter("sigma").as_double();
    mcl_config.map_threshold = get_parameter("map_threshold").as_double();
    mcl_config.distance_map_max_value = get_parameter("max_distance").as_double();
    mcl_config.variance = get_parameter("variance").as_double();

    // MCL以外の値
    this->declare_parameter("map_resolution", 0.05);
    this->declare_parameter("error_ganma", 0.4);
    this->declare_parameter("error_vc", 0.05);

    resolution = get_parameter("map_resolution").as_double();
    ganma = get_parameter("error_ganma").as_double();
    vc = get_parameter("error_vc").as_double();

    RCLCPP_INFO(get_logger(), "number of particles, resolution = %d, %f", mcl_config.particle_num, resolution);
  }

private:
  MCLConfig mcl_config;
  GridMap map;
  MCL2D mcl = MCL2D();
  vector<vector<vector<int>>> map_;

  double resolution;
  double ganma;
  double vc;

  void adaptResolution(SensorData &ps)
  {
    for (auto &p : ps)
    {
      p.x /= resolution;
      p.y /= resolution;
      p.z /= resolution;
    }
  }

  void updateGridMap(SensorData pointcloud)
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

    std::queue<std::pair<GridType, GridType>> q;
    for (auto p : pointcloud)
    {
      auto pos = GridPos(p);
      map[pos] = GridValue{1.0, 0.0};
      q.emplace(pos, pos);
    }

    while (not q.empty())
    {
      auto v = q.front();
      q.pop();
      auto current_pos = v.first;
      auto root_pos = v.second;

      std::vector<GridType> round{GridType(1, 0), GridType(-1, 0), GridType(0, 1), GridType(0, -1)};
      for (auto &r : round)
      {
        auto new_pos = current_pos + r;
        auto distance_vec = new_pos - root_pos;
        auto distance = distance_map[abs(distance_vec.first)][abs(distance_vec.second)];
        auto map_itr = map.find(new_pos);
        if (distance <= max_range and (map_itr == map.end() or distance < map_itr->second.distance))
        {
          double prob = map_itr == map.end() ? 0.0 : map_itr->second.prob;
          map[current_pos] = GridValue{prob, distance};
          q.emplace(new_pos, root_pos);
        }
      }
    }
  }

  void registerMap(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    SensorData pointcloud;
    pcl::fromROSMsg(*msg, pointcloud);

    adaptResolution(pointcloud);

    updateGridMap(pointcloud);
  }

  void scanCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    MotionModel motion_model{0, 0};
    SensorModel sensor_model;

    pcl::fromROSMsg(*msg, sensor_model.data);

    adaptResolution(sensor_model.data);

    if (map_.size() <= 0)
    {
      RCLCPP_INFO(get_logger(), "init");
      updateGridMap(sensor_model.data);

      mcl = MCL2D(map, mcl_config);
      return;
    }

    RCLCPP_INFO(get_logger(), "mcl update");
    mcl.update(motion_model, sensor_model, map);
    RCLCPP_DEBUG(get_logger(), "position(%f,%f,%f deg)", mcl.pose.x * resolution, mcl.pose.y * resolution, mcl.pose.deg);

    RCLCPP_INFO(get_logger(), "calc error");
    vector<pair<double, double>> points;
    for (auto &p : mcl.current_particles)
    {
      points.emplace_back(p.x, p.y);
    }
    auto error_value = calc_error_ellipse(points, ganma, vc);
    RCLCPP_DEBUG(get_logger(), "error value:%f", error_value);

    RCLCPP_INFO(get_logger(), "draw image");
    draw_image();
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
    if (map_.size() <= 0)
    {
      return;
    }

    // 最初値・最大値の取得
    GridType min_range(0, 0), max_range(0, 0);
    for (auto grid : map)
    {
      int x = grid.first.first;
      int y = grid.first.second;

      min_range.first = std::min(min_range.first, x);
      min_range.second = std::min(min_range.second, y);

      max_range.first = std::max(max_range.first, x);
      max_range.second = std::max(max_range.second, y);
    }

    int width = max_range.first - min_range.first + 1;
    int height = max_range.second - min_range.second + 1;

    // 地図初期化
    map_ = vector<vector<vector<int>>>(height);
    for (int i = 0; i < height; i++)
    {
      vector<vector<int>> blank_row(width);
      for (int j = 0; j < width; j++)
      {
        blank_row[j] = vector<int>{255, 0, 255};
      }

      map_[i] = blank_row;
    }
    // 障害物の状態を表示
    for (auto grid : map)
    {
      if (grid.second.prob > 0.4)
      {
        map_[grid.first.second - min_range.second][grid.first.first - min_range.first] =
            vector<int>(3, (int)(255 * grid.second.prob));
      }
    }

    // パーティクルの表示
    auto max_weight = std::max_element(mcl.current_particles.begin(), mcl.current_particles.end(), [](const auto &a, const auto &b)
                                       { return a.weight < b.weight; });
    for (auto &p : mcl.current_particles)
    {
      auto x = (int)p.x - min_range.first;
      auto y = (int)p.y - min_range.second;
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
        auto y = -min_range.second + i, x = -min_range.first + j;
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
        auto y = (int)mcl.pose.y - min_range.second + i, x = (int)mcl.pose.x - min_range.first + j;
        if (0 > y or y >= height or 0 > x or x >= width)
          continue;

        map_[y][x][0] = 0;
        map_[y][x][1] = 255;
        map_[y][x][2] = 0;
      }
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription1_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription2_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(make_shared<MclNode>());
  rclcpp::shutdown();
  return 0;
}
