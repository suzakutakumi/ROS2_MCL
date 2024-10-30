#include <chrono>
#include <functional>
#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "MCL2D.hpp"
#include "Error.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/image.hpp>

// #include <opencv2/opencv.hpp>

using std::placeholders::_1;
using namespace std;

GridMap initGridMap(SensorModel sensor)
{
  GridMap map;
  for (auto s : sensor.data)
  {
    auto x_step = cos(s.degree * M_PI / 180);
    auto y_step = sin(s.degree * M_PI / 180);

    Pose pos;
    pos.x = 0;
    pos.y = 0;
    double distance = 0;
    while (true)
    {
      map[GridType((int)pos.x, (int)pos.y)] = 0;
      distance += 1.0;

      if (distance >= s.distance)
      {
        map[GridType((int)pos.x, (int)pos.y)] = 1;
        break;
      }
      if (distance >= sensor.max_range)
      {
        break;
      }

      pos.x += x_step;
      pos.y += y_step;
    }
  }
  return map;
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

class LidarSubscriber : public rclcpp::Node
{
public:
  LidarSubscriber() : Node("lidar_subscriber")
  {
    subscription1_ = create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&LidarSubscriber::scanCallback, this, _1));
    subscription2_ = create_subscription<sensor_msgs::msg::Image>("/map", 10, std::bind(&LidarSubscriber::registerMap, this, _1));

    publisher_ = create_publisher<sensor_msgs::msg::Image>("/map_image", 10);
    timer_ = create_wall_timer(500ms, std::bind(&LidarSubscriber::weight_publisher, this));

    this->declare_parameter("num_of_particle", 10000);
    this->declare_parameter("sigma", 0.04);
    this->declare_parameter("map_threshold", 0.5);
    this->declare_parameter("map_resolution", 0.05);
    this->declare_parameter("error_ganma", 0.4);
    this->declare_parameter("error_vc", 0.05);

    number_of_particle = get_parameter("num_of_particle").as_int();
    sigma = get_parameter("sigma").as_double();
    threshold = get_parameter("map_threshold").as_double();
    resolution = get_parameter("map_resolution").as_double();
    ganma = get_parameter("error_ganma").as_double();
    vc = get_parameter("error_vc").as_double();

    cout << "number of particles: " << number_of_particle << endl;
    cout << "sigma: " << sigma << endl;
    cout << "map threshold: " << threshold << endl;
    cout << "map resolution: " << resolution << endl;
  }

private:
  MCL2D mcl = MCL2D();
  GridMap map;
  vector<vector<vector<int>>> map_;

  double resolution;
  bool check_start = false;

  int number_of_particle;
  double sigma;
  double threshold;
  double ganma;
  double vc;

  void registerMap(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    int rows = msg->height;
    int cols = msg->width;

    for (int i = rows - 1; i >= 0; i--)
    {
      vector<int> row;
      for (int j = 0; j < cols; j++)
      {
        int color = msg->data[i * rows + j];
        map[GridType(j, i)] = (255 - color) / 255.0;
      }
    }

    cout << "map size: " << rows << "x" << cols << endl;

    if (not check_start)
    {
      mcl = MCL2D(number_of_particle, map, sigma, threshold);
      check_start = true;
    }
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    MotionModel motion_model{0, 0};

    SensorModel sensor_model;
    sensor_model.max_range = msg->range_max / resolution;

    for (unsigned long i = 0; i < msg->ranges.size(); ++i)
    {
      if (/*msg->range_max > msg->ranges[i] and*/ msg->ranges[i] > 0.0)
      {
        auto angle = msg->angle_min + i * msg->angle_increment;
        angle = deg_mod(angle * 180 / M_PI);
        auto distance = msg->ranges[i] / resolution;
        sensor_model.data.push_back(SensorData{distance, angle});
      }
    }
    if (not check_start)
    {
      RCLCPP_INFO(get_logger(), "init map");
      map = initGridMap(sensor_model);
      RCLCPP_INFO(get_logger(), "init mcl");
      mcl = MCL2D(number_of_particle, map, sigma, threshold);
      check_start = true;
      RCLCPP_INFO(get_logger(), "start mcl");
      return;
    }

    RCLCPP_INFO(get_logger(), "mcl update");
    mcl.update(motion_model, sensor_model, map);
    cout << mcl.pose.x * resolution << "," << mcl.pose.y * resolution << "," << mcl.pose.deg << "," << endl
         << endl;

    vector<pair<double, double>> points;
    for (auto &p : mcl.current_particles)
    {
      points.emplace_back(p.x, p.y);
    }
    auto error_value = calc_error_ellipse(points, ganma, vc);
    RCLCPP_INFO(get_logger(), "calc error: %lf", error_value);

    if (error_value > 0.5)
    {
      RCLCPP_INFO(get_logger(), "update map");
      update_map(sensor_model, error_value / 10);
    }

    RCLCPP_INFO(get_logger(), "draw image");
    draw_image();
  }

  void weight_publisher()
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

  void
  draw_image()
  {
    if (not check_start)
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
      if (grid.second > 0.4)
      {
        map_[grid.first.second - min_range.second][grid.first.first - min_range.first] =
            vector<int>(3, (int)(255 * grid.second));
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

  void update_map(SensorModel sensor, double init_value)
  {
    for (auto s : sensor.data)
    {
      auto x_step = cos((s.degree + mcl.pose.deg) * M_PI / 180);
      auto y_step = sin((s.degree + mcl.pose.deg) * M_PI / 180);

      Pose pos = mcl.pose;
      double distance = 0;
      while (true)
      {
        auto grid = GridType((int)pos.x, (int)pos.y);
        auto map_iter = map.find(grid);
        if (map_iter == map.end())
        {
          map[grid] = 0;
        }

        distance += 1.0;

        if (distance >= s.distance)
        {
          map[grid] += init_value;
          break;
        }
        if (distance >= sensor.max_range)
        {
          break;
        }

        pos.x += x_step;
        pos.y += y_step;
      }
    }
  }
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription1_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription2_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(make_shared<LidarSubscriber>());
  rclcpp::shutdown();
  return 0;
}
