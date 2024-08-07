#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "MCL2D.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/image.hpp>

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
    this->declare_parameter("map_threshold", 220);

    number_of_particle = get_parameter("num_of_particle").as_int();
    sigma = get_parameter("sigma").as_double();
    threshold = get_parameter("map_threshold").as_int();

    cout << "number of particles: " << number_of_particle << endl;
    cout << "sigma: " << sigma << endl;
    cout << "map threshold: " << threshold << endl;
  }

private:
  MCL2D mcl = MCL2D();
  Map map;
  double resolution = 0.05;
  bool check_start = false;

  int number_of_particle;
  double sigma;
  int threshold;

  void registerMap(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    int rows = msg->height;
    int cols = msg->width;

    map.clear();
    for (int i = rows - 1; i >= 0; i--)
    {
      vector<int> row;
      for (int j = 0; j < cols; j++)
      {
        int color = msg->data[i * rows + j];

        row.push_back(color);
      }
      map.push_back(row);
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
    if (not check_start)
    {
      return;
    }

    MotionModel motion_model{0, 0};

    SensorModel sensor_model;
    sensor_model.max_range = msg->range_max;
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

    mcl.update(motion_model, sensor_model, map);
    cout << (mcl.pose.x - (map[0].size() / 2 + 10)) * resolution << "," << (mcl.pose.y - (map.size() / 2 + 10)) * resolution << "," << mcl.pose.deg << "," << endl
         << endl;
  }

  void weight_publisher()
  {
    if (not check_start)
    {
      return;
    }
    // 描画
    vector<vector<vector<int>>> map_;
    for (auto rows : map)
    {
      vector<vector<int>> row;
      row.clear();
      for (auto v : rows)
      {
        vector<int> vs = {v, v, v};
        row.push_back(vs);
      }
      map_.push_back(row);
    }

    double max_weight = 0;
    for (auto p : mcl.current_particles)
    {
      max_weight = p.weight > max_weight ? p.weight : max_weight;
    }
    for (auto p : mcl.current_particles)
    {
      auto color = 200 - (int)(p.weight * 200 / max_weight);
      map_[(int)p.y][(int)p.x][0] = 255;
      map_[(int)p.y][(int)p.x][1] = color;
      map_[(int)p.y][(int)p.x][2] = color;
    }

    for (int i = -1; i <= 1; i++)
    {
      for (int j = -1; j <= 1; j++)
      {
        auto y = map.size() / 2 + 10, x = map[0].size() / 2 + 10;
        map_[y + i][x + j][0] = 0;
        map_[y + i][x + j][1] = 0;
        map_[y + i][x + j][2] = 255;
      }
    }

    for (int i = -1; i <= 1; i++)
    {
      for (int j = -1; j <= 1; j++)
      {
        auto y = (uint)mcl.pose.y + i, x = (uint)mcl.pose.x + j;
        if (0 > y or y >= map.size() or 0 > x or x >= map[0].size())
          continue;

        map_[y][x][0] = 0;
        map_[y][x][1] = 255;
        map_[y][x][2] = 0;
      }
    }

    reverse(map_.begin(), map_.end());

    sensor_msgs::msg::Image image;
    image.height = map.size();
    image.width = map[0].size();
    image.encoding = "rgb8";
    image.step = image.width * 3;

    image.data.clear();
    for (auto rows : map_)
    {
      for (auto v : rows)
      {
        image.data.push_back(v[0]);
        image.data.push_back(v[1]);
        image.data.push_back(v[2]);
      }
    }

    publisher_->publish(image);
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
