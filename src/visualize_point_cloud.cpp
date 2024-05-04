#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <fstream>
#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <geometry_msgs/msg/point32.hpp>

using namespace std::chrono_literals;

class VisualizePointCloud : public rclcpp::Node
{
  public:
    VisualizePointCloud()
    : Node("visualize_point_cloud"),
    map_file_path(std::string(PROJECT_PATH) + "/maps/graham_apt_Fri May  3 21:26:32 2024_map.csv")
    {
      point_cloud_publisher = create_publisher<sensor_msgs::msg::PointCloud>("orb_point_cloud", 10);
      timer = create_wall_timer(
      500ms, std::bind(&VisualizePointCloud::timer_callback, this));

      load_point_cloud();
    }

  private:
    void load_point_cloud()
    {
      point_cloud.header.frame_id = "point_cloud";
      point_cloud.header.stamp = get_clock()->now();

      std::ifstream file(map_file_path);
      // if (file.fail())
      // {
      //   RCLCPP_ERROR_STREAM(get_logger(), "Failed to open file: " << map_file_path);
      //   rclcpp::shutdown();
      // }
      std::string line;
      while (std::getline(file, line))
      {
        std::istringstream iss(line);
        std::string token;
        geometry_msgs::msg::Point32 point;
        int i = 0;
        while (std::getline(iss, token, ','))
        {
          if (i == 0)
          {
            point.x = std::stof(token);
          }
          else if (i == 1)
          {
            point.y = std::stof(token);
          }
          else if (i == 2)
          {
            point.z = std::stof(token);
          }
          i++;
        }
        point_cloud.points.push_back(point);
      }
      point_cloud.channels.push_back(sensor_msgs::msg::ChannelFloat32());
      point_cloud.channels[0].name = "intensity";
      point_cloud.channels[0].values.resize(point_cloud.points.size());
      for (int i = 0; i < point_cloud.points.size(); i++)
      {
        point_cloud.channels[0].values[i] = 1.0;
      }
      RCLCPP_INFO_STREAM(get_logger(), "Loaded point cloud with " << point_cloud.points.size() << " points");
    }
    void timer_callback()
    {
      point_cloud_publisher->publish(point_cloud);
    }
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr point_cloud_publisher;
    sensor_msgs::msg::PointCloud point_cloud;
    std::string map_file_path;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VisualizePointCloud>());
  rclcpp::shutdown();
  return 0;
}
