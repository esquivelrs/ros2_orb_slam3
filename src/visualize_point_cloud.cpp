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
#include <geometry_msgs/msg/quaternion.hpp>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

class VisualizePointCloud : public rclcpp::Node
{
  public:
    VisualizePointCloud()
    : Node("visualize_point_cloud")
    {
      // declare parameters
      declare_parameter("map_file_name", "2024-05-05_01:44:37 PM_graham_apt_imu_map.csv");
      std::string file_name = get_parameter("map_file_name").as_string();

      map_file_path = static_cast<std::string>(PROJECT_PATH) + "/maps/"+ file_name;

      point_cloud_publisher = create_publisher<sensor_msgs::msg::PointCloud>("orb_point_cloud", 10);
      // Initialize the transform broadcaster
      tf_broadcaster =
        std::make_unique<tf2_ros::TransformBroadcaster>(*this);

      timer = create_wall_timer(
      500ms, std::bind(&VisualizePointCloud::timer_callback, this));

      load_point_cloud();
    }

  private:
    void load_point_cloud()
    {

      std::ifstream file(map_file_path);
      if (file.fail())
      {
        RCLCPP_ERROR_STREAM(get_logger(), "Failed to open file: " << map_file_path);
        rclcpp::shutdown();
      }
      std::string line;
      int line_num = 0;
      while (std::getline(file, line))
      {
        std::istringstream iss(line);
        std::string token;
        int i = 0;
        if (line_num == 0)
        {
          geometry_msgs::msg::Quaternion orientation;
          while (std::getline(iss, token, ','))
          {
            if (i == 0)
            {
              orientation.x = std::stof(token);
            }
            else if (i == 1)
            {
              orientation.y = std::stof(token);
            }
            else if (i == 2)
            {
              orientation.z = std::stof(token);
            }
            else if (i == 3)
            {
              orientation.w = std::stof(token);
            }
            i++;
          }
          line_num++;
          initial_orientation = tf2::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w);
          continue;
        }
        geometry_msgs::msg::Point32 point;
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
      point_cloud.header.frame_id = "point_cloud";
      point_cloud.header.stamp = get_clock()->now();
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
      geometry_msgs::msg::TransformStamped t;
      t.header.stamp = get_clock()->now();
      t.header.frame_id = "camera_link";
      t.child_frame_id = "point_cloud";

      t.transform.rotation.x = initial_orientation.x();
      t.transform.rotation.y = initial_orientation.y();
      t.transform.rotation.z = initial_orientation.z();
      t.transform.rotation.w = initial_orientation.w();
      tf_broadcaster->sendTransform(t);
      
      point_cloud.header.stamp = get_clock()->now();
      point_cloud_publisher->publish(point_cloud);
    }
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr point_cloud_publisher;
    sensor_msgs::msg::PointCloud point_cloud;
    tf2::Quaternion initial_orientation;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
    std::string map_file_path;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VisualizePointCloud>());
  rclcpp::shutdown();
  return 0;
}
