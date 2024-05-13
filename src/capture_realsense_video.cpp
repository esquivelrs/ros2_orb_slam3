#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <iostream>
#include <fstream>

#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace std::chrono_literals;
using std::placeholders::_1;

class CaptureVideo : public rclcpp::Node
{
  public:
    CaptureVideo()
    : Node("capture_video")
    {
      // declare parameters
      declare_parameter("video_name", "realsense_video.mp4");

      // get parameters
      video_name = get_parameter("video_name").as_string();

      image_sub = create_subscription<sensor_msgs::msg::Image>(
          "camera/color/image_raw", 10, std::bind(&CaptureVideo::image_callback, this, _1));

      imu_sub = create_subscription<sensor_msgs::msg::Imu>(
          "camera/imu", 10, std::bind(&CaptureVideo::imu_callback, this, _1));

      // grab the current time
      std::time_t now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
      std::stringstream ss;
      ss << std::put_time(std::localtime(&now), "%Y-%m-%d_%H:%M:%S");
      time_string = ss.str();

      // get the video file ready and open it
      std::string file_name = static_cast<std::string>(PROJECT_PATH) + "/videos/"+ time_string + "_" + video_name;
      output_video.open(file_name, cv::VideoWriter::fourcc('m', 'p', '4', 'v'), 30, cv::Size(1280, 720), false);
      if (!output_video.isOpened())
      {
        RCLCPP_ERROR(get_logger(), "Could not open video file for writing");
        rclcpp::shutdown();
      }

      // get the imu file ready and open it
      std::string imu_file_name = std::string(PROJECT_PATH) + "/videos/" + time_string + "_" + video_name.substr(0, video_name.length() - 4) + ".csv";
      imu_file.open(imu_file_name);
      if (!imu_file.is_open())
      {
        RCLCPP_ERROR(get_logger(), "Could not open imu file for writing");
        rclcpp::shutdown();
      } else {
        RCLCPP_INFO(get_logger(), "imu file opened");
      }
    }
    // ~CaptureVideo()
    // {
    //   imu_file.close();
    // }

  private:
    void imu_callback(const sensor_msgs::msg::Imu msg)
    {
      geometry_msgs::msg::Vector3 linear_acceleration = msg.linear_acceleration;

      double roll, pitch, yaw;
      roll = std::atan2(linear_acceleration.y, linear_acceleration.z);
      pitch = std::atan2(-linear_acceleration.x, std::sqrt(linear_acceleration.y * linear_acceleration.y + linear_acceleration.z * linear_acceleration.z));
      yaw = std::atan2(-linear_acceleration.y, -linear_acceleration.z);
      tf2::Quaternion q;
      q.setRPY(roll, pitch, yaw);

      geometry_msgs::msg::Quaternion imu_data;
      imu_data.x = q.x();
      imu_data.y = q.y();
      imu_data.z = q.z();
      imu_data.w = q.w();

      RCLCPP_INFO_STREAM(get_logger(), "imu_data: " << imu_data.x << ", " << imu_data.y << ", " << imu_data.z << ", " << imu_data.w);
      imu_file << imu_data.x << ", " << imu_data.y << ", " << imu_data.z << ", " << imu_data.w << std::to_string(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9) << std::endl;
    }
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
      auto cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
      output_video << cv_ptr->image;

      cv::imshow("image", cv_ptr->image);
      cv::waitKey(1);
    }
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
    std::string video_name;
    cv::VideoWriter output_video;
    std::ofstream imu_file;
    std::string time_string;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CaptureVideo>());
  rclcpp::shutdown();
  return 0;
}
