#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <Eigen/Dense>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/eigen.hpp>
#include <image_transport/image_transport.h>

//* ORB SLAM 3 includes
#include "System.h" //* Also imports the ORB_SLAM3 namespace

#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MonoRealSense : public rclcpp::Node
{
  public:
    MonoRealSense()
    : Node("monorealsense"),
      vocabulary_file_path(std::string(PROJECT_PATH)+ "/orb_slam3/Vocabulary/ORBvoc.txt.bin"),
      settings_file_path(std::string(PROJECT_PATH) + "/orb_slam3/config/Monocular/RealSense_D435i.yaml")
    {
      // declare parameters
      declare_parameter("sensor_type", "monocular");
      declare_parameter("use_pangolin", true);

      // get parameters
      std::string sensor_type_param = get_parameter("sensor_type").as_string();
      bool use_pangolin = get_parameter("use_pangolin").as_bool();
      ORB_SLAM3::System::eSensor sensor_type;
      if (sensor_type_param == "monocular")
      {
        sensor_type = ORB_SLAM3::System::MONOCULAR;
      }
      else
      {
        RCLCPP_ERROR(get_logger(), "Sensor type not recognized");
      }

      image_sub = create_subscription<sensor_msgs::msg::Image>(
          "camera/color/image_raw", 10, std::bind(&MonoRealSense::image_callback, this, _1));

      imu_sub = create_subscription<sensor_msgs::msg::Imu>(
          "camera/imu", 10, std::bind(&MonoRealSense::imu_callback, this, _1));

      pAgent = std::make_shared<ORB_SLAM3::System>(vocabulary_file_path, 
          settings_file_path,
          sensor_type,
          use_pangolin);
    }

  private:
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
      imu_msg = *msg;
    }
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
      auto cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
      double timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

      cv::imshow("image", cv_ptr->image);
      cv::waitKey(1);
      ORB_SLAM3::IMU::Point imu_data(imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z,
          imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z, timestamp);
      const vector<ORB_SLAM3::IMU::Point> vImuMeas = {imu_data};
      Sophus::SE3f Tcw = pAgent->TrackMonocular(cv_ptr->image, timestamp, vImuMeas);
      vector<ORB_SLAM3::MapPoint*>map_points = pAgent->GetAllMapPoints();
      // RCLCPP_INFO_STREAM(get_logger(), "Pose: " << Tcw.matrix());
    }
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;

    sensor_msgs::msg::Imu imu_msg;
    std::shared_ptr<ORB_SLAM3::System> pAgent;
    std::string vocabulary_file_path;
    std::string settings_file_path;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MonoRealSense>());
  rclcpp::shutdown();
  return 0;
}
