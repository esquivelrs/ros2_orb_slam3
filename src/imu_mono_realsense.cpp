#include <rclcpp/callback_group.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/logging.hpp>
#include <std_srvs/srv/empty.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <ros2_orb_slam3/msg/pose_delta.hpp>
#include <ros2_orb_slam3/srv/april_tag_detection.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <iostream>
#include <fstream>
#include <fstream>
#include <chrono>
#include <ctime>
#include <signal.h>

#include <Eigen/Dense>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/eigen.hpp>

// this is orb_slam3
#include "System.h"

#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1, std::placeholders::_2;

class ImuMonoRealSense : public rclcpp::Node
{
  public:
    ImuMonoRealSense()
    : Node("imu_mono_realsense"),
      vocabulary_file_path(std::string(PROJECT_PATH)+ "/orb_slam3/Vocabulary/old_ORBvoc.txt.bin"),
      pose_ready(false),
      count(0),
      count2(0),
      scale_factor(0.0)
    {

      // declare parameters
      declare_parameter("sensor_type", "imu-monocular");
      declare_parameter("use_pangolin", true);
      declare_parameter("use_live_feed", false);
      declare_parameter("file_name", "output");
      declare_parameter("epsilon", 25.0); // pixels

      // get parameters
      sensor_type_param = get_parameter("sensor_type").as_string();
      bool use_pangolin = get_parameter("use_pangolin").as_bool();
      use_live_feed = get_parameter("use_live_feed").as_bool();
      file_name = get_parameter("file_name").as_string();
      eps = get_parameter("epsilon").as_double();

      // define callback groups
      image_callback_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      imu_callback_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      pose_delta_callback_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      apriltag_callback_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

      rclcpp::SubscriptionOptions image_options;
      image_options.callback_group = image_callback_group;
      rclcpp::SubscriptionOptions imu_options;
      imu_options.callback_group = imu_callback_group;
      rclcpp::SubscriptionOptions pose_delta_options;
      pose_delta_options.callback_group = pose_delta_callback_group;

      // set the sensor type based on parameter
      ORB_SLAM3::System::eSensor sensor_type;
      if (sensor_type_param == "monocular")
      {
        sensor_type = ORB_SLAM3::System::MONOCULAR;
        settings_file_path = std::string(PROJECT_PATH) + "/orb_slam3/config/Monocular/RealSense_D435i.yaml";
      } else if (sensor_type_param == "imu-monocular") {
        sensor_type = ORB_SLAM3::System::IMU_MONOCULAR;
        settings_file_path = std::string(PROJECT_PATH) + "/orb_slam3/config/Monocular-Inertial/RealSense_D435i.yaml";
      }
      else
      {
        RCLCPP_ERROR(get_logger(), "Sensor type not recognized");
        rclcpp::shutdown();
      }

      // setup orb slam object
      pAgent = std::make_shared<ORB_SLAM3::System>(
          vocabulary_file_path, 
          settings_file_path,
          sensor_type,
          use_pangolin,
          0);
      image_scale = pAgent->GetImageScale();

      // create subscriptions
      image_sub = create_subscription<sensor_msgs::msg::Image>(
          "camera/color/image_raw", 10, std::bind(&ImuMonoRealSense::image_callback, this, _1), image_options);

      imu_sub = create_subscription<sensor_msgs::msg::Imu>(
          "camera/imu", 10, std::bind(&ImuMonoRealSense::imu_callback, this, _1), imu_options);

      pose_delta_sub = create_subscription<ros2_orb_slam3::msg::PoseDelta>(
          "pose_delta", 10, std::bind(&ImuMonoRealSense::pose_delta_callback, this, _1), pose_delta_options);

      // create clients
      apriltag_client = create_client<ros2_orb_slam3::srv::AprilTagDetection>("apriltag_detection", rmw_qos_profile_services_default, apriltag_callback_group);

      while (!apriltag_client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
          RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
          rclcpp::shutdown();
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
      }

      // create services
      save_map_service = create_service<std_srvs::srv::Empty>(
          "save_map", std::bind(&ImuMonoRealSense::save_map_callback, this, _1, _2));
    }

    // ~ImuMonoRealSense()
    // {
    //   vector<ORB_SLAM3::MapPoint*>map_points = pAgent->GetAllMapPoints();
    //   save_map_to_csv(map_points);
    //   pAgent->Shutdown();
    // }

  private:
    void save_map_callback(const std::shared_ptr<std_srvs::srv::Empty::Request>,
        std::shared_ptr<std_srvs::srv::Empty::Response>)
    {
      vector<ORB_SLAM3::MapPoint*>map_points = pAgent->GetAllMapPoints();
      save_map_to_csv(map_points);
    }

    void save_map_to_csv(vector<ORB_SLAM3::MapPoint*> map_points)
    {

      // calculate the scale factor by averaging things
      for (size_t i = 0; i < scale_factors.size(); i++) {
        scale_factor += scale_factors[i];
      }
      scale_factor /= scale_factors.size();
      RCLCPP_INFO_STREAM(get_logger(), "final scale factor: " << scale_factor);
      std::ofstream map_file;
      std::ofstream unscaled_map_file;
      // add date and time to the map file name
      std::string time_string;
      if(use_live_feed) {
        std::time_t now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
        time_string = std::ctime(&now);
        time_string = time_string.substr(0, time_string.length() - 1) + "_";
      }
      std::string map_file_name = std::string(PROJECT_PATH) + "/maps/" + file_name + "_" + time_string + "map" + ".csv";
      std::string unscaled_map_file_name = std::string(PROJECT_PATH) + "/unscaled_maps/" + file_name + "_" + time_string + "map" + ".csv";
      map_file.open(map_file_name);
      unscaled_map_file.open(unscaled_map_file_name);
      
      if(map_file.is_open() && unscaled_map_file.is_open())
      {
        RCLCPP_INFO_STREAM(get_logger(), "Saving map to " << map_file_name);
        RCLCPP_INFO_STREAM(get_logger(), "Saving unscaled map to " << unscaled_map_file_name);
        map_file << initial_orientation->a[0] << "," << initial_orientation->a[1]
          << "," << initial_orientation->a[2] << "," << initial_orientation->w[0]
          << "," << initial_orientation->w[1] << "," << initial_orientation->w[2]
          << std::endl;
        unscaled_map_file << initial_orientation->a[0] << "," << initial_orientation->a[1]
          << "," << initial_orientation->a[2] << "," << initial_orientation->w[0]
          << "," << initial_orientation->w[1] << "," << initial_orientation->w[2]
          << std::endl;
        RCLCPP_INFO_STREAM(get_logger(), "Size of the map: " << map_points.size());
        for (auto &map_point : map_points)
        {
          Eigen::Vector3f pos = map_point->GetWorldPos();
          map_file << std::fixed << std::setprecision(3) << pos[0] * scale_factor << "," << pos[1] * scale_factor << "," << pos[2] * scale_factor << std::endl;
          unscaled_map_file << std::fixed << std::setprecision(3) << pos[0] << "," << pos[1] << "," << pos[2] << std::endl;
        }
        map_file.close();
      }
      else
      {
        if (!map_file.is_open()) {
          RCLCPP_ERROR(get_logger(), "Could not open map file for writing");
        } else {
          RCLCPP_ERROR(get_logger(), "Could not open unscaled map file for writing");
        }
      }
      RCLCPP_INFO_STREAM(get_logger(), "Map saved to " << map_file_name);
      RCLCPP_INFO_STREAM(get_logger(), "Unscaled map saved to " << unscaled_map_file_name);
    }

    void pose_delta_callback(const ros2_orb_slam3::msg::PoseDelta msg)
    {
      pose_delta = msg;
      pose_ready = true;
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
      auto cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
      timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

      cv::Mat frame = cv_ptr->image;

      if(image_scale != 1.f) {
        cv::resize(frame, frame, cv::Size(frame.cols*image_scale, frame.rows*image_scale));
      }

      if (sensor_type_param == "monocular") {
        Sophus::SE3f Tcw = pAgent->TrackMonocular(frame, timestamp);

        // only on the first slam iteration
        if (!Tcw.matrix().isIdentity() && count2 == 0) {
          // get the apriltag data
          auto request = std::make_shared<ros2_orb_slam3::srv::AprilTagDetection::Request>();
          auto result_future = apriltag_client->async_send_request(request);
          
          std::future_status status = result_future.wait_for(1s);
          while (status != std::future_status::ready) {
            status = result_future.wait_for(1s);
          }
          if (status == std::future_status::ready) {
            RCLCPP_INFO_STREAM(get_logger(), "apriltag detection ready");
            auto result = result_future.get();
            if (result->id != 1) {
              return;
            }
            std::vector<cv::KeyPoint> key_points = pAgent->GetTrackedKeyPoints();
            std::vector<ORB_SLAM3::MapPoint*> map_points = pAgent->GetTrackedMapPoints();
            std::array<geometry_msgs::msg::Point, 4> corners = result->corners;
            for (size_t i = 0; i < key_points.size(); i++) {
              if (map_points.at(i) != 0) {
                geometry_msgs::msg::Point center = result->centre; // europeans spell center wrong, not me
                if (std::sqrt(std::pow(center.x - key_points.at(i).pt.x, 2) + std::pow(center.y - key_points.at(i).pt.y, 2)) < eps) {
                  cv::Mat frame = pAgent->GetCurrentFrame();
                  cv::circle(frame, key_points.at(i).pt, 5, cv::Scalar(0, 0, 255), 2);
                  cv::circle(frame, cv::Point(center.x, center.y), 5, cv::Scalar(0, 255, 0), 2);
                  cv::line(frame, key_points.at(i).pt, cv::Point(center.x, center.y), cv::Scalar(255, 0, 0), 2);
                  cv::imshow("frame", frame);
                  cv::waitKey(1);
                  RCLCPP_INFO_STREAM(get_logger(), "keypoint: " << key_points.at(i).pt.x << ", " << key_points.at(i).pt.y);
                  RCLCPP_INFO_STREAM(get_logger(), "center: " << center.x << ", " << center.y);
                  // map array and keyframe array elements are linked by indices
                  Eigen::Vector3f pos = map_points.at(i)->GetWorldPos();
                  // scale_factor = (pos[2]/result->pose_trans[2]);
                  scale_factors.push_back(result->pose_trans[2]/pos[2]);
                  RCLCPP_INFO_STREAM(get_logger(), "scale factor: " << scale_factors.back());
                }
              }
            }
          } else {
            RCLCPP_INFO_STREAM(get_logger(), "apriltag detection not ready");
          }


          initial_orientation = current_imu_meas;
          count2++;
        }


      } else if (sensor_type_param == "imu-monocular"){
        Sophus::SE3f Tcw = pAgent->TrackMonocular(frame, timestamp, vImuMeas);
      }
      count ++;

      cv::imshow("frame", frame);
      cv::waitKey(1);
    }

    void imu_callback(const sensor_msgs::msg::Imu msg)
    {
      if (count > 0) {
        vImuMeas.clear();
        count = 0;
      }

      ORB_SLAM3::IMU::Point imu_meas(
          static_cast<float>(msg.linear_acceleration.x), static_cast<float>(msg.linear_acceleration.y), static_cast<float>(msg.linear_acceleration.z),
          static_cast<float>(msg.angular_velocity.x), static_cast<float>(msg.angular_velocity.y), static_cast<float>(msg.angular_velocity.z),
          msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9);
      // vImuMeas.push_back(imu_meas);
      current_imu_meas = std::make_shared<ORB_SLAM3::IMU::Point>(imu_meas);
      // RCLCPP_INFO_STREAM(get_logger(), "imu timestamp: " << std::fixed << std::setprecision(9) << msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9);


    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
    rclcpp::Subscription<ros2_orb_slam3::msg::PoseDelta>::SharedPtr pose_delta_sub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub;
    rclcpp::Client<ros2_orb_slam3::srv::AprilTagDetection>::SharedPtr apriltag_client;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr save_map_service;
    rclcpp::CallbackGroup::SharedPtr image_callback_group;
    rclcpp::CallbackGroup::SharedPtr imu_callback_group;
    rclcpp::CallbackGroup::SharedPtr pose_delta_callback_group;
    rclcpp::CallbackGroup::SharedPtr apriltag_callback_group;

    sensor_msgs::msg::Imu imu_msg;
    std::shared_ptr<ORB_SLAM3::IMU::Point> current_imu_meas;

    bool use_live_feed;
    std::string file_name;
    std::string sensor_type_param;

    cv::VideoCapture input_video;
    std::ifstream imu_file;
    std::ifstream video_timestamp_file;

    std::vector<geometry_msgs::msg::Vector3> vGyro;
    std::vector<double> vGyro_times;
    std::vector<geometry_msgs::msg::Vector3> vAccel;
    std::vector<double> vAccel_times;
    std::vector<ORB_SLAM3::IMU::Point> vImuMeas;

    std::shared_ptr<ORB_SLAM3::System> pAgent;
    std::string vocabulary_file_path;
    std::string settings_file_path;
    float image_scale;
    double timestamp;

    // save the differences between the translation portions of the trajectories
    // here, and then I can use that to scale everything here?
    double scale_factor;
    std::vector<double> scale_factors;
    ros2_orb_slam3::msg::PoseDelta pose_delta;
    std::vector<float> sum_pose_delta;
    std::vector<float> sum_Tcw_delta;
    Sophus::SE3f Tcw_prev;

    std::shared_ptr<ORB_SLAM3::IMU::Point> initial_orientation;
    double eps;
    bool pose_ready;
    int count;
    int count2;
};
std::shared_ptr<ImuMonoRealSense> node = nullptr;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto n = std::make_shared<ImuMonoRealSense>();
  executor.add_node(n);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
