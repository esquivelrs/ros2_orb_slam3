#include <apriltag/tag16h5.h>
#include <apriltag/tag25h9.h>
#include <apriltag/tag36h10.h>
#include <apriltag/tag36h11.h>
#include <apriltag/tagCircle21h7.h>
#include <apriltag/tagCircle49h12.h>
#include <apriltag/tagCustom48h12.h>
#include <apriltag/tagStandard41h12.h>
#include <apriltag/tagStandard52h13.h>
#include <apriltag/apriltag.h>
#include <apriltag/apriltag_pose.h>
#include <apriltag_msgs/msg/april_tag_detection.hpp>

#include <ros2_orb_slam3/msg/pose_delta.hpp>
#include <ros2_orb_slam3/srv/april_tag_detection.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <opencv2/opencv.hpp>

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <memory>

#include "System.h"

#include "rclcpp/rclcpp.hpp"
#include "ros2_orb_slam3/srv/detail/april_tag_detection__struct.hpp"
using std::placeholders::_1, std::placeholders::_2;

struct my_pose
{
  apriltag_pose_t the_pose;
  int id;
};

class AprilTagNode : public rclcpp::Node
{
  public:
    AprilTagNode()
    : Node("apriltag_node")
    {
      // declare parameters
      declare_parameter("apriltag_family", "tagStandard41h12");
      declare_parameter("tagsize", 0.115);

      // get parameters
      apriltag_family = get_parameter("apriltag_family").as_string();
      tagsize = get_parameter("tagsize").as_double();

      // create apriltag family
      if (apriltag_family == "tag16h5")
      {
        tf = tag16h5_create();
      }
      else if (apriltag_family == "tag25h9")
      {
        tf = tag25h9_create();
      }
      else if (apriltag_family == "tag36h10")
      {
        tf = tag36h10_create();
      }
      else if (apriltag_family == "tag36h11")
      {
        tf = tag36h11_create();
      }
      else if (apriltag_family == "tagCircle21h7")
      {
        tf = tagCircle21h7_create();
      }
      else if (apriltag_family == "tagCircle49h12")
      {
        tf = tagCircle49h12_create();
      }
      else if (apriltag_family == "tagCustom48h12")
      {
        tf = tagCustom48h12_create();
      }
      else if (apriltag_family == "tagStandard41h12")
      {
        tf = tagStandard41h12_create();
      }
      else if (apriltag_family == "tagStandard52h13")
      {
        tf = tagStandard52h13_create();
      }
      else
      {
        RCLCPP_ERROR(get_logger(), "Invalid apriltag family");
      }

      // create apriltag detector
      detector = apriltag_detector_create();
      tf = tagStandard41h12_create();
      apriltag_detector_add_family(detector, tf);

      // declare subscribers
      image_sub = create_subscription<sensor_msgs::msg::Image>(
          "camera/color/image_raw", 10, std::bind(&AprilTagNode::image_callback, this, _1));
      camera_intrinsics_sub = create_subscription<sensor_msgs::msg::CameraInfo>(
          "camera/color/camera_info", 10, std::bind(&AprilTagNode::camera_intrinsics_callback, this, _1));

      // declare publishers
      pose_delta_pub = create_publisher<ros2_orb_slam3::msg::PoseDelta>("pose_delta", 10);

      // declare services
      apriltag_detection_srv = create_service<ros2_orb_slam3::srv::AprilTagDetection>("apriltag_detection", 
        std::bind(&AprilTagNode::apriltag_detection_callback, this, _1, _2));

      prev_poses.push_back(my_pose{apriltag_pose_t(), -1});
    }

    ~AprilTagNode()
    {
      apriltag_detector_destroy(detector);
      if (apriltag_family == "tag16h5")
      {
        tag16h5_destroy(tf);
      }
      else if (apriltag_family == "tag25h9")
      {
        tag25h9_destroy(tf);
      }
      else if (apriltag_family == "tag36h10")
      {
        tag36h10_destroy(tf);
      }
      else if (apriltag_family == "tag36h11")
      {
        tag36h11_destroy(tf);
      }
      else if (apriltag_family == "tagCircle21h7")
      {
        tagCircle21h7_destroy(tf);
      }
      else if (apriltag_family == "tagCircle49h12")
      {
        tagCircle49h12_destroy(tf);
      }
      else if (apriltag_family == "tagCustom48h12")
      {
        tagCustom48h12_destroy(tf);
      }
      else if (apriltag_family == "tagStandard41h12")
      {
        tagStandard41h12_destroy(tf);
      }
      else if (apriltag_family == "tagStandard52h13")
      {
        tagStandard52h13_destroy(tf);
      }
    }

  private:
    void apriltag_detection_callback(const std::shared_ptr<ros2_orb_slam3::srv::AprilTagDetection_Request>,
        std::shared_ptr<ros2_orb_slam3::srv::AprilTagDetection_Response> response)
    {
      RCLCPP_INFO(get_logger(), "Service called");
      
      geometry_msgs::msg::Point center;
      center.x = info.det->c[0];
      center.y = info.det->c[1];

      std::array<geometry_msgs::msg::Point, 4> corners;
      response->id = info.det->id;
      response->centre = center;
      response->corners = corners;
      response->pose_trans = {apriltag_pose.t->data[0], apriltag_pose.t->data[1], apriltag_pose.t->data[2]};
      response->pose_rotation = {
        apriltag_pose.R->data[0], apriltag_pose.R->data[1], apriltag_pose.R->data[2],
        apriltag_pose.R->data[3], apriltag_pose.R->data[4], apriltag_pose.R->data[5],
        apriltag_pose.R->data[6], apriltag_pose.R->data[7], apriltag_pose.R->data[8]
      };
    }
    void camera_intrinsics_callback(const sensor_msgs::msg::CameraInfo msg)
    {
      camera_intrinsics = msg;
      info.tagsize = tagsize;
      info.fx = camera_intrinsics.k[0];
      info.fy = camera_intrinsics.k[4];
      info.cx = camera_intrinsics.k[2];
      info.cy = camera_intrinsics.k[5];
    }

    // double get_scale_factor(apriltag_detection_t *det, apriltag_pose_t *pose)
    // {
    //
    // }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
      if (camera_intrinsics.height == 0 || camera_intrinsics.width == 0)
      {
        RCLCPP_ERROR(get_logger(), "Camera intrinsics not received yet");
        return;
      }
      auto cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
      cv::Mat frame = cv_ptr->image;

      image_u8_t im = {cv_ptr->image.cols, cv_ptr->image.rows, cv_ptr->image.cols, cv_ptr->image.data};
      zarray_t *detections = apriltag_detector_detect(detector, &im);
      // RCLCPP_INFO_STREAM(get_logger(), "Detected " << zarray_size(detections) << " tags\n");
      for (int i = 0; i < zarray_size(detections); i++)
      {
        apriltag_detection_t *det;
        zarray_get(detections, i, &det);
        // RCLCPP_INFO_STREAM(get_logger(), "Detected tag " << det->id);
        // RCLCPP_INFO_STREAM(get_logger(), "Detected tag center " << det->c[0] << ", " << det->c[1]);

        // Figure out tag pose
        info.det = det;

        apriltag_pose_t pose;
        double err = estimate_tag_pose(&info, &pose);
        apriltag_pose = pose;
        bool found_pose = false;
        for (size_t i = 0; i < prev_poses.size(); i ++){
          if (prev_poses.at(i).id == det->id) {
            found_pose = true;

            // Make pose delta object
            ros2_orb_slam3::msg::PoseDelta pose_delta;
            pose_delta.header.stamp = msg->header.stamp;

            // Create translation part of the object
            std_msgs::msg::Float32MultiArray pose_delta_translation;
            pose_delta_translation.data = {
              static_cast<float>(pose.t->data[0]) - static_cast<float>(prev_poses.at(i).the_pose.t->data[0]), 
              static_cast<float>(pose.t->data[1]) - static_cast<float>(prev_poses.at(i).the_pose.t->data[1]),
              static_cast<float>(pose.t->data[2]) - static_cast<float>(prev_poses.at(i).the_pose.t->data[2])};
            pose_delta.translation = pose_delta_translation;
            // RCLCPP_INFO_STREAM(get_logger(), "Translation: " << pose_delta_translation.data.at(0) << ", " << pose_delta_translation.data.at(1) << ", " << pose_delta_translation.data.at(2));

            std_msgs::msg::Float32MultiArray pose_delta_rotation;
            pose_delta_rotation.data = {
              static_cast<float>(pose.R->data[0]) - static_cast<float>(prev_poses.at(i).the_pose.R->data[0]),
              static_cast<float>(pose.R->data[1]) - static_cast<float>(prev_poses.at(i).the_pose.R->data[1]),
              static_cast<float>(pose.R->data[2]) - static_cast<float>(prev_poses.at(i).the_pose.R->data[2]),
              static_cast<float>(pose.R->data[3]) - static_cast<float>(prev_poses.at(i).the_pose.R->data[3]),
              static_cast<float>(pose.R->data[4]) - static_cast<float>(prev_poses.at(i).the_pose.R->data[4]),
              static_cast<float>(pose.R->data[5]) - static_cast<float>(prev_poses.at(i).the_pose.R->data[5]),
              static_cast<float>(pose.R->data[6]) - static_cast<float>(prev_poses.at(i).the_pose.R->data[6]),
              static_cast<float>(pose.R->data[7]) - static_cast<float>(prev_poses.at(i).the_pose.R->data[7]),
              static_cast<float>(pose.R->data[8]) - static_cast<float>(prev_poses.at(i).the_pose.R->data[8])};
            pose_delta.rotation = pose_delta_rotation;
            pose_delta_pub->publish(pose_delta);
            prev_poses.at(i) = my_pose{pose, det->id};
          }
        }
        if (!found_pose) {
          prev_poses.push_back(my_pose{pose, det->id});
          return;
        }
        // RCLCPP_INFO_STREAM(get_logger(), "Tag pose error " << err);
        // RCLCPP_INFO_STREAM(get_logger(), "Tag pose R " << pose.R->data[0] << ", " << pose.R->data[1] << ", " << pose.R->data[2]);
        // RCLCPP_INFO_STREAM(get_logger(), "Tag pose R " << pose.R->data[3] << ", " << pose.R->data[4] << ", " << pose.R->data[5]);
        // RCLCPP_INFO_STREAM(get_logger(), "Tag pose R " << pose.R->data[6] << ", " << pose.R->data[7] << ", " << pose.R->data[8]);
        // RCLCPP_INFO_STREAM(get_logger(), "Tag pose t " << pose.t->data[0] << ", " << pose.t->data[1] << ", " << pose.t->data[2]);

        // Draw tag outline
        line(frame, cv::Point(det->p[0][0], det->p[0][1]),
                 cv::Point(det->p[1][0], det->p[1][1]),
                 cv::Scalar(0, 0xff, 0), 2);
        line(frame, cv::Point(det->p[0][0], det->p[0][1]),
                 cv::Point(det->p[3][0], det->p[3][1]),
                 cv::Scalar(0, 0, 0xff), 2);
        line(frame, cv::Point(det->p[1][0], det->p[1][1]),
                 cv::Point(det->p[2][0], det->p[2][1]),
                 cv::Scalar(0xff, 0, 0), 2);
        line(frame, cv::Point(det->p[2][0], det->p[2][1]),
                 cv::Point(det->p[3][0], det->p[3][1]),
                 cv::Scalar(0xff, 0, 0), 2);

        std::stringstream ss;
        ss << det->id;
        cv::String text = ss.str();
        int fontface = cv::FONT_HERSHEY_SCRIPT_SIMPLEX;
        double fontscale = 1.0;
        int baseline;
        cv::Size textsize = cv::getTextSize(text, fontface, fontscale, 2,
                                        &baseline);
        putText(frame, text, cv::Point(det->c[0]-textsize.width/2.0,
                                   det->c[1]+textsize.height/2.0),
                fontface, fontscale, cv::Scalar(0xff, 0x99, 0), 2);
      }

      // Draw detection outlines
        for (int i = 0; i < zarray_size(detections); i++) {
          apriltag_detection_info_t info;
        

          apriltag_detection_t *det;
          zarray_get(detections, i, &det);
        }
        apriltag_detections_destroy(detections);

        cv::imshow("Tag Detections", frame);
        cv::waitKey(1);

    }
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_intrinsics_sub;
    rclcpp::Publisher<ros2_orb_slam3::msg::PoseDelta>::SharedPtr pose_delta_pub;
    rclcpp::Service<ros2_orb_slam3::srv::AprilTagDetection>::SharedPtr apriltag_detection_srv;
    // would making this a service call be better? I'm sending over the difference
    // in change in pose to another node. This happens whenever I get a new image,
    // and the other node is also going to be getting many images. However I probalby
    // want to check if the image messages in either node have a the exact same time
    // stamp when they hit their respective callbacks. If they don't then it makes
    // it a lot more complicated to compare change in pose.

    // no, need a custom message type with timestamp and pose message(!) to send the
    // pose to the orb_slam3 node to be compared.

    sensor_msgs::msg::CameraInfo camera_intrinsics;
    std::vector<my_pose> prev_poses;
    apriltag_detection_info_t info;

    apriltag_detection_t apriltag_detection;
    apriltag_pose_t apriltag_pose;
    
    std::string apriltag_family;
    apriltag_family_t *tf;
    apriltag_detector_t *detector;

    double tagsize;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AprilTagNode>());
  rclcpp::shutdown();
  return 0;
}
