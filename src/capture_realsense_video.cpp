#include <sensor_msgs/msg/image.hpp>
#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace std::chrono_literals;
using std::placeholders::_1;

class MonoRealSense : public rclcpp::Node
{
  public:
    MonoRealSense()
    : Node("capture_video")
    {
      // declare parameters
      declare_parameter("video_name", "realsense_video.mp4");

      // get parameters
      video_name = get_parameter("video_name").as_string();

      image_sub = create_subscription<sensor_msgs::msg::Image>(
          "camera/color/image_raw", 10, std::bind(&MonoRealSense::image_callback, this, _1));

      std::string file_name = static_cast<std::string>(PROJECT_PATH) + "/videos/"+ video_name;
      output_video.open(file_name, 0, 30, cv::Size(1280, 720));
      RCLCPP_INFO_STREAM(get_logger(), "project_path: " << PROJECT_PATH);
      if (!output_video.isOpened())
      {
        RCLCPP_ERROR(get_logger(), "Could not open video file for writing");
        rclcpp::shutdown();
      }
    }
    ~MonoRealSense()
    {
    }

  private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
      auto cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
      output_video << cv_ptr->image;

      cv::imshow("image", cv_ptr->image);
      cv::waitKey(1);
    }
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;
    std::string video_name;
    cv::VideoWriter output_video;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MonoRealSense>());
  rclcpp::shutdown();
  return 0;
}
