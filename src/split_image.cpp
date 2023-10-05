#include <string>
#include <string_view>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/highgui.hpp>
#include "image_transport/image_transport.hpp"

class ImgSplitter : public rclcpp::Node
{
public:
  ImgSplitter()
  : Node("img_splitter")
  {

    // parameters
    declare_parameter("display_images", false);
    display_images = get_parameter("display_images").as_bool();
    RCLCPP_INFO_STREAM(get_logger(), "Displaying Images: " << display_images);
    //Subscribers
    sub_raw_ = std::make_shared<image_transport::CameraSubscriber>(
        image_transport::create_camera_subscription(
          this,
          "image_raw",
          std::bind(&ImgSplitter::raw_callback, this, std::placeholders::_1, std::placeholders::_2),
          "compressed",
          rclcpp::QoS {10}.get_rmw_qos_profile()
        )
    );

    //Publishers
    pub_raw_left_ = std::make_shared<image_transport::CameraPublisher>(
        image_transport::create_camera_publisher(
          this,
          "/left/image_raw",
          rclcpp::QoS {10}.get_rmw_qos_profile()
        )
      );

    pub_raw_right_ = std::make_shared<image_transport::CameraPublisher>(
        image_transport::create_camera_publisher(
          this,
          "/right/image_raw",
          rclcpp::QoS {10}.get_rmw_qos_profile()
        )
      );
  }

  //Destroy windows
  ~ImgSplitter()
  {
    cv::destroyAllWindows();
  }

private:
  bool display_images;
  std::shared_ptr<image_transport::CameraSubscriber> sub_raw_;
  std::shared_ptr<image_transport::CameraPublisher> pub_raw_left_;
  std::shared_ptr<image_transport::CameraPublisher> pub_raw_right_;

  void raw_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr& img,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr&
  ) {
    // img->encoding gives rgb8 but it really is bgr8
    const cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(*img, "bgr8");
    const auto w = cv_ptr->image.size().width;
    const auto h = cv_ptr->image.size().height;
    // corner x, corner y, width, height
    const cv::Mat left = cv_ptr->image(cv::Rect(0, 0, 0.5*w, h)).clone();
    const cv::Mat right = cv_ptr->image(cv::Rect(0.5*w, 0, 0.5*w, h)).clone();
    if (display_images){
      show_img("Whole", cv_ptr->image);
      show_img("left", left);
      show_img("right", right);
    }
    std_msgs::msg::Header header;
    header.stamp = get_clock()->now();
    // for now camera info still blank
    sensor_msgs::msg::CameraInfo camera_info_left_;
    camera_info_left_.header = header;
    sensor_msgs::msg::CameraInfo camera_info_right_;
    camera_info_right_.header = header;
    const auto msg_left = cv_bridge::CvImage(header, "bgr8", left).toImageMsg();
    pub_raw_left_->publish(*msg_left, camera_info_left_);
    const auto msg_right = cv_bridge::CvImage(header, "bgr8", right).toImageMsg();
    pub_raw_right_->publish(*msg_right, camera_info_right_);
  }

  void show_img(const std::string name, const cv::Mat img){
    cv::namedWindow(name);
    cv::imshow(name, img);
    cv::waitKey(1);
  }

};



int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImgSplitter>());
  rclcpp::shutdown();
  return 0;
}
