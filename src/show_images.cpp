#include <string>
#include <string_view>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/highgui.hpp>
#include "image_transport/image_transport.hpp"

class ShowImages : public rclcpp::Node
{
public:
  ShowImages()
  : Node("show_images")
  {

    //Subscribers
    sub_raw_ = std::make_shared<image_transport::CameraSubscriber>(
        image_transport::create_camera_subscription(
          this,
          "image_raw",
          std::bind(&ShowImages::raw_callback, this, std::placeholders::_1, std::placeholders::_2),
          "compressed",
          rclcpp::QoS {10}.get_rmw_qos_profile()
        )
    );

    sub_left_raw_ = std::make_shared<image_transport::CameraSubscriber>(
        image_transport::create_camera_subscription(
          this,
          "left/image_raw",
          std::bind(&ShowImages::raw_left_callback, this, std::placeholders::_1, std::placeholders::_2),
          "compressed",
          rclcpp::QoS {10}.get_rmw_qos_profile()
        )
    );

    sub_right_raw_ = std::make_shared<image_transport::CameraSubscriber>(
        image_transport::create_camera_subscription(
          this,
          "right/image_raw",
          std::bind(&ShowImages::raw_right_callback, this, std::placeholders::_1, std::placeholders::_2),
          "compressed",
          rclcpp::QoS {10}.get_rmw_qos_profile()
        )
    );

  }

  //Destroy windows
  ~ShowImages()
  {
    cv::destroyAllWindows();
  }

private:
  std::shared_ptr<image_transport::CameraSubscriber> sub_raw_;
  std::shared_ptr<image_transport::CameraSubscriber> sub_right_raw_;
  std::shared_ptr<image_transport::CameraSubscriber> sub_left_raw_;

  void raw_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr& img,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr&
  ) {
    // img->encoding gives rgb8 but it really is bgr8
    const cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(*img, "bgr8");
    show_img("Whole", cv_ptr->image);
  }

  void raw_left_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr& img,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr&
  ) {
    // img->encoding gives rgb8 but it really is bgr8
    const cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(*img, "bgr8");
    show_img("Left", cv_ptr->image);
  }

  void raw_right_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr& img,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr&
  ) {
    // img->encoding gives rgb8 but it really is bgr8
    const cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(*img, "bgr8");
    show_img("Right", cv_ptr->image);
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
  rclcpp::spin(std::make_shared<ShowImages>());
  rclcpp::shutdown();
  return 0;
}
