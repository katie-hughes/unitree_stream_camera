#include <string>
#include <string_view>
#include "rclcpp/rclcpp.hpp"
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
    //Subscribers
    // sub_raw_ = std::make_shared<image_transport::CameraSubscriber>(
    //     image_transport::create_camera_subscription(
    //       this,
    //       "image_raw",
    //       std::bind(&ImgSplitter::raw_callback, this, std::placeholders::_1, std::placeholders::_2),
    //       "compressed",
    //       rclcpp::QoS {10}.get_rmw_qos_profile()
    //     )
    // );
    sub_raw_ = create_subscription<sensor_msgs::msg::Image>(
      "/head/front/cam/image_rect/right",
      10,
      std::bind(&ImgSplitter::raw_callback, this, std::placeholders::_1)
    );

    RCLCPP_INFO_STREAM(get_logger(), "img_subscriber node started");

  }

  //Destroy windows
  ~ImgSplitter()
  {
    cv::destroyAllWindows();
  }

private:
  // std::shared_ptr<image_transport::CameraSubscriber> sub_raw_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_raw_;

  void raw_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr& img
    // const sensor_msgs::msg::CameraInfo::ConstSharedPtr&
  ) {
    RCLCPP_INFO_STREAM(get_logger(), "received image");
    const auto cv_img = cv_bridge::toCvCopy(*img, img->encoding)->image;
    const auto w = cv_img.size().width;
    const auto h = cv_img.size().height;
    // corner, corner, width, height
    const auto left = cv_img(cv::Rect(0, 0, 0.5*w, h)).clone();
    const auto right = cv_img(cv::Rect(0.5*w, 0, 0.5*w, h)).clone();
    const std::string left_name = "left";
    const std::string right_name = "right";
    cv::namedWindow(right_name);
    cv::imshow(right_name, right);
    cv::namedWindow(left_name);
    cv::imshow(left_name, left);
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
