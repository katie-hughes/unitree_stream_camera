#include <string>
#include <string_view>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/highgui.hpp>
#include "image_transport/image_transport.hpp"

void show_image(const std::string & window, const sensor_msgs::msg::Image::ConstSharedPtr& img);

class ImgSplitter : public rclcpp::Node
{
public:
  ImgSplitter()
  : Node("img_splitter")
  {
    //Subscribers
    sub_raw_ = std::make_shared<image_transport::CameraSubscriber>(
        image_transport::create_camera_subscription(
          this,
          "image_raw",
          std::bind(&ImgSubscriber::raw_callback, this, std::placeholders::_1, std::placeholders::_2),
          "compressed",
          rclcpp::QoS {10}.get_rmw_qos_profile()
        )
    );

    RCLCPP_INFO_STREAM(get_logger(), "img_subscriber node started");

  }

  //Destroy windows
  ~ImgSplitter()
  {
    cv::destroyAllWindows();
  }

private:
  std::shared_ptr<image_transport::CameraSubscriber> sub_raw_;

  void raw_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr& img,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr&
  ) {
    show_image("raw img", img);
  }

};

void show_image(const std::string & window, const sensor_msgs::msg::Image::ConstSharedPtr& img) {
  cv::namedWindow(window);
  cv::imshow(window, cv_bridge::toCvCopy(*img, img->encoding)->image);
  cv::waitKey(1);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImgSplitter>());
  rclcpp::shutdown();
  return 0;
}
