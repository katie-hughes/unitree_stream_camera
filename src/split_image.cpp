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
    sub_raw_ = std::make_shared<image_transport::CameraSubscriber>(
        image_transport::create_camera_subscription(
          this,
          "image_raw",
          std::bind(&ImgSplitter::raw_callback, this, std::placeholders::_1, std::placeholders::_2),
          "compressed",
          rclcpp::QoS {10}.get_rmw_qos_profile()
        )
    );
    // sub_raw_ = create_subscription<sensor_msgs::msg::Image>(
    //   "image_raw",
    //   10,
    //   std::bind(&ImgSplitter::raw_callback, this, std::placeholders::_1)
    // );

    RCLCPP_INFO_STREAM(get_logger(), "img_subscriber node started");

  }

  //Destroy windows
  ~ImgSplitter()
  {
    cv::destroyAllWindows();
  }

private:
  std::shared_ptr<image_transport::CameraSubscriber> sub_raw_;
    // rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_raw_;

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
    show_img("Whole", cv_ptr->image);
    show_img("left", left);
    show_img("right", right);
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
