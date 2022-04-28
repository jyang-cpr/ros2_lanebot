#include <memory>

#include "cv_bridge/cv_bridge.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/objdetect.hpp"
#include "opencv2/videoio.hpp"

using std::placeholders::_1;

static const std::string OPENCV_WINDOW = "Lanebot Window";

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("minimal_subscriber")
    {
      subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/color/preview/image", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
      cv::namedWindow(OPENCV_WINDOW);
    }

    ~MinimalSubscriber()
    {
      cv::destroyWindow(OPENCV_WINDOW);
    }

  private:
    void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg) const
    {
      cv_bridge::CvImagePtr cv_ptr;
      cv::Mat greyMat;
      cv::Mat smoothMat;

      try
      {
          cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
          cv::cvtColor(cv_ptr->image, greyMat, cv::COLOR_BGR2GRAY);
          cv::GaussianBlur(greyMat, smoothMat, cv::Size( 5, 5), 0, 0);
      }
      catch (cv_bridge::Exception& e)
      {
          RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());

          return;
      }

      cv::imshow(OPENCV_WINDOW, smoothMat);
      cv::waitKey(3);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}