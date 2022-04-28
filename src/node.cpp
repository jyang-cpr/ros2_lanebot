#include <memory>
#include <string>
#include <vector>

#include "cv_bridge/cv_bridge.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/objdetect.hpp"
#include "opencv2/videoio.hpp"

using std::placeholders::_1;

static const std::string RAW_WINDOW = "Raw Window";
static const std::string GREY_WINDOW = "Grey Window";
static const std::string EDGES_WINDOW = "Edges Window";
static const std::string SEGMENT_WINDOW = "Segment Window";
static const std::string LANES_WINDOW = "Lanes Window";

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("minimal_subscriber")
    {
      subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/color/preview/image", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
      cv::namedWindow(RAW_WINDOW);
      cv::namedWindow(GREY_WINDOW);
      cv::namedWindow(EDGES_WINDOW);
      //cv::namedWindow(SEGMENT_WINDOW);
      cv::namedWindow(LANES_WINDOW);
    }

    ~MinimalSubscriber()
    {
      cv::destroyWindow(RAW_WINDOW);
      cv::destroyWindow(GREY_WINDOW);
      cv::destroyWindow(EDGES_WINDOW);
      //cv::destroyWindow(SEGMENT_WINDOW);
      cv::destroyWindow(LANES_WINDOW);
    }

  private:
    void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg) const
    {
      cv_bridge::CvImagePtr cv_ptr;
      cv::Mat greyMat;
      cv::Mat smoothMat;
      cv::Mat edgesMat;
      cv::Mat maskMat;
      cv::Mat segmentMat;
      cv::Mat lanesMat;

      try
      {
        // Dimensions 250 x 400
        cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
        cv::cvtColor(cv_ptr->image, greyMat, cv::COLOR_BGR2GRAY);
        cv::GaussianBlur(greyMat, smoothMat, cv::Size( 5, 5), 0, 0);
        cv::Canny(smoothMat, edgesMat, 100, 130, 3);

        maskMat = cv::Mat::zeros(edgesMat.size(), edgesMat.type());

        cv::Point corners[1][3];
        corners[0][0] = cv::Point(0, maskMat.rows - 1);
        corners[0][1] = cv::Point(maskMat.cols - 1, maskMat.rows - 1);
        corners[0][2] = cv::Point((maskMat.cols - 1) / 2, (maskMat.rows - 1) / 1.75);
        const cv::Point* cornerList[1] = {corners[0]}; 
        int numPoints = 3; 
        int numPolygons = 1; 
        cv::fillPoly(maskMat, cornerList, &numPoints, numPolygons, cv::Scalar(255, 255, 255), 8);

        cv::bitwise_and(maskMat, edgesMat, segmentMat);

        lanesMat = cv_ptr->image.clone();
        std::vector<cv::Vec4i> lines;
        cv::HoughLinesP(segmentMat, lines, 1, CV_PI/180, 50, 50, 10 );
        for( size_t i = 0; i < lines.size(); i++ )
        {
            cv::Vec4i l = lines[i];
            line(lanesMat, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0, 0, 255), 3, cv::LINE_AA);
        }
      }
      catch (cv_bridge::Exception& e)
      {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());

        return;
      }

      cv::imshow(RAW_WINDOW, cv_ptr->image);
      cv::imshow(GREY_WINDOW, greyMat);
      cv::imshow(EDGES_WINDOW, edgesMat);
      //cv::imshow(SEGMENT_WINDOW, segmentMat);
      cv::imshow(LANES_WINDOW, lanesMat);
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