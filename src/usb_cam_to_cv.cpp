#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>

cv_bridge::CvImagePtr cv_ptr;
cv::Mat cam_image;

static const std::string OPENCV_WINDOW = "Demo window";
static const std::string OPENCV_WINDOW1 = "Demo window1";

bool new_image = false;

void topic_to_cv(sensor_msgs::Image msg)
{
  try
  {
  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  cam_image = cv_ptr -> image;
  new_image = true;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
}


int main(int argc , char ** argv)
{
  ros::init(argc, argv, "converter");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("usb_cam/image_raw", 1, topic_to_cv);
  cv::namedWindow(OPENCV_WINDOW);
  cv::namedWindow(OPENCV_WINDOW1);
  
  cv::SimpleBlobDetector detector;
  std::vector<cv::KeyPoint> keypoints;
  

  while(ros::ok())
  {
    ros::spinOnce();
    if (new_image)
    {
      std::cout << "1" << std::endl;
      cv::Mat cam_im_with_keypoints;
      std::cout << "2" << std::endl;
      cv::cvtColor(cam_image, cam_im_with_keypoints, cv::COLOR_BGR2GRAY);
      std::cout << "3" << std::endl;
      cv::imshow(OPENCV_WINDOW, cam_image);
      cv::imshow(OPENCV_WINDOW1, cam_im_with_keypoints);
      // detector.detect(cam_im_with_keypoints, keypoints);
      // std::cout << "4" << std::endl;
      // cv::drawKeypoints(cam_image, keypoints, cam_im_with_keypoints, cv::Scalar(0, 0, 255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
      // std::cout << "5" << std::endl;
      // cv::imshow(OPENCV_WINDOW, cam_im_with_keypoints);
      // std::cout << "6" << std::endl;
      cv::waitKey(3);
      new_image = false;
    }
  }
  return 0;
}