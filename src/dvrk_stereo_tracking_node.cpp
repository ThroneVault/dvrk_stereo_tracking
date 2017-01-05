#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include "dvrk_stereo_tracking_node.hpp"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

using namespace cv;
using namespace std;


bool croppedFlag = false;
cv_bridge::CvImagePtr cv_ptr;

void viewCropLeftCb(const sensor_msgs::ImageConstPtr& msg)
{
  croppedFlag = true;

  cout << "viewcropletcbcalled";
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

void viewCrop() {
  ros::NodeHandle nh;

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("/stereo/left/image_rect_color", 1, viewCropLeftCb);
  while (!croppedFlag)
  {
    ros::spinOnce();
  }

  Mat frame;
  frame = cv_ptr->image;

  Rect crop_rect(250,0 ,390, 480);

  Mat dstFrame = Mat::ones(frame.size(), frame.type());
  Mat binary_mask = Mat(frame.size(), CV_8UC1, Scalar::all(0)); ;
  rectangle(binary_mask, crop_rect, 255, CV_FILLED);
  cvtColor(binary_mask, binary_mask, CV_GRAY2BGR);
  bitwise_and(binary_mask, frame, dstFrame);


  cv::imshow("temp", dstFrame);
  cv::waitKey(0);

  cout << "left image done";

  // for right
  croppedFlag = false;

  sub = it.subscribe("/stereo/right/image_rect_color", 1, viewCropLeftCb);
  while (!croppedFlag)
  {
    ros::spinOnce();
  }


  frame = cv_ptr->image;


  dstFrame = Mat::ones(frame.size(), frame.type());
  binary_mask = Mat(frame.size(), CV_8UC1, Scalar::all(0)); ;
  rectangle(binary_mask, crop_rect, 255, CV_FILLED);
  cvtColor(binary_mask, binary_mask, CV_GRAY2BGR);
  bitwise_and(binary_mask, frame, dstFrame);


  cv::imshow("temp", dstFrame);
  cv::waitKey(0);
}

void viewThresholded()
{
  ros::NodeHandle nh;

  croppedFlag = false;

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("/stereo/left/image_rect_color", 1, viewCropLeftCb);
  while (!croppedFlag)
  {
    ros::spinOnce();
  }

  Mat frame;
  frame = cv_ptr->image;

  Mat input_hsv_frame, lower_red_hue_range;

  cv::cvtColor(frame, input_hsv_frame, cv::COLOR_BGR2HSV);

  cv::inRange(input_hsv_frame, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), lower_red_hue_range);

  cv::imshow("temp", lower_red_hue_range);
  cv::waitKey(0);

  cout << "left image done";

}

void track()
{
}

int main(int argc, char *argv[]) {


  ros::init(argc, argv, "image_listener");

  //while (1)
  //{

  cout << "1) View Crop\n2)View Thresholded\n3)Start Tracking\n Ctrl-C to exit";
  int option;
  //cin >> option;

  //switch (option)
  //{
  //case 1: cout << "1 selected";
  //viewCrop();
  //break;
  //case 2: cout << "2 selected";
  viewThresholded();
  //break;
  //case 0: exit(0);
  //break;
  //default: cout << "invalid option";
  //break;
  //}
  //}


  return 0;
}
