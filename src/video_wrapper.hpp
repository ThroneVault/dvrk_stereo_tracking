#include<opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"
#include <iostream>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

// class to open video file and read frame by frame.
// also allows you to set crop

using namespace cv;
using namespace std;

class VideoWrapper
{
  public:
    VideoWrapper()
    {
      ros::NodeHandle nh;
      cv::namedWindow("view");
      cv::startWindowThread();
      image_transport::ImageTransport it(nh);
      //string left_camera_topic ("/stereo/left/uvc_camera_stereo_left/camera_info_url");
      string left_camera_topic ("/stereo/left/image_rect_color");
      image_transport::Subscriber sub = it.subscribe(left_camera_topic, 1, &VideoWrapper::imageCallback, this);
      ros::spin();

    }

    void imageCallback(const sensor_msgs::ImageConstPtr& msg)
    {
      try
      {

            cv_ptr = cv_bridge::toCvCopy(msg, "bgr8")->image;

        cv::imshow("view", cv_ptr);
        cv::waitKey(30);
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
      }
    }

    void SetCrop(Rect crop_rect)
    {
      crop_rect_ = crop_rect;
      crop_isset_ = true;
    }

    Mat GetFrame()
    {
      return cv_ptr;
    }
      //Mat frame;
      //cap_ >> frame;

      //if (crop_isset_)
      //{

        //Mat dstFrame = Mat::ones(frame.size(), frame.type());
        //Mat binary_mask = Mat(frame.size(), CV_8UC1, Scalar::all(0)); ;
        //rectangle(binary_mask, crop_rect_, 255, CV_FILLED);
        //cvtColor(binary_mask, binary_mask, CV_GRAY2BGR);
        //bitwise_and(binary_mask, frame, dstFrame);


        //return dstFrame;
      //}
      //else
        //return frame;
    //}

  private:
    cv::Rect crop_rect_;
    string filename_;
    VideoCapture cap_;
    bool crop_isset_;
    Mat cv_ptr;


};
