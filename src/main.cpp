#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include "video_wrapper.hpp"
#include "frame_processor.hpp"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
    cv::waitKey(30);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char *argv[]) {


  /*
   * TODO
   *
   * Convert to use bagfile directly
   * convert to catkin project
   * publish topic directly
   * write python code to publish to transform
   * camera calibration can be used as a yaml file
   */


  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  cv::namedWindow("view");
  cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  //string left_camera_topic ("/stereo/left/uvc_camera_stereo_left/camera_info_url");
  string left_camera_topic ("/stereo/left/image_rect_color");
  image_transport::Subscriber sub = it.subscribe(left_camera_topic, 1, imageCallback);


  //VideoWrapper left_vw = VideoWrapper(argv[1]);
  //FrameProcessor left_fp = FrameProcessor();

  //VideoWrapper right_vw = VideoWrapper(argv[2]);
  //FrameProcessor right_fp = FrameProcessor();

  //Mat left_frame, right_frame, left_thresholded_frame, right_thresholded_frame;
  //Point left_center, right_center;

  //Rect crop_rect(250,0 ,390, 480);
  //left_vw.SetCrop(crop_rect);
  //right_vw.SetCrop(crop_rect);


  //// TODO check if frame exists
  //while (1)
  //{
    //left_frame = left_vw.GetFrame();
    //left_center = left_fp.GetRedCenter(left_frame);
    //left_thresholded_frame = left_fp.GetThresholdedImage(left_frame);
    //cout << "left_center is [" << left_center.x << "," << left_center.y << "]"<< endl;
    //imshow("left", left_thresholded_frame);
    //waitKey(0);

    //right_frame = right_vw.GetFrame();
    //right_center = right_fp.GetRedCenter(right_frame);
    //right_thresholded_frame = right_fp.GetThresholdedImage(right_frame);
    //cout << "right_center is [" << right_center.x << "," << right_center.y << "]"<< endl;
    //imshow("right", right_thresholded_frame);
    //waitKey(0);

    //// open camera matrix and triangulate point

  //}

  return 0;
}
