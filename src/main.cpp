#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include "video_wrapper.hpp"
#include "frame_processor.hpp"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

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

  VideoWrapper left_vw = VideoWrapper();
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
    imshow("right", left_vw.GetFrame());
    waitKey(0);

    //// open camera matrix and triangulate point

  //}

  return 0;
}
