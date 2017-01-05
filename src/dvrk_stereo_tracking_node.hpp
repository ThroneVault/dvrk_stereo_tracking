#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"
#include <iostream>

class StereoTracking
{
  public:
    StereoTracking()
    {
    }

    void storeImageCb(const sensor_msgs::ImageConstPtr& msg)
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


