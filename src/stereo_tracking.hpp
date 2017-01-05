#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"
#include <iostream>
#include <ros/ros.h>
#include "std_msgs/Int8.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

using namespace cv;
using namespace ros;
using namespace std;

class StereoTracking
{
  public:
    StereoTracking();

    void StoreImageCb(const sensor_msgs::ImageConstPtr& msg);

    void ViewCrop();

    void ViewThresholded();

    void TrackBlob();

    void TrackBlobLeftCb(const sensor_msgs::ImageConstPtr& msg);
    void TrackBlobRightCb(const sensor_msgs::ImageConstPtr& msg);
    Mat GetRGBThresholdedImage(Mat input_bgr_frame);
    Point GetCenter (Mat input_frame);
    Mat GetCroppedImage(Mat input_bgr_frame, Rect crop_rect);
    Mat DrawCrosshair(Mat input_frame, Point center);

  private:
    bool flag_;
    cv_bridge::CvImagePtr cv_ptr_;
    ros::NodeHandle nh_;
    image_transport::Subscriber sub_;

    cv::Scalar hsv_lower_threshold_, hsv_upper_threshold_;
    cv::Scalar bgr_lower_threshold_, bgr_upper_threshold_;

    Mat frame_;

    Rect crop_rect_, left_crop_rect_, right_crop_rect_;
    Mat dstFrame_;
    Mat binary_mask_;

    Mat input_hsv_frame, lower_red_hue_range;
    Publisher left_xy_pub, right_xy_pub;
    std_msgs::Int8 left_xy, right_xy;

    int left_x, left_y;
    int right_x, right_y;

    int min_points_;
};
