#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"
#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>
#include "std_msgs/Int8.h"
#include "geometry_msgs/Point.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

using namespace cv;
using namespace ros;
using namespace std;

enum CameraSide { LEFT, RIGHT };

class StereoTracking
{
  public:
    StereoTracking();

    void StoreImageCb(const sensor_msgs::ImageConstPtr& msg);

    void SetCrop();
    void ViewCrop();
    static void MouseHandlerStatic(int event, int x, int y, int flags, void* that);
    void MouseHandler(int event, int x, int y, int flags);

    void ViewThresholded();

    void TrackBlob();

    void TrackBlobLeftCb(const sensor_msgs::ImageConstPtr& msg);
    void TrackBlobRightCb(const sensor_msgs::ImageConstPtr& msg);
    Mat GetRGBThresholdedImage(Mat input_bgr_frame);
    Point GetCenter (Mat input_frame);
    Mat GetCroppedImage(Mat input_bgr_frame, Rect crop_rect);
    Mat DrawCrosshair(Mat input_frame, Point center);
    void TriangulatePoints();
    Mat SegmentStem (Mat input_frame);
    Vec4f FitLine (Mat input_frame);
    void DrawLine(Mat img, Vec4f line_params, int thickness, Scalar color);
    Point GetClosestPoint(Mat input_frame, Vec4f line_params);

  private:
    bool flag_;
    cv_bridge::CvImagePtr cv_ptr_;
    ros::NodeHandle nh_;
    image_transport::Subscriber sub_;

    cv::Scalar hsv_lower_threshold_, hsv_upper_threshold_;
    cv::Scalar stem_hsv_lower_threshold_, stem_hsv_upper_threshold_;
    cv::Scalar bgr_lower_threshold_, bgr_upper_threshold_;

    Mat frame_;

    Rect crop_rect_, left_crop_rect_, right_crop_rect_;
    Mat dstFrame_;
    Mat binary_mask_;

    Mat input_hsv_frame, lower_red_hue_range;
    Publisher left_xy_pub, right_xy_pub, raw_xyz_pub;
    geometry_msgs::Point left_xy, right_xy, raw_xyz;

    int left_x, left_y;
    int right_x, right_y;

    int min_points_;
    Mat projection_matrix_left_, projection_matrix_right_;

    int drag, select_flag;

    cv::Point point1, point2;    
    //cv::Mat frame;
    const char* src_window;
    bool callback;
    CameraSide current_camera_;
};

