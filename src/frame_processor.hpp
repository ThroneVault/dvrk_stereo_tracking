#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"
#include <iostream>

// class to open video file and read frame by frame.
// also allows you to set crop

using namespace cv;
using namespace std;

//struct Point {
//float x;
//float y;
//};

class FrameProcessor 
{
  public:
    FrameProcessor()
    {
      min_points_ = 20;
    }

  // have option to select crop area
    Mat GetThresholdedImage(Mat input_bgr_frame)
    {
      Mat input_hsv_frame, lower_red_hue_range;

      cv::cvtColor(input_bgr_frame, input_hsv_frame, cv::COLOR_BGR2HSV);

      cv::inRange(input_hsv_frame, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), lower_red_hue_range);

      return lower_red_hue_range;
    }

    Point GetRedCenter(Mat input_bgr_frame)
    {

      Mat input_hsv_frame, lower_red_hue_range;

      cv::cvtColor(input_bgr_frame, input_hsv_frame, cv::COLOR_BGR2HSV);

      cv::inRange(input_hsv_frame, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), lower_red_hue_range);

      std::vector<cv::Point2i> locations;   // output, locations of non-zero pixels 
      cv::findNonZero(lower_red_hue_range, locations);

      long int sum_x, sum_y;
      sum_x = 0; sum_y = 0;

      for (vector<Point2i>::iterator it = locations.begin(); it != locations.end(); ++it)
      {
        sum_x += (*it).x;
        sum_y += (*it).y;
      }

      Point center;

      if (locations.size() > min_points_)
      {
        center.x = sum_x / locations.size();
        center.y = sum_y / locations.size();
        return center;
      }
      else
      {
        center.x = -1;
        center.y = -1;
        return center;
      }

    }

    void SetMinPoints(int min_points)
    {
      min_points_ = min_points;
    }

  private:
    int min_points_;

};
