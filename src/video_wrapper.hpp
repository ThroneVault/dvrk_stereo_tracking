#include<opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"
#include <iostream>

// class to open video file and read frame by frame.
// also allows you to set crop

using namespace cv;
using namespace std;

class VideoWrapper
{
  public:
    VideoWrapper(string filename)
    {

      cap_.open(filename);

    }

    void SetCrop(Rect crop_rect)
    {
      crop_rect_ = crop_rect;
      crop_isset_ = true;
    }

    Mat GetFrame()
    {
      Mat frame;
      cap_ >> frame;



      if (crop_isset_)
      {

        Mat dstFrame = Mat::ones(frame.size(), frame.type());
        Mat binary_mask = Mat(frame.size(), CV_8UC1, Scalar::all(0)); ;
        rectangle(binary_mask, crop_rect_, 255, CV_FILLED);
        cvtColor(binary_mask, binary_mask, CV_GRAY2BGR);
        bitwise_and(binary_mask, frame, dstFrame);


        return dstFrame;
      }
      //return frame(crop_rect_);
      else
        return frame;
    }

  private:
    cv::Rect crop_rect_;
    string filename_;
    VideoCapture cap_;
    bool crop_isset_;


};
