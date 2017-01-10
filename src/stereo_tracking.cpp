#include "stereo_tracking.hpp"


StereoTracking::StereoTracking() {
  flag_ = false;
  left_crop_rect_ = Rect(250, 50,250, 430);
  right_crop_rect_ = Rect(150, 50, 275, 430);
  left_xy_pub = nh_.advertise<geometry_msgs::Point>("left_xy", 5);
  right_xy_pub = nh_.advertise<geometry_msgs::Point>("right_xy", 5);
  hsv_lower_threshold_ = cv::Scalar(60, 89, 185);
  hsv_upper_threshold_ = cv::Scalar(86, 64, 225);

  bgr_lower_threshold_ = cv::Scalar(121, 185, 185);
  bgr_upper_threshold_ = cv::Scalar(167, 225, 200);
  //hsv_lower_threshold_ = cv::Scalar(0, 100, 100);
  //hsv_upper_threshold_ = cv::Scalar(10, 255, 255);
  
  stem_hsv_lower_threshold_ = cv::Scalar(14, 0, 37);
  stem_hsv_upper_threshold_ = cv::Scalar(100, 58, 51);

  left_x = -1;
  left_y = -1;

  min_points_ = 20;


  // read calibration file
  String parameters_left_path = ros::package::getPath("dvrk_stereo_tracking") + "/stereo_cam0_opencv.yaml";
  FileStorage fs_left(parameters_left_path, FileStorage::READ);
  fs_left["projection_matrix"] >> projection_matrix_left_;
  cout << "projection_matrix_left_" << projection_matrix_left_;

  String parameters_right_path = ros::package::getPath("dvrk_stereo_tracking") + "/stereo_cam0_opencv.yaml";
  FileStorage fs_right(parameters_right_path, FileStorage::READ);
  fs_right["projection_matrix"] >> projection_matrix_right_;
  cout << "projection_matrix_right_" << projection_matrix_right_;


}

void StereoTracking::StoreImageCb(const sensor_msgs::ImageConstPtr& msg)
{
  flag_ = true;

  try
  {
    cv_ptr_ = cv_bridge::toCvCopy(msg, "bgr8");
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

void StereoTracking::MouseHandlerStatic(int event, int x, int y, int flags, void* that)
{
  // below code is to bypass opencv restriction of not allowing non-static member callbacks. TODO - understand
  StereoTracking* temp = reinterpret_cast<StereoTracking*>(that);
  temp->MouseHandler(event, x, y, flags);
}

void StereoTracking::MouseHandler(int event, int x, int y, int flags)
{
    if (event == CV_EVENT_LBUTTONDOWN && !drag && !select_flag)
    {
        /* left button clicked. ROI selection begins */
        point1 = cv::Point(x, y);
        drag = 1;
    }

    if (event == CV_EVENT_MOUSEMOVE && drag && !select_flag)
    {
        /* mouse dragged. ROI being selected */
        cv::Mat img1 = frame_.clone();
        point2 = cv::Point(x, y);
        cv::rectangle(img1, point1, point2, CV_RGB(255, 0, 0), 3, 8, 0);
        cv::imshow(src_window, img1);
    }

    if (event == CV_EVENT_LBUTTONUP && drag && !select_flag)
    {
        cv::Mat img2 = frame_.clone();
        point2 = cv::Point(x, y);
        drag = 0;
        select_flag = 1;
        cv::imshow(src_window, img2);
        callback = true;
        if ((current_camera_) == LEFT)
          left_crop_rect_ = Rect(point1, point2);
        else
          right_crop_rect_ = Rect(point1, point2);
        destroyWindow(src_window);
    }
}

void StereoTracking::SetCrop() {
  image_transport::ImageTransport it(nh_);

  flag_ = false;
  current_camera_ = LEFT;
  drag = 0;
  select_flag = 0;
  src_window = "Select ROI";
  callback = false;
  sub_ = it.subscribe("/stereo/left/image_rect_color", 1, &StereoTracking::StoreImageCb, this);

  while (!flag_)
  {
    ros::spinOnce();
  }

  frame_ = cv_ptr_->image;
  cv::namedWindow( src_window, CV_WINDOW_AUTOSIZE);
  cv::imshow(src_window, frame_);
  cv::setMouseCallback(src_window, StereoTracking::MouseHandlerStatic, this);
  cv::waitKey(0);


  flag_ = false;
  drag = 0;
  select_flag = 0;
  src_window = "Select ROI";
  callback = false;
  current_camera_ = RIGHT;
  sub_ = it.subscribe("/stereo/right/image_rect_color", 1, &StereoTracking::StoreImageCb, this);

  while (!flag_)
  {
    ros::spinOnce();
  }

  frame_ = cv_ptr_->image;
  cv::namedWindow( src_window, CV_WINDOW_AUTOSIZE);
  cv::imshow(src_window, frame_);
  cv::setMouseCallback(src_window, StereoTracking::MouseHandlerStatic, this);
  cv::waitKey(0);

}

void StereoTracking::ViewCrop() {
  image_transport::ImageTransport it(nh_);
  sub_ = it.subscribe("/stereo/left/image_rect_color", 1, &StereoTracking::StoreImageCb, this);

  while (!flag_)
  {
    ros::spinOnce();
  }

  frame_ = cv_ptr_->image;

  dstFrame_ = Mat::ones(frame_.size(), frame_.type());
  binary_mask_ = Mat(frame_.size(), CV_8UC1, Scalar::all(0)); ;
  rectangle(binary_mask_, left_crop_rect_, 255, CV_FILLED);
  cvtColor(binary_mask_, binary_mask_, CV_GRAY2BGR);
  bitwise_and(binary_mask_, frame_, dstFrame_);


  cv::imshow("temp", dstFrame_);
  cv::waitKey(0);

  cout << "left image done";

  // for right
  flag_ = false;

  sub_ = it.subscribe("/stereo/right/image_rect_color", 1, &StereoTracking::StoreImageCb, this);
  while (!flag_)
  {
    ros::spinOnce();
  }

  frame_ = cv_ptr_->image;

  dstFrame_ = Mat::ones(frame_.size(), frame_.type());
  binary_mask_ = Mat(frame_.size(), CV_8UC1, Scalar::all(0)); ;
  rectangle(binary_mask_, right_crop_rect_, 255, CV_FILLED);
  cvtColor(binary_mask_, binary_mask_, CV_GRAY2BGR);
  bitwise_and(binary_mask_, frame_, dstFrame_);


  cv::imshow("temp", dstFrame_);
  cv::waitKey(0);
}

void StereoTracking::ViewThresholded()
{
  flag_ = false;

  image_transport::ImageTransport it(nh_);
  image_transport::Subscriber sub_ = it.subscribe("/stereo/left/image_rect_color", 1, &StereoTracking::StoreImageCb, this);
  while (!flag_)
  {
    ros::spinOnce();
  }

  frame_ = cv_ptr_->image;

  cv::cvtColor(frame_, input_hsv_frame, cv::COLOR_BGR2HSV);
  cv::inRange(input_hsv_frame, hsv_lower_threshold_, hsv_upper_threshold_, lower_red_hue_range);

  //cv::inRange(frame_, bgr_lower_threshold_, bgr_upper_threshold_, lower_red_hue_range);

  cv::imshow("temp", lower_red_hue_range);
  cv::waitKey(0);

  cout << "left image done";
}


Mat StereoTracking::GetRGBThresholdedImage(Mat input_bgr_frame)
{
  Mat output;
  cv::inRange(input_bgr_frame, bgr_lower_threshold_, bgr_upper_threshold_, output);

  return output;
}

Mat StereoTracking::GetCroppedImage(Mat input_bgr_frame, Rect crop_rect)
{
  Mat dstFrame, binarymask;

  dstFrame = Mat::ones(input_bgr_frame.size(), input_bgr_frame.type());
  binarymask = Mat(input_bgr_frame.size(), CV_8UC1, Scalar::all(0)); ;
  rectangle(binarymask, crop_rect, 255, CV_FILLED);
  cvtColor(binarymask, binarymask, CV_GRAY2BGR);
  bitwise_and(binarymask, input_bgr_frame, dstFrame);

  return dstFrame;
}

Point StereoTracking::GetCenter (Mat input_frame)
{
  std::vector<cv::Point2i> locations;   // output, locations of non-zero pixels 
  cv::findNonZero(input_frame, locations);

  long int sum_x = 0, sum_y =0;

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


Mat StereoTracking::DrawCrosshair(Mat input_frame, Point center)
{
  Point pt1, pt2;

  int length = 10;
 
  pt1 = Point(center.x - length, center.y);
  pt2 = Point(center.x + length, center.y);
  line(input_frame, pt1, pt2, Scalar(0, 0, 255),  2, 8);  //crosshair horizontal

  pt1 = Point(center.x, center.y - length);
  pt2 = Point(center.x, center.y + length);
  line(input_frame, pt1, pt2, Scalar(0, 0, 255),  2, 8);  //crosshair vertical
  
  return input_frame;
}

//Point StereoTracking::FindLowestPoint(Mat input_frame)
//{
//}

void StereoTracking::DrawLine(Mat img, Vec4f line_params, int thickness, Scalar color)
{
    //double theMult = max(img.rows,img.cols);
    //cout << "theMult " << theMult << endl << endl;
    //// calculate start point
    //Point startPoint;
    //startPoint.x = line_params[2]- theMult*line_params[0];// x0
    //startPoint.y = line_params[3] - theMult*line_params[1];// y0
    //cout << "startPoint " << startPoint;
    //// calculate end point
    //Point endPoint;
    //endPoint.x = line_params[2]+ theMult*line_params[0];//x[1]
    //endPoint.y = line_params[3] + theMult*line_params[1];//y[1]
    //cout << "endPoint " << startPoint;

    //// draw overlay of bottom line_paramss on image
    ////cvClipLine(cvGetSize(img), &startPoint, &endPoint);
    //line(img, startPoint, endPoint, color, thickness, 8, 0);
    //
    
    float large_val = 500;
    
    Point startPoint;
    startPoint.x = line_params[2]- large_val * line_params[0];// x0 - m*vx
    startPoint.y = line_params[3] - large_val * line_params[1];// 
    cout << "startPoint " << startPoint;
     //calculate end point
     
    Point endPoint;
    endPoint.x = line_params[2]+ large_val*line_params[0];//x[1]
    endPoint.y = line_params[3] + large_val*line_params[1];//y[1]
    cout << "endPoint " << startPoint;
    
    Mat rgb;
    cvtColor(img, rgb, CV_GRAY2BGR); 
    //line(rgb, Point (0,0), Point (100,100), Scalar(0,0,255), 3, 8, 0);
    line(rgb, startPoint, endPoint, Scalar(0,0,255), 3, 8, 0);

    cv::imshow("DrawLine", rgb);
    cv::waitKey(10);
}

void StereoTracking::FitLine (Mat input_frame)
{

  std::vector<cv::Point2i> locations;   // output, locations of non-zero pixels 
  cv::findNonZero(input_frame, locations);

  Vec4f line_params;
  fitLine(locations, line_params, CV_DIST_L2, 0, 0.01, 0.01);

  cout << "line parameters is " << line_params << endl;

  DrawLine(input_frame, line_params, 10, CV_RGB(255, 0, 0));

}

Mat StereoTracking::SegmentStem (Mat input_frame) 
{
  Mat input_hsv_frame;
  Mat segmented_stem;
  cv::cvtColor(input_frame, input_hsv_frame, cv::COLOR_BGR2HSV);
  cv::inRange(input_hsv_frame, stem_hsv_lower_threshold_, stem_hsv_upper_threshold_, segmented_stem);
  return segmented_stem;
}


void StereoTracking::TriangulatePoints()
{
  cv::Mat point4D(4, 1, CV_32FC1);

  float c0[] = {left_x, left_y};
  cv::Mat cam0pnts(2,1,CV_32FC1, c0);
  cout << "cam0pnts" << cam0pnts << endl;

  float c1[] = {right_x, right_y};
  cv::Mat cam1pnts(2,1,CV_32FC1, c1);
  cout << "cam1pnts" << cam1pnts << endl;

  cv::triangulatePoints(projection_matrix_left_,projection_matrix_right_,cam0pnts,cam1pnts,point4D);

  cout << "projection_matrix_left_" << projection_matrix_left_ << endl;
  cout << "projection_matrix_right_" << projection_matrix_right_ << endl;

  cout << "triangulated Point is " << point4D <<endl;
}

void StereoTracking::TrackBlobLeftCb(const sensor_msgs::ImageConstPtr& msg) {

  Mat input_frame, input_hsv_frame, lower_red_hue_range;
  input_frame = GetCroppedImage(cv_bridge::toCvCopy(msg, "bgr8") -> image, left_crop_rect_);

  Mat segmented_stem = SegmentStem(input_frame);
  FitLine(segmented_stem);


  //input_frame = GetRGBThresholdedImage(input_frame);

  //Point center = GetCenter(input_frame);

  //// TODO fix. too naive
  //left_x = center.x;
  //left_y = center.y;

  //input_frame = DrawCrosshair(cv_bridge::toCvCopy(msg, "bgr8") -> image, center);

  ////cv::imshow("left", input_frame);
  //cv::imshow("left", segmented_stem);
  //cv::waitKey(10);

  //left_xy.x = center.x;
  //left_xy.y = center.y;
  //left_xy.z = -1;
  //left_xy_pub.publish(left_xy);
}

void StereoTracking::TrackBlobRightCb(const sensor_msgs::ImageConstPtr& msg) {

  Mat input_frame, input_hsv_frame, lower_red_hue_range;
  input_frame = GetCroppedImage(cv_bridge::toCvCopy(msg, "bgr8") -> image, right_crop_rect_);
  input_frame = GetRGBThresholdedImage(input_frame);

  Point center = GetCenter(input_frame);
  right_x = center.x;
  right_y = center.y;

  input_frame = DrawCrosshair(cv_bridge::toCvCopy(msg, "bgr8") -> image, center);

  cv::imshow("right", input_frame);
  cv::waitKey(10);

  right_xy.x = center.x;
  right_xy.y = center.y;
  right_xy.z = -1;
  right_xy_pub.publish(right_xy);

  if (left_x != -1)
  {
    TriangulatePoints();

  }
}

void StereoTracking::TrackBlob()
{
  image_transport::ImageTransport it(nh_);
  image_transport::Subscriber left_sub = it.subscribe("/stereo/left/image_rect_color", 1, &StereoTracking::TrackBlobLeftCb, this);
  //image_transport::Subscriber right_sub = it.subscribe("/stereo/right/image_rect_color", 1, &StereoTracking::TrackBlobRightCb, this);

  ros::spin();
}
