#include "stereo_tracking.hpp"


StereoTracking::StereoTracking() {
  flag_ = false;
  left_crop_rect_ = Rect(250, 50,250, 430);
  right_crop_rect_ = Rect(150, 50, 275, 430);
  left_xy_pub = nh_.advertise<std_msgs::Int8>("left_xy", 5);
  right_xy_pub = nh_.advertise<std_msgs::Int8>("right_xy", 5);
  hsv_lower_threshold_ = cv::Scalar(60, 89, 185);
  hsv_upper_threshold_ = cv::Scalar(86, 64, 225);

  bgr_lower_threshold_ = cv::Scalar(121, 185, 185);
  bgr_upper_threshold_ = cv::Scalar(167, 225, 200);
  //hsv_lower_threshold_ = cv::Scalar(0, 100, 100);
  //hsv_upper_threshold_ = cv::Scalar(10, 255, 255);
  left_x = -1;
  left_y = -1;

  min_points_ = 50;
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
 
  Mat rgb_frame;
  cvtColor(input_frame, rgb_frame, CV_GRAY2BGR); 
  
  pt1 = Point(center.x - length, center.y);
  pt2 = Point(center.x + length, center.y);
  line(rgb_frame, pt1, pt2, Scalar(0, 0, 255),  2, 8);  //crosshair horizontal

  pt1 = Point(center.x, center.y - length);
  pt2 = Point(center.x, center.y + length);
  line(rgb_frame, pt1, pt2, Scalar(0, 0, 255),  2, 8);  //crosshair vertical
  
  return rgb_frame;
}

void StereoTracking::TrackBlobLeftCb(const sensor_msgs::ImageConstPtr& msg) {

  Mat input_frame, input_hsv_frame, lower_red_hue_range;
  input_frame = GetCroppedImage(cv_bridge::toCvCopy(msg, "bgr8") -> image, left_crop_rect_);
  input_frame = GetRGBThresholdedImage(input_frame);

  Point center = GetCenter(input_frame);

  // TODO fix. too naive
  left_x = center.x;
  left_y = center.y;

  input_frame = DrawCrosshair(input_frame, center);

  cv::imshow("left", input_frame);
  cv::waitKey(10);



  //left_xy.data = 17;
  //left_xy_pub.publish(left_xy);
}

void StereoTracking::TrackBlobRightCb(const sensor_msgs::ImageConstPtr& msg) {

  Mat input_frame, input_hsv_frame, lower_red_hue_range;
  input_frame = GetCroppedImage(cv_bridge::toCvCopy(msg, "bgr8") -> image, right_crop_rect_);
  input_frame = GetRGBThresholdedImage(input_frame);

  Point center = GetCenter(input_frame);
  right_x = center.x;
  right_y = center.y;

  input_frame = DrawCrosshair(input_frame, center);

  cv::imshow("right", input_frame);
  cv::waitKey(10);

  //right_xy.data = 18;
  //right_xy_pub.publish(right_xy);
  //
  if (left_x != -1)
  {

// triangulate
  }
}

void StereoTracking::TrackBlob()
{
  image_transport::ImageTransport it(nh_);
  image_transport::Subscriber left_sub = it.subscribe("/stereo/left/image_rect_color", 1, &StereoTracking::TrackBlobLeftCb, this);
  image_transport::Subscriber right_sub = it.subscribe("/stereo/right/image_rect_color", 1, &StereoTracking::TrackBlobRightCb, this);

  ros::spin();
}