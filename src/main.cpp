#include "stereo_tracking.hpp"

using namespace std;

int main(int argc, char *argv[]) {


  ros::init(argc, argv, "image_listener");

  StereoTracking st;

  //while (1)
  //{

  cout << "1) View Crop\n2)View Thresholded\n3)Start Tracking\n Ctrl-C to exit";
  int option;
  //cin >> option;

  //switch (option)
  //{
  //case 1: cout << "1 selected";
  //viewCrop();
  //break;
  //case 2: cout << "2 selected";
  st.ViewThresholded();
  //break;
  //case 2: cout << "2 selected";
  //st.TrackBlob();
  //break;
  //case 0: exit(0);
  //break;
  //default: cout << "invalid option";
  //break;
  //}
  //}


  return 0;
}
