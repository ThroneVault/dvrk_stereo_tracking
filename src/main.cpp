#include "stereo_tracking.hpp"

using namespace std;

int main(int argc, char *argv[]) {


  ros::init(argc, argv, "image_listener");

  StereoTracking st;

  while (1)
  {
    destroyAllWindows();

    cout << "\n\n1) Set Crop\n2) View cropped Image\n3) Start Tracking\n Ctrl-C to exit\n\n";
    int option;
    cin >> option;

    switch (option)
    {
      case 1: cout << "1 selected";
              st.SetCrop();
              break;

      case 2: cout << "2 selected";
              st.ViewCrop();
              break;

      case 3: cout << "3 selected";
              st.TrackBlob();
              break;

      case 0: exit(0);
              break;

      default: cout << "invalid option";
               break;
    }
  }

  return 0;
}
