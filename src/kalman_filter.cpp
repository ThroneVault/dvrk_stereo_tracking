// subscribe to the left_xy and right_xy topic.
// maintain memory here
//
// use eigen library
//
//
#include <Eigen/Dense>
#include <iostream>
#include <ros/ros.h>

using namespace Eigen;
using namespace std;



int main()
{
  int num_states = 4;
  int num_measurements = 4;

  MatrixXd A(num_states, num_states); // system dynamics 
  MatrixXd C(num_measurements, num_states); // output
  MatrixXd Q(num_states, num_states); // process noise covariance 
  MatrixXd R(num_measurements, num_measurements); // Measurement noise covariance

  //A << 5,5,5,5,5;


  // create callbacks 
  // track both images separately
  


  return 0;
}
