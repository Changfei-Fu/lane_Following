#include <iostream>
#include <ros/ros.h>
#include "opencv2/opencv.hpp"

#include "cv_bridge/cv_bridge.h"
#include <geometry_msgs/Twist.h>
#include <Eigen/Core>
#include "Eigen/Dense"
#include <cmath>

using namespace cv;
using namespace std;

class Lane_Following{
private:
  ros::Subscriber img_sub;
  ros::Publisher cmd_vel_pub_;
  void subs_callback(const sensor_msgs::CompressedImageConstPtr& msg);

  Point prevpt1 = Point(50, 60);
  Point prevpt2 = Point(250, 60);
  Point cpt[2];
  Point fpt;
  int minlb[2];
  double ptdistance[2];
  double threshdistance[2];
  vector<double> mindistance1;
  vector<double> mindistance2;
  int error1,error2;
public:
    Lane_Following();
    Mat Homography();
    Mat frame;
};
