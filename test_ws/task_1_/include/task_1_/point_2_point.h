#ifndef POINT_2_POINT_H
#define POINT_2_POINT_H

#include "math.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <sstream>
#include <iostream>
#include<string>
#include<fstream>
using namespace std;

#define Kh 1.5707796327

class Point2Point {
public:
  Point2Point();
  void move_robot_(float speed,float rotation);
  void stop();
  ros::Publisher pub;
  geometry_msgs::Twist Move_cmd;
  float velocity[4];
  float angular[4];
  int wait_time[4];
private:
  ros::NodeHandle nh;
};

#endif
