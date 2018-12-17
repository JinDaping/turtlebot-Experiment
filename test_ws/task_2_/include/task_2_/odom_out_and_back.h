#ifndef ODOM_OUT_AND_BACK_H
#define ODOM_OUT_AND_BACK_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include "math.h"
#include <sstream>
#include <iostream>
#include <string>
#include <fstream>
#include <task_2_/PID.h>
using namespace std;

typedef std::pair<tf::Vector3, double> Pose_Type;

#define Kh 1.5707796327 //误差原因2

class Point2Point {
public:
  Point2Point();
  void move_robot_(float speed,float rotation);
  void stop();
  Pose_Type Get_RobotPose(tf::TransformListener &listener);
  void PID_Init_();
  void PID_Contrl_(float targetX, float taregtY, float targetA);
  ros::Publisher pub;
  geometry_msgs::Twist Move_cmd;
  tf::TransformListener listener;
  float velocity[4];
  float angular[4];
  int wait_time[4];
  Pose_Type Pose_Current;
  Pose_Type Pose_tmp;
  bool flag;
  float linear_duration;
  float rotation_duration;
  int ticks;
  int k;
  int rate;

  float back_px_;
  float back_py_;
  float back_angular_;
  float target_px_;
  float target_py_;
  float target_pa_;
  PID_AbsoluteType PoseXPID;
  PID_AbsoluteType PoseYPID;
  PID_AbsoluteType YawPID;

private:
  ros::NodeHandle nh;
};

#endif
