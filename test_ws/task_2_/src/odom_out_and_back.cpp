#include <task_2_/odom_out_and_back.h>

Point2Point::Point2Point() {
  flag = true;
  k = 0;
  rate = 200;
  velocity[0] = 0.1;
  angular[0] = 0.5;
  wait_time[0] = 1;
  for (int i = 1; i < 4; i++) {
    velocity[i] = velocity[i - 1] + 0.05;
    angular[i] = angular[i - 1] + 0.25;
    wait_time[i] = wait_time[i - 1] + 1;
  }
  PID_Init_();
  pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
}
void Point2Point::stop() {
  Move_cmd.linear.x = 0.0;
  Move_cmd.linear.y = 0.0;
  Move_cmd.linear.z = 0.0;

  Move_cmd.angular.x = 0.0;
  Move_cmd.angular.y = 0.0;
  Move_cmd.angular.z = 0.0;
}

void Point2Point::move_robot_(float speed, float rotation) {
  Move_cmd.linear.x = speed;
  Move_cmd.linear.y = 0.0;
  Move_cmd.linear.z = 0.0;

  Move_cmd.angular.x = 0.0;
  Move_cmd.angular.y = 0.0;
  Move_cmd.angular.z = -rotation;
}
Pose_Type Point2Point::Get_RobotPose(tf::TransformListener &plistener) {
  tf::StampedTransform transform;
  tf::Quaternion Orientation;
  tf::Vector3 Position;
  double Roll, Pitch, Yaw;

  try {
    if (flag) {
      plistener.waitForTransform("/odom", "/base_footprint", ros::Time(0),
                                 ros::Duration(1.0));
      flag = false;
    }
    plistener.lookupTransform("/odom", "/base_footprint", ros::Time(0),
                              transform);
  } catch (tf::TransformException ex) {
    ROS_ERROR("%s", ex.what());
    ros::Duration(1.0).sleep();
  }
  Position = transform.getOrigin();
  Orientation = transform.getRotation();
  tf::Matrix3x3(Orientation).getRPY(Roll, Pitch, Yaw);

  return std::make_pair(Position, Yaw);
}

void Point2Point::PID_Init_() {
  // X
  PoseXPID.kp = 0;
  PoseXPID.ki = 0;
  PoseXPID.kd = 0;

  PoseXPID.errNow = 0;
  PoseXPID.errOld = 0;
  PoseXPID.errILim = 0.5;
  PoseXPID.OutMAX = 1;
  PoseXPID.ctrOut = 0;
  // Y
  PoseYPID.kp = 0;
  PoseYPID.ki = 0;
  PoseYPID.kd = 0;

  PoseYPID.errNow = 0;
  PoseYPID.errOld = 0;
  PoseYPID.errILim = 0.5;
  PoseYPID.OutMAX = 1;
  PoseYPID.ctrOut = 0;

  // angular
  YawPID.kp = 0;
  YawPID.ki = 0;
  YawPID.kd = 0;

  YawPID.errNow = 0;
  YawPID.errOld = 0;
  YawPID.errILim = 0.5;
  YawPID.OutMAX = 1.25;
  YawPID.ctrOut = 0;
}
void Point2Point::PID_Contrl_(float targetX, float targetY, float targetA) {
  Pose_Current = Get_RobotPose(listener);
  back_px_ = Pose_Current.first.x();
  back_py_ = Pose_Current.first.y();
  // cout << "Pose_Current.first.x=" << point2point_.Pose_Current.first.x()
  //      << endl;
  // cout << "Pose_Current.first.y=" << point2point_.Pose_Current.first.y()
  //      << endl;
  // cout << "Pose_Current.first.yaw=" << point2point_.Pose_Current.second
  //      << endl;
  // X
  PoseXPID.errNow = targetX - back_px_;
  PID_AbsoluteMode(&PoseXPID);
  if (PoseXPID.ctrOut > PoseXPID.OutMAX) {
    PoseXPID.ctrOut = PoseXPID.OutMAX;
  } else if (PoseXPID.ctrOut < -PoseXPID.OutMAX) {
    PoseXPID.ctrOut = -PoseXPID.OutMAX;
  }
  PoseYPID.errNow = targetY - back_py_;
  PID_AbsoluteMode(&PoseYPID);
  if (PoseYPID.ctrOut > PoseYPID.OutMAX) {
    PoseYPID.ctrOut = PoseYPID.OutMAX;
  } else if (PoseYPID.ctrOut < -PoseYPID.OutMAX) {
    PoseYPID.ctrOut = -PoseYPID.OutMAX;
  }
  YawPID.errNow = targetA - back_angular_;
  PID_AbsoluteMode(&YawPID);
  if (YawPID.ctrOut > YawPID.OutMAX) {
    YawPID.ctrOut = YawPID.OutMAX;
  } else if (YawPID.ctrOut < -YawPID.OutMAX) {
    YawPID.ctrOut = -YawPID.OutMAX;
  }

  if ((k + 1) % 2 == 1) {
    move_robot_(PoseXPID.ctrOut, YawPID.ctrOut);
  } else {
    move_robot_(PoseYPID.ctrOut, YawPID.ctrOut);
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "point2point_node_");
  Point2Point point2point_;
  double target_p_[5][3] = {
      {0, 0, 0}, {1, 0, Kh}, {1, -1, 2 * Kh}, {0, -1, 3 * Kh}, {0, 0, 4 * Kh}};
  ros::Rate loop_rate(point2point_.rate);

  while (ros::ok()) {
    for (; point2point_.k < 5; point2point_.k++) {
      if (point2point_.k > 0) {
        // angular
        while (target_p_[point2point_.k][2] > point2point_.target_pa_) {
          point2point_.PID_Contrl_(0, 0, target_p_[point2point_.k][2]);
          point2point_.pub.publish(point2point_.Move_cmd);
          loop_rate.sleep();
        }
        // linear
        if (point2point_.k % 2 == 1) {
          while (target_p_[point2point_.k][0] > point2point_.back_px_) {
            point2point_.PID_Contrl_(target_p_[point2point_.k][0], 0, target_p_[point2point_.k][2]);
            point2point_.pub.publish(point2point_.Move_cmd);
            loop_rate.sleep();
          }
        } else {
          while (target_p_[point2point_.k][1] > point2point_.back_py_) {
            point2point_.PID_Contrl_(0, target_p_[point2point_.k][1], target_p_[point2point_.k][2]);
            point2point_.pub.publish(point2point_.Move_cmd);
            loop_rate.sleep();
          }
        }
      }
    }
    point2point_.stop();
    point2point_.pub.publish(point2point_.Move_cmd);
    ros::shutdown();
  }
  return 0;
}