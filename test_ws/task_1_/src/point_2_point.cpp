#include <task_1_/point_2_point.h>

Point2Point::Point2Point() {
  velocity[0] = 0.1;
  angular[0] = 0.5;
  wait_time[0] = 1;
  for (int i = 1; i < 4; i++) {
    velocity[i] = velocity[i - 1] + 0.05;
    angular[i] = angular[i - 1] + 0.25;
    wait_time[i] = wait_time[i - 1] + 1;
  }
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

int main(int argc, char **argv) {
  ros::init(argc, argv, "point2point_node_");
  Point2Point point2point_;
  float linear_duration;
  float rotation_duration;
  int ticks;
  int k = 0;
  int rate =500;
  ros::Rate loop_rate(rate);
  while (ros::ok()) {  
    for (; k < 4; k++) {
      // linear    
      point2point_.move_robot_(point2point_.velocity[k],0);
      linear_duration = 1.0 / point2point_.velocity[k];
      ticks = static_cast<int>(linear_duration * rate);
      for (int j = 0; j < ticks; ++j) {
        point2point_.pub.publish(point2point_.Move_cmd);
        //  Sleep for the time remaining to let us hit our 100Hz publish  //10ms
        loop_rate.sleep();
      }
      // stop 
      point2point_.stop();
      point2point_.pub.publish(point2point_.Move_cmd);
      ros::Duration(point2point_.wait_time[k]).sleep();     
      // angular
      point2point_.move_robot_(0,point2point_.angular[k]);
      rotation_duration = Kh / point2point_.angular[k];
      ticks= static_cast<int>(rotation_duration * rate);//误差原因1
      for (int j = 0; j < ticks; ++j) {
        point2point_.pub.publish(point2point_.Move_cmd);
        //  Sleep for the time remaining to let us hit our 100Hz publish  //10ms
        loop_rate.sleep();
      }
      point2point_.stop();
      point2point_.pub.publish(point2point_.Move_cmd);
    }
    point2point_.stop();
    point2point_.pub.publish(point2point_.Move_cmd);
    ros::shutdown();
  }
  return 0;
}