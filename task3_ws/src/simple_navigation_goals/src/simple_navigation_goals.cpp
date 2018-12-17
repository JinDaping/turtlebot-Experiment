#include "std_msgs/String.h"
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <ros/ros.h>
#include <sstream>
using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
    MoveBaseClient;

int main(int argc, char **argv) {
  ros::init(argc, argv, "simple_navigation_goals");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<std_msgs::String>("goalflag", 1000);
  ros::Rate loop_rate(1000);
  std_msgs::String msg;
  std::stringstream ss;
  // tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // wait for the action server to come up
  while (!ac.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  // we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // goal.target_pose.pose.position.x = 1.0;
  goal.target_pose.pose.orientation.w = 1.0;

  cout << "Input: \n";
  float x_site;
  float y_site;

  cout << "请输入x坐标: ";
  cin >> x_site;
  cout << "请输入y坐标: ";
  cin >> y_site;
  ROS_INFO("Run");
  goal.target_pose.pose.position.x = x_site;
  goal.target_pose.pose.position.y = y_site;

  // goal.target_pose.pose.orientation.w = y_site;

  // ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  ac.waitForResult();

  if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    // ROS_INFO("Hooray, the base moved 1 meter forward");
    ss << "true";
    msg.data = ss.str();
    pub.publish(msg);
    ROS_INFO("Completed");
  } else
    ROS_INFO("Failed");
  // ROS_INFO("The base failed to move forward 1 meter for some reason");
  ros::spinOnce();
  loop_rate.sleep();
  return 0;
}
