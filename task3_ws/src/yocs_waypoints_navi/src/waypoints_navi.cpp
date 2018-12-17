/**
 * License: BSD
 *   https://raw.github.com/yujinrobot/yujin_ocs/license/LICENSE
 */
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include "yocs_waypoints_navi/waypoints_navi.hpp"
#include <nav_msgs/Odometry.h>

// #include "yocs_waypoints_navi/carDriver.h"
// #include "yocs_waypoints_navi/readDataAll.h"

/*
 * TODO
 *  * think about how to best visualise the waypoint(s)/trajectory(ies) which are being executed
 *  * add RViz interface to yocs_waypoint_provider
 */

namespace yocs
{

WaypointsGoalNode::WaypointsGoalNode() : mode_(NONE),
                                         state_(IDLE),   //空闲
                                         frequency_(5), // 5 hz
                                         close_enough_(0.1), // 10 cm
                                         goal_timeout_(60.0), // 60 sec
                                         idle_status_update_sent_(false),
                                         move_base_ac_("move_base", true),
                                         mode(0),tank_id_(0),distance_alignment_(0.0),current_goal(0),target_path("path")
{}

WaypointsGoalNode::~WaypointsGoalNode()
{}

bool WaypointsGoalNode::init()
{
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  pnh.param("frequency",      frequency_,     1.0);
  pnh.param("close_enough",   close_enough_,  0.3);  // close enough to next waypoint　　足够接近下一个航路点
  pnh.param("goal_timeout",   goal_timeout_, 30.0);  // maximum time to reach a waypoint
  pnh.param("robot_frame",    robot_frame_,    std::string("/base_footprint"));
  pnh.param("world_frame",    world_frame_,    std::string("/map"));

  // reset goal way points
  waypoints_.clear();
  waypoints_it_ = waypoints_.end();

  while ((move_base_ac_.waitForServer(ros::Duration(1.0)) == false) && (ros::ok() == true))
  {
    ROS_WARN_THROTTLE(1, "Waiting for move_base action server to come up...");
  }
  waypoints_sub_  = nh.subscribe("waypoints",  1, &WaypointsGoalNode::waypointsCB, this);
  trajectories_sub_  = nh.subscribe("trajectories",  1, &WaypointsGoalNode::trajectoriesCB, this);
  tcp_sub_ = nh.subscribe("mode_distance_tank",10,&WaypointsGoalNode::cbOdom,this);
  
  
  
//   nav_ctrl_sub_  = nh.subscribe("nav_ctrl", 1, &WaypointsGoalNode::navCtrlCB, this);
  status_pub_  = nh.advertise<yocs_msgs::NavigationControlStatus>("nav_ctrl_status", 1, true);


  
  // zwy
  status_goal_pub=nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("status_goal", 10);

  // zwy
  
  
  return true;
}

// zwy
void WaypointsGoalNode::cbOdom(const nav_msgs::Odometry::ConstPtr &msg)
{
    
   mode = int(msg->pose.pose.position.x);
//    std::cout<<"reached!!"<<mode<<std::endl;
   tank_id_ = int(msg->pose.pose.position.y);
   distance_alignment_ =  msg->pose.pose.position.z;
   
    if(tank_id_ == 0)
   {
     target_path = "path";
  } else if(tank_id_ == 1)
   {
     target_path = "path1";
  } else  if(tank_id_ == 2)
  {
     target_path = "path2";
  }
  else if (tank_id_ == 3) //1 to 2
    target_path = "path12";
  else if(tank_id_ ==4)  //2 to 1
   {
   target_path = "path21";
 }
 else if(tank_id_==5)   // 1 back home
 {
   target_path = "back1 ";
}
 else if(tank_id_==6) //2 back home
 {
   target_path = "back2";
}
}

//zwy

void WaypointsGoalNode::waypointsCB(const yocs_msgs::WaypointList::ConstPtr& wps)
{
  wp_list_ = *wps;
  ROS_INFO_STREAM("Received " << wp_list_.waypoints.size() << " way points.");
}

void WaypointsGoalNode::trajectoriesCB(const yocs_msgs::TrajectoryList::ConstPtr& trajs)
{
  traj_list_ = *trajs;
  ROS_INFO_STREAM("Received " << traj_list_.trajectories.size() << " trajectories.");
}



bool WaypointsGoalNode::cancelAllGoals(double timeout)
{
  actionlib::SimpleClientGoalState goal_state = move_base_ac_.getState();
  if ((goal_state != actionlib::SimpleClientGoalState::ACTIVE) &&
      (goal_state != actionlib::SimpleClientGoalState::PENDING) &&
      (goal_state != actionlib::SimpleClientGoalState::RECALLED) &&
      (goal_state != actionlib::SimpleClientGoalState::PREEMPTED))
  {
    // We cannot cancel a REJECTED, ABORTED, SUCCEEDED or LOST goal
    ROS_WARN("Cannot cancel move base goal, as it has %s state!", goal_state.toString().c_str());
    publishStatusUpdate(yocs_msgs::NavigationControlStatus::ERROR);
    return true;
  }

  ROS_INFO("Canceling move base goal with %s state...", goal_state.toString().c_str());
  move_base_ac_.cancelAllGoals();
  if (move_base_ac_.waitForResult(ros::Duration(timeout)) == false)
  {
    ROS_WARN("Cancel move base goal didn't finish after %.2f seconds: %s",
             timeout, goal_state.toString().c_str());
    publishStatusUpdate(yocs_msgs::NavigationControlStatus::ERROR);
    return false;
  }

  ROS_INFO("Cancel move base goal succeed. New state is %s", goal_state.toString().c_str());
  publishStatusUpdate(yocs_msgs::NavigationControlStatus::CANCELLED);
  return true;
}

void WaypointsGoalNode::resetWaypoints()
{
  ROS_DEBUG("Full reset: clear markers, delete waypoints and goal and set state to IDLE");
  waypoints_.clear();
  waypoints_it_ = waypoints_.end();
  goal_  = NOWHERE;
  mode_  = NONE;
}

void WaypointsGoalNode::spin()
{
  move_base_msgs::MoveBaseGoal mb_goal;
  
  
  ros::Rate rate(frequency_);

  while (ros::ok())
  {
    rate.sleep();
    ros::spinOnce();
    
  
 std::cout<<"mode: "<<mode<<std::endl<<std::endl;
  std::cout<<"target_path:  "<<target_path<<std::endl<<std::endl;


  /////////       ｚｗｙ   
//        std::cout<<"mode"<<mode<<std::endl;
//       　mode =0;
  
    if ((state_ == IDLE) || (state_ == COMPLETED))
    {
      resetWaypoints();
      // If provided goal is among the way points or trajectories, add the way point(s) to the goal way point list
      bool goal_found = false;

      if (!goal_found)
      {
        for (unsigned int traj = 0; traj < traj_list_.trajectories.size(); ++traj)
        {
          if (target_path == traj_list_.trajectories[traj].name)
          {
//                target_path = "path";
            for (unsigned int wp = 0; wp < traj_list_.trajectories[traj].waypoints.size(); ++wp)
            {
		  geometry_msgs::PoseStamped pose;
	      if(wp !=( traj_list_.trajectories[traj].waypoints.size()-1))
	      {
		  pose.header = traj_list_.trajectories[traj].waypoints[wp].header;
		  pose.pose = traj_list_.trajectories[traj].waypoints[wp].pose;
	      }
	      else {
		  pose.header = traj_list_.trajectories[traj].waypoints[wp].header;
		  pose.pose.position.x = traj_list_.trajectories[traj].waypoints[wp].pose.position.x + distance_alignment_;
		  pose.pose.position.y = traj_list_.trajectories[traj].waypoints[wp].pose.position.y;
		  pose.pose.position.z = traj_list_.trajectories[traj].waypoints[wp].pose.position.z;
		  pose.pose.orientation = traj_list_.trajectories[traj].waypoints[wp].pose.orientation;
	      }
              waypoints_.push_back(pose);
            }
            
            waypoints_it_ = waypoints_.begin();
            goal_found = true;
            ROS_INFO_STREAM("Prepared to navigate along the trajectory '" << target_path << "'.");
            ROS_INFO_STREAM("# of way points = " << waypoints_.size());
//             target_path ="path";
          }
        }
      
  
      if (goal_found) 		//	改成mode ==1试一试
      {
	 
	        state_ =  START;
                mode_  =  GOAL;
	    
//            goal_found = false;
      }
      else
      {
        ROS_WARN_STREAM("Could not find provided way point or trajectory.");
      }
    }
    else
    {
      ROS_WARN_STREAM("Cannot start way point/trajectory execution, because navigator is currently active. "
                      << "Please stop current activity first.");
    }
  }
  
///////zwy   
 
    if (state_ == START)
    {


      if (waypoints_it_ < waypoints_.end())
      {
        mb_goal.target_pose.header.stamp = ros::Time::now();
        mb_goal.target_pose.header.frame_id = waypoints_it_->header.frame_id;
        mb_goal.target_pose.pose = waypoints_it_->pose;
//        mb_goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);  // TODO use the heading from robot loc to next (front)

        ROS_INFO("New goal: %.2f, %.2f, %.2f",
                 mb_goal.target_pose.pose.position.x, mb_goal.target_pose.pose.position.y,
                 tf::getYaw(mb_goal.target_pose.pose.orientation));
        move_base_ac_.sendGoal(mb_goal);

        publishStatusUpdate(yocs_msgs::NavigationControlStatus::RUNNING);

        state_ = ACTIVE;
      }
      else
      {
        ROS_ERROR_STREAM("Cannot start execution. Already at the last way point.");
        idle_status_update_sent_ = false;
        state_ = IDLE;
      }

      // TODO: This is a horrible workaround for a problem I cannot solve: send a new goal
      // when the previous one has been cancelled return immediately with succeeded state
      //
      // Marcus: Don't understand this case (yet). Commenting out until we need it.
//        int times_sent = 0;
//        while ((move_base_ac_.waitForResult(ros::Duration(0.1)) == true) &&
//               (move_base_ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED))
//        {
//          move_base_ac_.sendGoal(mb_goal);
//          times_sent++;
//        }
//        if (times_sent > 1)
//        {
//          ROS_WARN("Again the strange case of instantaneous goals... (goal sent %d times)", times_sent);
//        }
    }
    else if (state_ == ACTIVE)
    {
      actionlib::SimpleClientGoalState goal_state = move_base_ac_.getState();

      // We are still pursuing a goal...　　　我们仍在追求目标..
      if ((goal_state == actionlib::SimpleClientGoalState::ACTIVE) ||
          (goal_state == actionlib::SimpleClientGoalState::PENDING) ||                     //有待
          (goal_state == actionlib::SimpleClientGoalState::RECALLED) ||                   // 回顾
          (goal_state == actionlib::SimpleClientGoalState::PREEMPTED))                 //抢占
      {
        // check if we timed out
        if ((ros::Time::now() - mb_goal.target_pose.header.stamp).toSec() >= goal_timeout_)
        {
          ROS_WARN("Cannot reach goal after %.2f seconds; request a new one (current state is %s)",
                    goal_timeout_, move_base_ac_.getState().toString().c_str());
          if (waypoints_it_ < (waypoints_.end() - 1))
          {
            ROS_INFO_STREAM("Requesting next way point.");
            waypoints_it_++;
            state_ = START;
          }
          else
          {
            ROS_INFO_STREAM("No more way points to go to.");
            state_ = COMPLETED;
          }
        }
        // When close enough to current goal (except for the final one!), go for the
        // next waypoint, so we avoid the final slow approach and subgoal obsession
        if (waypoints_it_ < (waypoints_.end() - 1))
        {
          tf::StampedTransform robot_gb, goal_gb;
          try
          {
            tf_listener_.lookupTransform(world_frame_, robot_frame_, ros::Time(0.0), robot_gb);
          }
          catch (tf::TransformException& e)
          {
            ROS_WARN("Cannot get tf %s -> %s: %s", world_frame_.c_str(), robot_frame_.c_str(), e.what());
            continue;
          }

          mtk::pose2tf(mb_goal.target_pose, goal_gb);
          double distance = mtk::distance2D(robot_gb, goal_gb);
          if (distance <= close_enough_)
          {
            waypoints_it_++;
            state_ = START;
            ROS_INFO("Close enough to current goal (%.2f <= %.2f m).", distance, close_enough_);
            ROS_INFO_STREAM("Requesting next way point.");
          }
          else
          {
            // keep going until get close enough
          }
        }
        else
        {
          // keep going, since we approaching last way point
        }
      }
      else // actionlib::SimpleClientGoalState::SUCCEEDED, REJECTED, ABORTED, LOST
      {
        if (goal_state == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
          ROS_INFO("Go to goal successfully completed: %.2f, %.2f, %.2f",
                   mb_goal.target_pose.pose.position.x, mb_goal.target_pose.pose.position.y,
                   tf::getYaw(mb_goal.target_pose.pose.orientation));
          if (waypoints_it_ < (waypoints_.end() - 1))
          {
            ROS_INFO_STREAM("Requesting next way point.");
            waypoints_it_++;
            state_ = START;
          }
          else
          {
            ROS_INFO_STREAM("Reached final way point.");
            state_ = COMPLETED;
          }
        }
        else
        {
          ROS_ERROR("Go to goal failed: %s.", move_base_ac_.getState().toString().c_str());
          if (waypoints_it_ < (waypoints_.end() - 1))
          {
            ROS_INFO_STREAM("Requesting next way point.");
            waypoints_it_++;
            state_ = START;
          }
          else
          {
            ROS_INFO_STREAM("No more way points to go to.");
            state_ = COMPLETED;
          }
        }
      }
    }
    else if(state_ == COMPLETED)
    {
      // publish update
      publishStatusUpdate(yocs_msgs::NavigationControlStatus::COMPLETED);
      idle_status_update_sent_ = false;
      state_ = IDLE;
    }
    else // IDLE
    {
      if (!idle_status_update_sent_)
      {
        publishStatusUpdate(yocs_msgs::NavigationControlStatus::IDLING);
        idle_status_update_sent_ = true;
      }
    }
      
  }
}

bool WaypointsGoalNode::equals(const geometry_msgs::PoseStamped& a, const geometry_msgs::PoseStamped& b)
{
  return ((a.pose.position.x == b.pose.position.x) &&
          (a.pose.position.y == b.pose.position.y) &&
          (a.pose.position.z == b.pose.position.z));
  // TODO make decent, with rotation (tk::minAngle, I think) and frame_id and put in math toolkit
}

bool WaypointsGoalNode::equals(const geometry_msgs::Point& a, const geometry_msgs::Point& b)
{
  return ((a.x == b.x) && (a.y == b.y) && (a.z == b.z));
}

void WaypointsGoalNode::publishStatusUpdate(const uint8_t& status)
{
  
  
//   nav_msgs::Odometry goal_state;
  geometry_msgs::PoseWithCovarianceStamped  goal_state;
  yocs_msgs::NavigationControlStatus msg;
  
  
  
  if (status == yocs_msgs::NavigationControlStatus::IDLING)
  {
    msg.status = yocs_msgs::NavigationControlStatus::IDLING;
    msg.status_desc = "Idling";
    status_pub_.publish(msg);
  }
  else if (status == yocs_msgs::NavigationControlStatus::RUNNING)
  {
    msg.status = yocs_msgs::NavigationControlStatus::RUNNING;
    msg.status_desc = "Navigating to way point.";
    status_pub_.publish(msg);
  }
  else if (status == yocs_msgs::NavigationControlStatus::PAUSED)
  {
    msg.status = yocs_msgs::NavigationControlStatus::PAUSED;
    msg.status_desc = "Navigation on hold.";
    status_pub_.publish(msg);
  }
  else if (status == yocs_msgs::NavigationControlStatus::COMPLETED)
  {
    msg.status = yocs_msgs::NavigationControlStatus::COMPLETED;
    msg.status_desc = "Reached final destination.";
    status_pub_.publish(msg);
    
    goal_state.pose.pose.position.x=1;
    status_goal_pub.publish(goal_state);
  }
  else if (status == yocs_msgs::NavigationControlStatus::CANCELLED)
  {
    msg.status = yocs_msgs::NavigationControlStatus::CANCELLED;
    msg.status_desc = "Navigation cancelled.";
    status_pub_.publish(msg);
  }
  else if (status == yocs_msgs::NavigationControlStatus::ERROR)
  {
    msg.status = yocs_msgs::NavigationControlStatus::ERROR;
    msg.status_desc = "An error occurred.";
    status_pub_.publish(msg);
  }
  else
  {
    ROS_ERROR_STREAM("Cannot publish unknown status updated!");
  }
}

} // namespace yocs

