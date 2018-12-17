#ifndef CARDRIVER_H_
#define CARDRIVER_H_

#include <ros/ros.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <boost/thread/condition_variable.hpp>
#include <limits>

#include <stdlib.h>
#include <sstream>
#include <string>

#include <iostream>

#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tcp2ros/readDataAll.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>

#define PULSE_PER_CYCLE 1524
#define PULSE_MEASURE_FREQ 50
#define DEG_TO_ANGLE   57.324
#define pai 3.1415926

using namespace std;
class CarDriver
{
public:
	CarDriver(ros::NodeHandle &nh,double a,double b,double p);

	int getUpdateFreq() {return  update_freq_;}

	~CarDriver()
	{

	}
private:
	ros::NodeHandle nh_;

	ros::Publisher odom_pub;
	ros::Subscriber tcp_sub;
	ros::Subscriber imu_sub;

	tf::TransformBroadcaster odom_broadcaster;

	bool init;
	double a,b;
	double cmd_[4];
	double data[];
	double vel_[2];

	double pose_x;
	double pose_y;
	double pose_th;

	double vx;
	double vy;
	double vth;

	double last_data[4];
	bool get_imu;
	double init_imu_yaw;
        double last_imu_th;
	double imu_yaw;

	ros::Time updated_;
	int update_freq_;
	double pulse_cycle;

	void carVelConvert();
	void readCallback(const tcp2ros::readDataAll& data);
	void imuCallback(const sensor_msgs::Imu& imu_);

	inline uint8_t sgn(double x)
	{
		if (x > 0)
		return 1;
		return 0;
	}
};

#endif /* TANK_HW_H_ */
