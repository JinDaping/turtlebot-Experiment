#include "std_msgs/String.h"
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <time.h>

using namespace std;

class OpencvImg {
public:
  OpencvImg() : flag(false), goal_flag(false) {
    sub_ = n_.subscribe("/camera/rgb/image_raw", 1000,
                        &OpencvImg::image_receive, this);
     sub1_ = n_.subscribe("goalflag", 1000, &OpencvImg::flag_receive, this);
    

    videowriter = cv::VideoWriter(
        "/home/ubuntu/task3_ws/src/OpenCVPrc/data/kinect.avi",
        CV_FOURCC('M', 'J', 'P', 'G'), 25.0, cv::Size(640, 480));
    if (!videowriter.isOpened()) {
      cout << "The address of VideoWriter is error!" << endl;
      videowriter.release();
    }
  }
  void flag_receive(const std_msgs::String::ConstPtr &msg) {
    flag = true;
    ROS_INFO("I heard: [%s]", msg->data.c_str());
  }
  void image_receive(const sensor_msgs::ImageConstPtr &msg) {
    cv_bridge::CvImageConstPtr cv_ptr =
        cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
    if (flag) {
       cv::imwrite("/home/ubuntu/task3_ws/src/OpenCVPrc/data/frame1.jpg",
                   cv_ptr->image);
      flag = false;
      goal_flag = true;
      previous_time = time(&tv);
      current_time = time(&tv);     
    }
    if (goal_flag && current_time - previous_time < 14) {
      videowriter << cv_ptr->image;
      current_time = time(&tv);
      cv::imshow("image", cv_ptr->image);
    }
    else{
      goal_flag = false;
    }
     

    cv::waitKey(1);
  }

public:
  bool flag;

private:
  ros::NodeHandle n_;
  ros::Subscriber sub_;
  ros::Subscriber sub1_;
  cv::VideoWriter videowriter;

  bool goal_flag;
  time_t tv;
  time_t previous_time;
  time_t current_time;
};

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "opencv_img_node");
  OpencvImg opencv_img_;
   ros::spin();
  return 0;
}
