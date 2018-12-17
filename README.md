# turtlebot-Experiment

#task3_ws:
 Achieve turtlebot specified target point autonomous navigation, recording screen 10s
#Opencv_pro commond:
'''bash
#compile
cd task3_ws
catkin_make
source  devel/setup.bash 
#如果后边出现运行找不到包，编译时，单独编译该包
#launch 

roslaunch turtlebot_gazebo turtlebot_world.launch
roslaunch turtlebot_gazebo amcl_demo.launch map_file:=/home/ubuntu/task3_ws/src/OpenCVPrc/data/Map/home_mapv3.yaml
roslaunch turtlebot_rviz_launchers view_navigation.launch
#after launch rviz,init turtlebot pose. However,run commond in 下边

rosrun OpenCVPrc opencv_img

rosrun simple_navigation_goals simple_navigation_goals

'''
