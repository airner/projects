#!/bin/bash

sudo killall gzserver
sudo killall gzclient
sudo killall rviz
sudo killall roscore
sudo killall rosmaster

sleep 10
echo "roslaunch turtlebot_gazebo turtlebot_world.launch "
roslaunch turtlebot_gazebo turtlebot_world.launch &
sleep 20
echo "roslaunch robot_pose_ekf robot_pose_ekf.launch" 
roslaunch robot_pose_ekf robot_pose_ekf.launch &
sleep 20
echo "roslaunch odom_to_trajectory create_trajectory.launch" 
roslaunch odom_to_trajectory create_trajectory.launch &
sleep 20
echo "roslaunch turtlebot_teleop keyboard_teleop.launch" 
roslaunch turtlebot_teleop keyboard_teleop.launch &
sleep 20
echo "rosrun rviz rviz -d /home/robond/catkin_ws/src/EKFLab.rviz"
rosrun rviz rviz -d /home/robond/catkin_ws/src/EKFLab.rviz

