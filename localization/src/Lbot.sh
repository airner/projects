sudo killall gzserver
sudo killall gzclient
sudo killall rviz
sudo killall roscore
sudo killall rosmaster

sleep 30
echo "roslaunch li_bot li_world.launch"
roslaunch li_bot li_world.launch &
sleep 30
echo "roslaunch li_bot li_amcl.launch"
roslaunch li_bot li_amcl.launch &
sleep 30
echo "rosrun li_bot li_navigation_goal"
rosrun li_bot li_navigation_goal
