sudo killall gzserver
sudo killall gzclient
sudo killall rviz
sudo killall roscore
sudo killall rosmaster

sleep 30 
echo "roslaunch udacity_bot udacity_world.launch"
roslaunch udacity_bot udacity_world.launch &
sleep 30
echo "roslaunch udacity_bot amcl.launch"
roslaunch udacity_bot amcl.launch &
sleep 32
echo "rosrun udacity_bot navigation_goal"
rosrun udacity_bot navigation_goal
