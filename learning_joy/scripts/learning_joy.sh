source /catkin_ws/devel/setup.bash 
 sudo xboxdrv --silent & sudo chmod a+rw /dev/input/js1 & rosparam set joy_node/dev "/dev/input/js1" & roslaunch learning_joy turtle_joy.launch
