sudo xboxdrv --detach-kernel-driver & sudo bash -c "chmod a+rw /dev/input/js0"

sudo bash -c "source /opt/ros/kinetic/setup.bash && source ~/catkin_ws/devel/setup.bash && rosclean purge -y && roslaunch logcart_driver logcart_teleop.launch" & source ~/catkin_ws/devel/setup.bash & roslaunch freenect_launch freenect.launch & roslaunch logcart_driver logcart.sh & roslaunch logcart_mapping logcart_mapping.launch & roslaunch logcart_mapping pointcloudtolaserscan.launch
