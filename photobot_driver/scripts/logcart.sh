#!/usr/bin/env bash
sudo bash -c "chmod a+rw /dev/input/js1"
sudo bash -c "source /opt/ros/kinetic/setup.bash && source /home/huachong/catkin_ws/devel/setup.bash && rosclean purge -y && roslaunch logcart_driver logcart_teleop.launch"
