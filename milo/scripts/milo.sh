#!/usr/bin/env bash
sudo xboxdrv --detach-kernel-driver --silent & sudo bash -c "chmod a+rw /dev/input/js0"

sudo bash -c "source /opt/ros/kinetic/setup.bash && source ~/catkin_ws/devel/setup.bash && rosclean purge -y && roslaunch milo milo_teleop.launch"
