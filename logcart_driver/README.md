## Steps to compile source code and run the program:
$ cd ~/catkin_ws;
$ bash devel/setup.bash;
$ catkin_make -DCATKIN_WHITELIST_PACKAGES=”logcart_driver”;
$  find /dev/input/js0 – exec sudo chmod a+rw {} \;
$  echo password | sudo -S bash roslaunch logcart_driver logcart_teleop.launch

## Comments:
- These callbacks are currently not used in the application: 
  velocityRecvCallback, odmCallback
- This warning about missing mutex in the constructor is waived (also exists in Folo):
  "Warning: updateConfig() called on a dynamic_reconfigure.."

