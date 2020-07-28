#include <serial/serial.h>

// #define RASPBERRY_PI

#ifdef RASPBERRY_PI 
    #include <wiringPi.h>
#endif

#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/String.h>
#include <tf/tf.h>

#include <algorithm>
#include <cstdlib>
#include <cmath>
#include <string>
#include <vector>
#include <regex>


void initMilo()
{
std::cout<<"YES THIS IS BEING INITIALISED"<<std::endl;
}

int main(int argc, char **argv)
{
    double loop_hz;

    ros::init(argc, argv, "milo_hardware_interface");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(2); // multi-threaded non-blocking spinning
    ros::Time current_time, last_time;
    ros::Duration elapsed_time;

    current_time = ros::Time::now();
    last_time = ros::Time::now();
    initMilo();
    nh.param("/milo/hardware_interface/loop_hz", loop_hz, 10.0);
    ros::Rate rate(loop_hz);
    spinner.start();

    while (nh.ok())
    {

        current_time = ros::Time::now();
        elapsed_time = current_time - last_time;
        last_time = current_time;
        rate.sleep();
        std::cout<<"Loop Time"<<elapsed_time<<std::endl;
    }

    spinner.stop();
    return 0;
}
