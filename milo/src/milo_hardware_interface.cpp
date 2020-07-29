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

class Milo_hardware
{
public:
    Milo_hardware();
    // double max_l_speed,max_angular_speed,wheel_radius,wheel_distance;
private:
    ros::NodeHandle nh_;
    ros::Subscriber cmd_vel_sub_;
    ros::Subscriber button_sub_;
    void initMilo();
    



};



// Milo_hardware::Milo_hardware(): wheel_radius(1)
// {
//     nh_.param("milo_hardware_interface/wheel_radius", wheel_radius);

// }



int main(int argc, char **argv)
{
    double loop_hz,wheel_radius;

    ros::init(argc, argv, "milo_hardware_interface");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(2); // multi-threaded non-blocking spinning
    ros::Time current_time, last_time;
    ros::Duration elapsed_time;

    current_time = ros::Time::now();
    last_time = ros::Time::now();
    nh.param("/milo/hardware_interface/loop_hz", loop_hz, 10.0);
    nh.param("/milo_hardware_interface/wheel_radius",wheel_radius, 10.0);
    ros::Rate rate(loop_hz);
    spinner.start();

    while (nh.ok())
    {

        current_time = ros::Time::now();
        elapsed_time = current_time - last_time;
        last_time = current_time;
        rate.sleep();
        std::cout<<wheel_radius<<std::endl;
    }

    spinner.stop();
    return 0;
}
