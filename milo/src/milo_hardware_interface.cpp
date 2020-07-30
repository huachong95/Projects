#include <serial/serial.h>
#include <milo/Joystick.h>

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

class milo_hardware
{

private:
    void velocityRecvCallback(const geometry_msgs::Twist::ConstPtr &cmd);
    void buttonsRecvCallback(const milo::Joystick::ConstPtr &button);
    double saturate(double &value, double min, double max);
    ros::NodeHandle nh;
    ros::Subscriber cmd_vel_sub;
    ros::Subscriber button_sub;
    double wheel_distance, wheel_radius, max_l_speed,
        min_l_speed, max_ang_speed, min_ang_speed; 
    int baud_rate, log_level;
    std::string port_mbed;
    serial::Serial serial;
    bool isSerialPortDown;
    const double PI = 3.14159265358979323846;
    const double SEC_PER_MIN = 60.0;

public:
    milo_hardware(ros::NodeHandle &nh);

    double left_rpm, right_rpm,loop_hz;

    void Test();
    void initSerialPort();
    void readSerialPort();
    void writeSerialPort();
};

milo_hardware::milo_hardware(ros::NodeHandle &nh) : nh(nh), isSerialPortDown(false)
{
    nh.getParam("/log_level", log_level);
    nh.getParam("/milo_hardware_interface/wheel_distance", wheel_distance);
    nh.getParam("/milo_hardware_interface/wheel_radius", wheel_radius);
    nh.getParam("/milo_hardware_interface/max_l_speed", max_l_speed);
    nh.getParam("/milo_hardware_interface/min_l_speed", min_l_speed);
    nh.getParam("/milo_hardware_interface/max_ang_speed", max_ang_speed);
    nh.getParam("/milo_hardware_interface/min_ang_speed", min_ang_speed);
    nh.getParam("/milo_hardware_interface/port_mbed", port_mbed);
    nh.getParam("/milo_hardware_interface/baud_rate", baud_rate);
    nh.getParam("/milo_hardware_interface/loop_hz", loop_hz);

    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Level(log_level));
    std::cout<<"log_level"<<log_level<<std::endl;
    initSerialPort();
 

    cmd_vel_sub = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 100, &milo_hardware::velocityRecvCallback, this);
    button_sub = nh.subscribe<milo::Joystick>("/joy_buttons", 100, &milo_hardware::buttonsRecvCallback, this);
}
void milo_hardware::velocityRecvCallback(const geometry_msgs::Twist::ConstPtr &cmd)
{
    double linear, angular, left_mps, right_mps;
    linear = cmd->linear.y;
    angular = cmd->angular.x;
    if (linear >= 0)
    {
        linear = linear * max_l_speed;
    }
    else
    {
        linear = linear * min_l_speed;
    }

    if (angular >= 0)
    {
        angular = angular * max_ang_speed;
    }
    else
    {
        angular = angular * min_ang_speed;
    }

    linear = saturate(linear, min_l_speed, max_l_speed);
    angular = saturate(angular, min_ang_speed, max_ang_speed);

    left_mps = linear - angular * wheel_distance / 2.0f;
    right_mps = linear + angular * wheel_distance / 2.0f;

    left_rpm = left_mps * SEC_PER_MIN / (2.0f * PI * wheel_radius);
    right_rpm = right_mps * SEC_PER_MIN / (2.0f * PI * wheel_radius);
}

void milo_hardware::buttonsRecvCallback(const milo::Joystick::ConstPtr &button)
{
}

void milo_hardware::readSerialPort()
{
}

void milo_hardware::writeSerialPort()
{
    try
    {
        if (!isSerialPortDown)
        {
            // Stores string from the read buffer into tempStrBuff delimited by "enter"
            std::string tempStrBuff = serial.readline(1024, "\r");
        }
    }
    catch (const std::exception &e)
    {
        ROS_ERROR("Serial Port has crashed");
        serial.close();
        isSerialPortDown = true;
    }
}
double milo_hardware::saturate(double &value, double min, double max)
{
    if (value < min)
    {
        value = min;
    }
    else if (value > max)
    {
        value = max;
    }
}

void milo_hardware::initSerialPort()
{
    try
    {
        serial.setPort(port_mbed);
        serial.setBaudrate(baud_rate);
        serial::Timeout tout = serial::Timeout::simpleTimeout(10); // Millisecs
        serial.setTimeout(tout);
        serial.open();

        isSerialPortDown = false;

        /* Flush buffer */
        serial.readline();
    }
    catch (serial::IOException &e)
    {
        ROS_ERROR("Unable to open port %s", port_mbed.c_str());

        isSerialPortDown = true;
    }
}

void milo_hardware::Test()
{
    std::cout << "HI" << std::endl;
}
int main(int argc, char **argv)
{
    double loop_hz;
    ros::init(argc, argv, "milo_hardware_interface");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(2); // multi-threaded non-blocking spinning
    ros::Time current_time, last_time;
    ros::Duration elapsed_time;

    milo_hardware milo(nh); //Creating the object : milo

    current_time = ros::Time::now();
    last_time = ros::Time::now();
    ros::Rate rate(milo.loop_hz);
    spinner.start();

    while (nh.ok())
    {

        current_time = ros::Time::now();
        elapsed_time = current_time - last_time;
        last_time = current_time;
        rate.sleep();
        // std::cout << milo.left_rpm << std::endl;
    }

    spinner.stop();
    return 0;
}
