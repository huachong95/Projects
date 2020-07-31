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
    bool isValidNumber(std::string str);
    void processSerialMsg(std::string &tempStrBuff);

    ros::NodeHandle nh;
    ros::Subscriber cmd_vel_sub;
    ros::Subscriber button_sub;
    double wheel_distance, wheel_radius, max_l_speed,
        min_l_speed, max_ang_speed, min_ang_speed;
    double left_rpm, right_rpm;
    int log_level;
    const double PI = 3.14159265358979323846;
    const double SEC_PER_MIN = 60.0;
    std::string port_mbed;
    serial::Serial serial;
    bool isSerialPortDown;

    const int FIELDS_IN_SERIAL_MSG = 4;

    int baud_rate;

public:
    milo_hardware(ros::NodeHandle &nh);

    double loop_hz;
    void initSerialPort();
    void readSerialPort();
    void writeSerialPort();
    void checkSerialPort();
};

// Constructor for milo_hardware class
milo_hardware::milo_hardware(ros::NodeHandle &nh) : nh(nh)
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
    std::cout << "log_level" << log_level << std::endl;
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
    try
    {
        if (!isSerialPortDown)
        {
            // Stores string from the read buffer into tempStrBuff delimited by "enter"
            std::string tempStrBuff = serial.readline(1024, "\r");
            if (tempStrBuff.length() > 0)
            {
                processSerialMsg(tempStrBuff);
            }
        }
    }
    catch (const std::exception &e)
    {
        serial.close();
        isSerialPortDown = true;
    }
}

void milo_hardware::writeSerialPort()
{
    try
    {
        if (!isSerialPortDown)
        {
            std::string rpm_l = std::to_string((int)(left_rpm));
            std::string rpm_r = std::to_string((int)(right_rpm));

            std::string serialTxMsg = "?S," + rpm_l + "_" + rpm_r + ",/r";
            serial.write(serialTxMsg);
            ROS_DEBUG("SerialTxMsg: %s", serialTxMsg.c_str());
        }
    }
    catch (const std::exception &e)
    {
        serial.close();
        isSerialPortDown = true;
    }
}
void milo_hardware::checkSerialPort()
{
    if (isSerialPortDown)
    {
        initSerialPort();
        isSerialPortDown = false; //Reinitialised serial port, reset flag
        ROS_ERROR("Serial Port has crashed, re-initialising");
    }
    else
    {
    }
}

void milo_hardware::processSerialMsg(std::string &serialMsg)
{
    if (serialMsg.find("\r") != std::string::npos)
    {
        int indexMsgEnd = serialMsg.find("\r");
        ROS_DEBUG("indexMsgEnd position: %d", indexMsgEnd);

        if (std::count(serialMsg.begin(), serialMsg.end(), '_') == (FIELDS_IN_SERIAL_MSG - 1))
        {
            char buff[1024];
            std::strcpy(buff, serialMsg.c_str());
            char *header = strtok(buff, ",");  //Expects header "?H"
            char *payload = strtok(NULL, ","); //Expects <payload information>
            char *footer = strtok(NULL, ",");
            ROS_DEBUG("Header: %s, Payload %s, Footer %s", header, payload, footer);

            char *strSpeed1 = strtok(payload, "_");
            char *strSpeed2 = strtok(NULL, "_");
            char *strEncoder1 = strtok(NULL, "_");
            char *strEncoder2 = strtok(NULL, "_");
            ROS_DEBUG("Payload Strings: %s, %s, %s, %s", strSpeed1, strSpeed2, strEncoder1, strEncoder2);

            int speed1 = atoi((isValidNumber(strSpeed1) ? strSpeed1 : "0"));
            int speed2 = atoi((isValidNumber(strSpeed2) ? strSpeed2 : "0"));
            long encoder1 = atol((isValidNumber(strEncoder1) ? strEncoder1 : "0"));
            long encoder2 = atol((isValidNumber(strEncoder2) ? strEncoder2 : "0"));

            ROS_DEBUG("Payload Values: %d, %d, %ld, %ld", speed1, speed2, encoder1, encoder2);
        }
    }
    else
    {
        ROS_WARN("last serial msg index is not \r");
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

bool milo_hardware::isValidNumber(std::string str)
{
    bool isNum = true;
    for (int i = 0; i < str.length(); i++)
    {
        if (isdigit(str[i]) == false && str[i] != '-')
        {
            isNum = false;
            ROS_ERROR("Invalid number detected: %s", str.c_str());
            break;
        }
    }
    return isNum;
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

        milo.readSerialPort();
        milo.checkSerialPort();
        milo.writeSerialPort();

        rate.sleep();
    }

    spinner.stop();
    return 0;
}
