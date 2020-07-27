#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>
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

class LogcartHardware : public hardware_interface::RobotHW
{
private:
   
    const double PI = 3.14159265358979323846;
    const double WHEEL_SHAFT_RATIO = 32.5;
    const double ENC_PULSE_PER_REV = 748;
    const double ENC_COUNT_PER_REV = ENC_PULSE_PER_REV * 1;
    const double SEC_PER_MIN = 60.0;

    hardware_interface::JointStateInterface jnt_state_interface;
    hardware_interface::VelocityJointInterface vel_jnt_interface;
    ros::NodeHandle nh;
    ros::Subscriber cmd_sub, odom_sub;
    ros::Publisher pose2d_pub, battery_pub;

    // DEBUG = 1, INFO = 2, WARN = 4, ERROR = 8, FATAL = 16
    int debug_log_level = 0;

    double cmd[2] = {0, 0};
    double pos[2] = {0, 0};
    double vel[2] = {0, 0};
    double eff[2] = {0, 0};

    int baud_rate;
    std::string port_mbed;
    serial::Serial serial;
    bool isSerialPortDown;

    std::string serialStrBuffer;

    double wheel_base;
    double wheel_radius;
    double maxLinearVelocity, minLinearVelocity;
    double maxAngularVelocity, minAngularVelocity;

    std_msgs::String battery_level;

    void velocityRecvCallback(const geometry_msgs::Twist::ConstPtr &twist);
    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);

public:
    LogcartHardware(ros::NodeHandle &nh);
    ~LogcartHardware();

    void read();
    void write();
    void initSerialPort();
    void checkSerialPort();
    void processHeartbeatMessage(std::string &serialStr, int indexHeader);
    void processBattVoltageMessage(std::string &serialStr, int indexHeader);
    bool isValidNumber(std::string str);
};

const std::string HEARTBEAT_HEADER = "?H";
const std::string BATT_VOLTAGE_HEADER = "?V";

const int FIELDS_IN_HEARTBEAT_MSG = 8;
const int FIELDS_IN_BATT_VOLTAGE_MSG = 1;

LogcartHardware::LogcartHardware(ros::NodeHandle &nh)
    : nh(nh), baud_rate(115200), isSerialPortDown(false), serialStrBuffer("")
{

    // connect and register the joint state interface
    hardware_interface::JointStateHandle state_handle_left("left_wheel_joint", &pos[0], &vel[0], &eff[0]);
    jnt_state_interface.registerHandle(state_handle_left);

    hardware_interface::JointStateHandle state_handle_right("right_wheel_joint", &pos[1], &vel[1], &eff[1]);
    jnt_state_interface.registerHandle(state_handle_right);

    registerInterface(&jnt_state_interface);

    // connect and register the velocity joint interface
    hardware_interface::JointHandle vel_handle_left(state_handle_left, &cmd[0]);
    vel_jnt_interface.registerHandle(vel_handle_left);

    hardware_interface::JointHandle vel_handle_right(state_handle_right, &cmd[1]);
    vel_jnt_interface.registerHandle(vel_handle_right);

    registerInterface(&vel_jnt_interface);

    #ifdef RASPBERRY_PI 
    wiringPiSetup(); // for RPi GPIO control
    #endif

    ros::param::get("log_level", debug_log_level);

    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Level(debug_log_level));

    ros::param::get("~baud_rate", baud_rate); // private parameter
    ros::param::get("~port_mbed", port_mbed); // private parameter
    initSerialPort();

    nh.getParam("/base_controller/wheel_separation", wheel_base);
    nh.getParam("/base_controller/wheel_radius", wheel_radius);
    nh.getParam("/base_controller/linear/x/max_velocity", maxLinearVelocity);
    nh.getParam("/base_controller/linear/x/min_velocity", minLinearVelocity);
    nh.getParam("/base_controller/angular/z/max_velocity", maxAngularVelocity);
    nh.getParam("/base_controller/angular/z/min_velocity", minAngularVelocity);

    // cmd_vel is published by teleop_joy.cpp
    cmd_sub = nh.subscribe<geometry_msgs::Twist>("/base_controller/cmd_vel", 100, &LogcartHardware::velocityRecvCallback, this);
    odom_sub = nh.subscribe<nav_msgs::Odometry>("/base_controller/odom", 100, &LogcartHardware::odomCallback, this);

    pose2d_pub = nh.advertise<geometry_msgs::Pose2D>("/pose2d", 1000);
    battery_pub = nh.advertise<std_msgs::String>("/battery_state", 1000);
}

LogcartHardware::~LogcartHardware()
{
}

// Assign pos and vel variables based on joint state (vel in rad/s, pos in rad)
// From Folo (hardware.cpp)
void LogcartHardware::read()
{
    try
    {
        if (!isSerialPortDown)
        {
            // Store string from read buffer into buffer, delimited by "enter" or timeout, see serial::setTimeout()
            std::string tempStrBuff = serial.readline(1024, "\r");
            ROS_DEBUG("%s", tempStrBuff.c_str());

            /* Append the received serial string to the back of the existing string */
            std::replace(tempStrBuff.begin(), tempStrBuff.end(), '\r', '+');
            serialStrBuffer.append(tempStrBuff);
            // ROS_DEBUG("serialStrBuffer: %s", serialStrBuffer.c_str());

            /* NOTE: Assuming that each message received is always complete before another one is received */
            /* Serial read buffer string (i.e. "?H,XX_XX_XX_XX_XX_XX_XX_XX,\r" or "?V,40,\r") */

            /* If serial string buffer is not empty, search for valid headers */
            if(serialStrBuffer.length() > 0)
            {
                /* Find number of headers in serial string buffer */
                int numOfHeaders = std::count(serialStrBuffer.begin(), serialStrBuffer.end(), '?');

                ROS_DEBUG("Num of headers found: %d", numOfHeaders);

                /* Loop for number of headers found */
                for(int i=0; i<numOfHeaders; i++)
                {
                    bool isHeartBeatPresent = false, isBattVoltagePresent = false;
                    int indexHeartBeat = 9999, indexBattVoltage = 9999;

                    /* Check if heartbeat header is found (not equal to string::npos) */
                    if(serialStrBuffer.find(HEARTBEAT_HEADER) != std::string::npos)
                    {
                        isHeartBeatPresent = true;
                        indexHeartBeat = serialStrBuffer.find(HEARTBEAT_HEADER);

                        ROS_DEBUG("Heartbeat Index: %d", indexHeartBeat);
                    }

                    /* Check if batt voltage header is found (not equal to string::npos) */
                    if(serialStrBuffer.find(BATT_VOLTAGE_HEADER) != std::string::npos)
                    {
                        isBattVoltagePresent = true;
                        indexBattVoltage = serialStrBuffer.find(BATT_VOLTAGE_HEADER);

                        // ROS_DEBUG("BattVoltage Index: %d", indexHeartBeat);
                    }

                    /* If string::find() heartbeat and batt voltage headers are both found, check which index is smaller */
                    if(isHeartBeatPresent && isBattVoltagePresent)
                    {
                        /* Process the message type with a smaller "find" index. */
                        if(indexHeartBeat < indexBattVoltage)
                        {
                            /* Remove garbage message before header */
                            serialStrBuffer = serialStrBuffer.substr(indexHeartBeat);

                            /* Process heartbeat message */
                            processHeartbeatMessage(serialStrBuffer, 0);
                        }
                        else
                        {
                            /* Remove garbage message before header */
                            serialStrBuffer = serialStrBuffer.substr(indexBattVoltage);

                            /* Process batt voltage message */
                            processBattVoltageMessage(serialStrBuffer, 0);
                        }
                    }
                    /* Else, process the message found */
                    else
                    {
                        if(isHeartBeatPresent)
                        {
                            /* Remove garbage message before header */
                            serialStrBuffer = serialStrBuffer.substr(indexHeartBeat);

                            /* Process heartbeat message */
                            processHeartbeatMessage(serialStrBuffer, 0);
                        }
                        
                        if(isBattVoltagePresent)
                        {
                            /* Remove garbage message before header */
                            serialStrBuffer = serialStrBuffer.substr(indexBattVoltage);

                            /* Process batt voltage message */
                            processBattVoltageMessage(serialStrBuffer, 0);
                        }
                    }  
                }
            }
        }
    }
    catch (serial::SerialException &e)
    {
        ROS_ERROR("Motor Serial Port Crash");
        serial.close();
        isSerialPortDown = true; // likely cause even though Stop Button not physically connected to GPIO
    }
}

// Take cmd variable and send movement command
void LogcartHardware::write()
{
    // ROS_DEBUG("Cmd[0]: %.2f, Cmd[1]: %.2f", cmd[0], cmd[1]);

    std::string rpm1 = std::to_string((int)(cmd[0] * (SEC_PER_MIN / ((2 * PI)))));
    std::string rpm2 = std::to_string((int)(cmd[1] * (SEC_PER_MIN / ((2 * PI)))));

    try
    {
        if (!isSerialPortDown)
        {
            // ?S,XX_XX,\r
            std::string speedCmd =  "?S," + rpm1 + "_" + rpm2 + ",\r";
            // std::string speedCmd =  "?S," + std::to_string(80) + "_" + std::to_string(80) + ",\r";
            serial.write(speedCmd);
            
            ROS_DEBUG("Serial Write: %s", speedCmd.c_str());
        }
        else
        {
            ROS_WARN("Serial port is down!");
        }
    }
    catch (serial::SerialException &e)
    {
        ROS_ERROR("Motor Serial Port Crash");
        serial.close();
        isSerialPortDown = true;
    }
}

void LogcartHardware::checkSerialPort()
{
    if (isSerialPortDown)
    {
        ROS_ERROR("Re-initialize motor controller port");
        initSerialPort();
    }
}

void LogcartHardware::initSerialPort()
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

void LogcartHardware::processBattVoltageMessage(std::string &serialStr, int indexHeader)
{
    /* Check if batt voltage message is complete */
    if(serialStr.find("+") != std::string::npos)
    {
        int indexMsgEnd = serialStr.find("+");
        ROS_DEBUG("indexMsgEnd: %d", indexMsgEnd);
        
        /* Extract batt voltage message from serial string buffer */
        std::string strBattVoltage = serialStr.substr(indexHeader, (indexMsgEnd - indexHeader) + 1);
        ROS_DEBUG("strBattVoltage: %s", strBattVoltage.c_str());

        /* Check if number of fields is correct */
        if(std::count(strBattVoltage.begin(), strBattVoltage.end(), '_') == (FIELDS_IN_BATT_VOLTAGE_MSG - 1))
        {
            // Extract header, payload and footer from the batt voltage message
            char buff[1024];
            std::strcpy(buff, strBattVoltage.c_str()); 
            char* header = strtok(buff, ",");   // Expects: '?V'
            char* payload = strtok(NULL, ",");  // Expects: <payload>
            char* footer = strtok(NULL, ",");   // Expects: '+'
            ROS_DEBUG("Header: %s, Payload: %s, Footer: %s", header, payload, footer);

            char* strBattVoltage = payload;
            ROS_DEBUG("Payload String: %s", strBattVoltage);

            // If invalid characters are found, 0 will be used
            int battVoltage = atoi((isValidNumber(strBattVoltage) ? strBattVoltage : "0"));
            ROS_DEBUG("Payload Value: %d", battVoltage);
        }
        else
        {
            ROS_ERROR("Invalid number of fields detected in Batt Voltage message.");
        }

        /* Remove the batt voltage message from the serial string buffer */
        serialStr = serialStr.substr(indexMsgEnd);
    }
    /* Else if the message is incomplete */
    else
    {
        // Remove the string and header from the front of the serial string buffer
        // serialStr = serialStr.substr(indexHeader + BATT_VOLTAGE_HEADER.length());

        /* Print an error alert indicating unexpected termination of message */
        ROS_WARN("Batt Voltage message was incomplete.");
    }
}

void LogcartHardware::processHeartbeatMessage(std::string &serialStr, int indexHeader)
{
    /* Check if heartbeat message is complete */
    if(serialStr.find("+") != std::string::npos)
    {
        int indexMsgEnd = serialStr.find("+");
        ROS_DEBUG("indexMsgEnd: %d", indexMsgEnd);
        
        /* Extract heartbeat message from serial string buffer */
        std::string strHeartBeat = serialStr.substr(indexHeader, (indexMsgEnd - indexHeader) + 1);
        ROS_DEBUG("strHeartBeat: %s", strHeartBeat.c_str());

        /* Check if number of fields is correct */
        if(std::count(strHeartBeat.begin(), strHeartBeat.end(), '_') == (FIELDS_IN_HEARTBEAT_MSG - 1))
        {
            // Extract header, payload and footer from the heartbeat message
            char buff[1024];
            std::strcpy(buff, strHeartBeat.c_str()); 
            char* header = strtok(buff, ",");   // Expects: '?H'
            char* payload = strtok(NULL, ",");  // Expects: <payload>
            char* footer = strtok(NULL, ",");   // Expects: '+'
            ROS_DEBUG("Header: %s, Payload: %s, Footer: %s", header, payload, footer);

            char* strSpeed1         = strtok(payload, "_");
            char* strSpeed2         = strtok(NULL, "_");
            char* strEncoder1       = strtok(NULL, "_");
            char* strEncoder2       = strtok(NULL, "_");
            ROS_DEBUG("Payload Strings: %s, %s, %s, %s", strSpeed1, strSpeed2, strEncoder1, strEncoder2);

            // Extract values if received message is complete (not truncated)
            // For each parameter, if invalid characters are found, 0 will be used
            int speed1      = atoi((isValidNumber(strSpeed1)        ? strSpeed1 : "0"));
            int speed2      = atoi((isValidNumber(strSpeed2)        ? strSpeed2 : "0"));
            long encoder1   = atol((isValidNumber(strEncoder1)      ? strEncoder1 : "0"));
            long encoder2   = atol((isValidNumber(strEncoder2)      ? strEncoder2 : "0"));

            ROS_DEBUG("Payload Values: %d, %d, %ld, %ld", speed1, speed2, encoder1, encoder2);

            // Set values for ROS control
            vel[0] = speed1     * (2 * PI) / SEC_PER_MIN;
            vel[1] = speed2     * (2 * PI) / SEC_PER_MIN;
            pos[0] = encoder1   * (2 * PI) / ENC_COUNT_PER_REV;
            pos[1] = encoder2   * (2 * PI) / ENC_COUNT_PER_REV;
            ROS_INFO("vel[0]: %.2f, vel[1]: %.2f, pos[0]: %.2f, pos[1]: %.2f", vel[0],vel[1], pos[0], pos[1]);
        }
        else
        {
            ROS_ERROR("Invalid number of fields detected in Heartbeat message.");
        }

        /* Remove the heartbeat message from the serial string buffer */
        serialStr = serialStr.substr(indexMsgEnd);
    }
    /* Else if the message is incomplete */
    else
    {
        // Remove the string and header from the front of the serial string buffer
        // serialStr = serialStr.substr(indexHeader + HEARTBEAT_HEADER.length());

        /* Print an error alert indicating unexpected termination of message */
        ROS_WARN("Heartbeat message was incomplete.");
    }
}

bool LogcartHardware::isValidNumber(std::string str)
{
    bool isNum = true;
    for(int i=0; i<str.length(); i++)
    {
        if(isdigit(str[i]) == false && str[i] != '-')
        {
            isNum = false;
            ROS_ERROR("Invalid number detected: %s", str.c_str());
            break;
        }
    }
    return isNum;
}

// Translate joystick messages to velocity commands for setting motor speed
void LogcartHardware::velocityRecvCallback(const geometry_msgs::Twist::ConstPtr &twist)
{
    double linear, angular, left_speed, right_speed, left_speed_percent, right_speed_percent;

    // Extract linear and angular velocities from the message
    linear = twist->linear.x;
    angular = twist->angular.z;

    ROS_DEBUG("linear x: %.4f, angular.x: %.4f", twist->linear.x, twist->angular.z);

    // Calculate wheel speeds in m/s, considering turning
    left_speed = linear - (angular * (wheel_base / 2));
    right_speed = linear + (angular * (wheel_base / 2));

}

void LogcartHardware::odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    geometry_msgs::Pose2D current_pose;

    // linear position
    current_pose.x = msg->pose.pose.position.x;
    current_pose.y = msg->pose.pose.position.y;

    // quaternion to RPY conversion
    tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);

    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // angular positions
    current_pose.theta = yaw;
    pose2d_pub.publish(current_pose);
}

int main(int argc, char **argv)
{
    double loop_hz;

    ros::init(argc, argv, "mbed_interface");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(2); // multi-threaded non-blocking spinning
    ros::Time current_time, last_time;
    ros::Duration elapsed_time;

    LogcartHardware logcart(nh);
    controller_manager::ControllerManager cm(&logcart, nh);

    current_time = ros::Time::now();
    last_time = ros::Time::now();

    nh.param("/logcart_driver/hardware_interface/loop_hz", loop_hz, 0.1);
    ros::Rate rate(loop_hz);
    spinner.start();

    while (nh.ok())
    {
        logcart.read();
        logcart.checkSerialPort();
        current_time = ros::Time::now();
        elapsed_time = current_time - last_time;
        cm.update(ros::Time::now(), elapsed_time);
        last_time = current_time;
        logcart.write();
        rate.sleep();
    }

    spinner.stop();
    return 0;
}
