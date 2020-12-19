#include <serial/serial.h>
#include <milo/Joystick.h>
#include <chrono>

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
    double rpm2ms(double &rpm1);
    double ms2rpm(double &v1);
    void accelerationControl(double &rpm_l, double &rpm_r);
    double saturate(double &value, double min, double max);
    bool isValidNumber(std::string str);
    void processHeartbeatMessage(std::string &serialStr, int indexHeader);
    void processBattVoltageMessage(std::string &serialStr, int indexHeader);

    ros::NodeHandle nh;
    ros::Subscriber cmd_vel_sub;
    ros::Subscriber button_sub;
    double wheel_distance, wheel_radius, max_l_speed,
        min_l_speed, max_ang_speed, min_ang_speed, max_accel, min_accel;
    double left_rpm = 0.0, right_rpm = 0.0;
    int log_level;
    const double PI = 3.14159265358979323846;
    const double SEC_PER_MIN = 60.0;
    std::string port_mbed;
    serial::Serial serial;
    bool isSerialPortDown;
    std::string serialStrBuffer;
    const std::string HEARTBEAT_HEADER = "?H";
    const std::string BATT_VOLTAGE_HEADER = "?V";

    const int FIELDS_IN_HEARTBEAT_MSG = 8;
    const int FIELDS_IN_BATT_VOLTAGE_MSG = 1;

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
    nh.getParam("/milo_hardware_interface/max_accel", max_accel);
    nh.getParam("/milo_hardware_interface/min_accel", min_accel);
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
    double linear = 0.0, angular = 0.0, left_mps = 0.0, right_mps = 0.0;
    linear = cmd->linear.y;
    angular = cmd->angular.x;
    ROS_DEBUG("Linear Speed: %f",linear );
    if (linear >= 0.0)
    {
        linear = linear * max_l_speed;
    }
    else
    {
        linear = linear * -min_l_speed;
    }

    if (angular >= 0.0)
    {
        angular = angular * max_ang_speed;
    }
    else
    {
        angular = angular * -min_ang_speed;
    }

    linear = saturate(linear, min_l_speed, max_l_speed);
    angular = saturate(angular, min_ang_speed, max_ang_speed);
        ROS_DEBUG("Linear Speed: %f",linear );

    left_mps = linear - angular * wheel_distance / 2.0f;
    right_mps = linear + angular * wheel_distance / 2.0f;

    left_rpm = ms2rpm(left_mps);
    right_rpm = ms2rpm(right_mps);
    accelerationControl(left_rpm, right_rpm);
}

void milo_hardware::accelerationControl(double &rpm_l, double &rpm_r)
{

    // static double last_time = current_time;

    static double last_mps_l = 0.0, last_mps_r = 0.0;
    double loop_duration = 1 / loop_hz;

    double mps_l = rpm2ms(rpm_l); //converts left velocity in rpm to m/s
    double mps_r = rpm2ms(rpm_r); //converts right velocity in rpm to m/s

    double t_accel_l = (mps_l - last_mps_l) / loop_duration;
    double t_accel_r = (mps_r - last_mps_r) / loop_duration;
    double accel_l = saturate(t_accel_l, min_accel, max_accel);
    double accel_r = saturate(t_accel_r, min_accel, max_accel);

    mps_l = last_mps_l + accel_l * loop_duration;
    mps_r = last_mps_r + accel_r * loop_duration;

    last_mps_l = mps_l;
    last_mps_r = mps_r;

    //Enforces the acceleration limit into the required motor rpm
    rpm_l = ms2rpm(mps_l);
    rpm_r = ms2rpm(mps_r);

    ROS_DEBUG("2 RPM_L: %f, RPM_R: %f", rpm_l, rpm_r);
}

double milo_hardware::rpm2ms(double &rpm1)
{
    double v1 = wheel_radius * rpm1 * 2.0f * PI / SEC_PER_MIN;
    return v1;
}

double milo_hardware::ms2rpm(double &v1)
{
    double rpm1 = v1 * SEC_PER_MIN / (2.0f * PI * wheel_radius);
    return rpm1;
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
            /* Append the received serial string to the back of the existing string */
            std::replace(tempStrBuff.begin(), tempStrBuff.end(), '\r', '+');
            serialStrBuffer.append(tempStrBuff);
            // ROS_DEBUG("serialStrBuffer: %s", serialStrBuffer.c_str());

            /* NOTE: Assuming that each message received is always complete before another one is received */
            /* Serial read buffer string (i.e. "?H,XX_XX_XX_XX_XX_XX_XX_XX,\r" or "?V,40,\r") */

            /* If serial string buffer is not empty, search for valid headers */
            if (serialStrBuffer.length() > 0)
            {
                /* Find number of headers in serial string buffer */
                int numOfHeaders = std::count(serialStrBuffer.begin(), serialStrBuffer.end(), '?');

                // ROS_DEBUG("Num of headers found: %d", numOfHeaders);

                /* Loop for number of headers found */
                for (int i = 0; i < numOfHeaders; i++)
                {
                    bool isHeartBeatPresent = false, isBattVoltagePresent = false;
                    int indexHeartBeat = 9999, indexBattVoltage = 9999;

                    /* Check if heartbeat header is found (not equal to string::npos) */
                    if (serialStrBuffer.find(HEARTBEAT_HEADER) != std::string::npos)
                    {
                        isHeartBeatPresent = true;
                        indexHeartBeat = serialStrBuffer.find(HEARTBEAT_HEADER);

                        // ROS_DEBUG("Heartbeat Index: %d", indexHeartBeat);
                    }

                    /* Check if batt voltage header is found (not equal to string::npos) */
                    if (serialStrBuffer.find(BATT_VOLTAGE_HEADER) != std::string::npos)
                    {
                        isBattVoltagePresent = true;
                        indexBattVoltage = serialStrBuffer.find(BATT_VOLTAGE_HEADER);

                        // ROS_DEBUG("BattVoltage Index: %d", indexHeartBeat);
                    }

                    /* If string::find() heartbeat and batt voltage headers are both found, check which index is smaller */
                    if (isHeartBeatPresent && isBattVoltagePresent)
                    {
                        /* Process the message type with a smaller "find" index. */
                        if (indexHeartBeat < indexBattVoltage)
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
                        if (isHeartBeatPresent)
                        {
                            /* Remove garbage message before header */
                            serialStrBuffer = serialStrBuffer.substr(indexHeartBeat);

                            /* Process heartbeat message */
                            processHeartbeatMessage(serialStrBuffer, 0);
                        }

                        if (isBattVoltagePresent)
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
            // std::string serialTxMsg = "?S,50_60,/r";

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

void milo_hardware::processHeartbeatMessage(std::string &serialStr, int indexHeader)
{
    /* Check if heartbeat message is complete */
    if (serialStr.find("+") != std::string::npos)
    {
        int indexMsgEnd = serialStr.find("+");
        ROS_DEBUG("indexMsgEnd: %d", indexMsgEnd);

        /* Extract heartbeat message from serial string buffer */
        std::string strHeartBeat = serialStr.substr(indexHeader, (indexMsgEnd - indexHeader) + 1);
        // ROS_DEBUG("strHeartBeat: %s", strHeartBeat.c_str());

        /* Check if number of fields is correct */
        if (std::count(strHeartBeat.begin(), strHeartBeat.end(), '_') == (FIELDS_IN_HEARTBEAT_MSG - 1))
        {
            // Extract header, payload and footer from the heartbeat message
            char buff[1024];
            std::strcpy(buff, strHeartBeat.c_str());
            char *header = strtok(buff, ",");  // Expects: '?H'
            char *payload = strtok(NULL, ","); // Expects: <payload>
            char *footer = strtok(NULL, ",");  // Expects: '+'
            ROS_DEBUG("Header: %s, Payload: %s, Footer: %s", header, payload, footer);

            char *strSpeed1 = strtok(payload, "_");
            char *strSpeed2 = strtok(NULL, "_");
            char *strEncoder1 = strtok(NULL, "_");
            char *strEncoder2 = strtok(NULL, "_");
            ROS_DEBUG("Payload Strings: %s, %s, %s, %s", strSpeed1, strSpeed2, strEncoder1, strEncoder2);

            // Extract values if received message is complete (not truncated)
            // For each parameter, if invalid characters are found, 0 will be used
            int speed1 = atoi((isValidNumber(strSpeed1) ? strSpeed1 : "0"));
            int speed2 = atoi((isValidNumber(strSpeed2) ? strSpeed2 : "0"));
            long encoder1 = atol((isValidNumber(strEncoder1) ? strEncoder1 : "0"));
            long encoder2 = atol((isValidNumber(strEncoder2) ? strEncoder2 : "0"));

            ROS_DEBUG("Payload Values: %d, %d, %ld, %ld", speed1, speed2, encoder1, encoder2);
        }
        serialStr = serialStr.substr(indexMsgEnd);
    }
    else
    {
        ROS_WARN("last serial msg index is not \r");
    }
    /* Remove the heartbeat message from the serial string buffer */
}

void milo_hardware::processBattVoltageMessage(std::string &serialStr, int indexHeader)
{
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
        ROS_DEBUG("MAIN");
        // milo.readSerialPort();
        // milo.checkSerialPort();
        milo.writeSerialPort();

        rate.sleep();
    }

    spinner.stop();
    return 0;
}
