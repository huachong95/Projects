#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <milo/Joystick.h>

class TeleopJoy
{
public:
    TeleopJoy();
    ros::Publisher vel_pub_;
    ros::Publisher button_pub_;
    geometry_msgs::Twist twist;
    void publishTwist();

private:
    void joyCallback(const sensor_msgs::Joy::ConstPtr &joy);
    ros::NodeHandle nh_;

    ros::Subscriber joy_sub_;
    ros::Subscriber button_sub_;
    ros::Subscriber navi_sub_;
    milo::Joystick joy_buttons;

    int l_axis, a_axis;
    double l_scale_, a_scale_;
    bool trackingButtonActive;
    
};

TeleopJoy::TeleopJoy() : l_axis(1), a_axis(2), trackingButtonActive(false)
{
    nh_.param("/milo_teleop_joy/axis_linear", l_axis, l_axis);
    nh_.param("/milo_teleop_joy/axis_angular", a_axis, a_axis);
    nh_.param("/milo_teleop_joy/scale_linear", l_scale_, l_scale_);
    nh_.param("/milo_teleop_joy/scale_angular", a_scale_, a_scale_);


    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 100);
    button_pub_ = nh_.advertise<milo::Joystick>("joy_buttons", 100);

    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 100, &TeleopJoy::joyCallback, this);

}

void TeleopJoy::publishTwist()
{
    vel_pub_.publish(twist);
}

// Manipulate joystick data by scaling it and using independent axes
void TeleopJoy::joyCallback(const sensor_msgs::Joy::ConstPtr &joy)
{

    twist.angular.x = a_scale_ * joy->axes[a_axis];
    twist.linear.y = l_scale_ * joy->axes[l_axis];

    
    // vel_pub_.publish(twist);

    // TODO: Clean up the implementation below!!!!
    // enableTracking = 0 means joystick mode
    // enableTracking = 1 means follow-mode mode
    // enableTracking = 2 means navigation mode

    if (joy->buttons[0] == 1.0) // BUTTON A
    {
        ROS_INFO("A button pressed");
        joy_buttons.joystick_mode = 0;
        button_pub_.publish(joy_buttons);
    }

    if (joy->buttons[1] == 1.0) // BUTTON B
    {
        ROS_INFO("B button pressed");
        joy_buttons.joystick_mode = 1;
        button_pub_.publish(joy_buttons);
    }

    if (joy->buttons[2] == 1.0) // BUTTON X
    {
        ROS_INFO("X button pressed");
        joy_buttons.joystick_mode = 2;
        button_pub_.publish(joy_buttons);
    }

    if (joy->buttons[3] == 1.0) // BUTTON Y
    {
        ROS_INFO("Y button pressed");
        joy_buttons.joystick_mode = 3;
        button_pub_.publish(joy_buttons);
    }

    if (joy->buttons[4] == 1.0) // BUTTON LB
    {
        ROS_INFO("LB button pressed");
        joy_buttons.joystick_mode = 4;
        button_pub_.publish(joy_buttons);
    }

    if (joy->buttons[5] == 1.0) // BUTTON RB
    {
        ROS_INFO("B button pressed");
        joy_buttons.joystick_mode = 5;
        button_pub_.publish(joy_buttons);
    }

    if (joy->buttons[6] == 1.0) // BUTTON BACK
    {
        ROS_INFO("BACK button pressed");
        joy_buttons.joystick_mode = 6;
        button_pub_.publish(joy_buttons);
    }

    if (joy->buttons[7] == 1.0) // BUTTON START
    {
        ROS_INFO("START button pressed");
        joy_buttons.joystick_mode = 7;
        button_pub_.publish(joy_buttons);
    }
    if (joy->buttons[8] == 1.0) // BUTTON POWER
    {
        ROS_INFO("POWER button pressed");
        joy_buttons.joystick_mode = 8;
        button_pub_.publish(joy_buttons);
    }

    if (joy->buttons[9] == 1.0) // BUTTON LEFTJOY
    {
        ROS_INFO("LEFTJOY button pressed");
        joy_buttons.joystick_mode = 9;
        button_pub_.publish(joy_buttons);
    }

    if (joy->buttons[10] == 1.0) // BUTTON RIGHTJOY
    {
        ROS_INFO("RIGHTJOY button pressed");
        joy_buttons.joystick_mode = 10;
        button_pub_.publish(joy_buttons);
    }
}



int main(int argc, char **argv)
{
    double loop_hz;

    ros::init(argc, argv, "milo_teleop_joy");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(2); // multi-threaded non-blocking spinning
    TeleopJoy milo_teleop_joy;
    nh.getParam("/milo_teleop_joy/loop_hz", loop_hz);
    ros::Rate loop_rate(loop_hz);

    while (nh.ok())
    {
        milo_teleop_joy.publishTwist();
        ros::spinOnce();
        loop_rate.sleep();
        
    }
    return 0;
}
