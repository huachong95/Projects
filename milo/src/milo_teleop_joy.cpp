#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <milo/Joystick.h>

class TeleopJoy
{
public:
    TeleopJoy();

private:
    void joyCallback(const sensor_msgs::Joy::ConstPtr &joy);
    void naviCallback(const geometry_msgs::Twist::ConstPtr &navi);
    ros::NodeHandle nh_;
    ros::Publisher vel_pub_;
    ros::Publisher button_pub_;
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
    nh_.param("/teleop_joy_node/axis_linear", l_axis, l_axis);
    nh_.param("/teleop_joy_node/axis_angular", a_axis, a_axis);
    nh_.param("/teleop_joy_node/scale_linear", l_scale_, l_scale_);
    nh_.param("/teleop_joy_node/scale_angular", a_scale_, a_scale_);

    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/joystick/cmd_vel", 100);
    button_pub_ = nh_.advertise<milo::Joystick>("joy_buttons", 100);

    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 100, &TeleopJoy::joyCallback, this);
    navi_sub_ = nh_.subscribe<geometry_msgs::Twist>("/navi/cmd_vel",100, &TeleopJoy::naviCallback, this);
}
void TeleopJoy::naviCallback(const geometry_msgs::Twist::ConstPtr &navi){
    if (joy_buttons.enableTracking == 2) {
        geometry_msgs::Twist twist;
        twist.angular.z = navi->angular.z; 
        twist.linear.x = navi->linear.x;
        vel_pub_.publish(twist);
}
}
// Manipulate joystick data by scaling it and using independent axes
void TeleopJoy::joyCallback(const sensor_msgs::Joy::ConstPtr &joy)
{
    geometry_msgs::Twist twist;
    twist.angular.z = a_scale_ * joy->axes[a_axis];
    twist.linear.x = l_scale_ * joy->axes[l_axis];
    //vel_pub_.publish(twist);

    // TODO: Clean up the implementation below!!!!
    // enableTracking = 0 means joystick mode
    // enableTracking = 1 means follow-mode mode
    // enableTracking = 2 means navigation mode

    if (joy->buttons[5] == 1.0)
    {
        // joy->buttons[5] => RB button (1 when pressed else 0)
        ROS_INFO("RB button pressed - disable joy, disable navigation, follow-mode takes over.");
        joy_buttons.enableTracking = 1;
        button_pub_.publish(joy_buttons);
    }
    else if (joy->buttons[4] == 1.0)
    {
        // joy->buttons[4] => LB button (1 when pressed else 0)
        ROS_INFO("LB button pressed - disable follow-mode, disable joy, navigation takes over.");
        joy_buttons.enableTracking = 2;
        button_pub_.publish(joy_buttons);
    }
    else if (joy->buttons[7] == 1.0)
    {
        // joy->buttons[7] => RT button (1 when pressed else 0)
        ROS_INFO("RT button pressed - disable follow-mode, disable navigation, joy takes over");
        joy_buttons.enableTracking = 0;
        button_pub_.publish(joy_buttons);
    }

    // Publish joystick/cmd_vel
    if (joy_buttons.enableTracking == 0)
    {
        vel_pub_.publish(twist);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "milo_teleop_joy");
    TeleopJoy teleop_joy;
    ros::spin();
    return 0;
}
