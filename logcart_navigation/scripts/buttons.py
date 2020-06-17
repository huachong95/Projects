#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from actionlib_msgs.msg import GoalStatusArray, GoalID
from geometry_msgs.msg import PoseStamped

class Emergency:
    def __init__(self):
        self.joy = 0
        self.joy2 = 0
        self.goalstatus = None

    def joy_callback(self, msg):
        # "Store" message received.
        self.joy = msg.buttons[2]
        self.joy2 = msg.buttons[1]

    def goalstatus_callback(self, msg):
        # "Store" the message received.
        self.goalstatus = msg
        self.send_cancel_command()

    def send_cancel_command(self):
        stopbutton = self.joy
        gohomebutton = self.joy2
        statuslist = self.goalstatus.status_list

        if stopbutton == 1:
            if statuslist == []:
                rospy.loginfo("No Goal")

            else:
                pub = rospy.Publisher('move_base/cancel', GoalID, queue_size=1)
                cancelmessage = GoalID()
                cancelmessage.stamp.secs = 0
                cancelmessage.stamp.nsecs = 0
                cancelmessage.id = ''
                pub.publish(cancelmessage)

        if gohomebutton == 1:
            pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=1)
            gohomemessage = PoseStamped()
            gohomemessage.header.frame_id = 'map'
            gohomemessage.pose.position.x = -0.491820812225
            gohomemessage.pose.position.y = -0.333309173584
            gohomemessage.pose.orientation.z = 0.0956382859987
            gohomemessage.pose.orientation.w = 0.9954161533
            pub.publish(gohomemessage)

if __name__ == '__main__':

    rospy.init_node('joy_to_nav_node')
    
    emergency = Emergency()

    rospy.Subscriber('/joy', Joy, emergency.joy_callback)
    rospy.Subscriber('move_base/status', GoalStatusArray, emergency.goalstatus_callback)
    rospy.spin()
