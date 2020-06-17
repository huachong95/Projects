#!/usr/bin/env python

import os
import rospy
import threading
import requests
import json
import numpy as np
import math
import serial 
import time
import requests

from flask import Flask, request, abort, json
from flask_restful import Api, reqparse, Resource

from std_msgs.msg import String
from sensor_msgs.msg import Joy , LaserScan
from actionlib_msgs.msg import GoalStatusArray , GoalID
from geometry_msgs.msg import PoseWithCovarianceStamped , PoseStamped , Twist, Pose2D
from logcart_follow.msg import LeaderTrack
from nav_msgs.msg import Odometry

app = Flask(__name__)

# # ROS topic publishing initialization
pose_init = rospy.Publisher('initial_pos', PoseWithCovarianceStamped, queue_size=1)
pose_home = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=1)
cancel_cmd = rospy.Publisher('move_base/cancel', GoalID, queue_size=1)
joy_pub = rospy.Publisher('joy', Joy, queue_size=1)

class Info:
    def goalstatus_callback(self, msg):
        if len(msg.status_list) > 0:
            self.goalstatus = msg.status_list[0].status
        else: 
            self.goalstatus = 0
        #print(self.goalstatus)
    
    def heartbeat_callback(self, msg):
        self.battery = msg.data
        #print(self.battery) 

    def cmd_vel_callback(self , msg):
        self.linear_x = msg.linear.x
        self.angular_z = msg.angular.z
        # print(self.linear_x) 
        # print(self.angular_z)

    def odom_callback(self , msg):
        self.odom = msg.twist.twist.linear.x
        #print(self.odom)
    
    def position_callback(self , msg):
        self.position_x = msg.pose.pose.position.x
        self.position_y = msg.pose.pose.position.y
        self.orientation_z = msg.pose.pose.orientation.z
        self.orientation_w = msg.pose.pose.orientation.w
        # print(self.position_x)
        # print(self.orientation_z)

    def joy_callback(self , msg):
        self.autonomous = msg.buttons[4]
        self.follow = msg.buttons[5]
        self.joystick = msg.buttons[7]
        # print(self.autonomous)
        # print(self.follow)
        # print(self.joystick)
    #callback for rosmsg from pose2d(follow)
    def pose2d_callback(self,msg):
        pos_x =  msg.x
        pos_y = msg.y
        self.distance_1 =math.sqrt(math.pow(pos_x,2) + math.pow(pos_y,2))
        self.angle_1 = np.rad2deg(msg.theta)

    def leadertrack_callback(self,msg):
        pos_x = msg.centroid.x
        pos_y = -(msg.centroid.y)
        self.distance = math.sqrt(math.pow(pos_x,2) + math.pow(pos_y,2))
        self.angle = np.rad2deg(np.arctan2(pos_y, pos_x))
    
    def TrackingCallback(self,msg):
        self.display_text = str(msg.data)

    def TrajectoryCallback(self,msg):
        self.trajectory = str(msg.data)

    def ScanCallback(self, data):
        ranges = data.ranges
        # print(ranges)

        counter = 0
        angle_min_deg = np.rad2deg(data.angle_min)
        starting_angle = angle_min_deg
        angle_increment_deg = np.rad2deg(data.angle_increment)

        for x in ranges:
            counter = counter + 1
            starting_angle = starting_angle + angle_increment_deg
            # print(counter, ": Angle: ", starting_angle,", Range: ", x)
            if (starting_angle >= -10 and starting_angle <= 15 and 
                x >= 0.50000 and x <= 0.80000):
                self.target_available = "True"
                
            else:
                self.target_available = "False"
            
            #print(self.target_available)   
##Receiving of information from client
# Handling initial_pos 'POST' request from client.py


@app.route('/autonomous/initialpose/set', methods=['POST']) 
def autonomous_initialpose():
    req_pos = request.get_json()
    pose_x = req_pos['position_x']
    pose_y = req_pos['position_y']
    pose_z = req_pos['orientation_z']
    print(pose_x)
    print(pose_y)
    print(pose_z)
    initial_pos(pose_x,pose_y,pose_z)
    return json.dumps(request.get_json()), 201

# Handling home_pos 'POST' request from client.py
@app.route('/autonomous/goal/set', methods=['POST']) 
def autonomous_goal():
    req_home_pos = request.get_json()
    home_x = req_home_pos['position_x']
    home_y = req_home_pos['position_y']
    home_z = req_home_pos['orientation_z']
    home_w = req_home_pos['orientation_w']
    print(home_x)
    print(home_y)
    print(home_z)
    print(home_w)
    move_home(home_x,home_y,home_z,home_w)
    print("publish success")
    return json.dumps(request.get_json()), 201

#Handling send_cancel_command 'POST' request from client.py
@app.route ('/autonomous/cancel/set', methods=['POST','GET'])
def autonomous_cancel():
    # req_cancel_command = request.get_json()
    print("Cancelling specific goal")
    cancel_command()
    print("cancel command publish success")
    return json.dumps(request.get_json()), 201

##Sending of requested information to CLIENT
#handle GET request to check if target is available
@app.route('/follow/target', methods=['GET'])
def check_target():
    targetavailable = str(info.target_available)
    print(info.target_available+"http")
    target = {
        'target_available' : targetavailable
    }
    #print(target['target_available'])
    return json.dumps(target) , 201

@app.route("/autonomous/status/get", methods=['POST','GET'])
def autonomous_status():
    goal_state = int(info.goalstatus)
    if goal_state == 1:
        goal_status = 'unavailable'
    else:
        goal_status = 'available'
    state = {'goal_status': goal_status}
    #print(goal_status)
    return json.dumps(state) , 201

@app.route("/autonomous/pose/get", methods=['POST','GET'])
def autonomous_pose():
    pose = {
        'position_x' : info.position_x , 
        'position_y' : info.position_y ,
        'orientation_z' : info.orientation_z ,
        'orientation_w' : info.orientation_w
    }
    return json.dumps(pose) ,201


@app.route("/follow/pose2d", methods=['POST','GET'])
def pose2d():
    pose2d = { 'distance_1' : info.distance_1 ,
                    'angle_1' : info.angle_1}
    return json.dumps(pose2d) ,201

@app.route("/follow/leadertrack", methods=['POST','GET'])
def leader_track():    
    leadertrack_msg = { 'distance' : info.distance ,
                    'angle' : info.angle}
    return json.dumps(leadertrack_msg) ,201

@app.route("/mode/set", methods=['POST','GET'])
def mode_selection():
    data = request.get_json()
    mode = data['mode']
    joy = Joy()
    joy.buttons = [0] * 12
    joy.axes = [0.0]* 6
    
    if mode == 'joystick':
        joy.buttons[4] = 0
        joy.buttons[5] = 0
        joy.buttons[7] = 1
    elif mode == 'follow':
        joy.buttons[4] = 0
        joy.buttons[5] = 1
        joy.buttons[7] = 0
    elif mode == 'autonomous':
        joy.buttons[4] = 1
        joy.buttons[5] = 0
        joy.buttons[7] = 0
    elif mode == 'counterclockwise':
        joy.axes[2] = 1.0
    print(joy)
    joy_pub.publish(joy)
    return json.dumps(request.get_json()), 201

@app.route("/mode/get", methods = ['POST','GET'])
def mode_get():
    if info.autonomous == 1 :
        mode_state = 'autonomous'
    elif info.follow == 1 :
        mode_state = 'follow'
    elif info.joystick == 1 :
        mode_state = 'joystick'
    
    mode = {'mode_state' : mode_state}
    return json.dumps(mode) , 201

@app.route("/heartbeat/get",methods=['POST','GET'])
def heartbeat():
    heartbeat = {
        'battery': info.battery,
        'linear_x': info.linear_x ,
        'angular_z': info.angular_z,
        'odometry' : info.odom,
        'track_status' : info.display_text,
    }
    return json.dumps(heartbeat) , 201

# ROS topic publishing function
def initial_pos(x,y,z):
    init_pose = PoseWithCovarianceStamped()
    init_pose.header.frame_id = "map"
    init_pose.header.stamp = rospy.Time.now()
    init_pose.pose.pose.position.x = float(x)
    init_pose.pose.pose.position.y = float(y)
    init_pose.pose.pose.position.z = float(z)
    pose_init.publish(init_pose)

def move_home(x,y,z,w):
    home_pose = PoseStamped()
    home_pose.header.frame_id = "map"
    home_pose.header.stamp = rospy.Time.now()
    home_pose.pose.position.x = float(x)
    home_pose.pose.position.y = float(y)
    home_pose.pose.orientation.z = float(z)
    home_pose.pose.orientation.w = float(w)
    pose_home.publish(home_pose)

def cancel_command():
    cancelmessage = GoalID()
    cancelmessage.stamp.secs = 0
    cancelmessage.stamp.nsecs = 0
    cancelmessage.id =''
    cancel_cmd.publish(cancelmessage)
    print("TestFunction")
    

if __name__ == '__main__':
    info = Info()
    threading.Thread(target=lambda: rospy.init_node('REST_to_nav_node', disable_signals=True)).start()
    rospy.Subscriber('move_base/status', GoalStatusArray , info.goalstatus_callback) 
    rospy.Subscriber('/battery_state', String, info.heartbeat_callback, queue_size=1) 
    rospy.Subscriber('/base_controller/cmd_vel', Twist, info.cmd_vel_callback, queue_size=1)
    rospy.Subscriber('/base_controller/odom', Odometry, info.odom_callback, queue_size=1) 
    rospy.Subscriber('/amcl_pose',PoseWithCovarianceStamped,info.position_callback,queue_size=1)
    rospy.Subscriber('/joy', Joy, info.joy_callback, queue_size=1)
    rospy.Subscriber('/leadertrack', LeaderTrack, info.leadertrack_callback,queue_size=1)
    rospy.Subscriber('/track_status', String, info.TrackingCallback, queue_size=1)
    rospy.Subscriber('/pose2d', Pose2D, info.pose2d_callback, queue_size=1)
    rospy.Subscriber('/trajectory', String, info.TrajectoryCallback, queue_size=1)
    rospy.Subscriber('/scan_filtered', LaserScan, info.ScanCallback, queue_size=1)
    app.run(host = '192.168.10.148', port = 5000, threaded = True)
    
    
