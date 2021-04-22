#! /usr/bin/env python3
import rospy

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
from pressure_controller_ros.msg import CommandAction
from geometry_msgs.msg import Pose as PoseMsg
from apriltag_ros.msg import AprilTagDetectionArray

from tf.transformations import euler_from_quaternion

import time
import copy
import sys
import os
import numpy as np

#import sorotraj


class Controller:
    def __init__(self, pressure_server = "pressure_control"):
        self.pressure_server_name = pressure_server
        self.DEBUG = rospy.get_param(rospy.get_name()+"/DEBUG",False)
        self.config = rospy.get_param(rospy.get_name()+"/controller",None)
        self.type = self.config.get('type',None)
        self.params = self.config.get('parameters',None)
        self.controller_rate = float(rospy.get_param(rospy.get_name()+"/controller_rate",30))

        for key in self.params:
            self.params[key] = np.array(self.params[key])

        # Connect to the pressure controller command server
        #self.command_client = actionlib.SimpleActionClient(self.pressure_server_name, CommandAction)
        #self.command_client.wait_for_server()

        # Connect a callback function to the desired pose topic
        pose_topic = rospy.get_param(rospy.get_name()+"/pose_topic",'desired_pose')
        rospy.Subscriber(pose_topic, PoseMsg, self.set_desired_pose)

        # Connect a callback function to the tag detection topic
        tag_topic = rospy.get_param(rospy.get_name()+"/tag_topic",None)
        rospy.Subscriber(tag_topic, AprilTagDetectionArray, self.tag_callback)

        # Set up some parameters
        self.init_pose = None
        self.init_time = None

        self.last_pose = None
        self.last_time = None

        self.desired_pose = None


    def set_desired_pose(self, pose):

        pos_raw = pose.position
        ori_raw = pose.orientation

        pos = np.array([pos_raw.x, pos_raw.y, pos_raw.z])
        ori = euler_from_quaternion([ori_raw.x, ori_raw.y, ori_raw.z, ori_raw.w])
        ori = np.array(ori)

        self.desired_pose = {'position':pos, 'orientation':ori}



    def compute_step(self, curr_time, curr_pose):

        if self.desired_pose is None:
            rospy.loginfo("No desired pose detected yet")
            return

        
        pos_error = curr_pose['position'] - self.desired_pose['position']
        ori_error = curr_pose['orientation'] - self.desired_pose['orientation']

        print(pos_error)
        print(ori_error)

        time_diff = curr_time - self.last_time

        #pos_diff = curr_pose['position'] - self.last_pose['position']
        #ori_diff = curr_pose['orientation'] - self.last_pose['orientation']

        print(time_diff)

        # Compute control
        pressures = [0,0,0,0,0,0,0,0]

        if 'pid' in self.type:
            p = self.params['p']
            i = self.params['i']
            d = self.params['d']
            # Run 1 step of PID

        maxp = self.params['maxp']
        minp = self.params['minp']


        if 'planar' in self.type:
            x_trans = self.params['x_trans']
            y_trans = self.params['y_trans']
            z_rot   = self.params['z_rot']
            rest    = self.params['rest']

            xt_comp = pos_error[0]*p[0]*(x_trans-rest)
            yt_comp = pos_error[1]*p[1]*(y_trans-rest)

            zr_comp = ori_error[2]*p[5]*(z_rot-rest)

            all_press=np.array([xt_comp, yt_comp, zr_comp])
            pressures_raw = np.mean(all_press,axis=0)+rest

            # Clip pressures to thier min and max values
            pressures = np.min([pressures_raw,maxp], axis=0)
            pressures = np.max([pressures,minp], axis=0)


        self.last_time = curr_time
        return pressures

            

    def send_setpoint(self, pressures, transition_time=None):

        if transition_time is None:
            transition_time = 1/self.controller_rate
        
        goal = CommandGoal(command='set', args=[transition_time]+pressures, wait_for_ack = False)
        self.command_client.send_goal(goal)
        self.command_client.wait_for_result()


    def tag_callback(self, msg):
        detections = msg.detections

        # If no detection, do nothing
        if len(detections)==0:
            return

        # If there's a detection, get the data

        time = msg.header.stamp.to_sec()
        detection = detections[0]

        pose = detection.pose.pose.pose

        pos_raw = pose.position
        ori_raw = pose.orientation

        pos = np.array([pos_raw.x, pos_raw.y, pos_raw.z])
        ori = euler_from_quaternion([ori_raw.x, ori_raw.y, ori_raw.z, ori_raw.w])
        ori = np.array(ori)

        pose={'position':pos, 'orientation':ori}

        # Store the initial pose on the first detection event
        if self.init_pose is None:
            self.init_pose = copy.deepcopy(pose)
            self.init_time = copy.deepcopy(time)

            self.last_pose = copy.deepcopy(pose)
            self.last_time = copy.deepcopy(time)
            return

        # Calculate relative time and pose
        time_rel = time - self.init_time

        pose_rel = {}
        for key in pose:
            pose_rel[key] = pose[key] - self.init_pose[key]


        # Calculate Control
        pressures = self.compute_step(time_rel, pose_rel)
        print(pressures)
        #self.send_setpoint(pressures)


    def shutdown(self):
        #self.command_client.cancel_all_goals()
        pass



if __name__ == '__main__':
    try:
        rospy.init_node('controller_node', disable_signals=True)
        node = Controller()
        rospy.spin()

    except rospy.ROSInterruptException:
        node.shutdown()
        print("program interrupted before completion")