#! /usr/bin/env python
import rospy

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
from pressure_controller_ros.msg import CommandAction, CommandGoal
from geometry_msgs.msg import Pose as PoseMsg
from geometry_msgs.msg import Quaternion, Point
from apriltag_ros.msg import AprilTagDetectionArray

from tf.transformations import euler_from_quaternion, quaternion_from_euler

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

        self.num_channels = rospy.get_param('/pressure_control/num_channels',[])
        self.num_channels_total = sum(self.num_channels)

        for key in self.params:
            self.params[key] = np.array(self.params[key])

        # Connect to the pressure controller command server
        self.command_client = actionlib.SimpleActionClient(self.pressure_server_name, CommandAction)
        self.command_client.wait_for_server()
        self.set_data_stream(True)

        # Connect a callback function to the desired pose topic
        pose_topic = rospy.get_param(rospy.get_name()+"/pose_topic",'/desired_pose')
        measured_pose_topic = rospy.get_param(rospy.get_name()+"/measured_pose_topic",'/measured_pose')
        rospy.Subscriber(pose_topic, PoseMsg, self.set_desired_pose)

        self.pose_publisher = rospy.Publisher(measured_pose_topic, PoseMsg, queue_size=10)

        # Connect a callback function to the tag detection topic
        tag_topic = rospy.get_param(rospy.get_name()+"/tag_topic",None)
        rospy.Subscriber(tag_topic, AprilTagDetectionArray, self.tag_callback)

        # Set up some parameters
        self.init_pose = None
        self.init_time = None

        self.last_pose = None
        self.last_time = None

        self.desired_pose = None
        self.is_shutdown=False
        self.is_init = False

    def set_desired_pose(self, pose):
        if self.is_shutdown:
            return

        pos_raw = pose.position
        ori_raw = pose.orientation

        pos = np.array([pos_raw.x, pos_raw.y, pos_raw.z])
        ori = euler_from_quaternion([ori_raw.x, ori_raw.y, ori_raw.z, ori_raw.w])
        ori = np.array(ori)

        self.desired_pose = {'position':pos, 'orientation':ori}



    def compute_step(self, curr_time, curr_pose):

        if self.desired_pose is None:
            rospy.loginfo("No desired pose detected yet")
            return None

        
        pos_error = curr_pose['position'] - self.desired_pose['position']
        ori_error = curr_pose['orientation'] - self.desired_pose['orientation']

        print(pos_error)
        print(ori_error)

        time_diff = curr_time - self.last_time

        #pos_diff = curr_pose['position'] - self.last_pose['position']
        #ori_diff = curr_pose['orientation'] - self.last_pose['orientation']

        print(time_diff)

        # Compute control
        pressures = [0]*self.num_channels_total

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

        if isinstance(pressures, np.ndarray):
            pressures = pressures.tolist()

        psend=[transition_time]+pressures
        
        self.send_command('set', psend)


    def set_data_stream(self, value):
        if value:
            self.send_command('on', [])
        else:
            self.send_command('off', [])



    def send_command(self,cmd, args, wait_for_ack=False):
        goal = CommandGoal(command=cmd, args=args, wait_for_ack = wait_for_ack)
        self.command_client.send_goal(goal)
        self.command_client.wait_for_result()


    def publish_pose(self, pose_in):
        pos = pose_in['position'].tolist()
        ori = pose_in['orientation'].tolist()

        ori_quat = quaternion_from_euler(ori[0],ori[1],ori[2])
        pose_out=PoseMsg(position=Point(x=pos[0],y=pos[1],z=pos[2]),
                         orientation=Quaternion(x=ori_quat[0], y=ori_quat[1], z=ori_quat[2],w=ori_quat[3]),
                         )

        self.pose_publisher.publish(pose_out)

    def tag_callback(self, msg):
        if self.is_shutdown:
            return

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


        self.publish_pose(pose_rel)

        # Calculate Control
        pressures = self.compute_step(time_rel, pose_rel)
        if pressures is not None:
            #print(pressures)
            self.send_setpoint(pressures)
        elif not self.is_init:
            rest=self.params.get('rest',None)
            if rest is not None:
                self.send_setpoint(rest,1.0)
            self.is_init=True


    def shutdown(self):
        self.is_shutdown=True
        print('Setting all pressures to zero')
        self.send_setpoint([0]*self.num_channels_total, 2)
        time.sleep(2.2)
        print('Turning off data stream')
        self.set_data_stream(False)
        time.sleep(0.5)
        
        #self.command_client.cancel_all_goals()



if __name__ == '__main__':
    try:
        rospy.init_node('controller_node', disable_signals=False)
        node = Controller()
        rospy.on_shutdown(node.shutdown)
        rospy.spin()
    except:
        raise

    # except KeyboardInterrupt:
    #     print("Keyboard Interrup Triggered")
    #     node.shutdown()

    # except rospy.ROSInterruptException:
    #     print("ROS is shutting down")
    #     node.shutdown()
        