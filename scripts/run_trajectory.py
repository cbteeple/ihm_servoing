#! /usr/bin/env python3
import rospy

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
from geometry_msgs.msg import Pose as PoseMsg
from geometry_msgs.msg import Quaternion, Point

from tf.transformations import euler_from_quaternion, quaternion_from_euler

import copy
import sys
import os
import numpy as np

import sorotraj


class TrajSender:
    def __init__(self, pressure_server = "pressure_control"):
        self.pressure_server_name = pressure_server
        self.DEBUG = rospy.get_param(rospy.get_name()+"/DEBUG",False)
        config = rospy.get_param(rospy.get_name()+"/config",None)
        settings = rospy.get_param(rospy.get_name()+"/settings",None)
        self.traj_spec  = {'config':config, 'settings':settings}

        self.controller_rate = float(rospy.get_param(rospy.get_name()+"/trajectory_rate",30))
        self.num_reps = rospy.get_param(rospy.get_name()+"/num_reps",None)
        self.speed_factor = rospy.get_param(rospy.get_name()+"/speed_factor",None)

        # Make a publisher to the desired pose topic
        pose_topic = rospy.get_param(rospy.get_name()+"/pose_topic",'/desired_pose')
        self.pose_publisher = rospy.Publisher(pose_topic, PoseMsg, queue_size=10)

        # Set up some parameters
        self.init_pose = None
        self.init_time = None

        self.last_pose = None
        self.last_time = None

        self.desired_pose = None

        self.get_trajectory()


    def get_trajectory(self):
        traj = sorotraj.TrajBuilder(graph=False)

        # Generate the trajectory based on the definition. This will eventually be possible in sorotraj directly
        traj.definition=self.traj_spec
        traj.settings = self.traj_spec.get("settings",None)
        traj.config = self.traj_spec.get("config",None)
        traj.traj_type = str(traj.settings.get("traj_type"))
        traj.subsample_num = traj.config.get("subsample_num")
        traj.go()
        trajectory = traj.get_trajectory()
        interp = sorotraj.Interpolator(trajectory)
        trajectory_fn = interp.get_interp_function(
                        num_reps = self.num_reps,
                        speed_factor = self.speed_factor,
                        invert_direction = False,
                        as_list = False)

        self.trajectory_fn = trajectory_fn
        self.final_time = interp.get_final_time()


    def publish_pose(self, pose_in):

        pos = pose_in[0:3]
        ori = pose_in[3:6]

        ori_quat = quaternion_from_euler(ori[0],ori[1],ori[2])
        pose_out=PoseMsg(position=Point(x=pos[0],y=pos[1],z=pos[2]),
                         orientation=Quaternion(x=ori_quat[0], y=ori_quat[1], z=ori_quat[2],w=ori_quat[3]),
                         )

        self.pose_publisher.publish(pose_out)


    def run_trajectory(self):
        # helper variables
        r = rospy.Rate(self.controller_rate)
        success = False
        
        # Send the pressure trajectory in real time
        start_time=rospy.Time.now()
        curr_time = rospy.Duration(0.0)

        idx=0

        while curr_time.to_sec() < self.final_time and not rospy.is_shutdown():
            # Interpolate point to send from the trajectory given
            curr_time = rospy.Time.now()-start_time

            if curr_time.to_sec() >= self.final_time: 
                break
            
            new_pose = self.trajectory_fn(curr_time.to_sec()).tolist()

            # Send pressure setpoint to the controller
            self.publish_pose(new_pose)

            r.sleep()
            idx += 1


    def shutdown(self):
        pass



if __name__ == '__main__':
    try:
        rospy.init_node('controller_node', disable_signals=True)
        node = TrajSender()
        node.run_trajectory()

    except rospy.ROSInterruptException:
        node.shutdown()
        print("program interrupted before completion")