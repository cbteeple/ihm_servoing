#! /usr/bin/env python
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
import yaml
import errno
from datetime import datetime
import rosbag_recorder.srv as rbr
import video_recorder.srv as vrec

import sorotraj


class TrajSender:
    def __init__(self, pressure_server = "pressure_control"):
        self.pressure_server_name = pressure_server
        self.DEBUG = rospy.get_param(rospy.get_name()+"/DEBUG",False)
        self.save_data = rospy.get_param(rospy.get_name()+"/save_data",False)
        config = rospy.get_param(rospy.get_name()+"/config",None)
        settings = rospy.get_param(rospy.get_name()+"/settings",None)
        self.traj_spec  = {'config':config, 'settings':settings}

        self.controller_rate = float(rospy.get_param(rospy.get_name()+"/trajectory_rate",30))
        self.traj_name = rospy.get_param(rospy.get_name()+"/traj_name",'none.yaml').strip('.yaml')
        self.num_reps = rospy.get_param(rospy.get_name()+"/num_reps",None)
        self.speed_factor = rospy.get_param(rospy.get_name()+"/speed_factor",None)

        # Make a publisher to the desired pose topic
        pose_topic = rospy.get_param(rospy.get_name()+"/pose_topic",'/desired_pose')
        actual_pose_topic = rospy.get_param(rospy.get_name()+"/measured_pose_topic",'/measured_pose')
        self.pose_publisher = rospy.Publisher(pose_topic, PoseMsg, queue_size=10)

        # Set up some parameters
        self.init_pose = None
        self.init_time = None

        self.last_pose = None
        self.last_time = None

        self.desired_pose = None

        self.get_trajectory()

        if self.save_data:
            self.sh = SaveHandler()
            self.sh.save_camera=True
            self.sh.save_hand=True
            self.sh.save_traj=True
            self.sh.desired_pose_topic=pose_topic
            self.sh.actual_pose_topic=actual_pose_topic
            self.sh.DEBUG=self.DEBUG


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
        
        # Start Saving data
        if self.save_data:
            self.sh.createOutFolder(self.traj_name)
            self.sh.start_saving('rep_0')
            

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
        
        self.sh.stop_saving()


    def shutdown(self):
        if self.sh.saving_now:
            self.sh.start_saving()


class SaveHandler:
    def __init__(self, filepath_config = None):
        self.DEBUG=False
        if filepath_config is None:
            curr_path=os.path.dirname(os.path.abspath(__file__))
            filepath_config = os.path.join(curr_path,'..','config')
        self.save_data_folder = self.get_save_locations(os.path.join(filepath_config,'save_locations.yaml'))
        self.save_folder_curr = None
        self.save_camera = False
        self.save_hand = False
        self.save_tags = False
        self.save_traj = False
        self.saving_now = False
        self.desired_pose_topic = "/desired_pose"
        self.actual_pose_topic = "/measured_pose"

    def get_save_locations(self, config_filename):
        # Get save locations
        with open(config_filename,'r') as f:
            # use safe_load instead of load
            save_config = yaml.safe_load(f)
        
        # Try the user-specified directory
        data_folder_out = save_config.get('save_folder',None)
        directory_exists = True

        if data_folder_out is not None:
            if not os.path.exists(data_folder_out):
                try:
                    os.makedirs(data_folder_out)
                    directory_exists = True
                except OSError as e:
                    if e.errno != errno.EEXIST:
                        directory_exists = False

        if not directory_exists:
            if self.DEBUG:
                print('SAVE SETUP: Save directory did not exist. Trying default save directory')
            data_folder_out = save_config.get('save_folder_default',None)
            if data_folder_out is not None:
                if not os.path.exists(data_folder_out):
                    try:
                        os.makedirs(data_folder_out)
                        directory_exists = True
                    except OSError as e:
                        if e.errno != errno.EEXIST:
                            directory_exists = False
            
                else:
                    directory_exists = True

        
        if not directory_exists:
            print('ERROR: Please use an existing data directory in save_config.yaml')
            raise
        else:
            print('SAVE SETUP: Saving in: %s'%(data_folder_out))
        
        return(data_folder_out)
        

    def start_saving(self, out_filename):
        self.out_filename = self.createOutFile(out_filename)
        rospy.wait_for_service('rosbag_recorder/record_topics')

        if self.save_camera:
            rospy.wait_for_service('video_recorder/record')

        # generate the topic list
        topic_list = []
        if self.save_hand:
            topic_list.extend(['/pressure_control/echo','/pressure_control/pressure_data'])
        if self.save_traj:
            topic_list.extend([self.desired_pose_topic,self.actual_pose_topic])
        if self.save_tags:
            topic_list.extend(['/tag_detections'])

        try:
            service = rospy.ServiceProxy('rosbag_recorder/record_topics', rbr.RecordTopics)
            response = service(self.out_filename, topic_list)

            if self.save_camera:
                service = rospy.ServiceProxy('video_recorder/record', vrec.RecordVideo)
                response = service(self.out_filename.replace('.bag','.mp4'))

            self.saving_now = True
            return response.success
            
        except rospy.ServiceException, e:
            if self.DEBUG:
                print "Service call failed: %s"%e


    def stop_saving(self):
        try:
            service = rospy.ServiceProxy('rosbag_recorder/stop_recording', rbr.StopRecording)
            response = service(self.out_filename)

            if self.save_camera:
                service = rospy.ServiceProxy('video_recorder/stop_recording', vrec.StopRecording)
                response = service(self.out_filename.replace('.bag','.mp4'))
            
            self.saving_now = False
            return response.success
        except rospy.ServiceException, e:
            if self.DEBUG:
                print "Service call failed: %s"%e



    def createOutFolder(self,traj_name):
        now = datetime.now()
        self.save_folder_curr = os.path.join(traj_name, now.strftime("%Y%m%d_%H%M%S"))

        dirname = os.path.abspath(os.path.join(os.path.expanduser('~'),self.save_data_folder,self.save_folder_curr))
        if not os.path.exists(dirname):
            os.makedirs(dirname)

        


    def createOutFile(self,filename):

        outFile=os.path.abspath(os.path.join(os.path.expanduser('~'),self.save_data_folder,self.save_folder_curr,filename))
        #print(outFile)

        return "%s.bag" % (outFile)




if __name__ == '__main__':
    try:
        rospy.init_node('controller_node', disable_signals=True)
        node = TrajSender()
        node.run_trajectory()

    except rospy.ROSInterruptException:
        node.shutdown()
        print("program interrupted before completion")