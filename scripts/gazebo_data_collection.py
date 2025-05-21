#!/usr/bin/env python3
import math
import os.path
import time
import random
import rospy
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates,ModelState

from gazebo_msgs.srv import SetModelState
from squaternion import Quaternion
from std_msgs.msg import Float64,String
import tf
from geometry_msgs.msg import TransformStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np
import json
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from utils import rotate_matrix_coordinate,calculate_rotation_angle,detect_ball,check_distance
from parabola_fitter import *

class Command:
    def __init__(self,state="idle",vx=0,vy=0,vw=0,r1=0,r2=-90,r3=180,target_x=0,target_y=0,rv1=0,rv2=0,rv3=0):

        self.state = state
        self.vx = vx
        self.vy = vy
        self.vw = vw
        self.r1 = r1
        self.r2 = r2
        self.r3 = r3
        self.rv1 = rv1
        self.rv2 = rv2
        self.rv3 = rv3
        self.target_x = target_x
        self.target_y = target_y

    def encode(self):

        return json.dumps(self.__dict__)
    def set_attribute(self, **kwargs):
        self.state = kwargs.get('state')
        self.vx = float(kwargs.get('vx'))
        self.vy = float(kwargs.get('vy'))
        self.vw = float(kwargs.get('vw'))
        self.r1 = float(kwargs.get('r1'))
        self.r2 = float(kwargs.get('r2'))
        self.r3 = float(kwargs.get('r3'))
        self.rv1 = float(kwargs.get('rv1'))
        self.rv2 = float(kwargs.get('rv2'))
        self.rv3 = float(kwargs.get('rv3'))
        self.target_x = float(kwargs.get('target_x'))
        self.target_y = float(kwargs.get('target_y'))

    def decode(self,command_string):
        attribute_dict = json.loads(command_string)
        self.set_attribute(**attribute_dict)




class Robot:
    def __init__(self, robot_name,initial_state,state="idle"):

        self.robot_name=robot_name
        self.state=initial_state.state
        self.initial_state=initial_state

        # General Settings
        self.g = -9.81
        self.max_speed = 3
        self.camera_bias=[0.0,0,0.0]
        self.camera_rpy = [0, -0.785, 0.0]
        self.arm_pose = [-0.2, 0, 0.3]
        self.skip_frame=0
        self.min_frame_count=30

        self.save=False

        # Camera relatetd
        self.fx = 500
        self.fy = 500
        self.cx = 500
        self.cy = 500


        self.parabola_fitter=ParabolaFitter(camera_rpy_to=[0,45,-90],camera_rpy_back=[45,0,90],min_data_samples=self.min_frame_count)





        # Ros topics
        self.set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.command_topic = rospy.Subscriber(f'/{self.robot_name}/command', String, self.command_callback, queue_size=1)
        self.reset_topic = rospy.Subscriber(f'/{self.robot_name}/command', String, self.reset_callback,
                                              queue_size=1)

        self.target_topic = rospy.Subscriber(f'/{self.robot_name}/image_raw', Image, self.collecting_callback,
                                             queue_size=1)
        # self.target_topic = rospy.Subscriber("/gazebo/model_states", ModelStates, self.simulate_local_controller,
        #                                      queue_size=1)


        self.pub_arm = rospy.Publisher(f'/{self.robot_name}/Arm_position_controller/command',
                                              JointTrajectory,
                                              queue_size=1)

        self.chassis_publish_topic=rospy.Publisher(f'/{self.robot_name}/cmd_vel', Twist, queue_size=1)



        #### Memorys
        self.ball_memory=[]
        self.move_target=[0,0]
        self.frame_count=0
        #x,y,theta,t
        self.robot_pose=[0,0,0,0]
        self.robot_pose_initial = [0, 0, 0, 0]
        self.robot_pose_global = [0, 0, 0, 0]
        self.odometry=[]
        # vx,vy,omega
        self.move_msg = [0, 0, 0]
    def collecting_callback(self, data):
        present_time = rospy.Time.now().to_sec()
        time_step = present_time - self.robot_pose[3]

        # Camera Intrinsic
        x_pixel, y_pixel = detect_ball(data)
        # print(x_pixel,y_pixel)
        try:
            if x_pixel == -1:
                # self.ball_memory = []
                self.frame_count = 0
                self.robot_pose_initial = self.robot_pose_global
                self.robot_pose = [0, 0, 0, 0]
            elif not x_pixel == -1:
                self.frame_count += 1

                x_c = (x_pixel - self.cx) / self.fx
                y_c = (y_pixel - self.cy) / self.fy
                self.ball_memory.append([x_c,y_c,present_time])

        except rospy.ROSInterruptException:
            print("error")

    def reset_callback(self, data):
        # rospy.loginfo("message from topic%s: %s", self.robot_name, data.data)
        command = Command()
        command.decode(data.data)
        self.state = command.state
        if command.state=="reset":
            print("reset")
            self.save=True

            observations_dir = "/home/xinchi/monocular_fitting_dataset/observations"
            print(len(self.ball_memory))
            if os.path.isdir(observations_dir):
                number_of_files = len(os.listdir(observations_dir))
                np.save(os.path.join(observations_dir, f"{number_of_files}.npy"),
                        np.array(self.ball_memory))

            self.ball_memory=[]
            self.robot_pose = [0, 0, 0, rospy.Time.now().to_sec()]
            self.move_msg = [0, 0, 0]
            self.move_target = [0, 0]
            self.frame_count = 0
            self.robot_pose_initial = self.robot_pose_global
            # x = self.initial_state.x
            # y = self.initial_state.y
            if self.robot_name=="robot2":
                x = self.initial_state.x
                y = self.initial_state.y
                # self.state="catch"
            else:
                x = self.initial_state.x
                y = self.initial_state.y
            w = self.initial_state.w
            msg = Twist()
            msg.linear.x = 0
            msg.linear.y = 0
            msg.linear.z = 0
            self.chassis_publish_topic.publish(msg)

            trajectory = JointTrajectory()
            trajectory.joint_names = ['Revolute2','Revolute6','Revolute10']
            point = JointTrajectoryPoint()
            point.positions = [math.radians(self.initial_state.r1),math.radians(self.initial_state.r2),math.radians(self.initial_state.r3)]
            point.velocities = [self.initial_state.rv1,self.initial_state.rv2,self.initial_state.rv3]
            point.time_from_start = rospy.Duration(0.1)
            trajectory.points.append(point)
            self.pub_arm.publish(trajectory)

            state_msg = ModelState()
            state_msg.model_name = self.robot_name
            state_msg.pose.position.x=x
            state_msg.pose.position.y=y
            q = Quaternion.from_euler(0, 0, w, degrees=True)
            state_msg.pose.orientation.x = q.x
            state_msg.pose.orientation.y = q.y
            state_msg.pose.orientation.z = q.z
            state_msg.pose.orientation.w = q.w
            self.set_state(state_msg)



    def command_callback(self,data):
        # rospy.loginfo("message from topic%s: %s", self.robot_name, data.data)
        command = Command()
        command.decode(data.data)
        self.throw_target_pose = [command.target_x, command.target_y, 0]
        self.state=command.state
        trajectory = JointTrajectory()
        trajectory.joint_names = ['Revolute2', 'Revolute6', 'Revolute10']
        point = JointTrajectoryPoint()
        point.positions = [math.radians(command.r1), math.radians(command.r2),
                           math.radians(command.r3)]
        point.velocities = [command.rv1, command.rv2, command.rv3]
        point.time_from_start = rospy.Duration(0.1)
        trajectory.points.append(point)
        self.pub_arm.publish(trajectory)

class Ball:
    def __init__(self):
        self.set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.reset_topic= rospy.Subscriber(f'/ball/command', String, self.reset_callback, queue_size=1)
        self.landing_pose=[]
        self.record=True
        self.g=-9.81
    def reset_callback(self,data):
        command = Command()
        command.decode(data.data)
        # rospy.loginfo("message from topic%s: %s", "ball", data.data)
        # print(command.state)
        if command.state == "reset":
            state_msg = ModelState()
            state_msg.model_name = 'ball'
            x,y,z,vx,vy,vz = random.uniform(-5,-4),random.uniform(-0.5,0.5),random.uniform(1.5,2.5),random.uniform(6,9),random.uniform(-1,1),random.uniform(-1,1)
            state_msg.pose.position.x = x
            state_msg.pose.position.y = y
            state_msg.pose.position.z = z
            state_msg.twist.linear.x = vx
            state_msg.twist.linear.y = vy
            state_msg.twist.linear.z = vz
            self.set_state(state_msg)
            # rospy.sleep(1)
            ball_dir="/home/xinchi/monocular_fitting_dataset/ball"
            contact_t=-x/vx
            contact_y=y+contact_t*vy
            contact_z=z+contact_t*vz+0.5*self.g*contact_t**2
            if os.path.isdir(ball_dir):
                number_of_files=len(os.listdir(ball_dir))
                np.save(os.path.join(ball_dir,f"{number_of_files}.npy"),np.array([0,contact_y,contact_z,contact_t]))




if __name__ == "__main__":
    rospy.init_node("robot_lacrosse")


    initial_state1=Command()
    initial_state1.x = 0
    initial_state1.y = 0
    initial_state1.w = -180
    initial_state1.r1 = 0
    initial_state1.r2 = -90
    initial_state1.r3 = -180

    Robot("robot1",initial_state1)
    Ball()
    rospy.spin()
