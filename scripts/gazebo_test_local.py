#!/usr/bin/env python3
import math
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
        self.g = 10
        self.max_speed = 3
        self.camera_bias=[0.0,0,0.0]
        self.camera_rpy = [0, -0.785, 0.0]
        self.arm_pose = [-0.2, 0, 0.3]
        self.skip_frame=100
        self.min_frame_count=10

        self.save=False

        # Camera relatetd
        self.fx = 500.39832201574455
        self.fy = 500.39832201574455
        self.cx = 500.5
        self.cy = 500.5





        # Ros topics
        self.set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.command_topic = rospy.Subscriber(f'/{self.robot_name}/command', String, self.command_callback, queue_size=1)
        self.reset_topic = rospy.Subscriber(f'/{self.robot_name}/command', String, self.reset_callback,
                                              queue_size=1)
        self.move_topic = rospy.Subscriber("/gazebo/model_states", ModelStates, self.move_callback, queue_size=1)
        self.target_topic = rospy.Subscriber(f'/{self.robot_name}/image_raw', Image, self.local_controller,
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
        # vx,vy,omega
        self.move_msg = [0, 0, 0]



    def reset_callback(self, data):
        # rospy.loginfo("message from topic%s: %s", self.robot_name, data.data)
        command = Command()
        command.decode(data.data)
        self.state = command.state
        if command.state=="reset":
            print("reset")
            self.save=True

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

    def local_controller(self,data):

        present_time = rospy.Time.now().to_sec()
        time_step=present_time-self.robot_pose[3]

        # Camera Intrinsic
        x_pixel, y_pixel=detect_ball(data)
        try:
            if x_pixel==-1:
                self.ball_memory=[]
                self.frame_count=0
                self.robot_pose_initial = self.robot_pose_global
                self.robot_pose=[0,0,0,0]
            elif not x_pixel==-1:
                self.frame_count+=1
                # print(self.frame_count)

                self.robot_pose = [self.robot_pose[0]+self.move_msg[0]*time_step, self.robot_pose[1]+self.move_msg[1]*time_step,
                                   self.robot_pose[2]+self.move_msg[2]*time_step,self.robot_pose[3]+time_step]
                # robot_theta = self.robot_pose_global[2]
                # # print(math.degrees(robot_theta))
                # rotate_matrix = rotate_matrix_coordinate(0, 0, math.degrees(robot_theta))
                # robot_local = rotate_matrix @ np.array([self.robot_pose_global[0],self.robot_pose_global[1],0])
                # robot_local_initial = rotate_matrix @ np.array([self.robot_pose_initial[0],self.robot_pose_initial[1],0])
                # self.robot_pose = [robot_local[0] - robot_local_initial[0],
                #                    robot_local[1] - robot_local_initial[1],
                #                    0, self.robot_pose[3] + time_step]
                x_c = (x_pixel - self.cx) / self.fx
                y_c = (y_pixel - self.cy) / self.fy

                if self.frame_count>self.skip_frame:
                     self.ball_memory.append([x_c,y_c,present_time])
                if len(self.ball_memory)>self.min_frame_count:
                    # if len(self.ball_memory) >self.min_frame_count:
                    #     self.ball_memory.pop(0)
                    # print(len(self.ball_memory))
                    t_start=self.ball_memory[0][2]
                    g_x=0
                    g_y=self.g*math.cos(math.pi/4)
                    g_z=-self.g*math.sin(math.pi/4)
                    A=[]
                    B=[]
                    camera_angle=-self.camera_rpy[1]
                    for i in range(len(self.ball_memory)):
                        t=self.ball_memory[i][2] - t_start
                        # print(t)
                        # alpha=(self.ball_memory[i][0]+self.robot_pose[1])/(1+self.robot_pose[0]*math.cos(camera_angle))
                        # beta=(self.ball_memory[i][1]+self.robot_pose[0]*math.sin(camera_angle))/(1+self.robot_pose[0]*math.cos(camera_angle))
                        alpha=self.ball_memory[i][0]
                        beta=self.ball_memory[i][1]
                        A.append([1, t, 0, 0, -alpha,-alpha * t])
                        A.append([0, 0, 1, t, -beta,-beta * t])
                        # B.append([0.5 * t * t * (g_z * alpha - g_x)])
                        # B.append([0.5 * t * t * (g_z * beta - g_y)])
                        B.append([0.5 * t * t * (g_z * alpha - g_x)+(-self.robot_pose[0]*math.cos(math.pi/4)*alpha+self.robot_pose[1])])
                        B.append([0.5 * t * t * (g_z * beta - g_y)+(-self.robot_pose[0]*math.cos(math.pi/4)*beta)-(-self.robot_pose[0]*math.sin(math.pi/4))])


                    A_array = np.array(A)
                    B_array = np.array(B)
                    pseudo_inverse_A = np.linalg.pinv(A_array)
                    solved_parameters= pseudo_inverse_A@B_array

                    x0_c,vx_c,y0_c,vy_c,z0_c,vz_c = solved_parameters[0][0],solved_parameters[1][0],solved_parameters[2][0],solved_parameters[3][0],solved_parameters[4][0],solved_parameters[5][0]

                    rotate_matrix=rotate_matrix_coordinate(45,0,90)
                    ball_position_c=np.array([x0_c,y0_c,z0_c])
                    ball_velocity_c=np.array([vx_c,vy_c,vz_c])
                    ball_position_w=rotate_matrix@ball_position_c
                    ball_velocity_w=rotate_matrix@ball_velocity_c
                    x_ball_w,y_ball_w,z_ball_w=ball_position_w[0],ball_position_w[1],ball_position_w[2]
                    vx_ball_w,vy_ball_w,vz_ball_w=ball_velocity_w[0],ball_velocity_w[1],ball_velocity_w[2]
                    x_ball_w=x_ball_w+self.camera_bias[0]
                    z_ball_w=z_ball_w+self.camera_bias[2]
                    target_x = 0
                    target_y = 0

                    if vz_ball_w**2 - (z_ball_w-self.arm_pose[2]) * (-self.g)*2 >0:
                        drop_t = (-vz_ball_w - math.sqrt(vz_ball_w**2 - (z_ball_w-self.arm_pose[2]) * (-self.g)*2 )) / (-self.g)
                        target_x = x_ball_w + drop_t * vx_ball_w - self.arm_pose[0]
                        target_y = y_ball_w + drop_t * vy_ball_w - self.arm_pose[1]
                    distance_x = target_x - self.robot_pose[0]
                    distance_y = target_y - self.robot_pose[1]
                    vx= min(abs(distance_x)*1,1)*self.max_speed*(distance_x)/abs(distance_x)
                    vy= min(abs(distance_y)*1,1)*self.max_speed*(distance_y)/abs(distance_y)
                    self.move_msg=[vx,vy,0]

                    # if self.robot_name == "robot2":
                        # print(solved_parameters.T)
                        # print("velocity",[vx, vy, 0])
                        # print(self.ball_memory[-1])
                        # print(self.ball_memory[-1][2] - t_start)
                        # print(x_pixel, y_pixel)
                        # print(target_x+self.arm_pose[0],target_y+self.arm_pose[1])
                        # print(ball_velocity_w)
                        # print(self.robot_pose)
                        # print(self.robot_pose_global)
                        # print(self.robot_pose_initial)

        except:
            self.move_msg = [0, 0, 0]
    def simulate_local_controller(self,data):

        present_time = rospy.Time.now().to_sec()
        time_step = present_time - self.robot_pose[3]
        self.robot_pose = [self.robot_pose[0] + self.move_msg[0] * time_step,
                           self.robot_pose[1] + self.move_msg[1] * time_step,
                           self.robot_pose[2] + self.move_msg[2] * time_step, self.robot_pose[3] + time_step]
        robot_theta = self.robot_pose_global[2]
        # print(math.degrees(robot_theta))
        # rotate_matrix = rotate_matrix_coordinate(0, 0, math.degrees(robot_theta))
        # robot_local = rotate_matrix @ np.array([self.robot_pose_global[0],self.robot_pose_global[1],0])
        # robot_local_initial = rotate_matrix @ np.array([self.robot_pose_initial[0],self.robot_pose_initial[1],0])
        # self.robot_pose = [robot_local[0] - robot_local_initial[0],
        #                    robot_local[1] - robot_local_initial[1],
        #                    0, self.robot_pose[3] + time_step]
        # Camera Intrinsic
        # x_pixel, y_pixel = detect_ball(data)

        model_dict = {}
        for i in range(len(data.name)):
            model_dict[data.name[i]] = i
        robot_pose = data.pose[model_dict[self.robot_name]]
        robot_position = robot_pose.position
        ball_position = data.pose[model_dict['ball']].position

        q = Quaternion(robot_pose.orientation.x, robot_pose.orientation.y,
                       robot_pose.orientation.z, robot_pose.orientation.w)
        theta = q.to_euler(degrees=True)[0]
        relative_x_w = ball_position.x - robot_position.x
        relative_y_w = ball_position.y - robot_position.y
        relative_z_w = ball_position.z - robot_position.z
        relative_w=np.array([relative_x_w, relative_y_w, relative_z_w])
        # print(theta)
        relative_c=rotate_matrix_coordinate(0, 0, theta)@relative_w
        relative_c=np.array([relative_c[0]-self.camera_bias[0], relative_c[1]-self.camera_bias[1], relative_c[2]-self.camera_bias[2]])
        # print(relative_c)
        camera_coordinates_w= rotate_matrix_coordinate(0,45,-90)@relative_c
        f=camera_coordinates_w[2]
        # print(f)
        camera_coordinates_c=[camera_coordinates_w[0]/f,camera_coordinates_w[1]/f,camera_coordinates_w[2]/f]




        try:
            if relative_c[2] < 0.5 or relative_c[0]<0:
                self.ball_memory = []
                self.frame_count = 0
                self.robot_pose_initial = self.robot_pose_global
                # self.robot_pose = [0, 0, 0, 0]
            else:
                self.frame_count += 1
                # print(self.frame_count)
                # robot_theta = self.robot_pose_global[2]
                # # print(math.degrees(robot_theta))
                # rotate_matrix = rotate_matrix_coordinate(0, 0, math.degrees(robot_theta))
                # robot_local = rotate_matrix @ np.array([self.robot_pose_global[0],self.robot_pose_global[1],0])
                # robot_local_initial = rotate_matrix @ np.array([self.robot_pose_initial[0],self.robot_pose_initial[1],0])
                # self.robot_pose = [robot_local[0] - robot_local_initial[0],
                #                    robot_local[1] - robot_local_initial[1],
                #                    0, self.robot_pose[3] + time_step]

                x_c=camera_coordinates_c[0]
                y_c=camera_coordinates_c[1]
                # self.ball_memory.append([x_c, y_c, present_time])
                if self.frame_count>self.skip_frame and self.frame_count%10==0 and self.state=="catch":
                    self.ball_memory.append([x_c,y_c,present_time])
                    if len(self.ball_memory) > self.min_frame_count:
                        # if len(self.ball_memory) >self.min_frame_count+20:
                        #     self.ball_memory.pop(0)
                        # print(len(self.ball_memory))
                        t_start = self.ball_memory[0][2]
                        g_x = 0
                        g_y = self.g * math.cos(math.pi / 4)
                        g_z = -self.g * math.sin(math.pi / 4)
                        A = []
                        B = []
                        camera_angle = -self.camera_rpy[1]
                        for i in range(1,len(self.ball_memory)):
                            t = self.ball_memory[i][2] - t_start
                            # print(t)
                            # alpha=(self.ball_memory[i][0]+self.robot_pose[1])/(1+self.robot_pose[0]*math.cos(camera_angle))
                            # beta=(self.ball_memory[i][1]+self.robot_pose[0]*math.sin(camera_angle))/(1+self.robot_pose[0]*math.cos(camera_angle))
                            alpha = self.ball_memory[i][0]
                            beta = self.ball_memory[i][1]
                            A.append([1, t, 0, 0, -alpha, -alpha * t])
                            A.append([0, 0, 1, t, -beta, -beta * t])
                            # B.append([0.5 * t * t * (g_z * alpha - g_x)])
                            # B.append([0.5 * t * t * (g_z * beta - g_y)])
                            B.append([0.5 * t * t * (g_z * alpha - g_x) + (
                                        -self.robot_pose[0] * math.cos(math.pi / 4) * alpha + self.robot_pose[1])])
                            B.append([0.5 * t * t * (g_z * beta - g_y) + (
                                        -self.robot_pose[0] * math.cos(math.pi / 4) * beta) - (
                                                  -self.robot_pose[0] * math.sin(math.pi / 4))])

                        A_array = np.array(A)
                        B_array = np.array(B)
                        pseudo_inverse_A = np.linalg.pinv(A_array)
                        solved_parameters = pseudo_inverse_A @ B_array

                        x0_c, vx_c, y0_c, vy_c, z0_c, vz_c = solved_parameters[0][0], solved_parameters[1][0], \
                        solved_parameters[2][0], solved_parameters[3][0], solved_parameters[4][0], solved_parameters[5][0]

                        rotate_matrix = rotate_matrix_coordinate(45, 0, 90)
                        ball_position_c = np.array([x0_c, y0_c, z0_c])
                        ball_velocity_c = np.array([vx_c, vy_c, vz_c])
                        ball_position_w = rotate_matrix @ ball_position_c
                        ball_velocity_w = rotate_matrix @ ball_velocity_c
                        x_ball_w, y_ball_w, z_ball_w = ball_position_w[0], ball_position_w[1], ball_position_w[2]
                        vx_ball_w, vy_ball_w, vz_ball_w = ball_velocity_w[0], ball_velocity_w[1], ball_velocity_w[2]
                        x_ball_w = x_ball_w + self.camera_bias[0]
                        z_ball_w = z_ball_w + self.camera_bias[2]
                        target_x = 0
                        target_y = 0
                        if vz_ball_w ** 2 - (z_ball_w - self.arm_pose[2]) * (-self.g) * 2 > 0:
                            drop_t = (-vz_ball_w - math.sqrt(
                                vz_ball_w ** 2 - (z_ball_w - self.arm_pose[2]) * (-self.g) * 2)) / (-self.g)
                            target_x = x_ball_w + drop_t * vx_ball_w - self.arm_pose[0]
                            target_y = y_ball_w + drop_t * vy_ball_w - self.arm_pose[1]
                        self.move_target=[target_x,target_y]

                        if self.robot_name == "robot1":
                            # print(ball_position_c)
                            # print(self.robot_pose)
                            print(self.frame_count,len(self.ball_memory),self.move_target)


            distance_x = self.move_target[0] - self.robot_pose[0]
            distance_y = self.move_target[1] - self.robot_pose[1]
            vx = min(abs(distance_x) * 1, 1) * self.max_speed * (distance_x) / abs(distance_x)
            vy = min(abs(distance_y) * 1, 1) * self.max_speed * (distance_y) / abs(distance_y)
            # if self.robot_name == "robot1":
                # print(x_ball_w, y_ball_w, z_ball_w)
                # print(relative_w)
                # print([relative_x_w, relative_y_w,relative_z_w])
                # print("velocity",[vx, vy, 0])
                # print(self.ball_memory[-1])
                # print(self.ball_memory[-1][2] - t_start)
                # print(x_pixel, y_pixel)
                # print(camera_coordinates_c)
                # print(len(self.ball_memory))
                # print(self.ball_memory[-1])
                # print(target_x,target_y)
                # print(ball_velocity_w)
                # print(self.robot_pose)
                # print(self.robot_pose_global)
                # print(self.robot_pose_initial)
                # print(self.robot_pose)
                # print(self.move_target)
                # print(self.move_msg)
            self.move_msg = [vx, vy, 0]

        except:
            self.move_msg = [0, 0, 0]
    def move_callback(self,data):
        model_dict = {}
        for i in range(len(data.name)):
            model_dict[data.name[i]] = i
        robot_pose=data.pose[model_dict[self.robot_name]]
        robot_position = robot_pose.position
        ball_position = data.pose[model_dict['ball']].position

        q = Quaternion(robot_pose.orientation.x, robot_pose.orientation.y,
                       robot_pose.orientation.z, robot_pose.orientation.w)
        theta=q.to_euler(degrees=False)[0]

        self.robot_pose_global = [robot_position.x, robot_position.y, theta,0]

        msg = Twist()

        if self.state == "catch" and ball_position.z>self.arm_pose[2]:

            msg.linear.x=self.move_msg[0]
            msg.linear.y=self.move_msg[1]
            msg.linear.z = 0
            msg.angular.z = 0
        elif self.state == "rotate":
            self.ball_memory = []
            self.robot_pose = [0, 0, 0, 0]
            self.move_msg = [0, 0, 0]
            target_direction = [self.throw_target_pose[0] - robot_position.x,
                                self.throw_target_pose[1] - robot_position.y]
            self_direction = [math.cos(theta), math.sin(theta)]
            d_theta = calculate_rotation_angle(self_direction, target_direction)
            msg.angular.z = 5*d_theta
            msg.linear.x = 0
            msg.linear.y = 0
            msg.linear.z = 0
        else:
            msg.linear.x = 0
            msg.linear.y = 0
            msg.linear.z = 0
            msg.angular.z=0


        self.chassis_publish_topic.publish(msg)
class Ball:
    def __init__(self):
        self.set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.reset_topic= rospy.Subscriber(f'/ball/command', String, self.reset_callback, queue_size=1)
        self.record_topic = rospy.Subscriber("/gazebo/model_states", ModelStates, self.ball_recorder, queue_size=1)
        self.landing_pose=[]
        self.record=True
    def reset_callback(self,data):
        command = Command()
        command.decode(data.data)
        # rospy.loginfo("message from topic%s: %s", "ball", data.data)
        # print(command.state)
        if command.state == "reset":
            state_msg = ModelState()
            state_msg.model_name = 'ball'
            state_msg.pose.position.x = 2.7
            state_msg.pose.position.y = 0
            state_msg.pose.position.z = 0.5
            state_msg.twist.linear.x = 0.0
            state_msg.twist.linear.y = 0.0
            state_msg.twist.linear.z = 0.0
            self.set_state(state_msg)
            # rospy.sleep(1)

    def ball_recorder(self, data):

        model_dict = {}
        for i in range(len(data.name)):
            model_dict[data.name[i]] = i

        robot1_pose = data.pose[model_dict['robot1']]
        robot2_pose = data.pose[model_dict['robot2']]
        robot1_position = robot1_pose.position
        robot2_position = robot2_pose.position
        ball_position = data.pose[model_dict['ball']].position
        # # if ball_position.z < 0.290073:
        # print(ball_position.z)

        if check_distance(robot2_position, ball_position) >= 1:
            self.record=True
        if check_distance(robot2_position, ball_position) < 1 and ball_position.z < 0.3:
            if self.record==True:
                print(ball_position.z)
                self.record=False
                q = Quaternion(robot2_pose.orientation.x, robot2_pose.orientation.y,
                               robot2_pose.orientation.z, robot2_pose.orientation.w)
                theta = q.to_euler(degrees=False)[0]
                self.landing_pose.append(
                    [ball_position.x, ball_position.y, ball_position.z, robot2_position.x, robot2_position.y, theta])
                if len(self.landing_pose) % 20 == 0:
                    np.save("saved_pose.npy", self.landing_pose)




if __name__ == "__main__":
    rospy.init_node("robot_lacrosse")


    initial_state1=Command()
    initial_state1.x = 0
    initial_state1.y = 0
    initial_state1.w = 0
    initial_state1.r1 = 0
    initial_state1.r2 = -90
    initial_state1.r3 = -180
    initial_state2=Command()
    initial_state2.x =2.5
    initial_state2.y = 0
    initial_state2.w = 180
    initial_state2.r1 = 0
    initial_state2.r2 = -90
    initial_state2.r3 = -180
    Robot("robot1",initial_state1)
    Robot("robot2",initial_state2)
    Ball()
    rospy.spin()
