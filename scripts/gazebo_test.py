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
def calculate_rotation_angle(v1, v2):
    # 计算点积
    dot_product = np.dot(v1, v2)
    # 计算向量的模
    norm_v1 = np.linalg.norm(v1)
    norm_v2 = np.linalg.norm(v2)
    # 计算余弦值
    cos_theta = dot_product / (norm_v1 * norm_v2)
    # 计算叉积的 z 分量（二维向量叉积的结果是一个标量）
    cross_product = v1[0] * v2[1] - v1[1] * v2[0]
    # 使用 atan2 计算角度（结果在 -π 到 π 之间，对应 -180° 到 180°）
    angle = np.arctan2(cross_product, cos_theta)

    return angle

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
        self.set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.command_topic = rospy.Subscriber(f'/{self.robot_name}/command', String, self.command_callback, queue_size=1)
        self.reset_topic = rospy.Subscriber(f'/{self.robot_name}/command', String, self.reset_callback,
                                              queue_size=1)
        self.move_topic = rospy.Subscriber("/gazebo/model_states", ModelStates, self.move_callback, queue_size=1)


        # self.move_topic=rospy.Subscriber("/gazebo/model_states", ModelStates, self.move_callback, queue_size=1)
        # self.pub_revolute2 = rospy.Publisher(f'/{self.robot_name}/Revolute2_position_controller/command', Float64, queue_size=1)
        # self.pub_revolute6 = rospy.Publisher(f'/{self.robot_name}/Revolute6_position_controller/command', Float64, queue_size=1)
        # self.pub_revolute10 = rospy.Publisher(f'/{self.robot_name}/Revolute10_position_controller/command', Float64, queue_size=1)

        # self.pub_revolute2 = rospy.Publisher(f'/{self.robot_name}/Revolute2_position_controller/command', JointTrajectory,
        #                                      queue_size=1)
        # self.pub_revolute6 = rospy.Publisher(f'/{self.robot_name}/Revolute6_position_controller/command', JointTrajectory,
        #                                      queue_size=1)
        # self.pub_revolute10 = rospy.Publisher(f'/{self.robot_name}/Revolute10_position_controller/command', JointTrajectory,
        #                                       queue_size=1)
        self.pub_arm = rospy.Publisher(f'/{self.robot_name}/Arm_position_controller/command',
                                              JointTrajectory,
                                              queue_size=1)

        self.chassis_publish_topic=rospy.Publisher(f'/{self.robot_name}/cmd_vel', Twist, queue_size=1)

        self.g = 10
        self.max_speed = 1.5
        self.arm_pose = [-0.19, 0, 0.42]
    def reset_callback(self, data):
        rospy.loginfo("message from topic%s: %s", self.robot_name, data.data)
        command = Command()
        command.decode(data.data)
        self.state = command.state
        if command.state=="reset":
            # x = self.initial_state.x
            # y = self.initial_state.y
            if self.robot_name=="robot2":
                x = random.random()*5
                y = random.random()*5
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
        rospy.loginfo("message from topic%s: %s", self.robot_name, data.data)
        command = Command()
        command.decode(data.data)
        self.throw_target_pose = [command.target_x, command.target_y, 0]
        self.state=command.state
        # msg = Twist()
        # msg.linear.x = command.vx
        # msg.linear.y = command.vy
        # msg.angular.z = command.vw
        # self.chassis_publish_topic.publish(msg)

        trajectory = JointTrajectory()
        trajectory.joint_names = ['Revolute2', 'Revolute6', 'Revolute10']
        point = JointTrajectoryPoint()
        point.positions = [math.radians(command.r1), math.radians(command.r2),
                           math.radians(command.r3)]
        point.velocities = [command.rv1, command.rv2, command.rv3]
        point.time_from_start = rospy.Duration(0.1)
        trajectory.points.append(point)
        self.pub_arm.publish(trajectory)
    def move_callback(self,data):
        model_dict = {}
        for i in range(len(data.name)):
            model_dict[data.name[i]] = i
        robot_pose=data.pose[model_dict[self.robot_name]]
        robot_position = robot_pose.position
        ball_position = data.pose[model_dict['ball']].position
        ball_velocity = data.twist[model_dict['ball']].linear


        q = Quaternion(robot_pose.orientation.x, robot_pose.orientation.y,
                       robot_pose.orientation.z, robot_pose.orientation.w)
        theta=q.to_euler(degrees=False)[0]

        msg = Twist()
        if self.state == "catch" and ball_position.z>self.arm_pose[2]:
            t = (ball_velocity.z + math.sqrt(
                ball_velocity.z * ball_velocity.z + (ball_position.z - self.arm_pose[2]) * self.g * 2)) / self.g
            target_x = ball_position.x + t * ball_velocity.x
            target_y = ball_position.y + t * ball_velocity.y
            self.catch_target_pose = [target_x, target_y, 0]



            bias_x = math.cos(-theta) * self.arm_pose[0]
            bias_y = -math.sin(-theta) * self.arm_pose[0]
            target_x = self.catch_target_pose[0]
            target_y = self.catch_target_pose[1]
            robot_x = robot_position.x
            robot_y = robot_position.y
            distance_x = target_x - robot_x - bias_x
            distance_y = target_y - robot_y - bias_y

            vx= min(abs(distance_x)*10,1)*self.max_speed*(distance_x)/abs(distance_x)
            vy= min(abs(distance_y)*10,1)*self.max_speed*(distance_y)/abs(distance_y)
            x_relative = math.cos(theta) * vx + math.sin(theta) * vy
            y_relative = -math.sin(theta) * vx + math.cos(theta) * vy
            msg.linear.x=x_relative
            msg.linear.y=y_relative
            msg.linear.z = 0
            msg.angular.z = 0
        elif self.state == "rotate":
            target_direction = [self.throw_target_pose[0] - robot_position.x,
                                self.throw_target_pose[1] - robot_position.y]
            self_direction = [math.cos(theta), math.sin(theta)]
            d_theta = calculate_rotation_angle(self_direction, target_direction)
            msg.angular.z = 2*d_theta
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
    def reset_callback(self,data):
        command = Command()
        command.decode(data.data)
        rospy.loginfo("message from topic%s: %s", "ball", data.data)
        print(command.state)
        if command.state == "reset":
            state_msg = ModelState()
            state_msg.model_name = 'ball'
            state_msg.pose.position.x = -0.2
            state_msg.pose.position.y = 0
            state_msg.pose.position.z = 0.6
            state_msg.twist.linear.x = 0.0
            state_msg.twist.linear.y = 0.0
            state_msg.twist.linear.z = 0.0
            self.set_state(state_msg)
            # rospy.sleep(1)





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
    initial_state2.x = random.random()*5
    initial_state2.y = random.random()*5
    initial_state2.w = 180
    initial_state2.r1 = 0
    initial_state2.r2 = -90
    initial_state2.r3 = -180
    Robot("robot1",initial_state1)
    Robot("robot2",initial_state2)
    Ball()
    rospy.spin()
