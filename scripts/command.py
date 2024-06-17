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
from gazebo_test import Command
import numpy as np

def recorder(data,save_file="saved_pose.npy"):
    pose_list=[]
    model_dict = {}
    for i in range(len(data.name)):
        model_dict[data.name[i]] = i
    ball_position = data.pose[model_dict['ball']].position
    robot1_position = data.pose[model_dict['robot1']].position
    robot2_position = data.pose[model_dict['robot2']].position
    pose_list.append([ball_position.x,ball_position.y,robot2_position.x,robot2_position.y])

    np.append(save_file,np.array(pose_list))



def check_distance(position1,position2):
    return ((position1.x-position2.x)**2+(position1.y-position2.y)**2+(position1.z-position2.z)**2)**0.5

class Simulate:
    def __init__(self):
        self.set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        # self.ball_topic = rospy.Subscriber("/gazebo/model_states", ModelStates, self.reset_callback, queue_size=1)
        # self.throw_callback=rospy.Subscriber("/gazebo/model_states", ModelStates, self.throw_manager_callback, queue_size=1)
        self.timer_callback = rospy.Subscriber("/gazebo/model_states", ModelStates, self.timer_callback,
                                               queue_size=1)
        self.pub1 = rospy.Publisher('robot1/command', String, queue_size=1)
        self.pub2 = rospy.Publisher('robot2/command', String, queue_size=1)
        self.pub3 = rospy.Publisher('ball/command', String, queue_size=1)
        self.model_states=None

        command = Command()
        command.state = "reset"
        command_str = command.encode()
        self.pub1.publish(command_str)
        self.pub2.publish(command_str)
        self.pub3.publish(command_str)


        self.landing_pose=[]
    def timer_callback(self,data):
        self.model_states=data
    def simulate(self):
        # print(self.model_states)
        while not rospy.is_shutdown():
            data = self.model_states
            if data==None:
                continue
            model_dict = {}
            for i in range(len(data.name)):
                model_dict[data.name[i]] = i

            robot1_pose = data.pose[model_dict['robot1']]
            robot2_pose = data.pose[model_dict['robot2']]
            robot1_position = robot1_pose.position
            robot2_position = robot2_pose.position

            # print(1, check_distance(robot1_position, ball_position))
            # print(2, check_distance(robot2_position, ball_position))
            ball_position = data.pose[model_dict['ball']].position
            # if ball_position.z < 0.290073:
            #     print(ball_position.z)
            # if check_distance(robot2_position, ball_position) < 1 and 0.25 < ball_position.z < 0.35:
            #     q = Quaternion(robot2_pose.orientation.x, robot2_pose.orientation.y,
            #                    robot2_pose.orientation.z, robot2_pose.orientation.w)
            #     theta = q.to_euler(degrees=False)[0]
            #     self.landing_pose.append(
            #         [ball_position.x, ball_position.y, ball_position.z, robot2_position.x, robot2_position.y, theta])
            #     if len(self.landing_pose) % 2 == 0:
            #         np.save("saved_pose.npy", self.landing_pose)
            if ball_position.z < 0.290073:
                command = Command()
                command.state = "reset"
                command_str = command.encode()
                self.pub1.publish(command_str)
                self.pub2.publish(command_str)
                self.pub3.publish(command_str)
                time.sleep(1)
            elif check_distance(robot1_position, ball_position) < 0.5:
                target_distance=check_distance(robot1_position, robot2_position)
                target_v=math.sqrt(target_distance*10/math.sin(math.radians(45)))
                target_w=target_v/0.145
                time.sleep(1)
                command = Command()
                command.state = "rotate"
                command.target_x = robot2_position.x
                command.target_y = robot2_position.y
                command_str = command.encode()
                self.pub1.publish(command_str)
                command = Command()
                command.state = "rotate"
                command.target_x = robot1_position.x
                command.target_y = robot1_position.y
                command_str = command.encode()
                self.pub2.publish(command_str)
                time.sleep(1)
                command.state = "throw"
                command.r1 = 0
                command.r2 = -30
                command.r3 = -180
                command.rv2 = target_w
                command_str = command.encode()
                self.pub1.publish(command_str)
                time.sleep(0.2)
                command.r2 = -90
                command.state = "idle"

                command_str = command.encode()
                self.pub1.publish(command_str)
                command.state="catch"
                command_str = command.encode()
                self.pub2.publish(command_str)
            elif check_distance(robot2_position, ball_position) < 0.5:
                target_distance = check_distance(robot1_position, robot2_position)
                target_v = math.sqrt(target_distance * 10 / math.sin(math.radians(60)))
                target_w = target_v / 0.15
                time.sleep(1)

                command = Command()
                command.state = "rotate"
                command.target_x = robot2_position.x
                command.target_y = robot2_position.y
                command_str = command.encode()
                self.pub1.publish(command_str)
                command = Command()
                command.state = "rotate"
                command.target_x = robot1_position.x
                command.target_y = robot1_position.y
                command_str = command.encode()
                self.pub2.publish(command_str)

                time.sleep(1)
                command.state = "throw"
                command.r1 = 0
                command.r2 = -30
                command.r3 = -180
                command.rv2 = target_w
                command_str = command.encode()
                self.pub2.publish(command_str)
                time.sleep(0.2)
                command.r2 = -90
                command.state = "idle"

                command_str = command.encode()
                self.pub2.publish(command_str)
                command.state="catch"
                command_str = command.encode()
                self.pub1.publish(command_str)




if __name__ == '__main__':
    rospy.init_node("robot_lacrosse_command")
    simulation=Simulate()
    simulation.simulate()