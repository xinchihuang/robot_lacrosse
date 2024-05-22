#!/usr/bin/env python3
import time

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelState
from sensor_msgs.msg import JointState

from gazebo_msgs.srv import SetModelState
import message_filters
import collections
from squaternion import Quaternion
from comm_data import ControlData
from std_msgs.msg import Float64


def set_joint_position(joint_publisher, position):
    joint_command = Float64()
    joint_command.data = position
    joint_publisher.publish(joint_command)



class Simulation:
    def __init__(self, robot_num,max_velocity=0.5,stop_thresh=0.00,max_simulation_time_step = 1000):

        # basic settings
        self.robot_num = robot_num
        # communication related
        self.sub_topic_list = []
        self.chassis_topic_dict = collections.defaultdict()
        self.arm_topic_dict = collections.defaultdict()
        for index in range(self.robot_num):
            pose_topic=f'/odom'
            self.sub_topic_list.append(message_filters.Subscriber(pose_topic, Odometry))
        for index in range(self.robot_num):
            chassis_topic=f'/cmd_vel'
            self.chassis_topic_dict[index]=rospy.Publisher(chassis_topic, Twist, queue_size=10)
        self.pub_revolute2 = rospy.Publisher('/robot_arm_controller/Revolute2_position_controller/command', Float64, queue_size=10)
        self.pub_revolute6 = rospy.Publisher('/robot_arm_controller/Revolute6_position_controller/command', Float64, queue_size=10)
        self.pub_revolute10 = rospy.Publisher('/robot_arm_controller/Revolute10_position_controller/command', Float64, queue_size=10)



        ts = message_filters.ApproximateTimeSynchronizer(self.sub_topic_list, queue_size=10, slop=0.1,allow_headerless=True)
        ts.registerCallback(self.SimulateCallback)

        # robot related
        self.max_velocity=max_velocity
        self.stop_thresh=stop_thresh

        # simulation related
        self.time_step=0
        self.max_simulation_time_step = max_simulation_time_step
        # save related
        self.execute_stop=1




    def SimulateCallback(self, *argv):
        if self.execute_stop == 1:
            for index in range(0, self.robot_num):
                msg = Twist()
                msg.linear.x = 0
                msg.linear.y = 0
                print(msg.linear.x, msg.linear.y)
                self.chassis_topic_dict[index].publish(msg)
            self.execute_stop =0
        else:

            pose_list = []
            control_list=[]

            for index in range(self.robot_num):
                q=Quaternion(argv[index].pose.pose.orientation.x,argv[index].pose.pose.orientation.y,argv[index].pose.pose.orientation.z,argv[index].pose.pose.orientation.w)
                pose_index=[argv[index].pose.pose.position.x,argv[index].pose.pose.position.y,q.to_euler(degrees=False)[0]]
                pose_list.append(pose_index)
            print("___________")

            for index in range(0, self.robot_num):
                control_data=ControlData()
                control_data.velocity_x=0
                control_data.velocity_y=0
                control_data.omega=0
                control_list.append([control_data.velocity_x, control_data.velocity_y, control_data.omega])

            for index in range(0,self.robot_num):

                msg=Twist()
                if self.stop_thresh <abs(control_list[index][0])<self.max_velocity:
                    msg.linear.x = control_list[index][0]
                elif abs(control_list[index][0])>=self.max_velocity:
                    msg.linear.x = self.max_velocity*abs(control_list[index][0])/control_list[index][0]
                else:
                    msg.linear.x = 0
                if self.stop_thresh<abs(control_list[index][1])<self.max_velocity:
                    msg.linear.y = control_list[index][1]
                elif abs(control_list[index][1])>=self.max_velocity:
                    msg.linear.y = self.max_velocity*abs(control_list[index][1])/control_list[index][1]
                else:
                    msg.linear.y = 0

                # self.chassis_topic_dict[index].publish(msg)
                # msg.angular.z=10
                # print(control_list[index])
                # msg.linear.x,msg.linear.y,msg.angular.z=control_list[index][0],control_list[index][1],3
                self.chassis_topic_dict[index].publish(msg)
                if self.time_step %10==5:
                    set_joint_position(self.pub_revolute2, 1.0)
                    set_joint_position(self.pub_revolute6, 1.0)
                    set_joint_position(self.pub_revolute10,-1.3)
                elif self.time_step %10==0:
                    set_joint_position(self.pub_revolute2, 1.0)
                    set_joint_position(self.pub_revolute6, 1.0)
                    set_joint_position(self.pub_revolute10, 0.5)
                print(index)
            self.time_step+=1
            print(self.time_step)

            # self.execute_stop = 1

        if self.time_step>self.max_simulation_time_step:
            rospy.signal_shutdown(f"Stop after {self.time_step} steps")





if __name__ == "__main__":
    robot_num =1
    pose_list=[[0,0,0],
               ]
    rospy.wait_for_service('/gazebo/set_model_state')
    set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    rospy.init_node("robot_lacrosse")
    listener = Simulation(robot_num=robot_num,)

    pub_revolute2 = rospy.Publisher('/robot_arm_controller/Revolute2_position_controller/command', Float64,
                                         queue_size=10)
    pub_revolute6 = rospy.Publisher('/robot_arm_controller/Revolute6_position_controller/command', Float64,
                                         queue_size=10)
    pub_revolute10 = rospy.Publisher('/robot_arm_controller/Revolute10_position_controller/command', Float64,
                                          queue_size=10)

    set_joint_position(pub_revolute2, 1.0)
    set_joint_position(pub_revolute6, 1.0)
    set_joint_position(pub_revolute10, -1.3)

    state_msg = ModelState()
    state_msg.model_name = 'robomaster_ep_arm'
    state_msg.pose.position.x =0
    state_msg.pose.position.y = 0
    state_msg.pose.position.z = 0
    resp = set_state(state_msg)
    state_msg = ModelState()
    state_msg.model_name = 'ball'
    state_msg.pose.position.x = 0.0
    state_msg.pose.position.y = 0
    state_msg.pose.position.z = 0.7
    state_msg.twist.linear.x = 2.0  # 设置X方向的速度
    state_msg.twist.linear.y = 0.0
    state_msg.twist.linear.z = 0.0
    state_msg.twist.angular.x = 0.0
    state_msg.twist.angular.y = 0.0
    state_msg.twist.angular.z = 0.0
    state_msg.reference_frame = 'world'  # 参考坐标系
    resp = set_state(state_msg)
    # for i in range(robot_num):
    #     state_msg = ModelState()
    #     state_msg.model_name = 'rm_{}'.format(i)
    #     state_msg.pose.position.x = pose_list[i][0]
    #     state_msg.pose.position.y = pose_list[i][1]
    #     state_msg.pose.position.z = 0
    #     q=Quaternion.from_euler(0, 0, pose_list[i][2], degrees=False)
    #     state_msg.pose.orientation.x = q.x
    #     state_msg.pose.orientation.y = q.y
    #     state_msg.pose.orientation.z = q.z
    #     state_msg.pose.orientation.w = q.w
    #     resp = set_state(state_msg)
    rospy.spin()

