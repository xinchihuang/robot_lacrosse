#!/usr/bin/env python3
import os
import time

import numpy as np
import random
import rospy
from gazebo_msgs.msg import ModelStates, ModelState
from gazebo_msgs.srv import SetModelState


class Ball:
    def __init__(self):
        self.set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.move_topic = rospy.Subscriber("/gazebo/model_states", ModelStates, self.reset_callback, queue_size=1)
        self.ball_memory = []
        self.frame_count=0

    def reset_callback(self, data):
        self.frame_count += 1
        model_dict = {}
        for i in range(len(data.name)):
            model_dict[data.name[i]] = i
        ball_position = data.pose[model_dict['ball']].position
        if self.frame_count % 10 == 0:
            self.ball_memory.append([ball_position.x, ball_position.y, ball_position.z])
            # print(len(self.ball_memory))
        # print(ball_position.z)
        if ball_position.z < 0.3:

            if len(self.ball_memory)>50:
                print("save")
                files_and_dirs = os.listdir(
                    "/home/xinchi/catkin_ws/src/robot_lacrosse/scripts/simulate_parabola/")
                # Filter out directories, keeping only files
                files = [f for f in files_and_dirs if os.path.isfile(
                    os.path.join("/home/xinchi/catkin_ws/src/robot_lacrosse/scripts/simulate_parabola/", f))]
                number = len(files)
                np.save("/home/xinchi/catkin_ws/src/robot_lacrosse/scripts/simulate_parabola/" + str(
                    number) + ".npy", np.array(self.ball_memory[2:]))
                time.sleep(1)
            self.frame_count=0
            self.ball_memory = []
            state_msg = ModelState()
            state_msg.model_name = 'ball'
            state_msg.pose.position.x = random.random() * 2
            state_msg.pose.position.y = random.random() * 2
            state_msg.pose.position.z = random.random() * 2+1
            state_msg.twist.linear.x = random.random() * 4
            state_msg.twist.linear.y = random.random() * 4
            state_msg.twist.linear.z = random.random() * 4
            self.set_state(state_msg)




if __name__ == "__main__":
    rospy.init_node("robot_lacrosse")
    Ball()
    rospy.spin()
