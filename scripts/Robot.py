import math
import time
from scripts.data_process.check_parabola_point import check_parabola_point
from scripts.chassis_control.chassis_controller import optitrack_coordinate_to_world_coordinates, central_controller,landing_point_predictor,landing_point_predictor_lstm
from scripts.lstm_scratch import DynamicLSTM

import torch
from utils import calculate_rotation_angle
class Robot:
    def __init__(self, robot_name,chassis_executor=None,arm_executor=None):
        self.robot_name=robot_name
        self.chassis_executor=chassis_executor
        self.arm_executor = arm_executor
        self.model = DynamicLSTM(input_dim=2, hidden_dim=20, output_dim=1,
                            num_layers=2)  # Initialize the same model structure
        self.model.load_state_dict(torch.load("save_model.pth"))
        # General Settings
        self.g = 9.8
        self.max_speed = 3
        self.arm_pose = [-0.28, 0, 0.3]
        # landing prediction
        self.ball_memory=[]
        self.save_data=[]
        self.saved=False
        self.state=None
        self.parabola_state=False
        self.robot_self_pose=None
        self.robot_arm_list=[1,1,1]
    def generate_cotrol(self,x_world, y_world, z_world, theta_world):
        landing_target_x = None
        landing_target_y = None
        if len(self.ball_memory) >= 20:
            if check_parabola_point(self.ball_memory) == True:
                landing_target_x, landing_target_y, drop_t = landing_point_predictor_lstm(self.ball_memory,self.model,self.robot_self_pose, self.arm_pose[2])
            # landing_target_x, landing_target_y, drop_t=0,0,1
            # landing_time = drop_t - (self.ball_memory[-1][3] - self.ball_memory[0][3])
                print(landing_target_x,landing_target_y)
        if x_world ** 2 + y_world ** 2 < 4 and not landing_target_x == None and not landing_target_y == None:
            landing_target_x = landing_target_x - math.cos(theta_world) * self.arm_pose[0]
            landing_target_y = landing_target_y - math.sin(theta_world) * self.arm_pose[0]
            vx, vy, omega = central_controller([x_world, y_world, z_world], theta_world,
                                               [landing_target_x, landing_target_y, z_world], 0)
        else:
            vx, vy, omega = 0, 0, 0
        return vx, vy, omega
    def rotate(self,self_pose,direction_pose):
        target_direction = [direction_pose[0] - self_pose[0],
                            direction_pose[1] - self_pose[1]]
        self_direction = [math.cos(self_pose[3]), math.sin(self_pose[3])]

        d_theta = calculate_rotation_angle(self_direction, target_direction)
        # print(target_direction, self_direction,d_theta)
        # omega=math.degrees(d_theta)
        # print(d_theta)
        return 0,0,d_theta
    def execute(self,vx, vy, omega):
        self.chassis_executor.execute([vx, vy, omega])
    #
    def arm_throw_ball(self,desired_angle,desired_speed):
        arm_msg=self.arm_executor.throw(desired_angle,desired_speed)
        self.state="idle"
        return arm_msg
            # self.executor.stop_robot()
    def reset_arm(self):
        self.arm_executor.reset()
        self.state="idle"







