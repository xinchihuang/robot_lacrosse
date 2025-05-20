import math
import time
from scripts.data_process.check_parabola_point import check_parabola_point
from scripts.chassis_control.chassis_controller import optitrack_coordinate_to_world_coordinates, central_controller,landing_point_predictor,landing_point_predictor_lstm
from scripts.lstm_scratch import DynamicLSTM
from scripts.throw_ml import *
import torch
from scripts.utils import calculate_rotation_angle
class Robot:
    def __init__(self, name,chassis_executor=None,arm_executor=None):
        self.name=name
        self.chassis_executor=chassis_executor
        self.arm_executor = arm_executor
        self.chassis_model = DynamicLSTM(input_dim=2, hidden_dim=20, output_dim=1,
                            num_layers=2)  # Initialize the same model structure
        self.chassis_model.load_state_dict(torch.load("scripts/save_model.pth"))
        self.arm_model = SimpleMLP(input_dim=2, hidden_dim=100, output_dim=2)
        self.arm_model.load_state_dict(torch.load("scripts/save_model_throw.pth"))

        # General Settings
        self.g = 9.8
        self.max_speed = 3
        self.arm_pose = [-0.35, 0, 0.3]

        # landing prediction
        self.robot_state=None
        self.robot_self_pose=None

    def get_move_control(self,ball_memory):

        if len(ball_memory)<30:
            return 0,0,0
        # landing_target_x = None
        # landing_target_y = None
        # x_world, y_world, z_world, theta_world=self.robot_self_pose[0],self.robot_self_pose[1],self.robot_self_pose[2],self.robot_self_pose[3]
        theta_world = self.robot_self_pose[3]
        landing_target_x, landing_target_y, drop_t = landing_point_predictor_lstm(ball_memory,self.chassis_model,self.robot_self_pose, self.arm_pose[2])
        landing_target_x = landing_target_x - math.cos(theta_world) * self.arm_pose[0]
        landing_target_y = landing_target_y - math.sin(theta_world) * self.arm_pose[0]

        vx, vy, omega = central_controller(self.robot_self_pose[:3], theta_world,
                                           [landing_target_x, landing_target_y], 0)

        return vx, vy, omega
    def get_p_control(self,delta_position,k_p=1):
        # delta_position = [delta_position[0] - self.robot_self_pose[0], delta_position[1] - self.robot_self_pose[1]]
        # delta_position = [delta_position[0], delta_position[1]]
        # print(delta_position)
        # print(self.robot_self_pose)
        d_x= delta_position[0] - self.robot_self_pose[0]
        d_y= delta_position[1] - self.robot_self_pose[1]
        d_omega= delta_position[3] - self.robot_self_pose[3]
        # print(d_x,d_y)
        vx = k_p * d_x
        vy = k_p * d_y
        omega = k_p * d_omega
        return vx, vy, omega
    def get_rotate_control(self,direction_pose):
        target_direction = [direction_pose[0] - self.robot_self_pose[0],
                            direction_pose[1] - self.robot_self_pose[1]]
        self_direction = [math.cos(self.robot_self_pose[3]), math.sin(self.robot_self_pose[3])]

        d_theta = calculate_rotation_angle(self_direction, target_direction)
        # d_theta = math.degrees(d_theta)
        return 0,0,d_theta

    def get_arm_control_old(self, distance,height):

        desired_angle, desired_speed = cal_angle_speed(height, distance)
        feature = [[height, distance]]
        feature = torch.tensor(feature)
        output = self.arm_model(feature)
        min_angle, max_angle, min_vel, max_vel = 25, 35, 29, 31
        residual = linear_mapping(output.squeeze().detach().numpy()[0], 0, 1.0, min_angle,
                                  max_angle)
        desired_angle = desired_angle + residual
        desired_speed = 30
        return desired_angle,desired_speed
    def execute(self,vx, vy, omega):
        if self.robot_state=="idle":
            self.chassis_executor.execute([0,0,0])
        else:
            self.chassis_executor.execute([vx, vy, omega])
    #
    def arm_throw_ball(self,desired_angle,desired_speed):
        arm_msg=self.arm_executor.throw(desired_angle,desired_speed)
        # return arm_msg
            # self.executor.stop_robot()
    def reset_arm(self):
        self.arm_executor.reset()
    def stop(self):
        self.chassis_executor.stop()


class RobotOld:
    def __init__(self, name, chassis_executor=None, arm_executor=None):
        self.name = name
        self.chassis_executor = chassis_executor
        self.arm_executor = arm_executor
        self.chassis_model = DynamicLSTM(input_dim=2, hidden_dim=20, output_dim=1,
                                         num_layers=2)  # Initialize the same model structure
        self.chassis_model.load_state_dict(torch.load("save_model.pth"))
        self.arm_model = SimpleMLP(input_dim=2, hidden_dim=100, output_dim=2)
        self.arm_model.load_state_dict(torch.load("save_model_throw.pth"))

        # General Settings
        self.g = 9.8
        self.max_speed = 3
        self.arm_pose = [-0.35, 0, 0.3]

        # landing prediction
        self.save_data = []
        self.saved = False
        self.robot_state = None
        self.robot_self_pose = None

    def get_move_control(self, ball_memory):

        if len(ball_memory) < 30:
            return 0, 0, 0
        # landing_target_x = None
        # landing_target_y = None
        # x_world, y_world, z_world, theta_world=self.robot_self_pose[0],self.robot_self_pose[1],self.robot_self_pose[2],self.robot_self_pose[3]
        theta_world = self.robot_self_pose[3]
        landing_target_x, landing_target_y, drop_t = landing_point_predictor_lstm(ball_memory, self.chassis_model,
                                                                                  self.robot_self_pose,
                                                                                  self.arm_pose[2])
        landing_target_x = landing_target_x - math.cos(theta_world) * self.arm_pose[0]
        landing_target_y = landing_target_y - math.sin(theta_world) * self.arm_pose[0]

        vx, vy, omega = central_controller(self.robot_self_pose[:3], theta_world,
                                           [landing_target_x, landing_target_y], 0)

        return vx, vy, omega

    def get_rotate_control(self, direction_pose):
        target_direction = [direction_pose[0] - self.robot_self_pose[0],
                            direction_pose[1] - self.robot_self_pose[1]]
        self_direction = [math.cos(self.robot_self_pose[3]), math.sin(self.robot_self_pose[3])]

        d_theta = calculate_rotation_angle(self_direction, target_direction)
        # d_theta = math.degrees(d_theta)
        return 0, 0, d_theta

    def execute(self, vx, vy, omega):
        if self.robot_state == "idle":
            self.chassis_executor.execute([0, 0, 0])
        else:
            self.chassis_executor.execute([vx, vy, omega])

    #
    def arm_throw_ball(self, desired_angle, desired_speed):
        arm_msg = self.arm_executor.throw(desired_angle, desired_speed)
        # return arm_msg
        # self.executor.stop_robot()

    def reset_arm(self):
        self.arm_executor.reset()

    def stop(self):
        self.chassis_executor.stop()
class Launcher:
    def __init__(self, name,chassis_executor=None,arm_executor=None):
        self.name=name
        self.chassis_executor=chassis_executor
        self.arm_executor = arm_executor

    def execute(self,vx, vy, omega):
        self.chassis_executor.execute([vx, vy, omega])
    def launcher_throw_ball(self,desired_angle,desired_speed):
        arm_msg=self.arm_executor.launch(desired_angle,desired_speed)
        return arm_msg
            # self.executor.stop_robot()
    def reset_arm(self):
        self.arm_executor.reset()






