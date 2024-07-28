import math
import time
from scripts.data_process.check_parabola_point import check_parabola_point
from scripts.chassis_control.chassis_controller import optitrack_coordinate_to_world_coordinates, central_controller,landing_point_predictor

class Robot:
    def __init__(self, robot_name,chassis_executor=None,arm_executor=None):
        self.robot_name=robot_name
        self.chassis_executor=chassis_executor
        self.arm_executor = arm_executor
        # General Settings
        self.g = 10
        self.max_speed = 3
        self.arm_pose = [-0.25, 0, 0.3]
        # landing prediction
        self.ball_memory=[]
        self.save_data=[]
        self.saved=False
        self.state=None
        self.parabola_state=False
        self.robot_arm_list=[1,1,1]
    def generate_cotrol(self,x_world, y_world, z_world, theta_world):
        landing_target_x = None
        landing_target_y = None
        if len(self.ball_memory) >= 20:
            if check_parabola_point(self.ball_memory) == True:
                landing_target_x, landing_target_y, drop_t = landing_point_predictor(self.ball_memory, self.arm_pose[2])
            # landing_target_x, landing_target_y, drop_t=0,0,1
            # landing_time = drop_t - (self.ball_memory[-1][3] - self.ball_memory[0][3])
        if x_world ** 2 + y_world ** 2 < 2.25 and not landing_target_x == None and not landing_target_y == None:
            landing_target_x = landing_target_x - math.cos(theta_world) * self.arm_pose[0]
            landing_target_y = landing_target_y - math.sin(theta_world) * self.arm_pose[0]
            vx, vy, omega = central_controller([x_world, y_world, z_world], theta_world,
                                               [landing_target_x, landing_target_y, z_world], 0)
            # print("landing point",landing_target_x,landing_target_y)
            # if self.ball_memory[-1][2]<0.35:
            #     print("landing")
            # self.save_data.append(landing_time,landing_target_x,landing_target_y,self.ball_memory[-1][0],self.ball_memory[-1][1],self.ball_memory[-1][2])
            # print(landing_time,landing_target_x,landing_target_y,self.ball_memory[-1],len(self.ball_memory))
        else:
            vx, vy, omega = 0, 0, 0
        return vx, vy, omega
    def execute(self,vx, vy, omega):
        self.chassis_executor.execute([vx, vy, omega])
    #
    def arm_throw_ball(self):
        pass
            # self.executor.stop_robot()







