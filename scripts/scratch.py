import numpy as np
import matplotlib.pyplot as plt
import math
from utils import rotate_matrix_coordinate,calculate_rotation_angle,detect_ball,check_distance
ball_memory=np.load("robot1.npy")
class Robot:
    def __init__(self, robot_name,initial_state,state="idle"):
        self.robot_name=robot_name
        # General Settings
        self.g = 10
        self.max_speed = 1.5
        self.camera_bias=[0.1,0,0.15]
        self.camera_rpy = [0, -0.785, 0.0]
        self.arm_pose = [-0.2, 0, 0.3]
        self.min_frame_count=0

        self.save=False

        # Camera relatetd
        self.fx = 500.39832201574455
        self.fy = 500.39832201574455
        self.cx = 500.5
        self.cy = 500.5


        #### Memorys
        self.ball_memory=[]
        self.frame_count=0
        #x,y,theta,t
        self.robot_pose=[0,0,0,0]
        self.robot_pose_initial = [0, 0, 0, 0]
        self.robot_pose_global = [0, 0, 0, 0]
        # vx,vy,omega
        self.move_msg = [0, 0, 0]

    def simulate_local_controller(self):

        self.ball_memory=np.load("robot1.npy")
        self.ball_memory=np.array([self.ball_memory[599],self.ball_memory[600],self.ball_memory[601]])
        plt.scatter(self.ball_memory[:,0],self.ball_memory[:,1])
        plt.axis('equal')
        plt.show()

        if len(self.ball_memory) > self.min_frame_count:
            # if len(self.ball_memory) >self.min_frame_count:
            #     self.ball_memory.pop(0)
            # print(len(self.ball_memory))
            t_start = self.ball_memory[0][2]
            g_x = 0
            g_y = self.g * math.cos(math.pi / 4)
            g_z = -self.g * math.sin(math.pi / 4)
            A = []
            B = []
            camera_angle = -self.camera_rpy[1]
            for i in range(len(self.ball_memory)):
                t = self.ball_memory[i][2] - t_start
                print(t)
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
            print(x_ball_w, y_ball_w, z_ball_w,vx_ball_w, vy_ball_w, vz_ball_w)
            if vz_ball_w ** 2 - (z_ball_w - self.arm_pose[2]) * (-self.g) * 2 > 0:
                drop_t = (-vz_ball_w - math.sqrt(
                    vz_ball_w ** 2 - (z_ball_w - self.arm_pose[2]) * (-self.g) * 2)) / (-self.g)
                target_x = x_ball_w + drop_t * vx_ball_w - self.arm_pose[0]
                target_y = y_ball_w + drop_t * vy_ball_w - self.arm_pose[1]
            distance_x = target_x - self.robot_pose[0]
            distance_y = target_y - self.robot_pose[1]
            if distance_x>0:
                vx = min(abs(distance_x) * 1, 1) * self.max_speed * (distance_x) / abs(distance_x)
            else:
                vx=0
            if distance_y>0:
                vy = min(abs(distance_y) * 1, 1) * self.max_speed * (distance_y) / abs(distance_y)
            else:
                vy=0
            self.move_msg = [vx, vy, 0]
            print(distance_x, distance_y)
class_test=Robot("robot1",1)
class_test.simulate_local_controller()
# 0.09999984810234007 2.631759024147877e-07 0.14990338548878626 -3.756707601458217e-05 -4.4092717806960624e-05 0.036557950263426266
# 0.09994383329997437 -0.000125894576867611 0.18696210895765775 0.10614614206823407 -0.0015307385601591064 0.7187713445868923
# 0.09974935748055175 -0.0015097206775669086 0.8759787238940369 1.604749942057559 -0.00794352271122109 4.284814290530555
