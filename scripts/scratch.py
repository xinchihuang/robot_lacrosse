import socket
import math
import numpy as np
class RobotServer:
    def __init__(self,server_address):
        self.robot_state=None
        self.robot_self_pose=None
        self.server_address =server_address
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # server_address = ('192.168.0.105', 12345)  # Replace <server_ip_address> with the server's IP address
        self.client_socket.connect(self.server_address)
        print("Connected to server.")

    def send_chassis_rotate_data(self,state,robot1_pose,robot2_pose):
        robot1_pose_list = [round(num, 4) for num in robot1_pose]
        robot1_pose_str = ','.join(map(str, robot1_pose_list))
        robot2_pose_list = [round(num, 4) for num in robot2_pose]
        robot2_pose_str = ','.join(map(str, robot2_pose_list))
        command = f"{state};{robot1_pose_str};{robot2_pose_str}"
        self.client_socket.sendall(command.encode())
    def send_chassis_catch_data(self,state,robot1_pose,ball_memory):
        # ball_memory_list=[round(num, 4) for num in ball_memory]
        robot_pose_list=[round(num, 4) for num in robot1_pose]
        robot_pose_str=','.join(map(str, robot_pose_list))
        series_ball_memory=[]
        for i in range(len(ball_memory)):
            for j in range(len(ball_memory[i])):
                series_ball_memory.append(round(ball_memory[i][j],4))
        ball_memory_str=','.join(map(str, series_ball_memory))
        command = f"{state};{robot_pose_str};{ball_memory_str}"
        self.client_socket.sendall(command.encode())
    def send_arm_data(self,desired_angle, desired_speed,distance,height):
        command=f"throw;{desired_angle};{desired_speed};{distance};{height}"
        self.client_socket.sendall(command.encode())
    def reset_arm(self):
        command = "reset"
        self.client_socket.sendall(command.encode())
    def stop_chassis(self):
        command = "idle"
        self.client_socket.sendall(command.encode())
if __name__ == "__main__":

    robot1_pose=[0,1,0.1,math.pi]
    robot2_pose=[0,0,0.1,math.pi]
    ball_memory=np.load('saved_ball_data/0.npy')
    desired_angle=30
    desired_speed=30
    distance=2
    height=1.5
    # print(ball_memory)
    robot_server=RobotServer(('192.168.1.172', 12345))
    # robot_server.send_chassis_rotate_data("rotate",robot1_pose,robot2_pose)
    robot_server.send_arm_data(desired_angle, desired_speed,distance,height)