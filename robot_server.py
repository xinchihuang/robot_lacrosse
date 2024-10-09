import socket
from scripts.arm_control.arm_manager import Arm
from scripts.Robot import Robot
import random
from scripts.throw_ml import *
from scripts.robomaster_executor.robomaster_executor import *
def start_server():
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind(('0.0.0.0', 12345))
    server_socket.listen(100)
    print("Server is listening for connections...")
    arm = Arm()
    arm.enable_motors()
    chassis=RoboMasterExecutor()
    robot=Robot(name=1,arm_executor=arm,chassis_executor=chassis)

    while True:
        client_socket, addr = server_socket.accept()
        print(f"Connected to {addr}")
        try:
            while True:
                data = client_socket.recv(10240)
                if not data:
                    break
                data_list = data.decode().split(";")
                mode = data_list[0]
                # arm_data_str = ''
                print(mode)
                try:
                    if mode == 'throw':
                        desired_speed = float(data_list[1])
                        desired_angle = float(data_list[2])
                        distance = float(data_list[3])
                        height = float(data_list[4])
                        target_angle,target_speed=robot.get_arm_control_old(height=height,distance=distance)
                        print(target_angle,target_speed)
                        arm_data = arm.throw_to_angle_with_speed(target_angle=target_angle, target_speed=target_speed)
                        arm_data_str = str(arm_data)
                        print(arm_data_str)
                    elif mode == 'rotate':

                        robot_self_pose_list = data_list[1].split(",")
                        direction_pose_list = data_list[2].split(",")
                        robot_self_pose = [float(i) for i in robot_self_pose_list]
                        robot.robot_self_pose=robot_self_pose
                        direction_pose = [float(i) for i in direction_pose_list]
                        vx, vy, omega=robot.get_rotate_control(direction_pose)
                        chassis.execute([vx, vy, omega])
                    elif mode == 'catch':
                        robot_self_pose_list = data_list[1].split(",")
                        ball_memory_list = data_list[2].split(",")
                        robot_self_pose = [float(i) for i in robot_self_pose_list]
                        robot.robot_self_pose = robot_self_pose
                        ball_memory=[]
                        for i in range(0,len(ball_memory_list),3):
                            ball_pose=[float(ball_memory_list[i]),float(ball_memory_list[i+1]),float(ball_memory_list[i+2])]
                            ball_memory.append(ball_pose)
                        vx, vy, omega=robot.get_move_control(ball_memory)
                        chassis.execute([vx, vy, omega])
                    elif mode == 'idle':
                        chassis.stop()
                    elif mode == 'reset':
                        arm.reset_ball()
                    elif mode == 'stop':
                        arm.stop()
                        arm.bus.shutdown()
                        chassis.stop()
                        break  # Break the loop to close the client socket

                except ValueError as ve:
                    print(f"Value error: {ve}")
                except IndexError as ie:
                    print(f"Index error: {ie}")


                # client_socket.sendall(arm_data_str.encode())  # Echoes back the received data
            client_socket.close()
        except Exception as e:
            print(f"An error occurred: {e}")
        finally:
            if client_socket:
                client_socket.close()
                print("Connection closed")


if __name__ == '__main__':
    start_server()