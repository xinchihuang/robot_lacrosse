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
                        arm_data = arm.throw_to_angle_with_speed(target_angle=desired_angle, target_speed=desired_speed)
                        arm_data_str = str(arm_data)
                        # print(arm_data_str)

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