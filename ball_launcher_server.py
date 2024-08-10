import socket
from scripts.arm_control.arm_manager import Launcher
import random


def start_server():
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind(('0.0.0.0', 12345))
    server_socket.listen(100)
    print("Server is listening for connections...")
    launcher = Launcher()
    while True:
        client_socket, addr = server_socket.accept()
        print(f"Connected to {addr}")
        try:
            while True:
                data = client_socket.recv(10240)
                if not data:
                    break
                data_list = data.decode().split(",")
                mode = data_list[0]
                print(data_list)
                try:
                    if mode == 'launch':
                        target_angle = float(data_list[1])
                        target_speed = float(data_list[2])
                        launcher_data = launcher.move_at_acceleration(target_angle=target_angle, target_velocity=target_speed)
                except:
                    print("Invalid")

            client_socket.close()
        except Exception as e:
            print(f"An error occurred: {e}")
        finally:
            if client_socket:
                client_socket.close()
                print("Connection closed")


if __name__ == '__main__':
    start_server()