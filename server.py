import socket
from scripts.arm_control.arm_manager import Arm
import random
def start_server():
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind(('0.0.0.0', 12345))  # Bind to all interfaces on port 12345
    server_socket.listen(1)
    print("Server is listening for connections...")

    while True:
        client_socket, addr = server_socket.accept()
        print(f"Connected to {addr}")
        arm = Arm()
        arm.enable_motors()
        while True:
            data = client_socket.recv(1024)
            if not data:
                break
            data_list= data.decode().split(",")
            mode=data_list[0]
            if mode=="t":
                print(data)
                step = int(data_list[1])
                angle_1 = int(data_list[2])
                angle_2 = int(data_list[3])
                angle_3 = int(data_list[4])
                target_angle= int(data_list[5])
                # brake_angle=65
                target_speed = float(data_list[6])
                arm.go_to([[step, angle_1, angle_2, angle_3]])
                arm_data = arm.throw_to_angle_with_speed(target_angle=target_angle,target_speed=target_speed)
                arm.go_to([[step, angle_1, angle_2, angle_3]])
            elif mode=="u":
                arm.enable_motors()
            elif mode=="s":
                arm.stop()
            elif mode == "stop":
                arm.stop()
                arm.bus.shutdown()
            print(f"Received: {data.decode()}")
            client_socket.sendall(data)  # Echoes back the received data
        client_socket.close()

if __name__ == '__main__':
    start_server()
