import socket
from scripts.arm_control.arm_manager import Arm
import random
def start_server():
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind(('192.168.0.105', 12345))  # Bind to all interfaces on port 12345
    server_socket.listen(1)
    print("Server is listening for connections...")

    while True:
        client_socket, addr = server_socket.accept()
        print(f"Connected to {addr}")
        arm = Arm()
        arm.enable_motors()
        while True:
            data = client_socket.recv(10240)
            if not data:
                break
            data_list= data.decode().split(",")
            mode=data_list[0]
            if mode=="throw":
                target_angle= int(data_list[1])
                target_speed = float(data_list[2])
                arm_data = arm.throw_to_angle_with_speed(target_angle=target_angle,target_speed=target_speed)
                arm_data_str = str(arm_data)
            elif mode=="reset":
                arm.reset_ball()
            elif mode == "stop":
                arm.stop()
                arm.bus.shutdown()

            # print(f"Received: {arm_data_str}")
            client_socket.sendall(arm_data_str.encode())  # Echoes back the received data
        client_socket.close()

if __name__ == '__main__':
    start_server()