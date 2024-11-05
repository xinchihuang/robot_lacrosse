import socket
from scripts.arm_control.arm_manager import Arm
import random
def start_server():
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind(('192.168.0.102', 12345))  # Bind to all interfaces on port 12345
    server_socket.listen(5)
    print("Server is listening for connections...")

    while True:
        client_socket, addr = server_socket.accept()
        print(f"Connected to {addr}")
        while True:
            data = client_socket.recv(1024)
            if not data:
                break
            if data.decode()=="t":
                print(data)
                arm = Arm()
                step = 250
                angle_1 = 0
                angle_2 = 90
                angle_3 = 0
                brake_angle = random.randint(55, 65)
                # brake_angle=65
                torque = random.uniform(-12, -10)
                arm.go_to([[step, angle_1, angle_2, angle_3]])
                arm_data = arm.throw_to_angle_with_torque(brake_angle=brake_angle, torque=torque)
                arm.go_to([[step, angle_1, angle_2, angle_3]])
            print(f"Received: {data.decode()}")
            client_socket.sendall(data)  # Echoes back the received data
        client_socket.close()

if __name__ == '__main__':
    start_server()
