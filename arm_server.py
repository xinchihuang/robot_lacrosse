import socket
from scripts.arm_control.arm_manager import Arm
import random
def start_server():
    arm = Arm()
    arm.enable_motors()
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_socket:
        server_socket.bind(('0.0.0.0', 12345))  # 监听所有网络接口
        server_socket.listen(100)  # 可以调整监听的连接数
        print("Server is listening for connections...")
        while True:
            client_socket, addr = server_socket.accept()
            print(f"Connected to {addr}")
            try:
                while True:
                    data = client_socket.recv(10240)
                    if not data:
                        break
                    data_list= data.decode().split(",")
                    mode=data_list[0]
                    if mode=="throw":
                        target_angle= float(data_list[1])
                        target_speed = float(data_list[2])
                        arm_data = arm.throw_to_angle_with_speed(target_angle=target_angle,target_speed=target_speed)
                        # arm_data_str = str(arm_data)
                    elif mode=="reset":
                        arm.reset_ball()
                    elif mode == "stop":
                        arm.stop()
                        arm.bus.shutdown()
                    client_socket.sendall(data_list.encode())
            except:
                print("Error Input!")
                pass
            client_socket.close()

if __name__ == '__main__':
    start_server()