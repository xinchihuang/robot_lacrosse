import socket

class ArmExecutor:
    def __init__(self,server_address):
        self.server_address =server_address
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # server_address = ('192.168.0.105', 12345)  # Replace <server_ip_address> with the server's IP address
        self.client_socket.connect(self.server_address)
        print("Connected to server.")

    def throw(self,desired_angle,desired_speed):
        command = f"throw,{desired_angle},{desired_speed}"
        self.client_socket.sendall(command.encode())
    def reset(self):
        command = f"reset"
        self.client_socket.sendall(command.encode())
class LauncherExecutor:
    def __init__(self,server_address):
        self.server_address =server_address
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # server_address = ('192.168.0.105', 12345)  # Replace <server_ip_address> with the server's IP address
        self.client_socket.connect(self.server_address)
        print("Connected to server.")

    def launch(self,desired_angle,desired_speed):
        command = f"launch,{desired_angle},{desired_speed}"
        print(command)
        self.client_socket.sendall(command.encode())
    def reset(self):
        command = f"reset"
        self.client_socket.sendall(command.encode())
