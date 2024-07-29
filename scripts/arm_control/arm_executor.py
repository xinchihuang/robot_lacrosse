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
        return_msg=self.client_socket.sendall(command.encode())
        return return_msg
    def reset(self):
        command = f"reset"
        self.client_socket.sendall(command.encode())
