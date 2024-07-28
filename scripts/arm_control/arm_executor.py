import socket

class ArmExecutor:
    def __init__(self,sever_address):
        self.server_address =sever_address
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # server_address = ('192.168.0.105', 12345)  # Replace <server_ip_address> with the server's IP address
        self.client_socket.connect(self.server_address)
        print("Connected to server.")
    def send_command(self,command):
        self.client_socket.sendall(command.encode())
        return_data =self.client_socket.recv(10240)
        print(f"Received: {return_data.decode()}")

