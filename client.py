import socket

def connect_to_server():
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_address = ('192.168.0.104', 12345)  # Replace <server_ip_address> with the server's IP address
    client_socket.connect(server_address)
    print("Connected to server.")

    try:
        while True:
            message = input("Enter message to send: ")
            client_socket.sendall(message.encode())
            data = client_socket.recv(1024)
            print(f"Received: {data.decode()}")
    finally:
        client_socket.close()

if __name__ == '__main__':
    connect_to_server()
