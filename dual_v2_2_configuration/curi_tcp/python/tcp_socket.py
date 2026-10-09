import socket
import select

class TCPSocket:
    def __init__(self, server_ip, server_port, buffer_size, is_server):
        self.buffer_size = buffer_size
        self.receive_buffer = b""
        self.send_buffer = ""
        self.is_server = is_server
        
        # Initialize socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        
        if is_server:
            # SO_REUSEADDR allows immediate restart
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.sock.bind((server_ip if server_ip else '0.0.0.0', server_port))
            self.sock.listen(1)
            print(f"Waiting for connection on {server_port}...")
            self.client_sock, self.addr = self.sock.accept()
            # Set buffer size (TCP_SNDBUF)
            self.client_sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, buffer_size)
        else:
            self.sock.connect((server_ip, server_port))
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, buffer_size)
            self.client_sock = self.sock

    def select(self, usec):
        # select.select(inputs, outputs, exceptions, timeout_seconds)
        timeout = usec / 1_000_000.0
        ready_to_read, _, _ = select.select([self.client_sock], [], [], timeout)
        
        if ready_to_read:
            return self.receive()
        return 0

    def send(self, message):
        # Convert string to bytes
        data = message.encode('utf-8')
        return self.client_sock.send(data)

    def receive(self):
        try:
            self.receive_buffer = self.client_sock.recv(self.buffer_size)
            if not self.receive_buffer:
                return -1 # Disconnected
            return len(self.receive_buffer)
        except:
            return -1

    def print(self):
        if self.receive_buffer:
            print(self.receive_buffer.decode('utf-8'))

    def close(self):
        if self.is_server:
            self.client_sock.close()
        self.sock.close()