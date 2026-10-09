import select
import socket

class UdpSocket:
    name = 'udp'
    def __init__(self, local_ip, local_port, remote_ip, remote_port, buffer_size=4096):
        self._self_ip = local_ip
        self._self_port = local_port
        self._target_address = (remote_ip, remote_port)
        self._rx_buffer_size = buffer_size
        self._rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._rx.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, self._rx_buffer_size)
        self._tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        return
    
    def open(self):
        self._rx.bind((self._self_ip, self._self_port))
        self._tx.connect(self._target_address)
        print('open socket')

    def close(self):
        self._tx.close()
        self._rx.close()
        print('close socket')

    def send(self, message):
        try:
            self._tx.send(message.encode("ISO-8859-1"))
        except ConnectionRefusedError:
            # print("Connection refused. Client may not be available.")
            return
        
    def receive(self, dt = 0.001): # waiting time
        readable = select.select([self._rx], [], [], dt)[0]
        buf = ""
        if readable:
            for a in readable:
                # buf = a.recvfrom(self._rx_buffer_size)[0].decode("utf-8")
                buf = a.recvfrom(self._rx_buffer_size)[0].decode("ISO-8859-1")
        return buf