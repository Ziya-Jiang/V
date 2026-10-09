import sys
import signal
from functools import partial
import time
from udp_socket import UdpSocket

def signal_handler(udp, sig, frame):
    print('You pressed Ctrl+C!')
    udp.close()
    sys.exit(0)
    
if __name__ == '__main__':
    local_ip, local_port = "127.0.0.1", 10085
    remote_ip, remote_port = "127.0.0.1", 10086
    udp = UdpSocket(local_ip, local_port, remote_ip, remote_port)
    signal.signal(signal.SIGINT,partial(signal_handler, udp))
    udp.open()
    for i in range(10000):
        #Send data
        udp.send("1#2#3#4#5#")
        
        #Receive data
        data = udp.receive()
        if data: 
            print(data)
        
        time.sleep(0.05)
    udp.close()