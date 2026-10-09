from tcp_socket import TCPSocket

# Init server on port 30001
tcp = TCPSocket('', 30001, 1024, is_server=True)

try:
    while True:
        # Check for data with 1-second timeout (1,000,000 usec)
        result = tcp.select(1000000)
        
        if result > 0:
            print(f"Received:")
            tcp.print()
            tcp.send("ACK_FROM_SERVER")
        elif result == -1:
            print("Disconnected.")
            break
finally:
    tcp.close()