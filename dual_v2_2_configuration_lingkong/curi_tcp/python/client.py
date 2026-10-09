from tcp_socket import TCPSocket

# Connect to local robot simulator
tcp = TCPSocket('127.0.0.1', 30001, 1024, is_server=False)

try:
    # Send a move command
    command = "MOVE_J 45,0,0"
    print(f"Sending: {command}")
    tcp.send(command)

    # Wait for response (2-second timeout)
    if tcp.select(2000000) > 0:
        print("Robot Response:")
        tcp.print()
    else:
        print("No response.")

finally:
    tcp.close()