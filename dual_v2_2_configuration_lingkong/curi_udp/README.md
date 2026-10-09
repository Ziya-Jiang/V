# curi_udp
This is a general api for udp communication in c and python

Supported OS:
+ python: Linux & Windows
+ c code: Linux & Windows

## How to use
### 1. Initial the communication
+ Using **udp_init** function to initial and start the communication
```
udp_init(local_ip, local_port, remote_ip, remote_port)
```
### 2. Send and receive data
+ All the data are transfered in string data type and code/decode by using two functions (which you may change in your applications):
```
udp_pack()
udp_unpack()
```
+ Send data by using **udp_send** function
```
udp_send()
```
+ Blocking (waiting) for receive data by using **udp_receive** function
```
udp_receive()
```
+ Non-blocking for receive data by using **udp_get** function
```
udp_get(waiting_time_in_us)
```
### 3. Close the communication
+ Using **udp_init** function to close the communication
```
udp_close()
```
