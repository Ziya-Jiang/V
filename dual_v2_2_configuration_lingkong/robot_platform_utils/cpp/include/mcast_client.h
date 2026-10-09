#ifndef MCAST_CLIENT_H
#define MCAST_CLIENT_H

#include <iostream>
#include <memory>
#include <cstdint>

#include "cuarm_state.h"
#include "curi_udp.h"
#include "cuarm_message_handler.h"

template <typename UnpackT, typename PackT>
class McastClient{
    public:
        McastClient(char receive_ip[], int receive_port, char send_ip[], int send_port, 
            void (*unpack_func)(UnpackT*, char*) = nullptr,
            void (*pack_func)(PackT*, char*, int) = nullptr, 
            int buffer_size=4096);
        ~McastClient();
        void send(PackT* data);
        bool receive(UnpackT* data, int usec);
        void close();

    private:
        void (*pack_function_)(PackT*, char*, int) = nullptr;
        void (*unpack_function_)(UnpackT*, char*) = nullptr;
        std::unique_ptr<udp_node> udp_node_ptr_;
        int buffer_size_;
        bool first_send_success_ = false;
};

template <typename UnpackT, typename PackT>
McastClient<UnpackT, PackT>::McastClient(char receive_ip[], int receive_port, char send_ip[], int send_port, 
    void (*unpack_func)(UnpackT*, char*),
    void (*pack_func)(PackT*, char*, int), 
    int buffer_size){
    buffer_size_ = buffer_size;
    udp_node_ptr_ = std::make_unique<udp_node>();

    //Initialize UDP node and Sockets
    int receive_fd = socket(AF_INET, SOCK_DGRAM, 0);
    int reuse = 1;
    setsockopt(receive_fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)); // Allow reuse of local address before binding
    char ip[] = "0.0.0.0";
    int ret = udp_init1(udp_node_ptr_.get(), ip, receive_port, send_ip, send_port, buffer_size_, receive_fd);
    if (ret != 0){
        throw std::runtime_error("Failed to initialize UDP node, return code: " + std::to_string(ret));
    }

    // Join multicast group only when receive_ip is multicast.
    const in_addr_t addr = inet_addr(receive_ip);
    if (IN_MULTICAST(ntohl(addr))) {
        struct ip_mreq mreq;
        mreq.imr_multiaddr.s_addr = addr;
        mreq.imr_interface.s_addr = htonl(INADDR_ANY);
        if (setsockopt(udp_node_ptr_->receive_fd, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq)) < 0) {
            perror("setsockopt IP_ADD_MEMBERSHIP");
        }
    }

    pack_function_ = pack_func;
    unpack_function_ = unpack_func;
}

template <typename UnpackT, typename PackT>
McastClient<UnpackT, PackT>::~McastClient(){
    close();
}

template <typename UnpackT, typename PackT>
void McastClient<UnpackT, PackT>::send(PackT* data){
    if (!pack_function_) {
        throw std::runtime_error("Pack function is not defined.");
    }
    
    pack_function_(data, udp_node_ptr_->send_buffer, buffer_size_);
    udp_send(udp_node_ptr_.get(), buffer_size_);

    char buffer[1024];
    struct sockaddr_in send_addr = udp_node_ptr_->send_addr;
    socklen_t addr_len = sizeof(send_addr);
    ssize_t bytes = recvfrom(udp_node_ptr_->send_fd, buffer, sizeof(buffer) - 1, MSG_DONTWAIT, (struct sockaddr *)&send_addr, &addr_len);

    if (bytes > 0) {
        buffer[bytes] = '\0'; 
        std::cout << "Received feedback: " << buffer << std::endl;
    }
}

template <typename UnpackT, typename PackT>
bool McastClient<UnpackT, PackT>::receive(UnpackT* data, int usec){
    if (!unpack_function_) {
        throw std::runtime_error("Unpack function is not defined.");
    }

    if (udp_select(udp_node_ptr_.get(), usec, buffer_size_)) {
        unpack_function_(data, udp_node_ptr_->receive_buffer);
        return true;
    }
    return false;
}

template <typename UnpackT, typename PackT>
void McastClient<UnpackT, PackT>::close(){
    udp_close(udp_node_ptr_.get());
}

#endif