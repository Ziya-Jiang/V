#ifndef CUARM_UDP_H
#define CUARM_UDP_H

#include <cstring>
#include <iostream>
#include <memory>

#include "curi_udp/c/curi_udp.h"
#include "cuarm_state.h"
#include "cuarm_message_handler.h"

template <typename UnpackT, typename PackT>
class CuarmUdp{
    public:
        CuarmUdp(const std::string& local_ip, int local_port, const std::string& remote_ip, int remote_port, 
            void (*unpack_func)(UnpackT*, char*) = nullptr,
            void (*pack_func)(PackT*, char*, int) = nullptr, 
            int buffer_size=4096);
        ~CuarmUdp();
        void send(PackT* data);
        bool receive(UnpackT* data, int usec);
        void close();

    private:
        void (*pack_function_)(PackT*, char*, int) = nullptr;
        void (*unpack_function_)(UnpackT*, char*) = nullptr;
        std::unique_ptr<udp_node> udp_node_ptr_;
        int buffer_size_;
        std::string local_ip_;
        std::string remote_ip_;
};

template <typename UnpackT, typename PackT>
CuarmUdp<UnpackT, PackT>::CuarmUdp(const std::string& local_ip, int local_port, const std::string& remote_ip, int remote_port, 
    void (*unpack_func)(UnpackT*, char*),
    void (*pack_func)(PackT*, char*, int), 
    int buffer_size){
    buffer_size_ = buffer_size;
    udp_node_ptr_ = std::make_unique<udp_node>();
    local_ip_ = local_ip;
    remote_ip_ = remote_ip;
    
    //Initialize UDP node and Sockets
    int ret = udp_init(udp_node_ptr_.get(), local_ip_.c_str(), local_port, remote_ip_.c_str(), remote_port, buffer_size_);
    if (ret != 0){
        throw std::runtime_error("Failed to initialize UDP node, return code: " + std::to_string(ret));
    }

    pack_function_ = pack_func;
    unpack_function_ = unpack_func;
}

template <typename UnpackT, typename PackT>
CuarmUdp<UnpackT, PackT>::~CuarmUdp(){
    close();
}

template <typename UnpackT, typename PackT>
void CuarmUdp<UnpackT, PackT>::send(PackT* data){
    if (!pack_function_) {
        throw std::runtime_error("Pack function is not defined.");
    }

    std::memset(udp_node_ptr_->send_buffer, 0, static_cast<std::size_t>(buffer_size_));
    pack_function_(data, udp_node_ptr_->send_buffer, buffer_size_);
    const std::size_t payload_size =
        strnlen(udp_node_ptr_->send_buffer, static_cast<std::size_t>(buffer_size_));
    if (payload_size >= static_cast<std::size_t>(buffer_size_)) {
        throw std::runtime_error("Packed UDP payload exceeds the configured buffer.");
    }
    udp_send(udp_node_ptr_.get(), static_cast<int>(payload_size + 1));
}

template <typename UnpackT, typename PackT>
bool CuarmUdp<UnpackT, PackT>::receive(UnpackT* data, int usec){
    if (!unpack_function_) {
        throw std::runtime_error("Unpack function is not defined.");
    }

    if (udp_select(udp_node_ptr_.get(), usec, buffer_size_)) {
        if (udp_node_ptr_->receive_size <= 0 ||
            udp_node_ptr_->receive_size >= buffer_size_ ||
            udp_node_ptr_->receive_buffer[udp_node_ptr_->receive_size] != '\0') {
            return false;
        }
        unpack_function_(data, udp_node_ptr_->receive_buffer);
        return true;
    }
    return false;
}

template <typename UnpackT, typename PackT>
void CuarmUdp<UnpackT, PackT>::close(){
    udp_close(udp_node_ptr_.get());
}

#endif //CUARM_UDP_H