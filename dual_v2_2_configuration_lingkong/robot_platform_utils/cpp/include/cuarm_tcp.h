#ifndef CUARM_TCP_H
#define CUARM_TCP_H

#include <iostream>

#include "cuarm_state.h"
#include "cuarm_message_handler.h"

extern "C" {
    #include "curi_tcp/c/src/curi_tcp.h"
}
template <typename UnpackT, typename PackT>
class CuarmTcp{
    public:
        CuarmTcp(const std::string& server_ip, int server_port, bool is_server,
            void (*unpack_func)(UnpackT*, char*) = nullptr,
            void (*pack_func)(PackT*, char*, int) = nullptr, 
            int buffer_size=4096);
        ~CuarmTcp();
        void send(PackT* data);
        int  receive(UnpackT* data, int timeout_usec);
        bool server_has_client();
        bool server_wait_client(int timeout_usec);
        void close();

    private:
        void (*pack_function_)(PackT*, char*, int) = nullptr;
        void (*unpack_function_)(UnpackT*, char*) = nullptr;
        tcp_node* tcp_node_ptr_ = nullptr; // Use a raw pointer so C++ cannot auto-delete the C struct
        int buffer_size_;
        bool is_server_;
        std::string server_ip_;
        bool is_closed_ = false; 
};

template <typename UnpackT, typename PackT>
CuarmTcp<UnpackT, PackT>::CuarmTcp(const std::string& server_ip, int server_port, bool is_server,
    void (*unpack_func)(UnpackT*, char*),
    void (*pack_func)(PackT*, char*, int), 
    int buffer_size){
    buffer_size_ = buffer_size;    

    // Allocate memory for the tcp_node struct
    tcp_node_ptr_ = static_cast<tcp_node*>(std::malloc(sizeof(tcp_node)));
    if (!tcp_node_ptr_) {
        throw std::runtime_error("Failed to allocate memory for tcp_node struct.");
    }
    std::memset(tcp_node_ptr_, 0, sizeof(tcp_node));

    pack_function_ = pack_func;
    unpack_function_ = unpack_func;
    is_server_ = is_server;
    server_ip_ = server_ip;
    
    //Initialize TCP node and Sockets
    int ret = tcp_init(tcp_node_ptr_, server_ip_.c_str(), server_port, buffer_size_, is_server);
    if (ret != 0){
        if (is_server){
            throw std::runtime_error("Failed to initialize TCP server, return code: " + std::to_string(ret));
        }else{
            throw std::runtime_error("TCP Connection Failed, return code: " + std::to_string(ret));
        }
    }
}

template <typename UnpackT, typename PackT>
CuarmTcp<UnpackT, PackT>::~CuarmTcp(){
    close();
}

template <typename UnpackT, typename PackT>
void CuarmTcp<UnpackT, PackT>::send(PackT* data){
    if (!pack_function_) {
        throw std::runtime_error("TCP Pack function is not defined.");
    }
    
    pack_function_(data, tcp_node_ptr_->send_buffer, buffer_size_);
    tcp_send(tcp_node_ptr_, buffer_size_);
}

template <typename UnpackT, typename PackT>
int CuarmTcp<UnpackT, PackT>::receive(UnpackT* data, int timeout_usec){
    if (!unpack_function_) {
        throw std::runtime_error("TCP Unpack function is not defined.");
    }

    int bytes = tcp_select(tcp_node_ptr_, timeout_usec, buffer_size_);
    if (bytes > 0) {
        unpack_function_(data, tcp_node_ptr_->receive_buffer);
        return 0;
    }else if (bytes != -4){ // -4 is timeout
        if (is_server_) {
            // std::cout << "TCP Client disconnected. Available for new client..." << std::endl;
            tcp_server_clear_client(tcp_node_ptr_);
            return -1;
        }else{
            throw std::runtime_error("TCP Connection lost.");
        }
    }
    return 1; //Receive timeout
}

template <typename UnpackT, typename PackT>
bool CuarmTcp<UnpackT, PackT>::server_has_client(){
    return tcp_server_has_client(tcp_node_ptr_) == 1;
}

template <typename UnpackT, typename PackT>
bool CuarmTcp<UnpackT, PackT>::server_wait_client(int timeout_usec){
    int ret = tcp_server_wait_client(tcp_node_ptr_, timeout_usec, buffer_size_);
    if (ret < 0) {
        throw std::runtime_error("TCP Error waiting for client: " + std::to_string(ret));
    }else if (ret == 0){
        // std::cout << "TCP Client connected." << std::endl;
        return true;
    }
    return false; //Timeout
}

template <typename UnpackT, typename PackT>
void CuarmTcp<UnpackT, PackT>::close(){
    if (is_closed_) return;

    if (tcp_node_ptr_) {
        tcp_close(tcp_node_ptr_);
        std::free(tcp_node_ptr_);
    }
    is_closed_ = true;
}

#endif //CUARM_UDP_H