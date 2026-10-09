#ifndef MCAST_PRIORITY_SERVER_H
#define MCAST_PRIORITY_SERVER_H

#include <iostream>
#include <memory>

#include "cuarm_state.h"
#include "curi_udp.h"
#include "cuarm_message_handler.h"
#include "priority_queue.h"


// Multicast to all clients and select clients' commands by priority queue
template <typename UnpackT, typename PackT>
class McastPriorityServer{
    public:
        McastPriorityServer(char receive_ip[], int receive_port, char send_ip[], int send_port, 
            void (*unpack_func)(UnpackT*, char*) = nullptr,
            void (*pack_func)(PackT*, char*, int) = nullptr, 
            int buffer_size=4096);
        ~McastPriorityServer();
        void send(PackT* data);
        bool receive(int usec);
        bool popQueue(UnpackT* data);
        void clearQueue();
        void printQueue();
        void close();
        int getCurrentClientId() const;
        CommandPriorityLevel getCurrentClientPriority() const;

    private:
        void (*pack_function_)(PackT*, char*, int) = nullptr;
        void (*unpack_function_)(UnpackT*, char*) = nullptr;
        std::unique_ptr<udp_node> udp_node_ptr_;
        int buffer_size_;
        PriorityQueue<UnpackT> priority_queue_;
        int current_client_id_ = -1;
        CommandPriorityLevel current_client_priority_ = CommandPriorityLevel::kOther;
        long current_client_last_heartbeat_timestamp_ = 0;
        long heartbeat_timeout_us_ = 3000000;
};

template <typename UnpackT, typename PackT>
McastPriorityServer<UnpackT, PackT>::McastPriorityServer(char receive_ip[], int receive_port, char send_ip[], int send_port, 
    void (*unpack_func)(UnpackT*, char*),
    void (*pack_func)(PackT*, char*, int), 
    int buffer_size){
    buffer_size_ = buffer_size;
    udp_node_ptr_ = std::make_unique<udp_node>();

    //Initialize UDP node and Sockets
    int ret = udp_init(udp_node_ptr_.get(), receive_ip, receive_port, send_ip, send_port, buffer_size_);
    if (ret != 0){
        throw std::runtime_error("Failed to initialize UDP node, return code: " + std::to_string(ret));
    }

    //Set for multicast 
    int ttl = 1;
    setsockopt(udp_node_ptr_->send_fd, IPPROTO_IP, IP_MULTICAST_TTL, &ttl, sizeof(ttl));

    pack_function_ = pack_func;
    unpack_function_ = unpack_func;
}

template <typename UnpackT, typename PackT>
McastPriorityServer<UnpackT, PackT>::~McastPriorityServer(){
    close();
}

template <typename UnpackT, typename PackT>
void McastPriorityServer<UnpackT, PackT>::send(PackT* data){
    if (!pack_function_) {
        throw std::runtime_error("Pack function is not defined.");
    }
    
    pack_function_(data, udp_node_ptr_->send_buffer, buffer_size_);
    udp_send(udp_node_ptr_.get(), buffer_size_);
}

template <typename UnpackT, typename PackT>
bool McastPriorityServer<UnpackT, PackT>::receive(int usec){
    if (!unpack_function_) {
        throw std::runtime_error("Unpack function is not defined.");
    }

    bool has_current_client = false;
    udp_node* p = udp_node_ptr_.get();
	FD_SET(p->receive_fd, &(p->rset));
	struct timeval t;
	t.tv_sec = 0;
	t.tv_usec = usec;
	int nready = select(p->receive_fd + 1, &(p->rset), NULL, NULL, &t);
	if (nready > 0 && FD_ISSET(p->receive_fd, &(p->rset))) {
        // Now "drain" the buffer using non-blocking recv until there are no more packets left.
        while (true) {
            // Receive the packet and fill client_addr with sender info
            struct sockaddr_in client_addr;
            socklen_t addr_len = sizeof(client_addr);
            int bytes = recvfrom(p->receive_fd, p->receive_buffer, buffer_size_, MSG_DONTWAIT, (struct sockaddr *)&client_addr, &addr_len);
            unsigned short client_port = ntohs(client_addr.sin_port);
            if (bytes <= 0) break; 

            UnpackT temp;
            unpack_function_(&temp, p->receive_buffer);
            has_current_client |= (client_port == current_client_id_);
            // std::cout <<"bytes: " << bytes << ", client_port: " << client_port << ", priority: " << (int)temp.priority  << std::endl;
            if (temp.priority < current_client_priority_ || current_client_id_ == -1 || client_port == current_client_id_){
                typename PriorityQueue<UnpackT>::QueueData qd;
                qd.id = client_port;
                qd.priority = (int)temp.priority;
                qd.seq = temp.sequence;
                qd.received_at = get_time_now();
                qd.data = std::move(temp);
                priority_queue_.push(qd);
            }else{
                //Drop packet and return a failed connect response
                const char* reply = "Command rejected! Priority too low.";
                sendto(p->receive_fd, reply, strlen(reply), 0, (struct sockaddr*)&client_addr, addr_len);
            }
        }
    }else{
        return false;
    }

    long now = get_time_now();
    if (now - current_client_last_heartbeat_timestamp_ > heartbeat_timeout_us_){
        std::cout << "Heartbeat timeout, resetting current client" << std::endl;
        current_client_id_ = -1;
    }
    if (has_current_client) current_client_last_heartbeat_timestamp_ = now;

    if (priority_queue_.empty()) return false;
    return true;
}

template <typename UnpackT, typename PackT>
void McastPriorityServer<UnpackT, PackT>::close(){
    udp_close(udp_node_ptr_.get());
}

template <typename UnpackT, typename PackT>
bool McastPriorityServer<UnpackT, PackT>::popQueue(UnpackT* data){
    if (priority_queue_.empty()) return false;
    auto qd = priority_queue_.pop();
    *data = std::move(qd.data);

    //Update if new client's command is selected
    if (qd.id != current_client_id_){
        current_client_id_ = qd.id;
        current_client_priority_ = CommandPriorityLevel(qd.priority);
        current_client_last_heartbeat_timestamp_ = get_time_now();
    }

    return true;
}

template <typename UnpackT, typename PackT>
void McastPriorityServer<UnpackT, PackT>::clearQueue(){
    priority_queue_.clear();
}

template <typename UnpackT, typename PackT>
void McastPriorityServer<UnpackT, PackT>::printQueue(){
    priority_queue_.print();
}

template <typename UnpackT, typename PackT>
int McastPriorityServer<UnpackT, PackT>::getCurrentClientId() const{
    return current_client_id_;
}

template <typename UnpackT, typename PackT>
CommandPriorityLevel McastPriorityServer<UnpackT, PackT>::getCurrentClientPriority() const{
    return current_client_priority_;
}
#endif 