#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>

#if defined(_WIN32) || defined(WIN32)
#include <windows.h>
#else
#include <csignal>
#endif

#include "cuarm_udp.h"

namespace {

struct UdpTestMessage {
    char message[128];
};

void pack_message(UdpTestMessage* message, char* send_buffer, int buffer_size)
{
    std::snprintf(send_buffer, buffer_size, "%s", message->message);
}

void unpack_message(UdpTestMessage* message, char* receive_buffer)
{
    std::strncpy(message->message, receive_buffer, 127);
    message->message[127] = '\0';
}

std::atomic<bool> stop_requested(false);

void handle_sig(int)
{
    stop_requested = true;
}

}  // namespace

int main()
{
#if !defined(_WIN32) && !defined(WIN32)
    std::signal(SIGINT, handle_sig);
#endif

    const std::string kLocalIP = "127.0.0.1";
    const std::string kRemoteIP = "127.0.0.1";
    constexpr int kLocalPort = 30002;
    constexpr int kRemotePort = 30003;
    constexpr int kBufferSize = 1024;
    constexpr int kReceiveTimeoutUsec = 200000;
    constexpr auto kSendInterval = std::chrono::seconds(1);

    try {
        CuarmUdp<UdpTestMessage, UdpTestMessage> server(
            kLocalIP,
            kLocalPort,
            kRemoteIP,
            kRemotePort,
            unpack_message,
            pack_message,
            kBufferSize);

        std::cout << "Initializing UDP Server on " << kLocalIP << ":" << kLocalPort
                  << " (remote " << kRemoteIP << ":" << kRemotePort << ")..." << std::endl;
        std::cout << "Sending and receiving continuously. Press Ctrl+C to stop." << std::endl;

        int send_count = 0;
        auto last_send_time = std::chrono::steady_clock::now();

        while (!stop_requested) {
            UdpTestMessage incoming{};
            if (server.receive(&incoming, kReceiveTimeoutUsec)) {
                std::cout << "Received: " << incoming.message << std::endl;

                UdpTestMessage reply{};
                std::snprintf(reply.message, sizeof(reply.message), "SERVER_ACK: %s", incoming.message);
                server.send(&reply);
                std::cout << "Sent: " << reply.message << std::endl;
            }

            const auto now = std::chrono::steady_clock::now();
            if (now - last_send_time >= kSendInterval) {
                UdpTestMessage outgoing{};
                std::snprintf(outgoing.message, sizeof(outgoing.message), "SERVER_HEARTBEAT #%d", ++send_count);
                server.send(&outgoing);
                std::cout << "Sent: " << outgoing.message << std::endl;
                last_send_time = now;
            }
        }

        server.close();
        std::cout << "UDP server closed." << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "UDP server error: " << e.what() << std::endl;
        return 1;
    }

// Bypassing global exit teardown to prevent runtime library mismatch with Conda dependencies.
#if defined(_DEBUG) || !defined(NDEBUG)
    std::_Exit(0);
#else
    return 0;
#endif
}
