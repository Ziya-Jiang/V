#include <atomic>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <string>

#if defined(_WIN32) || defined(WIN32)
#include <windows.h>
#else
#include <csignal>
#endif

#include "cuarm_tcp.h"

namespace {

struct TcpTestCommand {
    char message[128];
};

struct TcpTestResponse {
    char message[128];
};

void pack_response(TcpTestResponse* response, char* send_buffer, int buffer_size)
{
    std::snprintf(send_buffer, buffer_size, "%s", response->message);
}

void unpack_command(TcpTestCommand* command, char* receive_buffer)
{
    std::strncpy(command->message, receive_buffer, 127);
    command->message[127] = '\0'; 
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
 
    const std::string kServerIP = "127.0.0.1";
    constexpr int kServerPort = 30001;
    constexpr int kBufferSize = 1024;
    constexpr int kSelectTimeoutUsec = 1000000;

    try {
        CuarmTcp<TcpTestCommand, TcpTestResponse> server(
            kServerIP,
            kServerPort,
            true,
            unpack_command,
            pack_response,
            kBufferSize);

        std::cout << "Initializing TCP Server on port " << kServerPort << "..." << std::endl;

        while (!stop_requested) {
            if (!server.server_has_client()) {
                std::cout << "Waiting for Client..." << std::endl;
                if (!server.server_wait_client(-1)) {
                    continue;
                }
                std::cout << "Client connected. Waiting for commands..." << std::endl;
            } else {
                TcpTestCommand command{};
                int ret = server.receive(&command, kSelectTimeoutUsec);
                if (ret == 0) {
                    std::cout << "Executing: " << command.message << std::endl;

                    TcpTestResponse response{};
                    std::strncpy(response.message, "COMMAND_DONE", 127);
                    server.send(&response);
                } else if (ret == -1) {
                    std::cout << "Client disconnected." << std::endl;
                }
            }
        }

        server.close();
        std::cout << "TCP server closed." << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "TCP server error: " << e.what() << std::endl;
        return 1;
    }

// Bypassing global exit teardown to prevent runtime library mismatch with Conda dependencies.
#if defined(_DEBUG) || !defined(NDEBUG)
    std::_Exit(0); 
#else
    return 0;
#endif
}
