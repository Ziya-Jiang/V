#include <cstdio>
#include <cstring>
#include <iostream>
#include <string>

#include "cuarm_tcp.h"

namespace {

struct TcpTestCommand {
    char message[128];
};

struct TcpTestResponse {
    char message[128];
};

void pack_command(TcpTestCommand* command, char* send_buffer, int buffer_size)
{
    std::snprintf(send_buffer, buffer_size, "%s", command->message);
}

void unpack_response(TcpTestResponse* response, char* receive_buffer)
{
    std::strncpy(response->message, receive_buffer, 127);
    response->message[127] = '\0'; 
}

}  // namespace

int main()
{
    const std::string kServerIP = "127.0.0.1";
    constexpr int kServerPort = 30001;
    constexpr int kBufferSize = 1024;
    constexpr int kResponseTimeoutUsec = 2000000;
    std::cout << "Connecting to TCP server at 127.0.0.1:" << kServerPort << "..." << std::endl;

    CuarmTcp<TcpTestResponse, TcpTestCommand> client(
        kServerIP,
        kServerPort,
        false,
        unpack_response,
        pack_command,
        kBufferSize);

    TcpTestCommand command{};
    std::strncpy(command.message, "MOVE_J 0,0,0", 127);
    std::cout << "Sending command: " << command.message << std::endl;
    client.send(&command);

    TcpTestResponse response{};
    int ret = client.receive(&response, kResponseTimeoutUsec);
    if (ret == 0) {
        std::cout << "Robot response: " << response.message << std::endl;
    } else if (ret == 1) {
        std::cout << "No response or timeout." << std::endl;
        return 1;
    }

    client.close();
    std::cout << "Connection closed." << std::endl;

// Bypassing global exit teardown to prevent runtime library mismatch with Conda dependencies.
#if defined(_DEBUG) || !defined(NDEBUG)
    std::_Exit(0); 
#else
    return 0;
#endif
}
