#ifndef CUARM_MESSAGE_HANDLER_H
#define CUARM_MESSAGE_HANDLER_H

#include <cstring>   // for strcpy
#include <type_traits> // for is_same_v
#include <stdexcept> // for runtime_error

#include "cuarm_state.h"

struct CuarmMessageHandler{
    static void pack_panel_command(PanelCommand* panel, char* send_buffer, int buffer_size);
    static void unpack_panel_command(PanelCommand* panel, char* receive_buffer);
    static void pack_panel_state(PanelState* state, char* send_buffer, int buffer_size);
    static void unpack_panel_state(PanelState* state, char* receive_buffer);

    static void pack_planner_command(PlannerCommand* command, char* send_buffer, int buffer_size);
    static void unpack_planner_command(PlannerCommand* command, char* receive_buffer);
    static void pack_planner_state(PlannerState* state, char* send_buffer, int buffer_size);
    static void unpack_planner_state(PlannerState* state, char* receive_buffer);

    //Panel Remote Page
    static void pack_remote_panel_command(RemotePanelCommand* command, char* send_buffer, int buffer_size);
    static void unpack_remote_panel_command(RemotePanelCommand* command, char* receive_buffer);

    //Rt config 
    static void pack_panel_config_command(PanelConfigCommand* command, char* send_buffer, int buffer_size);
    static void unpack_panel_config_command(PanelConfigCommand* command, char* receive_buffer);
    static void pack_rt_config_state(RtConfigState* state, char* send_buffer, int buffer_size);
    static void unpack_rt_config_state(RtConfigState* state, char* receive_buffer);

    private:
        /* gettimeofday() does not appear on linux without this. */
        // #define _BSD_SOURCE
        static int get_next_data_index(char* receive_buffer, int index);

        template<typename T>
        static void get_scan_format(char* buffer);

        template<typename T1, typename T2>
        static void scan_type(const char buffer[], int index, T1& destination);
};

template<typename T>
void CuarmMessageHandler::get_scan_format(char* buffer) {
    if constexpr (std::is_same_v<T, int>) strcpy(buffer, "%d");
    else if constexpr (std::is_same_v<T, float>) strcpy(buffer, "%f");
    else if constexpr (std::is_same_v<T, double>) strcpy(buffer, "%lf");
    else if constexpr (std::is_same_v<T, long>) strcpy(buffer, "%ld");
    else if constexpr (std::is_same_v<T, short>) strcpy(buffer, "%hd");
    else if constexpr (std::is_same_v<T, uint8_t>) strcpy(buffer, "%hhu");
    else if constexpr (std::is_same_v<T, char>) strcpy(buffer, "%c");
    else throw std::runtime_error("Unsupported type for get_scan_format");
    strcat(buffer, "#");
};

template<typename T1, typename T2>
void CuarmMessageHandler::scan_type(const char buffer[], int index, T1& destination) {
    T2 tmp;
    char fmt[10];
    get_scan_format<T2>(fmt);
    sscanf(&(buffer[index]), fmt, &tmp);
    destination = static_cast<T1>(tmp);
};

#endif