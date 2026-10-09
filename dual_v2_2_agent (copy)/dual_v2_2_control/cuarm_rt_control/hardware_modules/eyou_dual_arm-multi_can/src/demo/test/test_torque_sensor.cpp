#include <iostream>
#include <thread>
#include <iomanip>
#include <csignal>
#include "torque_sensor.h"
#include "eyou_joints.h"
#include "config_loader.h"
#include "usb_to_can.h"
#include "callback_dispatcher.h"
#include <signal.h>
#include "usb_to_can.h"
//////////////////////////////////////////////////////////sensor
bool exit_requested = false;

void signal_handler(int signal)
{
    exit_requested = true;
}
int main()
{
    signal(SIGINT, signal_handler);
    //Load configuration form config.yaml
    std::string base_path = PROJECT_SOURCE_DIR;
    ConfigLoader config_loader(base_path + "/config.yaml");
    RobotConfig& robot_config = config_loader.getRobotConfig();
    if (robot_config.torque_sensors[0].device_type == DEVICE_CANABLE)
    {
        canbox_init({robot_config.torque_sensors[0].can_channel, robot_config.torque_sensors[1].can_channel});
    }
    else if (robot_config.torque_sensors[0].device_type == DEVICE_USBCANII)
    {
        serial_can_open(CH0|CH1);
    }
    // Init eyou joints class, arm[0] is left arm, arm[1] is the right arm
    EyouJoints arm[NUM_ARM];
    TorSensor torque_sensors[NUM_ARM];
    for (int i = 0; i < NUM_ARM; ++i)
    {
        new (&torque_sensors[i]) TorSensor((CanDeviceType)(robot_config.torque_sensors[i].device_type), i, NUM_JOINTS_PER_ARM,
                                            robot_config.torque_sensors[i].ids.data(), robot_config.torque_sensors[i].can_channel);
    }
    


    std::ofstream fwrite_data("torque_sensor.txt");
    //------力传感器
    double torque_sensor_read[NUM_ARM][NUM_JOINTS_PER_ARM] = {0.0};
    double torque_sensor_zeropoint[NUM_ARM][NUM_JOINTS_PER_ARM] = {0.0};
    double current_zero[NUM_ARM][NUM_JOINTS_PER_ARM] = {0.0};
    double  joint_cur[NUM_ARM][NUM_JOINTS_PER_ARM] = {0.0};
    double dt = 0.01;
    for (int step = 0; step < 10; step++)
    {
        for (int i = 0; i < NUM_ARM; i++)
        {
            for(int j = 0;j<NUM_JOINTS_PER_ARM;++j)
            {
                if (!robot_config.torque_sensors[i].enabled[j]) continue;
                torque_sensors[i].read_sensor(j);
                torque_sensor_zeropoint[i][j] = torque_sensors[i].get_torque_data(j);
            }
        }
        std::this_thread::sleep_for(std::chrono::microseconds((int64_t)(dt*1000)));
    }
    
    double t =dt;
    int T = 10;
    auto start = std::chrono::steady_clock::now();
    auto next_time = start;
    while (t<T&&!exit_requested)
    {
        next_time += std::chrono::milliseconds((int64_t)(dt * 1000));
        for (int i = 0;i<NUM_ARM;++i)
        {
            for (int j = 0; j < NUM_JOINTS_PER_ARM; ++j)
            {
                if (!robot_config.torque_sensors[i].enabled[j]) continue;
                torque_sensors[i].read_sensor(j);
                torque_sensor_read[i][j] = torque_sensors[i].get_torque_data(j) - torque_sensor_zeropoint[i][j];
                std::cout<< "arm " << i << " | joint " << j+1 << " | torque sensor read: " << torque_sensor_read[i][j] << std::endl;
                fwrite_data << torque_sensor_read[i][j] << " ";
            }
        }
        t+=dt;
        fwrite_data << std::endl;
        std::this_thread::sleep_until(next_time);
    }
    fwrite_data.close();
    if (robot_config.torque_sensors[0].device_type == DEVICE_CANABLE)
    {
        canbox_close({robot_config.torque_sensors[0].can_channel, robot_config.torque_sensors[1].can_channel});
    }
    else if (robot_config.torque_sensors[0].device_type == DEVICE_USBCANII)
    {
        serial_can_close(CH0|CH1);
    }
    return 0;
}