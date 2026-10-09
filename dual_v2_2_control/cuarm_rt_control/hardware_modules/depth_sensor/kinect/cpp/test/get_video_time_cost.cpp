
#include "kinect.h"
#include "opencv2/opencv.hpp"
#include <chrono>
#include <thread>

int main(int argc, char** argv){
    int fps = 30;
    Kinect cam;
    cam.startVideoWriter("test", fps);
    cv::namedWindow("Image", cv::WINDOW_NORMAL);

    cv::Mat color;
    int delay_us = 35000;
    std::chrono::steady_clock::time_point t0, t1, t2, t3;
    t0 = std::chrono::steady_clock::now();
    int count = 0;
    while (true){
        cam.update();
        color = cam.getColorImage();
        t1 = std::chrono::steady_clock::now();

        cam.writeVideoFrame(color);
        t2 = std::chrono::steady_clock::now();

        int sleep_time = delay_us - std::chrono::duration_cast<std::chrono::microseconds>(t2 - t0).count();
        if (sleep_time > 0)
            std::this_thread::sleep_for(std::chrono::microseconds(sleep_time));
        t3 = std::chrono::steady_clock::now();

        std::cout << count << ", ";
        std::cout << "Capture cost[µs] = " << std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count() << ", ";
        std::cout << "Write cost[µs] = "   << std::chrono::duration_cast<std::chrono::microseconds>(t2 - t0).count() << ", ";
        std::cout << "Total cost[µs] = "   << std::chrono::duration_cast<std::chrono::microseconds>(t3 - t0).count() << std::endl;
        t0 = t3;
        count++;
    }
    return 0;
}