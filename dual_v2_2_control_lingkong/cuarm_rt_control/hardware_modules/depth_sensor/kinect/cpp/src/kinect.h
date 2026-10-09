#ifndef KINECT_H
#define KINECT_H

#include <string>

#include <k4a/k4a.hpp>
#include "opencv2/opencv.hpp"

class Kinect{
    public:
        Kinect();
        ~Kinect();
        bool update(); //assumes an inifinate timeout.
        bool update(int timeout_ms); //If set to 0, the function will return without blocking
        void setConfig(k4a_device_configuration_t config);
        cv::Mat getColorImage();
        cv::Mat getDepthImage();
        void saveImage(cv::Mat& img, const std::string& path);
        void startVideoWriter(const std::string& filename, double fps);
        void writeVideoFrame(const cv::Mat& img);
        void stopVideoWriter();
    private:
        void convertFrameSize(k4a_color_resolution_t resolution);
        cv::Mat convertToMat(k4a::image& src, bool deep_copy = true);
        k4a::device device_;
        k4a::capture capture_;
        k4a::calibration calibration_;
        cv::VideoWriter video_writer_;
        cv::Size frame_size_;
};

#endif