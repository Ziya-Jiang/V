#ifndef USB_CAM_H
#define USB_CAM_H

#include <string>
#include "opencv2/opencv.hpp"
class UsbCam{
    public:
        UsbCam(int id);
        ~UsbCam();
        cv::Mat getColorImage();
        void saveImage(cv::Mat& img, const std::string& path);
        void startVideoWriter(const std::string& filename, double fps);
        void writeVideoFrame(const cv::Mat& img);
        void stopVideoWriter();
    private:
        cv::VideoCapture capture_;
        cv::VideoWriter video_writer_;
        cv::Size frame_size_;
};

#endif