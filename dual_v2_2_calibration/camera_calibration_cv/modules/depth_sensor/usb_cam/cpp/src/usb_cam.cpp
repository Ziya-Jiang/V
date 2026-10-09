#include "usb_cam.h"

UsbCam::UsbCam(int id){
    capture_ = cv::VideoCapture(id, cv::CAP_V4L2);
    if (!capture_.isOpened()) { //check if video device has been initialised
        std::cout << "cannot open camera";
    }

    // Set the buffer size
    capture_.set(cv::CAP_PROP_BUFFERSIZE, 2);

    // You can check camera supported frame rate and resolution by typing terminal command"v4l2-ctl --list-formats-ext"
    // Set the frame rate
    int frameRate = 30; // desired frame rate in frames per second (fps)
    capture_.set(cv::CAP_PROP_FPS, frameRate);

    // Set the resolution
    int width = 640;  // desired width of the video frame
    int height = 480; // desired height of the video frame
    capture_.set(cv::CAP_PROP_FRAME_WIDTH, width);
    capture_.set(cv::CAP_PROP_FRAME_HEIGHT, height);
    frame_size_ = cv::Size(width, height);

    // // Verify the settings
    // double actual_frame_rate = capture_.get(cv::CAP_PROP_FPS);
    // double actual_width = capture_.get(cv::CAP_PROP_FRAME_WIDTH);
    // double actual_height = capture_.get(cv::CAP_PROP_FRAME_HEIGHT);
    // double acutal_buffer_size = capture_.get(cv::CAP_PROP_BUFFERSIZE);
    // std::cout << "Frame Rate: " << actual_frame_rate << std::endl;
    // std::cout << "Resolution: " << actual_width << "x" << actual_height << std::endl;
    // std::cout << "Buffer Size: " << acutal_buffer_size << std::endl;
}

UsbCam::~UsbCam(){
    capture_.release();
    video_writer_.release();
}

cv::Mat UsbCam::getColorImage(){
    cv::Mat frame;
    bool success = capture_.read(frame);
    return frame;
}

void UsbCam::saveImage(cv::Mat& img, const std::string& path){
    cv::imwrite(path, img);
}

void UsbCam::startVideoWriter(const std::string& filename, double fps){
    int codec = cv::VideoWriter::fourcc('X', 'V', 'I', 'D'); // Codec for AVI format
    video_writer_ = cv::VideoWriter(filename+".avi", codec, fps, frame_size_);
}

void UsbCam::writeVideoFrame(const cv::Mat& img){
    video_writer_.write(img);
}

void UsbCam::stopVideoWriter(){
    video_writer_.release();
}
