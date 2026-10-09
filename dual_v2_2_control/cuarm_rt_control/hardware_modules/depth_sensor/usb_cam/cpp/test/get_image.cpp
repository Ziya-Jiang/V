
#include "usb_cam.h"
#include "opencv2/opencv.hpp"

int main(int argc, char** argv){
    UsbCam cam(0);
    cv::namedWindow("Image", cv::WINDOW_NORMAL);
    while (true){
        cv::Mat color = cam.getColorImage();
        cam.saveImage(color, "test.jpg");
        cv::imshow("Image", color);
        cv::waitKey(10);
    }
    return 0;
}