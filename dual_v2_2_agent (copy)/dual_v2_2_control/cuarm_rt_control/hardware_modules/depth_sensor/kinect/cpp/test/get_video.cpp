

#include <assert.h>
#include <iostream>

#include "kinect.h"

int main()
{
    Kinect cam;
    int fps = 30;
    cam.startVideoWriter("test", fps);
    cv::Mat frame;
    int frame_count = 0;
    while (frame_count < 100){
        cam.update();
        frame = cam.getColorImage();
        cam.writeVideoFrame(frame);
        cv::waitKey(10);
        printf("frame_count: %d\n", frame_count);
        frame_count++;
    }
    cam.stopVideoWriter();
    return 0;
}