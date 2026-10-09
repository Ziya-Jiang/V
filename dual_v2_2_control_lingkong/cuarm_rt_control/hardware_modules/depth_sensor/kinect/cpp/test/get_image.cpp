

#include <assert.h>
#include <iostream>

#include "kinect.h"

int main()
{
    Kinect cam;
    cam.update();
    cv::Mat color = cam.getColorImage();
    cv::Mat depth = cam.getDepthImage();
    cam.saveImage(color, "test.png");
    return 0;
}