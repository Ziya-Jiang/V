#include "kinect.h"

Kinect::Kinect(){
    k4a_device_configuration_t device_config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    k4a_color_resolution_t resolution = K4A_COLOR_RESOLUTION_720P;
    convertFrameSize(resolution);
    
    device_config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
    device_config.color_resolution = resolution;
    device_config.depth_mode = K4A_DEPTH_MODE_NFOV_2X2BINNED;
    device_config.synchronized_images_only = true;
    device_ = k4a::device::open(K4A_DEVICE_DEFAULT);
    device_.start_cameras(&device_config);

    calibration_ = device_.get_calibration(device_config.depth_mode, device_config.color_resolution);
    update();
}

Kinect::~Kinect(){
    device_.close();
    video_writer_.release();
}

bool Kinect::update(){
    return device_.get_capture(&capture_);
}


bool Kinect::update(int timeout_ms){
    return device_.get_capture(&capture_, std::chrono::milliseconds(timeout_ms));
}

void Kinect::setConfig(k4a_device_configuration_t config){
    device_.stop_cameras();
    device_.start_cameras(&config);
}

cv::Mat Kinect::getColorImage(){
    k4a::image rbg = capture_.get_color_image();
    if (rbg.is_valid())
        return convertToMat(rbg);
    return cv::Mat();
}

cv::Mat Kinect::getDepthImage(){
    k4a::image depth = capture_.get_depth_image();
    if (depth.is_valid())
        return convertToMat(depth);
    return cv::Mat();
}

void Kinect::saveImage(cv::Mat& img, const std::string& path){
    cv::imwrite(path, img);
}

void Kinect::convertFrameSize(k4a_color_resolution_t resolution){
    switch (resolution){
        case K4A_COLOR_RESOLUTION_720P:
            frame_size_ = cv::Size(1280, 720);
            break;
        case K4A_COLOR_RESOLUTION_1080P:
            frame_size_ = cv::Size(1920, 1080);
            break;
        case K4A_COLOR_RESOLUTION_1440P:
            frame_size_ = cv::Size(2560, 1440);
            break;
        case K4A_COLOR_RESOLUTION_1536P:
            frame_size_ = cv::Size(2048, 1536);
            break;
        case K4A_COLOR_RESOLUTION_2160P:
            frame_size_ = cv::Size(3840, 2160);
            break;
        case K4A_COLOR_RESOLUTION_3072P:
            frame_size_ = cv::Size(4096, 3072);
            break;
        default:
            frame_size_ = cv::Size(1280, 720);
            break;
    }
}

cv::Mat Kinect::convertToMat(k4a::image& src, bool deep_copy){
    {
        assert( src.get_size() != 0 );

        cv::Mat mat;
        const int32_t width = src.get_width_pixels();
        const int32_t height = src.get_height_pixels();

        const k4a_image_format_t format = src.get_format();
        switch( format )
        {
            case k4a_image_format_t::K4A_IMAGE_FORMAT_COLOR_MJPG:
            {
                // NOTE: this is slower than other formats.
                std::vector<uint8_t> buffer( src.get_buffer(), src.get_buffer() + src.get_size() );
                mat = cv::imdecode( buffer, cv::IMREAD_ANYCOLOR );
                cv::cvtColor( mat, mat, cv::COLOR_BGR2BGRA );
                break;
            }
            case k4a_image_format_t::K4A_IMAGE_FORMAT_COLOR_NV12:
            {
                cv::Mat nv12 = cv::Mat( height + height / 2, width, CV_8UC1, src.get_buffer() ).clone();
                cv::cvtColor( nv12, mat, cv::COLOR_YUV2BGRA_NV12 );
                break;
            }
            case k4a_image_format_t::K4A_IMAGE_FORMAT_COLOR_YUY2:
            {
                cv::Mat yuy2 = cv::Mat( height, width, CV_8UC2, src.get_buffer() ).clone();
                cv::cvtColor( yuy2, mat, cv::COLOR_YUV2BGRA_YUY2 );
                break;
            }
            case k4a_image_format_t::K4A_IMAGE_FORMAT_COLOR_BGRA32:
            {
                cv::Mat rbga_mat = deep_copy ? cv::Mat( height, width, CV_8UC4, src.get_buffer() ).clone()
                                : cv::Mat( height, width, CV_8UC4, src.get_buffer() );
                cv::cvtColor(rbga_mat, mat, cv::COLOR_BGRA2BGR);
                break;
            }
            case k4a_image_format_t::K4A_IMAGE_FORMAT_DEPTH16:
            case k4a_image_format_t::K4A_IMAGE_FORMAT_IR16:
            {
                mat = deep_copy ? cv::Mat( height, width, CV_16UC1, reinterpret_cast<uint16_t*>( src.get_buffer() ) ).clone()
                                : cv::Mat( height, width, CV_16UC1, reinterpret_cast<uint16_t*>( src.get_buffer() ) );
                break;
            }
            case k4a_image_format_t::K4A_IMAGE_FORMAT_CUSTOM8:
            {
                mat = cv::Mat( height, width, CV_8UC1, src.get_buffer() ).clone();
                break;
            }
            case k4a_image_format_t::K4A_IMAGE_FORMAT_CUSTOM:
            {
                // NOTE: This is opencv_viz module format (cv::viz::WCloud).
                const int16_t* buffer = reinterpret_cast<int16_t*>( src.get_buffer() );
                mat = cv::Mat( height, width, CV_32FC3, cv::Vec3f::all( std::numeric_limits<float>::quiet_NaN() ) );
                mat.forEach<cv::Vec3f>(
                    [&]( cv::Vec3f& point, const int32_t* position ){
                        const int32_t index = ( position[0] * width + position[1] ) * 3;
                        point = cv::Vec3f( buffer[index + 0], buffer[index + 1], buffer[index + 2] );
                    }
                );
                break;
            }
            default:
                throw k4a::error( "Failed to convert this format!" );
                break;
        }
        return mat;
    }
}

void Kinect::startVideoWriter(const std::string& filename, double fps){
    int codec = cv::VideoWriter::fourcc('X', 'V', 'I', 'D'); // Codec for AVI format
    video_writer_ = cv::VideoWriter(filename+".avi", codec, fps, frame_size_);
}

void Kinect::writeVideoFrame(const cv::Mat& img){
    video_writer_.write(img);
}

void Kinect::stopVideoWriter(){
    video_writer_.release();
}
