//
// Created by rob-ot on 02.12.19.
//

#ifndef LIBUNICAM_FREENECTPROVIDER_H
#define LIBUNICAM_FREENECTPROVIDER_H


#include "unicam/UnicamDeviceProvider.h"
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>
#include <signal.h>

class FreenectProvider : public UnicamDeviceProvider {
public:
    struct frameData {
        cv::Mat depthFrame;
        cv::Mat irFrame;
        std::string timestamp;
        float centralDistance;
        double frameSize = 0.0;
    };

    void initializeCameras() override;
    std::list<std::string> getConnectedCameraTags() override; //returns the serial number or a number representing the camera
    UnicamCamera * getCameraByTag(std::string cameraTag) override ;
    UnicamDevices getCameraType() ; //Returns the type of current camera
    bool isMultSupported(); //Returns true if multicam is supported
    void spinOnce() override ;

private:
    libfreenect2::Freenect2Device *devtopause;
    libfreenect2::Freenect2 freenect2;
    libfreenect2::Freenect2Device *dev = 0;
    libfreenect2::PacketPipeline *pipeline = 0;

    std::string serial = "";

    bool enable_rgb = true;
    bool enable_depth = true;
    int deviceId = -1;
    size_t framemax = -1;
    libfreenect2::SyncMultiFrameListener* listener;
    libfreenect2::FrameMap frames;


};


#endif //LIBUNICAM_FREENECTPROVIDER_H
