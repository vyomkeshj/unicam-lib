//
// Created by rob-ot on 15.11.19.
//

#ifndef LIBUNICAM_XTIONPROVIDER_H
#define LIBUNICAM_XTIONPROVIDER_H


#include "unicam/UnicamDeviceProvider.h"
#include "../headers/io_interface.h"
#include "frame_data.h"

class XtionProvider : public UnicamDeviceProvider{
public:
    XtionProvider();
    void initializeCameras();
    std::list<std::string> getConnectedCameraTags(); //returns the serial number or a number representing the camera
    UnicamCamera * getCameraByTag(std::string cameraTag);
    UnicamDevices getCameraType(); //Returns the type of current camera
    bool isMultSupported(); //Returns true if multicam is supported

    void spinOnce();

private:
    std::string camera_name;
    std::string device_uri;

    openni::Status ni_status = openni::STATUS_OK;
    openni::Device device;
    openni::VideoStream *depth;


    unicam_xtion::IOInterface* iface;
};



#endif //LIBUNICAM_XTIONPROVIDER_H
