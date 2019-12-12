//
// Created by rob-ot on 19.11.19.
//

#include <iostream>
#include "../headers/XtionProvider.h"
#include "../headers/XtionCamera.h"

void XtionProvider::initializeCameras() {
    ni_status = openni::OpenNI::initialize();
    ni_status = device.open(openni::ANY_DEVICE);
    std::cout<<openni::OpenNI::getExtendedError()<<std::endl;
    if (ni_status != openni::STATUS_OK)
    {
        std::cout<<ni_status<<std::endl;
        openni::OpenNI::shutdown();
    }
    depth = new openni::VideoStream();



    ni_status = depth->create(device, openni::SENSOR_DEPTH);  //fixme: crashes here

    const openni::SensorInfo* sinfo = device.getSensorInfo(openni::SENSOR_DEPTH);
    const openni::Array< openni::VideoMode>& modesDepth = sinfo->getSupportedVideoModes();
    depth->stop();
    depth->setVideoMode(modesDepth[0]);


    if (ni_status == openni::STATUS_OK)
    {
        ni_status = depth->start();
        if (ni_status != openni::STATUS_OK)
        {
            //ROS_WARN("SimpleViewer: Couldn't start depth stream:\n%s", openni::OpenNI::getExtendedError());
            depth->destroy();
        }
    }
    else
    {
        //ROS_WARN("SimpleViewer: Couldn't find depth stream:\n%s", openni::OpenNI::getExtendedError());
    }


    if (!depth->isValid())
    {
        //ROS_ERROR("SimpleViewer: No valid streams. Exiting");
        openni::OpenNI::shutdown();
    }

    iface = new unicam_xtion::IOInterface(device, *depth);

    iface->initialize();
    iface->spinOnce();
    //openni::OpenNI::shutdown(); //todo: remove shutdown

}

std::list<std::string> XtionProvider::getConnectedCameraTags() {
    return std::list<std::string>();
}

UnicamCamera *XtionProvider::getCameraByTag(std::string cameraTag) {
    UnicamCamera* oneCamera = new XtionCamera(iface);
    return oneCamera;
}

UnicamDevices XtionProvider::getCameraType() {
    return UnicamDeviceProvider::getCameraType();
}

bool XtionProvider::isMultSupported() {
    return UnicamDeviceProvider::isMultSupported();
}

void XtionProvider::spinOnce() {
    iface->spinOnce();
}

XtionProvider::XtionProvider() {
}
