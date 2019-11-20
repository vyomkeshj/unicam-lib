//
// Created by rob-ot on 19.11.19.
//

#include "../headers/XtionProvider.h"
#include "../headers/XtionCamera.h"

void XtionProvider::initializeCameras() {
    ni_status = openni::OpenNI::initialize();
    ni_status = device.open(openni::ANY_DEVICE);
    if (ni_status != openni::STATUS_OK)
    {
        openni::OpenNI::shutdown();
    }
    depth = new openni::VideoStream();
    color = new openni::VideoStream();

    ni_status = depth->create(device, openni::SENSOR_DEPTH);  //fixme: crashes here
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

    ni_status = color->create(device, openni::SENSOR_COLOR);
    if (ni_status == openni::STATUS_OK)
    {
        ni_status = color->start();
        if (ni_status != openni::STATUS_OK)
        {
            //ROS_WARN("SimpleViewer: Couldn't start color stream:\n%s", openni::OpenNI::getExtendedError());
            color->destroy();
        }
    }
    else
    {
        //ROS_WARN("SimpleViewer: Couldn't find color stream:\n%s", openni::OpenNI::getExtendedError());
    }

    if (!depth->isValid() || !color->isValid())
    {
        //ROS_ERROR("SimpleViewer: No valid streams. Exiting");
        openni::OpenNI::shutdown();
    }

    iface = new unicam_xtion::IOInterface(device, *depth, *color);

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
