//
// Created by rob-ot on 02.12.19.
//

#include <iostream>
#include "../headers/FreenectProvider.h"

void FreenectProvider::initializeCameras() {
    if (freenect2.enumerateDevices() == 0) {
        std::cout << "no device connected!" << std::endl;
        return;
    }

    if (serial == "") {
        serial = freenect2.getDefaultDeviceSerialNumber();
    }
/// [discovery]

    if (pipeline) {
/// [open]
        dev = freenect2.openDevice(serial, pipeline);
/// [open]
    } else {
        dev = freenect2.openDevice(serial);
    }

    if (dev == 0) {
        std::cout << "failure opening device!" << std::endl;
        return;
    }

    devtopause = dev;
    int types = 0;
    if (enable_rgb)
        types |= libfreenect2::Frame::Color;
    if (enable_depth)
        types |= libfreenect2::Frame::Ir | libfreenect2::Frame::Depth;
    listener = new libfreenect2::SyncMultiFrameListener(types);

    dev->setColorFrameListener(listener);
    dev->setIrAndDepthFrameListener(listener);
/// [listeners]

/// [start]
    if (enable_rgb && enable_depth) {
        if (!dev->start())
            return;
    } else {
        if (!dev->startStreams(enable_rgb, enable_depth))
            return;
    }

}

void FreenectProvider::spinOnce() {
    if (!listener->waitForNewFrame(frames, 10*1000)) // 10 sconds
    {
        std::cout << "timeout!" << std::endl;
        return;
    }

}

std::list<std::string> FreenectProvider::getConnectedCameraTags() {
    return std::list<std::string>();
}

UnicamCamera *FreenectProvider::getCameraByTag(std::string cameraTag) {
    return nullptr;
}

UnicamDevices FreenectProvider::getCameraType() {
    return UnicamDeviceProvider::getCameraType();
}

bool FreenectProvider::isMultSupported() {
    return UnicamDeviceProvider::isMultSupported();
}
