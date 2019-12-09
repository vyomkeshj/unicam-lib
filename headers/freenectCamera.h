//
// Created by rob-ot on 02.12.19.
//

#ifndef LIBUNICAM_FREENECTCAMERA_H
#define LIBUNICAM_FREENECTCAMERA_H


#include "unicam/UnicamCamera.h"

class freenectCamera : public UnicamCamera {

    cv::Mat getDepthFrame() override;
    cv::Mat getIRFrame() override;
    cv::Mat getRGBFrame() override;
};


#endif //LIBUNICAM_FREENECTCAMERA_H
