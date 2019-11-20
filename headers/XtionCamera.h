//
// Created by rob-ot on 15.11.19.
//

#ifndef LIBUNICAM_XTIONCAMERA_H
#define LIBUNICAM_XTIONCAMERA_H


#include <OpenNI.h>
#include "unicam/UnicamCamera.h"
#include "io_interface.h"

class XtionCamera : public UnicamCamera {
    cv::Mat getRGBFrame();
     cv::Mat getIRFrame();
     cv::Mat getDepthFrame();

public:
    XtionCamera(unicam_xtion::IOInterface *inf);

private:
    unicam_xtion::IOInterface *inf;
};


#endif //LIBUNICAM_XTIONCAMERA_H
