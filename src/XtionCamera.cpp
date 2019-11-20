//
// Created by rob-ot on 19.11.19.
//

#include "../headers/XtionCamera.h"

cv::Mat XtionCamera::getRGBFrame() {
    return inf->getColorFrame();
}

cv::Mat XtionCamera::getIRFrame() {
    return cv::Mat();
}

cv::Mat XtionCamera::getDepthFrame() {
    return inf->getDepthFrame();
}

XtionCamera::XtionCamera(unicam_xtion::IOInterface *inf) {
this->inf = inf;
}