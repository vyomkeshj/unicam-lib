//
// Created by rob-ot on 2019-12-12.
//

#ifndef LIBUNICAM_FRAME_DATA_H
#define LIBUNICAM_FRAME_DATA_H


#include <opencv2/core/mat.hpp>
#include <string>

struct frame_data {
    cv::Mat depthFrame;
    std::string timestamp;
    float centralDistance;
    float hzDisparity;
    float vertDisparity;

    frame_data(cv::Mat depth, std::string dataTime, float centralDist, float hzDisp, float vertDisp) {
        depthFrame = depth;
        timestamp = dataTime;
        centralDistance = centralDist;
        hzDisparity = hzDisp;
        vertDisparity = vertDisp;
    }
};


#endif //LIBUNICAM_FRAME_DATA_H
