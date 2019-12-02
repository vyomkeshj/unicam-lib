//
// Created by rob-ot on 22.11.19.
//

#ifndef LIBUNICAM_CAMERAORIENTATIONCONTROLLER_H
#define LIBUNICAM_CAMERAORIENTATIONCONTROLLER_H


#include <string>
#include <opencv2/core/mat.hpp>
#include <fstream>
#include "../unicam/UnicamCamera.h"
#include "../unicam/UnicamDeviceProvider.h"

class CameraOrientationController {
public:
    explicit CameraOrientationController(const char *arduinoPort , UnicamCamera *camera, UnicamDeviceProvider *xtion);
    ~CameraOrientationController();
    cv::Vec3d recomputeNormal(cv::Mat &newDepthFrame);
    bool isFrameNormal(cv::Mat & depthFrame, int *horizontalDisparity, int *verticalDisparity);
    void alignCamera();
    void realignDevice();
    void computeDisparity(cv::Mat depthFrame);
    float computeSqrAverageDistance(int centerCol, int centerRow, int sqrDim, cv::Mat depthFrame);
private:
    int vertDiff, hzDiff = 0;
    int vertPos, hzPos = 120;
    int axesSize = 60;

    int errorThreshold = 60;
    int servoBaseInitPos = 120;
    int servoTopInitPos = 120;
    std::string arduinoPort;
    FILE *arduinoSerial;
    int hzNess, vertNess = 0;
    UnicamCamera* cameraControl;
    UnicamDeviceProvider *xtion;
};


#endif //LIBUNICAM_CAMERAORIENTATIONCONTROLLER_H
