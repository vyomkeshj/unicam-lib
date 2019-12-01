//
// Created by rob-ot on 22.11.19.
//

#ifndef LIBUNICAM_CAMERAORIENTATIONCONTROLLER_H
#define LIBUNICAM_CAMERAORIENTATIONCONTROLLER_H


#include <string>
#include <opencv2/core/mat.hpp>
#include <fstream>

class CameraOrientationController {
public:
    explicit CameraOrientationController(const char *arduinoPort );
    ~CameraOrientationController();
    cv::Vec3d recomputeNormal(cv::Mat &newDepthFrame);
    bool isFrameNormal(cv::Mat &depthFrame);
    void alignCamera(cv::Mat &newDepthFrame);
    void realignDevice(int verticalness, int horizontalness);

private:
    int vertDiff, hzDiff = 0;
    int vertPos, hzPos = 90;
    int axesSize = 60;

    int errorThreshold = 10;
    int servoBaseInitPos = 90;
    int servoTopInitPos = 90;
    std::string arduinoPort;
    FILE *arduinoSerial;
    int hzNess, vertNess = 0;

};


#endif //LIBUNICAM_CAMERAORIENTATIONCONTROLLER_H
