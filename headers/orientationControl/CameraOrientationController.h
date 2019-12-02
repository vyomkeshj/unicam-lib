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
    bool isFrameNormal(cv::Mat & depthFrame, int *horizontalDisparity, int *verticalDisparity);
    void alignCamera(cv::Mat &newDepthFrame);
    void realignDevice(cv::Mat &depthFrame);
    void computeDisparity(cv::Mat depthFrame);
    float computeSqrAverageDistance(int centerCol, int centerRow, int sqrDim, cv::Mat depthFrame);
private:
    int vertDiff, hzDiff = 0;
    int vertPos, hzPos = 120;
    int axesSize = 60;

    int errorThreshold = 2;
    int servoBaseInitPos = 120;
    int servoTopInitPos = 120;
    std::string arduinoPort;
    FILE *arduinoSerial;
    int hzNess, vertNess = 0;

};


#endif //LIBUNICAM_CAMERAORIENTATIONCONTROLLER_H
