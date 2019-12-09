//
// Created by rob-ot on 22.11.19.
//

#ifndef LIBUNICAM_CAMERAORIENTATIONCONTROLLER_H
#define LIBUNICAM_CAMERAORIENTATIONCONTROLLER_H
#define CAMERA_MINIMUM 400
#define DISTANCE_ERROR_THRESHOLD 10

#define FRAME_DIST_SQUARE_DIM 75   //adjust according to the distance tgt
#define DISTANCE_TARGET 1000

#include <string>
#include <opencv2/core/mat.hpp>
#include <fstream>
#include "../unicam/UnicamCamera.h"
#include "../unicam/UnicamDeviceProvider.h"

class CameraOrientationController {
public:

    struct frameData {
        cv::Mat depthFrame;
        std::string timestamp;
        float centralDistance;
    };

    explicit CameraOrientationController(const char *arduinoPort , UnicamCamera *camera, UnicamDeviceProvider *xtion);
    ~CameraOrientationController();
    cv::Vec3d recomputeNormal(cv::Mat &newDepthFrame);
    bool isFrameNormal(cv::Mat & depthFrame, int *horizontalDisparity, int *verticalDisparity);
    void computeDisparity(cv::Mat &depthFrame, int *horizontalDisparity, int *verticalDisparity);

    void alignCamera();
    void realignDevice();
    bool persistMatrix(cv::Mat data, int count, int hz, int vert);
    float computeSqrAverageDistance(int centerCol, int centerRow, int sqrDim, cv::Mat depthFrame);
private:
    int vertDiff, hzDiff = 0;
    int vertPos, hzPos = 120;
    int axesSize = 60;

    int errorThreshold = 15;
    int servoBaseInitPos = 120;
    int servoTopInitPos = 120;
    std::string arduinoPort;
    FILE *arduinoSerial;
    int hzNess, vertNess = 0;
    UnicamCamera* cameraControl;
    UnicamDeviceProvider *xtion;

    double computeFrameCentralDistance(cv::Mat &depthFrame);
};


#endif //LIBUNICAM_CAMERAORIENTATIONCONTROLLER_H
