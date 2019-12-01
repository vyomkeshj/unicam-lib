//
// Created by rob-ot on 22.11.19.
//

#include <iostream>
#include <opencv2/opencv.hpp>
#include "../headers/orientationControl/CameraOrientationController.h"



/*
 * Controls the camera orientation control hardware to orient the camera's normal perpendicular to the wall's surface
 * INPUT: arduinoPort to communicate with, cv::Mat depthFrame to compute orientation against
 * Steps:
 * 1. Detect the plane from center, expand.
 * 2. measure distance from centre, use the fov of the camera to get an approx translation from px to wo
 *
 *
 * **/
CameraOrientationController::CameraOrientationController(const char *arduinoPort) {
    this->arduinoPort = arduinoPort;
    arduinoSerial= fopen(arduinoPort, "w");
}

cv::Vec3d CameraOrientationController::recomputeNormal(cv::Mat &newDepthFrame) {
     alignCamera(newDepthFrame);
}

void CameraOrientationController::alignCamera(cv::Mat &newDepthFrame) {
    isFrameNormal(newDepthFrame);
    realignDevice(vertNess, hzNess);

}

bool CameraOrientationController::isFrameNormal(cv::Mat &depthFrame) {
    auto frame_data = depthFrame.data;
    //std::cout<<&depthFrame<<std::endl;
    int width = depthFrame.cols;
    int height = depthFrame.rows;
    int strideOffset = 50;
    int strideWidth = 30;


    int jumpWidth = depthFrame.step;
    bool hzMatch = 0;
    bool vertMatch = 0;

    int loopCounter = 0;
    int loopSumA = 0;
    int loopSumB = 0;
    for(int x = strideOffset; x<= width-strideOffset; x++) {
        for (int y = strideOffset; y<= strideOffset+strideWidth; y++) {
            loopCounter++;
            loopSumA+=frame_data[x*width+y];
        }
    }
    loopSumA/=loopCounter;
    loopCounter = 0;
    for(int x = strideOffset; x<= width-strideOffset; x++) {
        for (int y = height-strideOffset-strideWidth; y<= height-strideOffset; y++) {
            loopCounter++;
            loopSumB+=frame_data[x*width+x];
        }
    }

    loopSumB/=loopCounter;
    hzMatch = abs(loopSumA-loopSumB) < errorThreshold;
    std::cout<<"horizontalness:::::::::::::::::::::: "<<(loopSumA-loopSumB)<<std::endl;
    hzNess = (hzNess + (loopSumA-loopSumB))/2;
    loopCounter = 0;
    loopSumA = 0;
    loopSumB = 0;
    for(int y = strideOffset; y<= height-strideOffset; y++) {
        for (int x = strideOffset; x<= strideOffset+strideWidth; x++) {
            loopCounter++;
            loopSumA+=frame_data[x*width+y];

        }
    }
    loopSumA/=loopCounter;
    loopCounter = 0;
    for(int y = strideOffset; y<= height-strideOffset; y++) {
        for (int x = width-strideOffset-strideWidth; x<= width-strideOffset; x++) {
            loopCounter++;
            loopSumB+=frame_data[x*width+y];
        }
    }
    loopSumB/=loopCounter;
    vertMatch = abs(loopSumA-loopSumB) < errorThreshold;


    std::cout<<"verticalness: "<<(loopSumA-loopSumB)<<std::endl;
    vertNess = (vertNess + (loopSumA - loopSumB))/2;

    if(!hzMatch || !vertMatch)
    realignDevice(hzNess, vertNess);
    return hzMatch && vertMatch;
}

/**
 * Sends vertpos:hzpos
 * **/

int framecount = 30;
int step = 2;
void CameraOrientationController::realignDevice(int verticalness, int horizontalness) {
        if (framecount <= 0) {
            framecount = 10;
/**/    if (verticalness > errorThreshold/2 && servoTopInitPos > 60) {
                servoTopInitPos = servoTopInitPos-step;
                fprintf(arduinoSerial, "%d:%d\n",--servoTopInitPos, servoBaseInitPos);
                //std::cout << servoTopInitPos << ":" << servoBaseInitPos << std::endl;
                cv::waitKey(100);

            } else if (verticalness < -errorThreshold/2 && servoTopInitPos < 120) {
                servoTopInitPos = servoTopInitPos+step;

                fprintf(arduinoSerial, "%d:%d\n",++servoTopInitPos,servoBaseInitPos);
               // std::cout << servoTopInitPos << ":" << servoBaseInitPos << std::endl;
            }

            if (horizontalness > errorThreshold/2 && servoBaseInitPos > 80) {
                servoBaseInitPos = servoBaseInitPos-step;

                fprintf(arduinoSerial, "%d:%d\n", servoTopInitPos,--servoBaseInitPos);
            } else if (horizontalness < -errorThreshold/2 && servoBaseInitPos < 120) {
                servoBaseInitPos = servoBaseInitPos+step;
                fprintf(arduinoSerial, "%d:%d\n",servoTopInitPos,++servoBaseInitPos);

            }
        } else {
            framecount--;

        }
    }



CameraOrientationController::~CameraOrientationController() {
fclose(arduinoSerial);
}


