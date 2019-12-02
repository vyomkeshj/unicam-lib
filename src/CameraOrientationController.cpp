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
CameraOrientationController::CameraOrientationController(const char *arduinoPort, UnicamCamera* xtionCamera, UnicamDeviceProvider *xtion) : cameraControl(xtionCamera), xtion(xtion){
    this->arduinoPort = arduinoPort;
    arduinoSerial= fopen(arduinoPort, "w");
}

cv::Vec3d CameraOrientationController::recomputeNormal(cv::Mat &newDepthFrame) {
     //alignCamera(newDepthFrame);
}

bool aligned = false;
void CameraOrientationController::alignCamera() {
    if(!aligned) {
        realignDevice();
    } else {
        aligned = true;
    }
}

template<int I>
struct CvType {};

template<>
struct CvType<CV_64F> { typedef double type_t; };
template<>
struct CvType<CV_32F> { typedef float type_t; };
template<>
struct CvType<CV_8U> { typedef unsigned char type_t; };

template<>
struct CvType<CV_16U> { typedef unsigned short type_t; };


bool CameraOrientationController::isFrameNormal(cv::Mat &depthFrame, int *horizontalDisparity, int *verticalDisparity) {
    auto frame_data = depthFrame.data;
    int width = depthFrame.cols;
    int height = depthFrame.rows;
    const int type = CV_16U;

    int midRow = height/2;
    int midCol = width/2;

    int shiftStrideBy = 5;
    int shiftingStrideBorders = 20;

    int horizontalDirectionPointer =0;
    int verticalDirectionPointer =0;

    int iterCount = 1;

    //computeDisparity(depthFrame);


    for(int currentRow = shiftingStrideBorders; currentRow < (height - shiftingStrideBorders); currentRow++) { //iterate through rows in the current strides
        for(int colLeft = shiftingStrideBorders, colRight = (midCol + shiftingStrideBorders);                //iterate through points equally above and below the horizon
            (colLeft < midCol-shiftStrideBy) && (colRight < width-shiftStrideBy); colLeft+=shiftStrideBy, colRight+=shiftStrideBy) {
            if((depthFrame.at<CvType<type>::type_t>(currentRow, colLeft > 400) && depthFrame.at<CvType<type>::type_t>(currentRow, colRight))) {    // if both the points are non zero depth
                //std::cout<<"vert difference current = "<<colLeft<<", "<<colRight<<std::endl;
                //verticalDirectionPointer+= (frame_data[colLeft*width+currentRow] - frame_data[colRight*width+currentRow]); // vertical disparity compute
                verticalDirectionPointer+= (depthFrame.at<CvType<type>::type_t>(currentRow, colLeft)-depthFrame.at<CvType<type>::type_t>(currentRow, colRight));
                iterCount++;
                //verticalDirectionPointer/=2;
            }
        }
    }
    verticalDirectionPointer/=iterCount;
    //verticalDirectionPointer = -verticalDirectionPointer;
    std::cout<<"vertical disparity | = " <<verticalDirectionPointer<<std::endl;

    //perform corresponding pixel diff on the horizontal strides, shift stride each time
    iterCount = 1;
    for(int currentCol = shiftingStrideBorders; currentCol < (width - shiftingStrideBorders); currentCol++) { //iterate through columns in the current strides
        for(int rowUpper = shiftingStrideBorders, rowLower = (midRow + shiftingStrideBorders);                //iterate through points equally above and below the horizon
            ((rowUpper < midRow-shiftStrideBy) && (rowLower < height-shiftStrideBy)); rowUpper+=shiftStrideBy, rowLower+=shiftStrideBy) {
            if((depthFrame.at<CvType<type>::type_t>(rowUpper, currentCol) > 400 && (depthFrame.at<CvType<type>::type_t>(rowLower, currentCol) >400))) {    // if both the points are non zero depth
                //std::cout<<"horiz difference current = "<<rowUpper<<", "<<rowLower<<std::endl;
                horizontalDirectionPointer+= (depthFrame.at<CvType<type>::type_t>(rowUpper, currentCol)-depthFrame.at<CvType<type>::type_t>(rowLower, currentCol));

                iterCount++;
                //horizontalDirectionPointer/=2;
            }
        }
    }

    horizontalDirectionPointer/=iterCount;
    std::cout<<"                horizontal disparity  --- = " <<horizontalDirectionPointer<<std::endl;
    *horizontalDisparity = horizontalDirectionPointer;
    *verticalDisparity = verticalDirectionPointer;
    //horizontalDirectionPointer = -horizontalDirectionPointer;
    //realignDevice(depthFrame);

}

/*

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
            loopSumB+=frame_data[x*width+y];
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
            if (frame_data[x*width+y] != 0) {
                loopCounter++;
                loopSumA += frame_data[x * width + y];
            }
        }
    }
    loopSumA/=loopCounter;
    loopCounter = 0;
    for(int y = strideOffset; y<= height-strideOffset; y++) {
        for (int x = width-strideOffset-strideWidth; x<= width-strideOffset; x++) {
            if (frame_data[x*width+y] != 0) {
                loopCounter++;
                loopSumB += frame_data[x * width + y];
            }
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

*/
/**
 * Sends vertpos:hzpos
 * **/


int step = 1;
bool realigned = false;
void CameraOrientationController::realignDevice() {
if(!realigned) {
    for (int baseAngle = 147; baseAngle <= 154; baseAngle++) {
        for (int topAngle = 78; topAngle <= 95; topAngle++) {
            int verticalness, horizontalness;
            xtion->spinOnce();
            cv::Mat currentDepthImage = cameraControl->getDepthFrame();
            isFrameNormal(currentDepthImage, &horizontalness, &verticalness);
            cv::waitKey(100);
            if (horizontalness < errorThreshold && horizontalness > -errorThreshold && verticalness < errorThreshold &&
                verticalness > -errorThreshold) {
                std::cout << "calibrated" << std::endl;
            } else {
                fprintf(arduinoSerial, "%d:%d\n", baseAngle, topAngle);
                cv::waitKey(100);

                std::cout << "top angle = " << topAngle << " bottom angle " << baseAngle << std::endl;
            }
        }
    }
} else {
    realigned = true;
}
    }

    void CameraOrientationController::computeDisparity(cv::Mat depthFrame) {
        int sqrDim = 10;
        int width = depthFrame.cols;
        int height = depthFrame.rows;

        int midRow = height/2;
        int midCol = width/2;

        int boxCenterOffset = 20;
        float centerDist = computeSqrAverageDistance(midCol, midRow, sqrDim, depthFrame);
        //std::cout<<std::endl<<"                                                                  center dist"<<centerDist<<std::endl;


        float verticalVect = computeSqrAverageDistance(midCol, midRow-boxCenterOffset, sqrDim, depthFrame)-
                computeSqrAverageDistance(midCol, midRow+boxCenterOffset, sqrDim, depthFrame);

        std::cout<<"vertical disparity   "<<verticalVect<<std::endl;


        float horizontalVect = computeSqrAverageDistance(midCol-boxCenterOffset, midRow, sqrDim, depthFrame)-
                             computeSqrAverageDistance(midCol+boxCenterOffset, midRow, sqrDim, depthFrame);

        std::cout<<"horizontal disparity   "<<horizontalVect<<std::endl;

        //cv::waitKey(500);
    }

float CameraOrientationController::computeSqrAverageDistance(int centerCol, int centerRow, int sqrDim, cv::Mat depthFrame) {
        auto frame_data = depthFrame.data;
        int validPixelCount = 1;
        float regionAverage = 0;
        const int type = CV_16U;

        int width = depthFrame.cols;

        int rows = centerRow-sqrDim/2;
        int rowe = centerRow+sqrDim/2;

        for(int row2 = rows; row2 < rowe; row2 = row2 + 1) {
            std::cout<<std::endl;
            for(int col = (centerCol-sqrDim/2); col<(centerCol+(sqrDim/2)); col++) {

                //std::cout<<(depthFrame.at<CvType<type>::type_t>(row2, col))<<" ";
                float val = depthFrame.at<CvType<type>::type_t>(row2, col);

                if(val >= 400) {
                    //regionAverage +=frame_data[col*width+row];
                    regionAverage += val;
                    validPixelCount++;
                }
            }

            //std::cout<<validPixelCount<<std::endl;
            return regionAverage/validPixelCount;
        }
    }

CameraOrientationController::~CameraOrientationController() {
fclose(arduinoSerial);
}




