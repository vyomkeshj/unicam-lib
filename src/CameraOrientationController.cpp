//
// Created by rob-ot on 22.11.19.
//

#include <iostream>
#include <opencv2/opencv.hpp>
#include "../headers/orientationControl/CameraOrientationController.h"
#include "../headers/async_buf.h"


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

    int shiftStrideBy = 15;
    int shiftingStrideBorders = 60;

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
   // std::cout<<"                horizontal disparity  --- = " <<horizontalDirectionPointer<<std::endl;
    *horizontalDisparity = horizontalDirectionPointer;
    *verticalDisparity = verticalDirectionPointer;
    //horizontalDirectionPointer = -horizontalDirectionPointer;
    //realignDevice(depthFrame);

}


/**
 * Sends vertpos:hzpos
 * **/

int nSavedFrames = 150;
int step = 1;
bool realigned = false;
void CameraOrientationController::realignDevice() {
if(!realigned) {
    for (int baseAngle = 145; baseAngle <= 160; baseAngle++) {
        for (int topAngle = 78; topAngle <= 95; topAngle++) {
            int verticalness, horizontalness;
            xtion->spinOnce();
            cv::Mat currentDepthImage = cameraControl->getDepthFrame();
            computeDisparity(currentDepthImage, &horizontalness, &verticalness);
            cv::waitKey(100);
            if (horizontalness < errorThreshold && horizontalness > -errorThreshold && verticalness < errorThreshold &&
                verticalness > -errorThreshold) {
                std::cout << "calibrated" << std::endl;

                if(nSavedFrames>0) {
                    bool persisted = persistMatrix(currentDepthImage, nSavedFrames, horizontalness, verticalness);
                    if(persisted)
                    nSavedFrames--;
                } else {
                    std::cout<<"writing the matrices to file is now complete"<<std::endl;
                    exit(112); //exit the program when the writing is done
                }
            } else {
                std::cout << "top angle = " << topAngle << " bottom angle " << baseAngle << std::endl;
            }
        }
    }
} else {
    realigned = true;
}
    }


    void CameraOrientationController::computeDisparity(cv::Mat &depthFrame, int *horizontalDisparity, int *verticalDisparity) {
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


        *verticalDisparity = verticalVect;
        float horizontalVect = computeSqrAverageDistance(midCol-boxCenterOffset, midRow, sqrDim, depthFrame)-
                             computeSqrAverageDistance(midCol+boxCenterOffset, midRow, sqrDim, depthFrame);

        *horizontalDisparity = horizontalVect;

        if (horizontalVect < errorThreshold && horizontalVect > -errorThreshold && verticalVect < errorThreshold &&
            verticalVect > -errorThreshold) {


            if(nSavedFrames>0) {

                bool persisted = persistMatrix(depthFrame, nSavedFrames, horizontalVect, verticalVect);
                if(persisted)
                nSavedFrames--;
            }
        } else {
            std::cout<<"horizontal disparity   "<<horizontalVect<<"  vertical disparity = "<<verticalVect<<std::endl;
        }
        //cv::waitKey(500);
    }

float CameraOrientationController::computeSqrAverageDistance(int centerCol, int centerRow, int sqrDim, cv::Mat depthFrame) {
        auto frame_data = depthFrame.data;
        int validPixelCount = 1;
        float regionAverage = 0;
        const int type = CV_16U;

        int width = depthFrame.cols;
        int height = depthFrame.cols;


        for(int row = (centerRow-sqrDim/2); row <= (centerRow +(sqrDim/2)); row++) {
            for(int col = (centerCol-sqrDim/2); col<(centerCol+(sqrDim/2)); col++) {

                //std::cout<<(depthFrame.at<CvType<type>::type_t>(row2, col))<<" ";
                float val = depthFrame.at<CvType<type>::type_t>(row, col);

                if(val >= CAMERA_MINIMUM) {
                    //regionAverage +=frame_data[col*width+row];
                    regionAverage += val;
                    validPixelCount++;
                }
            }
        }
    //std::cout<<regionAverage/validPixelCount<<std::endl;
    return regionAverage/validPixelCount;

}

CameraOrientationController::~CameraOrientationController() {
fclose(arduinoSerial);
}

bool CameraOrientationController::persistMatrix(cv::Mat data, int count, int hz, int vert) {

    double centralDistance = computeFrameCentralDistance(data);
    double absDistanceDifference = abs(centralDistance - distanceTarget);
    if (absDistanceDifference > distanceErrorThreshold) {
        std::cout << "please move the stand to "
                  << distanceTarget << "it's currently at " << centralDistance << std::endl;
        return false;
    } else {
        std::cout << "persisting frame = " <<nSavedFrames<< std::endl;

        std::string fileName = "./xtion_at2/xtion-frame-stream-" + std::to_string(count) + ".yaml";
        async_buf sbuf(fileName);

        //cv::FileStorage writer(fileName, cv::FileStorage::WRITE);
        std::ostream astream(&sbuf);

        std::time_t result = std::time(nullptr);


        astream << "timestamp: " << std::asctime(std::localtime(&result)) << '\n';
        astream << "distance: " << centralDistance << '\n';
        astream << "horizontalness: " << hz << '\n';
        astream << "verticalness: " << vert << '\n';

        astream << "matrix: " << '\n';
        astream << "rows: " << data.rows << '\n';
        astream << "cols: " << data.cols << '\n';
        astream << "dt: " << "f" << '\n';
        astream << "data: " << data << '\n' << std::flush;
        return true;
    }
}

//computes the average distance from the wall
double CameraOrientationController::computeFrameCentralDistance(cv::Mat &depthFrame) {
    int width = depthFrame.cols;
    int height = depthFrame.rows;
    int middleSquareDim = 150;
    float loopCounter = 0.0;
    float loopSumA = 0.0;
    for(int y = height/2 - middleSquareDim/2; y<= height/2 + middleSquareDim/2; y++) {
        for (int x = width/2 - middleSquareDim/2; x<= width/2+middleSquareDim/2; x++) {
            ushort distance = depthFrame.at<ushort>(y,x);

            if(distance>CAMERA_MINIMUM) {           //try to filter out the bad distances
                loopCounter++;
                loopSumA +=distance;
            }
        }
    }
    return loopSumA/loopCounter;        //mean distance
}