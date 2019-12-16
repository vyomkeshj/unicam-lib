#include<iostream>
#include <sys/stat.h>
#include "headers/unicam/UnicamDeviceProvider.h"
#include "headers/RealsenseProvider.h"
#include "headers/XtionProvider.h"
#include "headers/orientationControl/CameraOrientationController.h"

using namespace std;

int main() {
    bool isFirstRun = true;
    bool isAlignedNow = false;
    bool isDistanceEqualToTarget = false;
    bool proceedToNextMeasurement = false;
    bool makeDelay = true;
    cv::Mat currentDepthFrameRef;
    int setDistance = 0;

    UnicamDeviceProvider *xtion = new XtionProvider();
    xtion->initializeCameras();
    UnicamCamera *camera = xtion->getCameraByTag("912112073363");

    CameraOrientationController* controller = new CameraOrientationController("/dev/ttyACM0", camera, xtion);
    while (true) {
        if(isFirstRun || proceedToNextMeasurement) {
            proceedToNextMeasurement = false;
            makeDelay = true;
            isFirstRun = false;
            std::cout<<"please enter the distance for calibration"<<std::endl;
            cin>>setDistance;
            mkdir(("depth"+std::to_string(setDistance)).c_str(), 0777);         //create the data storage dir
            controller->updateDistanceTarget(setDistance);
            std::cout<<std::endl<<" the distance has been set to "<<setDistance<<std::endl;

        }
        isAlignedNow = controller->realignDevice(currentDepthFrameRef);
        isDistanceEqualToTarget = controller->isAtExpectedDistance(currentDepthFrameRef);
        if(isAlignedNow && isDistanceEqualToTarget) {
            if(makeDelay) {
                std::cout<<"waiting for 10 seconds before starting to buffer frames"<<std::endl;
                cv::waitKey(10000);
                makeDelay = false;
            }
            proceedToNextMeasurement = controller->bufferedWriteToFile(currentDepthFrameRef);       //proceedToNextMeasurement is true when the current measurement frames have been persisted
        }

    }

}