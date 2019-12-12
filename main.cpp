#include<iostream>
#include <sys/stat.h>
#include "headers/unicam/UnicamDeviceProvider.h"
#include "headers/RealsenseProvider.h"
#include "headers/XtionProvider.h"
#include "headers/orientationControl/CameraOrientationController.h"

using namespace std;

int main() {
    bool saved = true;
    int setDistance = 0;

    UnicamDeviceProvider *xtion = new XtionProvider();
    xtion->initializeCameras();
    UnicamCamera *camera = xtion->getCameraByTag("912112073363");
    CameraOrientationController* controller = new CameraOrientationController("/dev/ttyACM0", camera, xtion);
    while (true) {
        if(saved == true) {
            saved = false;
            std::cout<<"please enter the distance for calibration"<<std::endl;
            cin>>setDistance;
            mkdir(("depth"+std::to_string(setDistance)).c_str(), 0777);
            controller->updateDistanceTarget(setDistance);

            std::cout<<std::endl<<"the distance has been set to "<<setDistance<<std::endl;

        }
        controller->realignDevice(saved);
    }

}