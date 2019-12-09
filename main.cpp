#include<iostream>
#include "headers/unicam/UnicamDeviceProvider.h"
#include "headers/RealsenseProvider.h"

using namespace std;

int main() {
    UnicamDeviceProvider *xtion = new RealsenseProvider();
    xtion->initializeCameras();
    UnicamCamera *camera = xtion->getCameraByTag("912112073363");
    //CameraOrientationController* controller = new CameraOrientationController("/dev/ttyACM0", camera, xtion);
    while (true) {
        xtion->spinOnce();
        cv::Mat matrix = camera->getDepthFrame();
        //controller->alignCamera();
        std::cout << matrix;
    }                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      return (0);

}
