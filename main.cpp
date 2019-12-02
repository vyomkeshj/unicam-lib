#include<iostream>
#include "headers/unicam/UnicamDeviceProvider.h"
#include "headers/RealsenseProvider.h"
#include "headers/XtionProvider.h"
#include "headers/orientationControl/CameraOrientationController.h"

using namespace std;

int main() {
    UnicamDeviceProvider *xtion = new XtionProvider();
    xtion->initializeCameras();
    UnicamCamera *camera = xtion->getCameraByTag("817612070540");
    CameraOrientationController* controller = new CameraOrientationController("/dev/ttyACM0", camera, xtion);
    while (true) {
        xtion->spinOnce();
        //cv::Mat matrix = camera->getDepthFrame();
        controller->alignCamera();

        //std::cout<<"y vect 0= "<<normal.val[0]<<" 1 = "<<normal.val[1]<<" 3 = "<<normal.val[2]<<std::endl;
        //std::cout << matrix;
    }                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      return (0);

}
