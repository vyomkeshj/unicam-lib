#include<iostream>
#include<fstream>
#include<cstdlib>
#include "headers/unicam/UnicamDeviceProvider.h"
#include "headers/RealsenseProvider.h"

using namespace std;

int main()
{
    UnicamDeviceProvider *realsense = new RealsenseProvider();
    realsense->initializeCameras();
    realsense->spinOnce();
    UnicamCamera *camera = realsense->getCameraByTag("912112073363");
    cv::Mat matrix = camera->getDepthFrame();

    return(0);
}

void tryAgain(UnicamCamera* camera) {
    cv::Mat matrix = camera->getDepthFrame();
    cv::imshow("image", matrix);
}