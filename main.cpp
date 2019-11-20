#include<iostream>
#include<fstream>
#include<cstdlib>
#include "headers/unicam/UnicamDeviceProvider.h"
#include "headers/RealsenseProvider.h"
#include "headers/XtionProvider.h"

using namespace std;

int main()
{
    UnicamDeviceProvider *xtion = new XtionProvider();
    xtion->initializeCameras();
    UnicamCamera *camera = xtion->getCameraByTag("912112073363");
    cv::Mat matrix = camera->getDepthFrame();
    matrix = (1.0000/4000.0)* matrix;
    std::cout<<matrix;
    cv::imshow("cert", matrix);
    cv::waitKey(100000);

    return(0);
}

