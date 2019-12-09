// Copyright 2018 Miso Robotics, Inc.
// Author: Ryan W. Sinnet (ryan@rwsinnet.com)

#include <OpenNI.h>

#include "../headers/io_interface.h"


int main(int argc, char** argv)
{
  std::string camera_name;
  std::string device_uri;

  openni::Status rc = openni::STATUS_OK;
  openni::Device device;
  openni::VideoStream depth, color;

  rc = openni::OpenNI::initialize();

  rc = device.open(openni::ANY_DEVICE);
  if (rc != openni::STATUS_OK)
  {
    //ROS_ERROR("SimpleViewer: Device open failed:\n%s", openni::OpenNI::getExtendedError());
    openni::OpenNI::shutdown();
    return 1;
  }

  rc = depth.create(device, openni::SENSOR_DEPTH);
  if (rc == openni::STATUS_OK)
  {
    rc = depth.start();
    if (rc != openni::STATUS_OK)
    {
      //ROS_WARN("SimpleViewer: Couldn't start depth stream:\n%s", openni::OpenNI::getExtendedError());
      depth.destroy();
    }
  }
  else
  {
    //ROS_WARN("SimpleViewer: Couldn't find depth stream:\n%s", openni::OpenNI::getExtendedError());
  }

  rc = color.create(device, openni::SENSOR_COLOR);
  if (rc == openni::STATUS_OK)
  {
    rc = color.start();
    if (rc != openni::STATUS_OK)
    {
      //ROS_WARN("SimpleViewer: Couldn't start color stream:\n%s", openni::OpenNI::getExtendedError());
      color.destroy();
    }
  }
  else
  {
    //ROS_WARN("SimpleViewer: Couldn't find color stream:\n%s", openni::OpenNI::getExtendedError());
  }

  if (!depth.isValid() || !color.isValid())
  {
    //ROS_ERROR("SimpleViewer: No valid streams. Exiting");
    openni::OpenNI::shutdown();
    return 2;
  }

  xtion2_ros::IOInterface iface(device, depth, color);
  iface.initialize();
  iface.spinOnce();
  openni::OpenNI::shutdown();
  return 0;
}
