
#include "../headers/io_interface.h"
#include <OpenNI.h>
#include <iostream>

namespace unicam_xtion
{
IOInterface::IOInterface(openni::Device& device, openni::VideoStream& depth)
  : device_(device), depth_stream_(depth), depth_new(false)
{
}

std::array<int, 2> IOInterface::initialize_stream(openni::VideoStream& stream)
{
  auto video_mode = stream.getVideoMode();
  return { { video_mode.getResolutionX(), video_mode.getResolutionY() } };
}

static bool validateStream(openni::VideoStream& stream, const std::string& stream_name)
{
  if (!stream.isValid())
  {
    std::cout<<stream_name<<" stream invalid"<<std::endl;
    return false;
  }
  return true;
}

bool IOInterface::initialize()
{
  const auto depth_resolution = initialize_stream(depth_stream_);
  if (!validateStream(depth_stream_, "Depth"))
    return false;



  width_ = depth_resolution[0];
  height_ = depth_resolution[1];

  streams_[0] = &depth_stream_;

  return true;
}

const cv::Mat IOInterface::getFrame(openni::VideoFrameRef& frame, const int data_type)
{
  // In a show of poor form, const_cast the data to avoid a memcpy. Hopefully no one tries to use them afterward.
  return cv::Mat(frame.getHeight(), frame.getWidth(), data_type, const_cast<void*>(frame.getData()));
}

void IOInterface::spinOnce()
{
  int changedIndex;
  openni::Status rc = openni::OpenNI::waitForAnyStream(streams_.data(), 2, &changedIndex);
  if (rc != openni::STATUS_OK)
  {
    std::cout<<("Wait failed");
    return;
  }

  //! \todo Get both frames here for sure.
  switch (changedIndex)
  {
    case 0:
      //! \todo Figure out how to use the timestamp from the frame.
      depth_stream_.readFrame(&depth_frame_);
      depth_new = true;
      break;
    default:
      std::cout<<("Failed waiting for next frame.");
  }
}
}
