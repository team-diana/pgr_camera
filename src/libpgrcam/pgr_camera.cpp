/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, UC Regents
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the University of California nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include "pgr_camera/pgr_camera.h"

static bool check_success(FlyCapture2::Error error) {
    if(error != FlyCapture2::PGRERROR_OK) {
        ROS_ERROR ("%s", error.GetDescription ());
        return false;
    }
    return true;
}

namespace pgr_camera
{

static unsigned int cameraNum = 0;

void init()
{
  FlyCapture2::BusManager busMgr;
  for (int tries = 0; tries < 5; ++tries)
  {
    check_success(busMgr.GetNumOfCameras(&cameraNum));
    if (cameraNum)
    {
      ROS_INFO ("Found %d cameras", cameraNum);
      for (unsigned int i = 0; i < cameraNum; i++)
      {
        unsigned int serNo;
        if(check_success(busMgr.GetCameraSerialNumberFromIndex(i, &serNo))) {
          ROS_INFO ("Camera %u: S/N %u", i, serNo);
        }
      }
    }
    return;
    usleep(200000);
  }
}

// FIXME: How to make this a member function?
void frameDone(FlyCapture2::Image * frame, const void *pCallbackData)
{
  Camera *camPtr = (Camera *)pCallbackData;
  if (camPtr->userCallback_.empty())
  {
    ROS_WARN ("User callback empty!");
    return;
  }
  
  // TODO: thread safety OK here?
  boost::lock_guard<boost::mutex> guard(camPtr->frameMutex_);
  camPtr->userCallback_(frame);
  //ROS_INFO("in frameDone");
}

size_t numCameras()
{
  return cameraNum;
}

Camera::Camera()
{
  Camera(0);
}

Camera::Camera(unsigned int serNo) :
  camIndex(0), camSerNo(serNo), frameRate(FlyCapture2::FRAMERATE_30)
{
  setup();
}

bool Camera::setup()
{
  ///////////////////////////////////////////////////////
  // Get a camera:
  FlyCapture2::BusManager busMgr;
  unsigned int N;
  check_success(busMgr.GetNumOfCameras(&N));
  if (camSerNo == 0)
  {
    FlyCapture2::PGRGuid guid;
    if(!check_success(busMgr.GetCameraFromIndex(camIndex, &guid))) return false;
    ROS_INFO ("did busMgr.GetCameraFromIndex(0, &guid)");
  }
  else
  {
    FlyCapture2::PGRGuid guid;
    if(!check_success(busMgr.GetCameraFromSerialNumber(camSerNo, &guid))) return false;
  }
  ROS_INFO ("Setup check_successful");
  return true;

}

void Camera::setFrameCallback(boost::function<void(FlyCapture2::Image *)> callback)
{
  userCallback_ = callback;
}

//typedef void (Camera::*funcPtr)(Image*, void*);

void Camera::initCam()
{
  FlyCapture2::BusManager busMgr;

  FlyCapture2::PGRGuid guid;
  check_success(busMgr.GetCameraFromIndex(0, &guid));
  ROS_INFO ("Camera GUID = %u %u %u %u", guid.value[0], guid.value[1], guid.value[2], guid.value[3]);

  check_success(camPGR.Connect(&guid));

  // Set to software triggering:
  FlyCapture2::TriggerMode triggerMode;
  check_success(camPGR.GetTriggerMode(&triggerMode));

  // Set camera to trigger mode 0
  triggerMode.onOff = false;

  check_success(camPGR.SetTriggerMode(&triggerMode));

  // Set other camera configuration stuff:
  FlyCapture2::FC2Config fc2Config;
  check_success(camPGR.GetConfiguration(&fc2Config));
  fc2Config.grabMode = FlyCapture2::DROP_FRAMES; // supposedly the default, but just in case..
  check_success(camPGR.SetConfiguration(&fc2Config));
  
  ROS_INFO ("Setting video mode to VIDEOMODE_640x480Y8, framerate to FRAMERATE_30...");
  if(check_success(camPGR.SetVideoModeAndFrameRate(FlyCapture2::VIDEOMODE_640x480Y8, FlyCapture2::FRAMERATE_30))) {
    ROS_INFO ("...check_success");
  }
  
  FlyCapture2::EmbeddedImageInfo embedInfo;
  embedInfo.frameCounter.onOff = true;
  check_success(camPGR.SetEmbeddedImageInfo(&embedInfo));

  FlyCapture2::CameraInfo camInfo;
  check_success(camPGR.GetCameraInfo(&camInfo));
  ROS_INFO ("camInfo.driverName = %s", camInfo.driverName);
  ROS_INFO ("camInfo.firmwareVersion = %s", camInfo.firmwareVersion);
  ROS_INFO ("camInfo.isColorCamera = %d", camInfo.isColorCamera);
}

void Camera::start()
{
  if (camPGR.IsConnected())
  {
    ROS_INFO ("IsConnected returned true");
  }
  else
    ROS_INFO ("IsConnected returned false");

  if(check_success(camPGR.StartCapture(frameDone, (void *)this))) {
    ROS_INFO ("StartCapture succeeded.");
  }

}

void Camera::stop()
{
  check_success(camPGR.StopCapture());
}

void Camera::SetVideoModeAndFramerate(unsigned int width, unsigned int height, string format, double rate)
{
  // TODO: support fractional frame rates
  // TODO: support colour cameras
  // TODO: support additional modes
  using namespace FlyCapture2;
  ROS_INFO_STREAM("Requested Width: " << width << " Height: " << height << " Format: '" << format <<"'");
  
  bool unknown = false;
  VideoMode vidMode;
  FrameRate frameRate;
  switch (width)
  {
    case 640:
      ROS_INFO("Height: %d",height);
      switch (height)
      {
        case 480:
          if (format == "Y8")
            vidMode = VIDEOMODE_640x480Y8;
          else if (format == "Y16")
            vidMode = VIDEOMODE_640x480Y16;
          else if (format == "RGB")
            vidMode = VIDEOMODE_640x480RGB;
          else {
            unknown = true;
            }
          break;
        default:
          unknown = true;
          break;
      }
      break;
    case 1280:
      switch (height)
      {
        case 960:
          if (format == "Y8")
          {
            vidMode = VIDEOMODE_1280x960Y8;
          }
          else if (format == "Y16")
          {
            vidMode = VIDEOMODE_1280x960Y16;
          }
          else if (format == "RGB")
          {
            vidMode = VIDEOMODE_1280x960RGB;
          }
          else {
            unknown = true;
          }
          break;
        default:
          unknown = true;
          break;
      }
      break;
    default:
      unknown = true;
      break;
  }

  if (unknown)
  {
    ROS_ERROR ("Unknown/unsupported video mode - mode not set");
    return;
  }

  // The following hardcoded numbers are from testing with
  // a FireflyMV USB (mono) camera using FlyCap2's configuration GUI
  // TODO: determine whether they are camera specific and if so, so this a better way..
  if (format == "FORMAT7")
  {
    frameRate = FRAMERATE_FORMAT7;
  }
  else if (rate >= 1.151 && rate < 7.606)
  {
    frameRate = FRAMERATE_7_5;
  }
  else if (rate >= 7.606 && rate < 15.211)
  {
    frameRate = FRAMERATE_15;
  }
  else if (rate >= 15.211 && rate < 30.430)
  {
    frameRate = FRAMERATE_30;
  }
  else if (rate >= 30.430 && rate < 60.861)
  {
    frameRate = FRAMERATE_60;
  }
  else
  {
    ROS_ERROR ("Unsupported frame rate");
    return;
  }

  ROS_INFO ("Attempting to set mode for width = %u height = %u format = %s frame_rate = %f",
      width, height, format.c_str(), rate);
  if(!check_success(camPGR.SetVideoModeAndFrameRate(vidMode, frameRate)))
  {
    ROS_ERROR ("Video mode and frame rate not set");
    ROS_ERROR ("vidMode = %u", vidMode);
    return;
  }
  ROS_INFO ("Video mode and frame rate set");

  FlyCapture2::Property prop;
  prop.type = FRAME_RATE;
  prop.autoManualMode = false;
  prop.onOff = true;
  prop.absControl = true;
  prop.absValue = rate;
  check_success(camPGR.SetProperty(&prop));
}

void Camera::SetExposure(bool _auto, bool onoff, unsigned int value)
{
  FlyCapture2::Property prop;
  prop.type = FlyCapture2::AUTO_EXPOSURE;
  prop.autoManualMode = _auto;
  prop.onOff = onoff;
  prop.valueA = value;
  check_success(camPGR.SetProperty(&prop));
}

void Camera::SetGain(bool _auto, float value)
{
  FlyCapture2::Property prop;
  prop.type = FlyCapture2::GAIN;
  prop.autoManualMode = _auto;
  prop.onOff = true;
  prop.absValue = value;
  check_success(camPGR.SetProperty(&prop));
}


void Camera::SetShutter (bool _auto, float value)
{
  FlyCapture2::Property prop;
  prop.type = FlyCapture2::AUTO_EXPOSURE;
  prop.autoManualMode = _auto;
  prop.onOff = true;
  prop.absValue = value;
  check_success(camPGR.SetProperty(&prop));
}

} // namespace pgrcamera
