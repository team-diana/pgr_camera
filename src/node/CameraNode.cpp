/*********************************************************************
\\* Software License Agreement (BSD License)
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

#include <CameraNode.h>
#include "pgr_camera/PgrCameraConfig.h"
#include "libflycapcam/FlycapCameraManager.h"

// ROS communication
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/fill_image.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <image_transport/image_transport.h>
#include <camera_calibration_parsers/parse_ini.h>
#include <std_msgs/String.h>
#include <polled_camera/publication_server.h>
#include <camera_info_manager/camera_info_manager.h>

// Dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <driver_base/SensorLevels.h>

#include <XmlRpcValue.h>

// Standard libs
#include <boost/scoped_ptr.hpp>
#include <boost/bind.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/format.hpp>
#include <boost/program_options.hpp>
#include <vector>
#include <sstream>
#include <fstream>
#include <sys/stat.h>
#include <cstdlib>
#include <unistd.h>

#include <team_diana_lib/logging/logging.h>
#include <team_diana_lib/strings/strings.h>

using namespace Td;
using namespace std;
using namespace FlyCapture2;


CameraNode::CameraNode(const ros::NodeHandle &nodeHandle) :
  nodeHandler(nodeHandle),
  imageTransport(nodeHandle),
  cameraInfoManager(nodeHandle),
  dynamicReconfigureServer(nodeHandle)
{}

DynamicReconfigureServer &CameraNode::getDynamicReconfigureServer()
{
  return dynamicReconfigureServer;
}

void CameraNode::configure(pgr_camera::PgrCameraConfig &config, uint32_t level)
{
  baseConfigure(config,  level);
}


void CameraNode::baseConfigure(pgr_camera::PgrCameraConfig &config,  uint32_t level)
{
  ros_info("Reconfigure request received");
  loadIntrinsics(config.intrinsics_ini, getFlycapCamera()->getSerialNumber());
}

CameraNode::~CameraNode()
{
  stop();
}

void CameraNode::init()
{
  ros_info("Setting up camera");
  char cameraName[200] = "image_raw";
  cameraPublisher = imageTransport.advertiseCamera(cameraName, 1);
  ros_info("Setup done");
}

void CameraNode::start()
{
  ROS_INFO("Starting up camera");
  getFlycapCamera()->start();
}

void CameraNode::stop()
{
  publicationServer.shutdown();
  cameraPublisher.shutdown();
  getFlycapCamera()->stop();
}

bool CameraNode::frameToImage(FlyCapture2::Image *frame, sensor_msgs::Image &image)
{

  // NOTE: 16-bit and Yuv formats not supported
  static const char *BAYER_ENCODINGS[] = { "none", "bayer_rggb8",
                                         "bayer_grbg8", "bayer_gbrg8", "bayer_bggr8", "unknown"
                                         };
  std::string encoding;

  //unsigned int bpp = frame->GetBitsPerPixel();
  FlyCapture2::BayerTileFormat bayerFmt;
  bayerFmt = frame->GetBayerTileFormat();
  //ROS_INFO("bayer is %u", bayerFmt);
  if (bayerFmt == FlyCapture2::NONE) {
    encoding = sensor_msgs::image_encodings::MONO8;
  }
  else {
    encoding = BAYER_ENCODINGS[bayerFmt];
  }

  return sensor_msgs::fillImage(image, encoding, frame->GetRows(),
                                frame->GetCols(), frame->GetStride(), frame->GetData());
}

bool CameraNode::processFrame(FlyCapture2::Image *frame, sensor_msgs::Image &img, sensor_msgs::CameraInfo &cam_info,  ros::Time timestamp)
{
  img.header.stamp = cam_info.header.stamp = timestamp;

  if (!frameToImage(frame, img)) {
    return false;
  }
  cam_info.height = img.height;
  cam_info.width = img.width;

  return true;
}

void CameraNode::retrieveAndPublishFrame(ros::Time timestamp)
{
  FlyCapture2::Image image;
  flycapcam::FlycapResult result = getFlycapCamera()->retrieveFrame(image);
  if(result) {
      publishImage(image, timestamp);
  } else {
    ros_error (toString("error while retrieving frame: ", result.getError().GetDescription()) );
  }
}

void CameraNode::publishImage(FlyCapture2::Image& frame, ros::Time timestamp)
{
  if (processFrame(&frame, sensorImage, cameraInfo, timestamp)) {
    ROS_INFO("Publish image, timestamp is %lu",  timestamp.toNSec());
    cameraPublisher.publish(sensorImage, cameraInfo, timestamp);
  }
}

void CameraNode::loadIntrinsics(string inifile, unsigned int cameraSerialNumber)
{
  // Read in calibration file

  ROS_INFO("OVERRIDING GIVEN ID FILE!");
  inifile = boost::str(boost::format("intrinsics%1%.ini") % cameraSerialNumber);

  char cwd[2048];
  if (getcwd(cwd, sizeof(cwd)) != NULL) {
    ROS_INFO("Searching intrinsics %s file in %s", inifile.c_str(),  cwd);
  }
  std::string cameraName = Td::toString(cameraSerialNumber);

  ROS_INFO("Loading calibration for camera %d with name %s",  cameraSerialNumber, cameraName.c_str());
  ifstream fin(inifile.c_str());
  if (fin.is_open()) {
    if (camera_calibration_parsers::readCalibrationIni(inifile, cameraName, cameraInfo)) {
      ROS_INFO("Loaded calibration for camera '%s' from intrinsics %s", cameraName.c_str(),  inifile.c_str());
    }
    else {
      ROS_WARN("Failed to load intrinsics from camera");
    }
    fin.close();
  }
  else {
    ROS_WARN("Intrinsics file not found: %s", inifile.c_str());
  }
}
