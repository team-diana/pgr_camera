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

#include <PgrCamera.h>
#include "pgr_camera/PgrCameraConfig.h"
#include "PgrCameraFactory.h"

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

#include "PgrCameraNode.h"
#include "CameraSynchronizer.h"
#include <team_diana_lib/logging/logging.h>

using namespace Td;


PgrCameraNode::PgrCameraNode(const ros::NodeHandle &nodeHandle,
                             shared_ptr<pgr_camera::PgrCamera> pgrCamera) :
  nodeHandler(nodeHandle),
  imageTransport(nodeHandle),
  cameraInfoManager(nodeHandle),
  dynamicReconfigureServer(nodeHandle),
  pgrCamera(pgrCamera),
  state(SETTING_UP)
{
}

bool PgrCameraNode::isSetupDone()
{
  return state != SETTING_UP;
}

DynamicReconfigureServer &PgrCameraNode::getDynamicReconfigureServer()
{
  return dynamicReconfigureServer;
}

void PgrCameraNode::configure(pgr_camera::PgrCameraConfig &config, uint32_t level)
{
  baseConfigure(config,  level);
  currentConfig = config;
}

void PgrCameraNode::setupConfigure()
{
  pgr_camera::PgrCameraConfig deflt;
  dynamicReconfigureServer.getConfigDefault(deflt);
  pgr_camera::PgrCameraConfig min,  max;
  min = max = deflt;

  baseSetupConfigure(min, max);

  dynamicReconfigureServer.setConfigMin(min);
  dynamicReconfigureServer.setConfigMax(max);
}

void PgrCameraNode::baseSetupConfigure(pgr_camera::PgrCameraConfig &min, pgr_camera::PgrCameraConfig &max)
{

  PropertyInfo gain = getPropertyInfo(GAIN);
  min.gain = gain.min;
  max.gain = gain.max;
  PropertyInfo shutter = getPropertyInfo(SHUTTER);
  min.shutter = shutter.min;
  max.shutter = shutter.max;
  PropertyInfo framerate = getPropertyInfo(FRAME_RATE);
  min.frame_rate = framerate.min;
  max.frame_rate = framerate.max;
}

void PgrCameraNode::baseConfigure(pgr_camera::PgrCameraConfig &config,  uint32_t level)
{
  ros_info("Reconfigure request received");

  if (level >= (uint32_t) driver_base::SensorLevels::RECONFIGURE_STOP) {
    stop();
  }

  loadIntrinsics(config.intrinsics_ini, pgrCamera->getSerialNumber());

  if (config.auto_exposure) {
    pgrCamera->SetExposure(true, true);
  }
  else {
    pgrCamera->SetExposure(false, true);
  }

  if (config.auto_shutter) {
    pgrCamera->SetShutter(true);
  }
  else {
    pgrCamera->SetShutter(false, (float) config.shutter);
  }


  if (config.auto_gain) {
    pgrCamera->SetGain(true);
  }
  else {
    pgrCamera->SetGain(false, (float) config.gain);
  }

  float frameRate = pgrCamera->GetFrameRate();
  ROS_INFO("Current framerate is: %f",  frameRate);
  pgrCamera->SetFrameRate(false, (float) config.frame_rate);

  sensorImage.header.frame_id = cameraInfo.header.frame_id = config.frame_id;

  if (level >= (uint32_t) driver_base::SensorLevels::RECONFIGURE_STOP) {
    start();
  }
}

PropertyInfo PgrCameraNode::getPropertyInfo(PropertyType propertyType)
{
  return pgrCamera->getPropertyInfo(propertyType);
}

PgrCameraNode::~PgrCameraNode()
{
  stop();
  pgrCamera.reset();
}

void PgrCameraNode::setup()
{
  if (state != SETTING_UP) {
    return;
  }

  ros_info("Setting up camera");
  pgrCamera->setFrameCallback(boost::bind(&PgrCameraNode::publishImage, this, _1, _2));
  char cameraName[200] = "image_raw";
  cameraPublisher = imageTransport.advertiseCamera(cameraName, 1);
  streamEnabledService = nodeHandler.advertiseService("enable_stream", &PgrCameraNode::enableStreamCallback,  this);
  oneShotService = nodeHandler.advertiseService("enable_oneshot",  &PgrCameraNode::enableOneShot,  this);
  std::string cameraDidPublishPublisherName = "image_published";
  cameraDidPublishPublisher = nodeHandler.advertise<pgr_camera::published> (cameraDidPublishPublisherName,  1000);

  state = STOP;
  ros_info("Setup done");

  //setupConfigure();
}

void PgrCameraNode::start()
{
  if (state == PgrCameraNode::RUNNING) {
    return;
  }

  ROS_INFO("Starting up camera");
  pgrCamera->start();
  state = PgrCameraNode::RUNNING;
  ROS_INFO("Camera started");
}

void PgrCameraNode::stop()
{
  if (state == STOP) {
    return;
  }

  pgrCamera->stop();               // Must stop camera before streaming_pub_.
  publicationServer.shutdown();
  cameraPublisher.shutdown();

  state = STOP;
}

bool PgrCameraNode::frameToImage(FlyCapture2::Image *frame, sensor_msgs::Image &image)
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

bool PgrCameraNode::processFrame(FlyCapture2::Image *frame, sensor_msgs::Image &img, sensor_msgs::CameraInfo &cam_info,  ros::Time timestamp)
{
  img.header.stamp = cam_info.header.stamp = timestamp;

  if (!frameToImage(frame, img)) {
    return false;
  }
  cam_info.height = img.height;
  cam_info.width = img.width;

  diagnosticsData.frameCount++;
  return true;
}

void PgrCameraNode::retrieveFrame()
{
  pgrCamera->retrieveFrame();
}

void PgrCameraNode::publishImage(FlyCapture2::Image *frame, int camIndex)
{
  ros::Time timestamp = ros::Time::now();
  publishImageWithTimestamp(frame,  camIndex,  timestamp);
}

void PgrCameraNode::overrideFrameCallback(std::function < void (FlyCapture2::Image *, unsigned int)  > callback)
{
  pgrCamera->setFrameCallback(callback);
}

bool PgrCameraNode::enableStreamCallback(pgr_camera::booleanRequest &request, pgr_camera::booleanResponse &response)
{
  enablePublishing(request.enabled);
  response.enabled = request.enabled;
  return true;
}

bool PgrCameraNode::enableOneShot(pgr_camera::oneshotRequest &request, pgr_camera::oneshotResponse &response)
{
  enableOneShot(request.enabled, request.oneshot_count);
  response.enabled = request.enabled;
  return true;
}

void PgrCameraNode::publishImageWithTimestamp(FlyCapture2::Image *frame, int camIndex,  ros::Time timestamp)
{
  if (processFrame(frame, sensorImage, cameraInfo, timestamp)) {
    ROS_INFO("Publish image of camera n.%d , timestamp is %lu",  camIndex, timestamp.toNSec());
    cameraPublisher.publish(sensorImage, cameraInfo, timestamp);

    diagnosticsData.publishedCount++;
    diagnosticsData.oneshotCount++;

    pgr_camera::published published;
    published.published = true;
    published.count = diagnosticsData.publishedCount;
    published.oneshot_count = diagnosticsData.oneshotCount;
    cameraDidPublishPublisher.publish(published);
  }
}

void PgrCameraNode::loadIntrinsics(string inifile, unsigned int cameraSerialNumber)
{
  // Read in calibration file

  ROS_INFO("OVERRIDING GIVEN ID FILE!");
  inifile = boost::str(boost::format("intrinsics%1%.ini") % cameraSerialNumber);

  char cwd[2048];
  if (getcwd(cwd, sizeof(cwd)) != NULL) {
    ROS_INFO("Searching intrinsics %s file in %s", inifile.c_str(),  cwd);
  }
  std::string cameraName;

  ROS_INFO("Loading calibration for camera %d",  cameraSerialNumber);
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

