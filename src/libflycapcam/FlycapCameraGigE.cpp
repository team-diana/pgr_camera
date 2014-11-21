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

#include "libflycapcam/FlycapCameraGigE.h"

#include <team_diana_lib/strings/strings.h>

#include <mutex>
#include <functional>

namespace flycapcam {

using namespace FlyCapture2;
using namespace Td;

FlycapCameraGigE::FlycapCameraGigE(std::unique_ptr< FlyCapture2::GigECamera > camera,
                                   FlyCapture2::PGRGuid guid,
                                   SerialNumber serialNumber,
                                   FlyCapture2::InterfaceType interfaceType)
  : FlycapCameraBase(guid, serialNumber, interfaceType),
    state(NOT_INITIALIZED)
{
  this->camera = std::move(camera);
}

FlycapCameraGigE::~FlycapCameraGigE()
{
  if(state == CAPTURING) {
    camera->StopCapture();
  }
}

FlyCapture2::CameraBase& FlycapCameraGigE::getCamera() const
{
  return *camera;
}

void FlycapCameraGigE::initCam()
{
  state = READY;
}

void FlycapCameraGigE::start()
{
  if(state != READY) {
    std::cerr << "Unexpected state during start(): " << state;
  }
  camera->StartCapture();

  state = CAPTURING;
}

void FlycapCameraGigE::stop()
{
  if(state != CAPTURING) {
    std::cerr << "Unexpected state during start(): " << state;
  }
  camera->StartCapture();
  state = READY;
}

FlycapResult FlycapCameraGigE::retrieveFrame(Image& image)
{
  Error error;
  std::cout << toString("retrieving frame of camera ", getSerialNumber()) << std::endl;

  error = camera->RetrieveBuffer(&image);

  if(error != PGRERROR_OK) {
    std::cerr << (Td::toString("Error for camera: ", getSerialNumber()
                               , " : " , error.GetDescription())) << std::endl;
  } else {
    std::cout << (Td::toString("Retrieve Buffer Ok for camera: ", getSerialNumber()
                               )) << std::endl;
  }

  return FlycapResult(error);
}

FlycapResult FlycapCameraGigE::getPacketSize(unsigned int& packetSize) const
{
  Error error;
  GigEProperty property;

  property.propType = FlyCapture2::PACKET_SIZE;

  if((error = camera->GetGigEProperty(&property)) == PGRERROR_OK) {
    packetSize = property.value;
  }

  return FlycapResult(error);
}

FlycapResult FlycapCameraGigE::getPacketDelay(unsigned int& packetDelay) const
{
  Error error;
  GigEProperty property;

  property.propType = PACKET_DELAY;

  if((error = camera->GetGigEProperty(&property)) == PGRERROR_OK) {
    packetDelay = property.value;
  }

  return FlycapResult(error);
}

FlycapResult FlycapCameraGigE::isPacketResendEnabled(bool& enabled)
{
  Error error;
  GigEConfig config;

  if((error = camera->GetGigEConfig(&config)) == PGRERROR_OK) {
    enabled = config.enablePacketResend;
  }

  return FlycapResult(error);
}


FlycapResult FlycapCameraGigE::setPacketSize(unsigned int packetSize)
{

  Error error;
  GigEProperty property;
  property.propType = FlyCapture2::PACKET_SIZE;
  property.value = packetSize;

  auto setPacketDelayImpl = [&]() {
    error = camera->SetGigEProperty(&property);
  };

  stopRunFunctionRestartHelper(setPacketDelayImpl);

  return FlycapResult(error);
}

FlycapResult FlycapCameraGigE::setPacketDelay(unsigned int packetDelay)
{

  Error error;
  GigEProperty property;
  property.propType = PACKET_DELAY;
  property.value = packetDelay;

  auto setPacketDelayImpl = [&]() {
    error = camera->SetGigEProperty(&property);
  };

  stopRunFunctionRestartHelper(setPacketDelayImpl);

  return FlycapResult(error);
}

FlycapResult FlycapCameraGigE::setPacketResendEnabled(bool enabled)
{
  Error error;
  GigEConfig config;
  config.enablePacketResend = enabled;

  stopRunFunctionRestartHelper([&]() {
    error =camera->SetGigEConfig(&config);
  });

  return FlycapResult(error);
}

FlycapResult FlycapCameraGigE::getAvailableGigEPacketSize(unsigned int& packetSize)
{
  Error error;

  stopRunFunctionRestartHelper([&]() {
    error =camera->DiscoverGigEPacketSize(&packetSize);
  });

  return FlycapResult(error);
}

FlycapResult FlycapCameraGigE::getGigEChannelsInfo(std::vector< GigEStreamChannel >& channelsInfo)
{
  Error error;
  channelsInfo.clear();

  unsigned int numOfChannels;
  if((error = camera->GetNumStreamChannels(&numOfChannels)) == PGRERROR_OK) {
     std::cout << " number of channels is " << numOfChannels << std::endl;
     for(int i = 0; i << numOfChannels; i++) {
       GigEStreamChannel channelInfo;
       if((error = camera->GetGigEStreamChannelInfo(i, &channelInfo)) == PGRERROR_OK) {
          channelsInfo.push_back(channelInfo);
       } else {
          std::cerr << "error while retriving info for channel " << i <<
            " : " << error.GetDescription() << std::endl;
         channelsInfo.clear();
         break;
      }
    }
  }

  return FlycapResult(error);
}

FlycapResult FlycapCameraGigE::getCameraStats(CameraStats& cameraStats)
{
  Error error;

  error = camera->GetStats(&cameraStats);

  if(error != PGRERROR_OK) {
    std::cerr << "Error while getting camera stats" << std::endl;
  }

  return FlycapResult(error);
}

void FlycapCameraGigE::stopRunFunctionRestartHelper(std::function< void() > fun)
{
  std::lock_guard<std::mutex> lock(cameraMutex);

  if(state == CAPTURING) {
    // TODO: if needed, add some sleep between functions.
    stop();
    fun();
    start();
  } else {
    fun();
  }
}

}
