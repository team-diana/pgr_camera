/********************************************************************
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

#ifndef FLYCAP_CAMERA_BASE_H
#define FLYCAP_CAMERA_BASE_H

#include "libflycapcam/FlycapTypes.h"
#include "libflycapcam/FlycapCamera.h"

#include "flycapture/FlyCapture2.h"
#include "flycapture/CameraBase.h"
#include "flycapture/Image.h"

#include <boost/timer.hpp>
#include <boost/asio.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/asio/deadline_timer.hpp>

#include <string>
#include <functional>
#include <memory>
#include <mutex>

namespace flycapcam
{

class FlycapCameraBase : public FlycapCamera
{
public:

  FlycapCameraBase(FlyCapture2::PGRGuid guid,
               SerialNumber serialNumber,
               FlyCapture2::InterfaceType interfaceType);

  virtual ~FlycapCameraBase() = 0;

  void initCam() override;
  void start() override;
  void stop() override;

  FlycapResult retrieveFrame(FlyCapture2::Image& image) override;
  FlycapResult getExposure(unsigned int& value) const override;
  FlycapResult getGain(float& value) const override;
  FlycapResult getShutter(float& value) const override;
  FlycapResult getFrameRate(float& framerate) const override;

  FlycapResult setExposure(bool automatic, bool onoff, unsigned int value = 50) override;
  FlycapResult setGain(bool automatic, float value = 0.0) override;
  FlycapResult setShutter(bool automatic, float value = 0.015) override;
  FlycapResult setFrameRate(bool automatic,  float value = 60) override;

  FlycapResult getAsyncBusSpeed(FlyCapture2::BusSpeed& busSpeed) const override;
  FlycapResult getGrabMode(FlyCapture2::GrabMode& grabMode) const override;
  FlycapResult getGrabTimeout(int& grabTimeout) const override;
  FlycapResult getBandwidthAllocation(FlyCapture2::BandwidthAllocation& bandwidthAllocation) const override;
  FlycapResult isHighPerformanceRetrieveBufferEnabled(bool& enabled) const override;
  FlycapResult getNumBuffers(unsigned int& numBuffers) const override;

  FlycapResult setAsyncBusSpeed(FlyCapture2::BusSpeed busSpeed) override;
  FlycapResult setGrabMode(FlyCapture2::GrabMode grabMode) override;
  FlycapResult setGrabTimeout(int grabTimeout) override;
  FlycapResult setBandwidthAllocation(FlyCapture2::BandwidthAllocation bandwidthAllocation) override;
  FlycapResult setHighPerformanceRetrieveBufferEnabled(bool enabled) override;
  FlycapResult setNumBuffers(int numBuffers) override;

  SerialNumber getSerialNumber() const override;
  FlyCapture2::PGRGuid getGuid() const override;
  FlyCapture2::InterfaceType getInterfaceType() const override;

protected:
  FlyCapture2::CameraBase& getCamera() const override;
  std::mutex cameraMutex;

private:
  FlycapResult updateConfigParameterHelper( std::function<FlyCapture2::FC2Config (const FlyCapture2::FC2Config&)> update);

private:
  unsigned int serialNumber;
  FlyCapture2::PGRGuid guid;
  FlyCapture2::InterfaceType interfaceType;
};

}

#endif
