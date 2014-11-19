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

#ifndef FLYCAP_CAMERA_H
#define FLYCAP_CAMERA_H

#include "libflycapcam/FlycapTypes.h"

#include "flycapture/FlyCapture2.h"
#include "flycapture/CameraBase.h"
#include "flycapture/Image.h"

#include <string>
#include <functional>

namespace flycapcam
{

class FlycapCamera
{
public:
  virtual void initCam();
  virtual void start();
  virtual void stop();

  virtual ~FlycapCamera() = 0;

  virtual FlycapResult retrieveFrame(FlyCapture2::Image& image);
  virtual void setFrameCallback(std::function <void (FlyCapture2::Image *, unsigned int)> callback);

  virtual FlycapResult getExposure(unsigned int& value) const;
  virtual FlycapResult getGain(float& value) const;
  virtual FlycapResult getShutter(float& value) const;
  virtual FlycapResult getFrameRate(float& framerate) const;

  virtual FlycapResult setExposure(bool automatic, bool onoff, unsigned int value = 50);
  virtual FlycapResult setGain(bool automatic, float value = 0.0);
  virtual FlycapResult setShutter(bool automatic, float value = 0.015);
  virtual FlycapResult setFrameRate(bool automatic,  float value = 60);

  virtual FlycapResult getAsyncBusSpeed(FlyCapture2::BusSpeed& busSpeed) const;
  virtual FlycapResult getGrabMode(FlyCapture2::GrabMode& grabMode) const;
  virtual FlycapResult getGrabTimeout(int& grabTimeout) const;
  virtual FlycapResult getBandwidthAllocation(FlyCapture2::BandwidthAllocation& bandwidthAllocation) const;
  virtual FlycapResult isHighPerformanceRetrieveBufferEnabled(bool& enabled) const;
  virtual FlycapResult getNumBuffers(unsigned int& numBuffers) const;

  virtual FlycapResult setAsyncBusSpeed(FlyCapture2::BusSpeed busSpeed);
  virtual FlycapResult setGrabMode(FlyCapture2::GrabMode grabMode);
  virtual FlycapResult setGrabTimeout(int grabTimeout);
  virtual FlycapResult setBandwidthAllocation(FlyCapture2::BandwidthAllocation bandwidthAllocation);
  virtual FlycapResult setHighPerformanceRetrieveBufferEnabled(bool enabled);
  virtual FlycapResult setNumBuffers(int numBuffers);

  virtual SerialNumber getSerialNumber() const;
  virtual FlyCapture2::PGRGuid getGuid() const;
  virtual FlyCapture2::InterfaceType getInterfaceType() const;

protected:
  virtual FlyCapture2::CameraBase& getCamera() const;

};

}

#endif
