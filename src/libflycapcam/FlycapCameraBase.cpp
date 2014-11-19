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

#include "libflycapcam/FlycapCameraBase.h"

namespace flycapcam {
  using namespace FlyCapture2;

FlycapCameraBase::FlycapCameraBase(FlyCapture2::PGRGuid guid, SerialNumber serialNumber, FlyCapture2::InterfaceType interfaceType)
{
  this->guid = guid;
  this->serialNumber = serialNumber;
  this->interfaceType = interfaceType;
}

FlycapResult FlycapCameraBase::getExposure(unsigned int& value) const
{
  Property prop;
  Error error;
  prop.type = AUTO_EXPOSURE;
  if((error = getCamera().GetProperty(&prop)) == PGRERROR_OK) {
    value = prop.valueA;
  }

  return FlycapResult(error);
}

FlycapResult FlycapCameraBase::getGain(float& value) const
{
  Property prop;
  Error error;
  prop.type = GAIN;
  if((error = getCamera().GetProperty(&prop)) == PGRERROR_OK) {
    value = prop.absValue;
  }

  return FlycapResult(error);
}

FlycapResult FlycapCameraBase::getShutter(float& value) const
{
  Property prop;
  Error error;
  prop.type = SHUTTER;
  if((error = getCamera().GetProperty(&prop)) == PGRERROR_OK) {
    value = prop.absValue;
  }

  return FlycapResult(error);
}

FlycapResult FlycapCameraBase::getFrameRate(float& value) const
{
  Property prop;
  Error error;
  prop.type = FlyCapture2::FRAME_RATE;
  if((error = getCamera().GetProperty(&prop)) == PGRERROR_OK) {
    value = prop.absValue;
  }

  return FlycapResult(error);
}

FlycapResult FlycapCameraBase::setExposure(bool automatic, bool onoff, unsigned int value)
{
  Property prop;
  Error error;

  prop.type = AUTO_EXPOSURE;
  prop.autoManualMode = automatic;
  prop.onOff = onoff;
  prop.valueA = value;

  error = getCamera().SetProperty(&prop);

  return FlycapResult(error);
}

FlycapResult FlycapCameraBase::setGain(bool automatic, float value)
{
  Property prop;
  Error error;

  prop.type = GAIN;
  prop.autoManualMode = automatic;
  prop.absValue = value;
  prop.onOff = true;

  error = getCamera().SetProperty(&prop);

  return FlycapResult(error);
}

FlycapResult FlycapCameraBase::setShutter(bool automatic, float value)
{
  Property prop;
  Error error;

  prop.type = FlyCapture2::SHUTTER;
  prop.autoManualMode = automatic;
  prop.absValue = value;
  prop.onOff = true;

  error = getCamera().SetProperty(&prop);

  return FlycapResult(error);
}

FlycapResult FlycapCameraBase::setFrameRate(bool automatic, float value)
{
  Property prop;
  Error error;

  prop.type = FlyCapture2::SHUTTER;
  prop.autoManualMode = automatic;
  prop.absValue = value;
  prop.onOff = true;

  error = getCamera().SetProperty(&prop);

  return FlycapResult(error);
}

FlycapResult FlycapCameraBase::getBandwidthAllocation(FlyCapture2::BandwidthAllocation& bandwidthAllocation) const
{
  FC2Config config;
  Error error;

  if(( error = getCamera().GetConfiguration(&config)) == PGRERROR_OK ) {
    bandwidthAllocation = config.bandwidthAllocation;
  }

  return FlycapResult(error);
}

FlycapResult FlycapCameraBase::getAsyncBusSpeed(BusSpeed& busSpeed) const
{
  FC2Config config;
  Error error;

  if(( error = getCamera().GetConfiguration(&config)) == PGRERROR_OK ) {
    busSpeed = config.asyncBusSpeed;
  }

  return FlycapResult(error);
}

FlycapResult FlycapCameraBase::getGrabMode(GrabMode& grabMode) const
{
  FC2Config config;
  Error error;

  if(( error = getCamera().GetConfiguration(&config)) == PGRERROR_OK ) {
    grabMode = config.grabMode;
  }

  return FlycapResult(error);
}


FlycapResult FlycapCameraBase::getGrabTimeout(int& grabTimeout) const
{
  FC2Config config;
  Error error;

  if(( error = getCamera().GetConfiguration(&config)) == PGRERROR_OK ) {
    grabTimeout = config.grabTimeout;
  }

  return FlycapResult(error);
}

FlycapResult FlycapCameraBase::getNumBuffers(unsigned int& numBuffers) const
{
  FC2Config config;
  Error error;

  if(( error = getCamera().GetConfiguration(&config)) == PGRERROR_OK ) {
    numBuffers = config.numBuffers;
  }

  return FlycapResult(error);
}

FlycapResult FlycapCameraBase::isHighPerformanceRetrieveBufferEnabled(bool& enabled) const
{
  FC2Config config;
  Error error;

  if(( error = getCamera().GetConfiguration(&config)) == PGRERROR_OK ) {
    enabled = config.highPerformanceRetrieveBuffer;
  }

  return FlycapResult(error);
}

FlycapResult FlycapCameraBase::updateConfigParameterHelper(std::function< FC2Config (const FC2Config&)> update)
{
  FC2Config config;
  Error error;

  if((error = getCamera().GetConfiguration(&config)) != PGRERROR_OK) {
    return FlycapResult(error);
  }

  FC2Config newConfig = update(config);

  error = getCamera().SetConfiguration(&newConfig);
  return FlycapResult(error);
}


FlycapResult FlycapCameraBase::setBandwidthAllocation(BandwidthAllocation bandwidthAllocation)
{
  return updateConfigParameterHelper([bandwidthAllocation](FC2Config config) {
    config.bandwidthAllocation = bandwidthAllocation;
    return config;
  });
}

FlycapResult FlycapCameraBase::setAsyncBusSpeed(BusSpeed busSpeed)
{
  return updateConfigParameterHelper([busSpeed](FC2Config config) {
    config.asyncBusSpeed = busSpeed;
    return config;
  });
}

FlycapResult FlycapCameraBase::setGrabMode(GrabMode grabMode)
{
  return updateConfigParameterHelper([grabMode](FC2Config config) {
    config.grabMode = grabMode;
    return config;
  });
}

FlycapResult FlycapCameraBase::setGrabTimeout(int grabTimeout)
{
  return updateConfigParameterHelper([grabTimeout](FC2Config config) {
    config.grabTimeout = grabTimeout;
    return config;
  });
}

FlycapResult FlycapCameraBase::setHighPerformanceRetrieveBufferEnabled(bool enabled)
{
  return updateConfigParameterHelper([enabled](FC2Config config) {
    config.highPerformanceRetrieveBuffer = enabled;
    return config;
  });
}

FlycapResult FlycapCameraBase::setNumBuffers(int numBuffers)
{
  return updateConfigParameterHelper([numBuffers](FC2Config config) {
    config.numBuffers = numBuffers;
    return config;
  });
}

PGRGuid FlycapCameraBase::getGuid() const
{
  return guid;
}

InterfaceType FlycapCameraBase::getInterfaceType() const
{
  return interfaceType;
}

SerialNumber FlycapCameraBase::getSerialNumber() const
{
  return serialNumber;
}

}

