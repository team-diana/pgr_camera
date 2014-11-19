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

#ifndef FLYCAP_CAMERA_GIGE_H
#define FLYCAP_CAMERA_GIGE_H

#include "libflycapcam/FlycapTypes.h"
#include "libflycapcam/FlycapCameraBase.h"

#include "flycapture/FlyCapture2.h"
#include "flycapture/CameraBase.h"
#include "flycapture/GigECamera.h"
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

class FlycapCameraGigE : public FlycapCameraBase
{

  enum FlycapCameraGigEState {
    NOT_INITIALIZED,
    READY,
    CAPTURING
  };

public:

  FlycapCameraGigE(std::unique_ptr<FlyCapture2::GigECamera> camera,
                   FlyCapture2::PGRGuid guid,
                   SerialNumber serialNumber,
                   FlyCapture2::InterfaceType interfaceType);

  virtual ~FlycapCameraGigE();

  void initCam() override;
  void start() override;
  void stop() override;

  FlycapResult retrieveFrame(FlyCapture2::Image& image) override;

  FlycapResult getPacketSize(unsigned int& packetSize) const;
  FlycapResult getPacketDelay(unsigned int& packetDelay) const;

  FlycapResult setPacketSize(unsigned int packetSize);
  FlycapResult setPacketDelay(unsigned int packetDelay);


protected:
  FlyCapture2::CameraBase& getCamera() const override;

private:
  void stopRunFunctionRestartHelper(std::function<void()> fun);

private:
  std::unique_ptr<FlyCapture2::GigECamera> camera;
  FlycapCameraGigEState state;

};

}

#endif
