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

#ifndef PGRCAMERA_H
#define PGRCAMERA_H

#include "PgrTypes.h"

#include "flycapture/FlyCapture2.h"
#include "flycapture/CameraBase.h"
#include "flycapture/Image.h"

#include <ros/ros.h>
#include <string>
#include <cstdlib>
#include <memory>
#include <mutex>
#include <functional>

#define PRINT_ERROR {ROS_ERROR(error.GetDescription());}
#define PGRERROR_OK FlyCapture2::PGRERROR_OK
#define PRINT_ERROR_AND_RETURN_FALSE {ROS_ERROR(error.GetDescription()); return false;}

namespace pgr_camera {

class PgrCameraFactory;

class PgrCamera
{
    friend class pgr_camera::PgrCameraFactory;

public:
    PgrCamera ( std::shared_ptr<FlyCapture2::CameraBase> camera, FlyCapture2::PGRGuid guid,
                SerialNumber serialNumber, FlyCapture2::InterfaceType interfaceType );

public:
    void start ();
    void stop ();

    // TODO: remove initCam function, use multiple methods instead.
    void initCam();

    static void frameDone ( FlyCapture2::Image *frame,const void *pCallbackData );
    void setFrameCallback ( std::function < void ( FlyCapture2::Image *, unsigned int )  > callback );


    void SetBinning ( unsigned int horz, unsigned int vert );
    void SetExposure ( bool _auto, bool onoff, unsigned int value = 50 );
    void SetGain ( bool _auto, float value = 0.0 );
    void SetShutter ( bool _auto, float value = 0.015 );
    float GetFrameRate();
    void SetFrameRate ( bool _auto,  float value = 60 );
    void SetGigESettings ( unsigned int packetSize,  unsigned int packetDelay );
    FlyCapture2::PropertyInfo getPropertyInfo ( FlyCapture2::PropertyType type );


    void SetGigEPacketSize ( unsigned int packetSize );
    void SetGigEPacketDelay ( unsigned int packetDelay );
    FlyCapture2::GigECamera* castToGigECamera ( FlyCapture2::CameraBase* cameraBase );
    FlyCapture2::GigEImageSettings getCurrentGigEImageSettings();
    unsigned int getCurrentPacketSize();
    unsigned int getCurrentPacketDelay();
    unsigned int getGigEProperty ( FlyCapture2::GigEProperty gigeProperty );


    std::shared_ptr<FlyCapture2::CameraBase> getCamera() {
        return camPGR;
    }

    SerialNumber getSerialNumber() {
        return camSerNo;
    }

    FlyCapture2::PGRGuid getGuid() {
        return guid;
    }

    unsigned int getCamIndex() {
        return camIndex;
    }

    void setCamIndex ( unsigned int camIndex ) {
        this->camIndex = camIndex;
    }

    FlyCapture2::InterfaceType getInterfaceType() {
        return interfaceType;
    }

    void enableCallback ( bool enable ) {
        this->callbackEnabled = enable;
    }

    void PrintCameraInfo ( FlyCapture2::CameraInfo *pCamInfo );

    // FIXME: following should really be private, but I can't see how to make the compiler
    // happy if they are..
    std::function < void ( FlyCapture2::Image *, unsigned int ) > userCallback;
    std::mutex frameMutex_;


private:
    unsigned int camSerNo;
    unsigned int camIndex;
    std::shared_ptr<FlyCapture2::CameraBase> camPGR;
    //FlyCapture2::GigECamera gigecamPGR;
    FlyCapture2::PGRGuid guid;
    FlyCapture2::Image rawPGRImage;
    FlyCapture2::FrameRate frameRate;
    FlyCapture2::InterfaceType interfaceType;
    bool callbackEnabled;
};

}

#endif
// PGRCAMERA_H
