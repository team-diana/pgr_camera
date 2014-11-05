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

#include "pgr_camera/PgrCamera.h"
#include <cstdio>
#include <cstring>
#include <cstdlib>

#define SLEEP_TIME 1

using namespace std;

PgrCamera::PgrCamera ( shared_ptr< CameraBase > camera, PGRGuid guid, unsigned int serialNumber, InterfaceType interfaceType )
     :  camPGR ( camera ),  guid ( guid ),  camSerNo ( serialNumber ),  interfaceType ( interfaceType ), callbackEnabled(true)
{

}


// FIXME: How to make this a member function?
void PgrCamera::frameDone ( FlyCapture2::Image * frame,const void *pCallbackData )
{
     PgrCamera *camPtr = ( PgrCamera * ) pCallbackData;
     if ( camPtr->callbackEnabled && !camPtr->userCallback.empty() ) {
          // TODO: thread safety OK here?
          boost::lock_guard<boost::mutex> guard ( camPtr->frameMutex_ );
          camPtr->userCallback ( frame,  camPtr->getCamIndex() );
          //ROS_INFO("in frameDone");
     } else {
          ROS_WARN ( "User callback empty!" );
     }
}


void PgrCamera::setFrameCallback ( boost::function<void ( Image *, unsigned int ) > callback )
{
     userCallback = callback;
     enableCallback(true);
     ROS_INFO ( "CALLBACK setted" );
}

void PgrCamera::initCam()
{
     // Start capturing images:
     FlyCapture2::Error error;
     FlyCapture2::PGRGuid guid;

     // Set to software triggering:
     FlyCapture2::TriggerMode triggerMode;
     if ( ( error = camPGR->GetTriggerMode ( &triggerMode ) ) != PGRERROR_OK )
          PRINT_ERROR;

     // Set camera to trigger mode 0
     triggerMode.onOff = false;

     if ( ( error = camPGR->SetTriggerMode ( &triggerMode ) ) != PGRERROR_OK )
          PRINT_ERROR;

     // Set other camera configuration stuff:
     FlyCapture2::FC2Config fc2Config;
     if ( ( error = camPGR->GetConfiguration ( &fc2Config ) ) != PGRERROR_OK )
          PRINT_ERROR;
     fc2Config.grabMode = FlyCapture2::DROP_FRAMES; // supposedly the default, but just in case..
     if ( ( error = camPGR->SetConfiguration ( &fc2Config ) ) != PGRERROR_OK )
          PRINT_ERROR;
     ROS_INFO ( "Setting video mode to VIDEOMODE_640x480Y8, framerate to FRAMERATE_30..." );

     FlyCapture2::EmbeddedImageInfo embedInfo;
     embedInfo.frameCounter.onOff = true;
     if ( ( error = camPGR->SetEmbeddedImageInfo ( &embedInfo ) ) != PGRERROR_OK )
          PRINT_ERROR;

     FlyCapture2::CameraInfo camInfo;
     if ( ( error = camPGR->GetCameraInfo ( &camInfo ) ) != PGRERROR_OK )
          PRINT_ERROR;
     ROS_INFO ( "camInfo.driverName = %s", camInfo.driverName );
     ROS_INFO ( "camInfo.firmwareVersion = %s", camInfo.firmwareVersion );
     ROS_INFO ( "camInfo.isColorCamera = %d", camInfo.isColorCamera );
}

void PgrCamera::start()
{
     FlyCapture2::Error error;
     if ( camPGR->IsConnected() ) {
          ROS_INFO ( "IsConnected returned true" );
     } else
          ROS_INFO ( "IsConnected returned false" );

     if ( ( error = camPGR->StartCapture ( frameDone, ( const void * ) this ) ) != PGRERROR_OK ) {
          ROS_ERROR ( error.GetDescription () );
     } else {
          ROS_INFO ( "StartCapture succeeded." );
     }
}

void PgrCamera::stop()
{
     FlyCapture2::Error error;
     if ( ( error = camPGR->StopCapture() ) != PGRERROR_OK )
          PRINT_ERROR;
}

void PgrCamera::SetBinning ( unsigned int horz, unsigned int vert )
{
// 	ROS_INFO ("Setting  the binning ");
// 	using namespace FlyCapture2;
// 	GigECamera gigeCamera;
// 	Error error;
// 	if ((error = error = gigeCamera.Connect(&guidPGR))
// 			!= PGRERROR_OK) {
// 		ROS_ERROR (error.GetDescription());
// 		return ;
// 	}
//
// 	if ((error = gigeCamera.SetGigEImageBinningSettings(horz, vert))
// 			!= PGRERROR_OK)
// 		ROS_ERROR (error.GetDescription());
// 	else
// 		ROS_INFO ("...success");
}

void PgrCamera::SetExposure ( bool _auto, bool onoff, unsigned int value )
{
     FlyCapture2::Property prop;
     FlyCapture2::Error error;
     prop.type = FlyCapture2::AUTO_EXPOSURE;
     prop.autoManualMode = _auto;
     prop.onOff = onoff;
     prop.valueA = value;
     if ( ( error = camPGR->SetProperty ( &prop ) ) != PGRERROR_OK ) {
          ROS_ERROR ( error.GetDescription () );
     }
}

void PgrCamera::SetGain ( bool _auto, float value )
{
     FlyCapture2::Property prop;
     FlyCapture2::Error error;
     prop.type = FlyCapture2::GAIN;
     prop.autoManualMode = _auto;
     prop.onOff = true;
     prop.absValue = value;
     if ( ( error = camPGR->SetProperty ( &prop ) ) != PGRERROR_OK ) {
          ROS_ERROR ( error.GetDescription () );
     }

     return;
}


void PgrCamera::SetShutter ( bool _auto, float value )
{
     FlyCapture2::Property prop;
     FlyCapture2::Error error;
     prop.type = FlyCapture2::AUTO_EXPOSURE;
     prop.autoManualMode = _auto;
     prop.onOff = true;
     prop.absValue = value;
     if ( ( error = camPGR->SetProperty ( &prop ) ) != PGRERROR_OK ) {
          //ROS_ERROR (error.GetDescription());
          ROS_ERROR ( "Unable to set binning" );
     }

     return;
}

void PgrCamera::SetFrameRate ( bool automatic,  float value )
{
     FlyCapture2::Property prop;
     FlyCapture2::Error error;
     prop.type = FlyCapture2::FRAME_RATE;
     prop.autoManualMode = automatic;
     //prop.onOff = true;
     if ( automatic == false) {
	  prop.valueA = value;
	  prop.valueB = value;
          prop.absValue = value;
	  prop.onOff = true;
	  prop.absControl = true;
     }


     ROS_INFO("Trying to set frame rate to %f: automatic is on: %d", value, automatic);
     if ( ( error = camPGR->SetProperty ( &prop ) ) != PGRERROR_OK ) {
          ROS_ERROR ( "Unable to set framerate:" );
          ROS_ERROR ( error.GetDescription() );
     }
}

void PgrCamera::SetGigEPacketSize ( unsigned int packetSize )
{
     ROS_INFO ( "Current Packet Size: %d",  getCurrentPacketSize() );
     SetGigESettings ( packetSize,  getCurrentPacketDelay() );
}

void PgrCamera::SetGigEPacketDelay ( unsigned int packetDelay )
{
     ROS_INFO ( "Current Packet Delay: %d",  getCurrentPacketDelay() );
     SetGigESettings ( getCurrentPacketDelay(),  packetDelay );
}


GigECamera* PgrCamera::castToGigECamera ( CameraBase* cameraBase )
{
     using namespace FlyCapture2;
     Error error;
     ROS_INFO ( "Casting to GigE" );
     GigECamera* gigeCamera = dynamic_cast<GigECamera*> ( camPGR.get() );
     if ( gigeCamera == 0 ) {
          ROS_INFO ( "camera is not a GigE camera" );
     }
     return gigeCamera;
}

GigEImageSettings PgrCamera::getCurrentGigEImageSettings()
{
     using namespace FlyCapture2;
     Error error;
     GigEImageSettings imageSettings;
     GigECamera* gigeCamera = castToGigECamera ( camPGR.get() );
     if ( !gigeCamera ) {
          ROS_ERROR ( "Impossible to get current GigE settings" );
          return imageSettings;
     }

     return imageSettings;
}

unsigned int PgrCamera::getCurrentPacketSize()
{
     GigEProperty property;
     property.propType = FlyCapture2::PACKET_SIZE;
     return getGigEProperty ( property );
}

unsigned int PgrCamera::getCurrentPacketDelay()
{
     GigEProperty property;
     property.propType = FlyCapture2::PACKET_DELAY;
     return getGigEProperty ( property );
}

unsigned int PgrCamera::getGigEProperty ( GigEProperty gigeProperty )
{
     using namespace FlyCapture2;
     Error error;

     GigECamera* gigeCamera = castToGigECamera ( camPGR.get() );
     if ( !gigeCamera ) {
          ROS_ERROR ( "Unable to get GigE property" );
          return 0;
     }

     // Set the packet size and delay to the camera
     error = gigeCamera->GetGigEProperty ( &gigeProperty );
     if ( error != PGRERROR_OK ) {
          ROS_ERROR ( "Unable to get packet size" );
          return 0;
     }

     return gigeProperty.value;
}

void PgrCamera::SetGigESettings ( unsigned int packetSize,  unsigned int packetDelay )
{
     using namespace FlyCapture2;
     Error error;

     GigECamera* gigeCamera = castToGigECamera ( camPGR.get() );
     if ( !gigeCamera ) {
          ROS_ERROR ( "Unable to set GigE settings. This is not a GigE camera" );
          return;
     }

     ROS_INFO("Current packet size : %u delay: %u ", getCurrentPacketSize(), getCurrentPacketDelay());

     error = gigeCamera->StopCapture();
     bool restartCamera;
     if ( error != PGRERROR_OK ) { //&& error != PGRERROR_NOT_STARTED )
          ROS_ERROR ( "unable to stop capture" );
          ROS_ERROR ( error.GetDescription() );
          restartCamera = false;
          return;
     } else {
          restartCamera = true;
     }
     sleep ( SLEEP_TIME );

     // Set the packet size and delay to the camera
     GigEProperty packetSizeProp;
     packetSizeProp.propType = PACKET_SIZE;
     packetSizeProp.value = packetSize;
     error = gigeCamera->SetGigEProperty ( &packetSizeProp );
     if ( error != PGRERROR_OK ) {
          ROS_ERROR ( "Unable to set packet size" );
          PRINT_ERROR ( error );
          return;
     }

     sleep ( SLEEP_TIME );

     GigEProperty packetDelayProp;
     packetDelayProp.propType = PACKET_DELAY;
     packetDelayProp.value = packetDelay;
     error = gigeCamera->SetGigEProperty ( &packetDelayProp );
     if ( error != PGRERROR_OK ) {
          ROS_ERROR ( "Unable to set packet delay" );
          PRINT_ERROR ( error );
          return;
     }

     sleep ( SLEEP_TIME );

     if ( restartCamera == true ) {
	start();
     }

     ROS_INFO ( "GigE settings set" );
     sleep ( SLEEP_TIME );
}

PropertyInfo PgrCamera::getPropertyInfo ( FlyCapture2::PropertyType type )
{
     FlyCapture2::Error error;
     FlyCapture2::PropertyInfo info;
     info.type = type;
     error = camPGR->GetPropertyInfo (&info);
     if ( error != PGRERROR_OK ) {
          ROS_ERROR ( error.GetDescription() );
     }

     return info;
}

       float PgrCamera::GetFrameRate()
{
     FlyCapture2::Property frmRate;
     FlyCapture2::Error error;
     frmRate.type = FlyCapture2::FRAME_RATE;
     error = camPGR->GetProperty ( &frmRate );
     if ( error != PGRERROR_OK ) {
          ROS_ERROR ( error.GetDescription() );
          return -1;
     }

     return frmRate.absValue;
}

void PgrCamera::PrintCameraInfo ( FlyCapture2::CameraInfo* pCamInfo )
{
     char macAddress[64];
     sprintf (
          macAddress,
          "%02X:%02X:%02X:%02X:%02X:%02X",
          pCamInfo->macAddress.octets[0],
          pCamInfo->macAddress.octets[1],
          pCamInfo->macAddress.octets[2],
          pCamInfo->macAddress.octets[3],
          pCamInfo->macAddress.octets[4],
          pCamInfo->macAddress.octets[5] );

     char ipAddress[32];
     sprintf (
          ipAddress,
          "%u.%u.%u.%u",
          pCamInfo->ipAddress.octets[0],
          pCamInfo->ipAddress.octets[1],
          pCamInfo->ipAddress.octets[2],
          pCamInfo->ipAddress.octets[3] );

     char subnetMask[32];
     sprintf (
          subnetMask,
          "%u.%u.%u.%u",
          pCamInfo->subnetMask.octets[0],
          pCamInfo->subnetMask.octets[1],
          pCamInfo->subnetMask.octets[2],
          pCamInfo->subnetMask.octets[3] );

     char defaultGateway[32];
     sprintf (
          defaultGateway,
          "%u.%u.%u.%u",
          pCamInfo->defaultGateway.octets[0],
          pCamInfo->defaultGateway.octets[1],
          pCamInfo->defaultGateway.octets[2],
          pCamInfo->defaultGateway.octets[3] );

     printf (
          "\n*** CAMERA INFORMATION ***\n"
          "Serial number - %u\n"
          "Camera model - %s\n"
          "Camera vendor - %s\n"
          "Sensor - %s\n"
          "Resolution - %s\n"
          "Firmware version - %s\n"
          "Firmware build time - %s\n"
          "GigE version - %u.%u\n"
          "User defined name - %s\n"
          "XML URL 1 - %s\n"
          "XML URL 2 - %s\n"
          "MAC address - %s\n"
          "IP address - %s\n"
          "Subnet mask - %s\n"
          "Default gateway - %s\n\n",
          pCamInfo->serialNumber,
          pCamInfo->modelName,
          pCamInfo->vendorName,
          pCamInfo->sensorInfo,
          pCamInfo->sensorResolution,
          pCamInfo->firmwareVersion,
          pCamInfo->firmwareBuildTime,
          pCamInfo->gigEMajorVersion,
          pCamInfo->gigEMinorVersion,
          pCamInfo->userDefinedName,
          pCamInfo->xmlURL1,
          pCamInfo->xmlURL2,
          macAddress,
          ipAddress,
          subnetMask,
          defaultGateway );
}
