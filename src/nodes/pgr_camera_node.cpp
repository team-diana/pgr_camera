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

// ROS communication
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/fill_image.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <std_msgs/String.h>
#include <pgr_camera/pgr_camera.h>

// Dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <driver_base/SensorLevels.h>
#include "pgr_camera/PGRCameraConfig.h"

// Standard libs
#include <boost/scoped_ptr.hpp>
#include <boost/bind.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/format.hpp>
#include <sstream>
#include <fstream>
#include <sys/stat.h>

class PGRCameraNode
{
private:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::CameraPublisher streaming_pub_;
  ros::ServiceServer set_camera_info_srv_;
  dynamic_reconfigure::Server < pgr_camera::PGRCameraConfig > reconfigure_srv_;
  camera_info_manager::CameraInfoManager cinfo_;

  string frame_id;
  string camera_info_url_;
  // Camera
  std::scoped_ptr < pgr_camera::Camera > cam_;
  bool running;

public:
    PGRCameraNode (const ros::NodeHandle & node_handle):nh_ (node_handle), it_ (nh_), cinfo_(node_handle), cam_ (NULL), running (false)
  {
    // Two-stage initialization: in the constructor we open the requested camera. Most
    // parameters controlling capture are set and streaming started in configure(), the
    // callback to dynamic_reconfig.
    pgr_camera::printCameras();

    ros::NodeHandle local_nh("~");

    // TODO: add facility to set intrinsics

    int serNo;
    if(local_nh.getParam("serial_number", serNo)) {
        cam_.reset (new pgr_camera::Camera (serNo));
    } else {
        cam_.reset (new pgr_camera::Camera ());
    }
    cam_->initCam ();
    cinfo_.setCameraName(cam_->getName());
    
    
    reconfigure_srv_.setCallback (boost::bind (&PGRCameraNode::configure, this, _1, _2));
  }

  void configure (pgr_camera::PGRCameraConfig & config, uint32_t level)
  {
    ROS_INFO ("Reconfigure request received");

    if (level >= (uint32_t) driver_base::SensorLevels::RECONFIGURE_STOP)
      stop ();
    
    if (camera_info_url_ != config.camera_info_url)
    {
      // set the new URL and load CameraInfo (if any) from it
      if (cinfo_.validateURL(config.camera_info_url))
        {
          cinfo_.loadCameraInfo(config.camera_info_url);
        }
      else
        {
          // new URL not valid, use the old one
          config.camera_info_url = camera_info_url_;
        }
    }
    
    /* 
     * If Shutter and Gain are both set to auto, then auto exposure
     * will tune each one to maintain a constant exposure at each pixel. 
     *
     * If only one of Shutter/Gain is set to auto, then that value
     * will be tuned to enforce the constant exposure value at each
     * pixel, while the other value is not changed.
     *
     * If both Shutter and Gain are set to manual, then auto exposure
     * has no effect.
     *
     */

    // Exposure
    if (config.auto_exposure)
      cam_->SetExposure (true, true);
    else    
      cam_->SetExposure (false, true);


    // Shutter
    if (config.auto_shutter)
      cam_->SetShutter (true);
    else
      cam_->SetShutter (false, (float)config.shutter);
    
    // Gain
    if(config.auto_gain)
      cam_->SetGain(true);
    else
      cam_->SetGain(false, (float)config.gain);

    // video mode / framerate
    cam_->SetVideoModeAndFramerate (config.width, config.height, config.format, config.frame_rate);

    
    // TF frame
    frame_id = config.frame_id;

    if (level >= (uint32_t) driver_base::SensorLevels::RECONFIGURE_STOP)
      start ();
  }

  ~PGRCameraNode ()
  {
    stop ();
  }

  void start ()
  {
    if (running)
      return;

    cam_->setFrameCallback (boost::bind (&PGRCameraNode::publishImage, this, _1));
    streaming_pub_ = it_.advertiseCamera ("image_raw", 1);
    cam_->start ();
    running = true;
  }

  void stop ()
  {
    if (!running)
      return;

    cam_->stop ();              // Must stop camera before streaming_pub_.
    streaming_pub_.shutdown ();

    running = false;
  }

  void publishImage (FlyCapture2::Image * frame)
  {
    sensor_msgs::Image img;
    sensor_msgs::CameraInfo cam_info(cinfo_.getCameraInfo());
    
    /// @todo Use time from frame?
    img.header.stamp = cam_info.header.stamp = ros::Time::now ();
    img.header.frame_id = cam_info.header.frame_id = frame_id;

    //unsigned int bpp = frame->GetBitsPerPixel();
    
    std::string encoding;
    switch(frame->GetBayerTileFormat ()) {
      case FlyCapture2::NONE: encoding = sensor_msgs::image_encodings::MONO8      ; break;
      case FlyCapture2::RGGB: encoding = sensor_msgs::image_encodings::BAYER_RGGB8; break;
      case FlyCapture2::GRBG: encoding = sensor_msgs::image_encodings::BAYER_GRBG8; break;
      case FlyCapture2::GBRG: encoding = sensor_msgs::image_encodings::BAYER_GBRG8; break;
      case FlyCapture2::BGGR: encoding = sensor_msgs::image_encodings::BAYER_BGGR8; break;
      default: ROS_ERROR("unknown BayerTileFormat"); return;
    }

    if(!sensor_msgs::fillImage (img, encoding, frame->GetRows (),
          frame->GetCols (), frame->GetStride (), frame->GetData ())) {
      ROS_ERROR("fillImage failed");
      return;
    }
    cam_info.height = img.height;
    cam_info.width = img.width;
    //              cam_info.roi.x_offset = frame->RegionX;
    //              cam_info.roi.y_offset = frame->RegionY;
    //              cam_info.roi.height = frame->Height;
    //              cam_info.roi.width = frame->Width;

    streaming_pub_.publish (img, cam_info);
  }

};

int
main (int argc, char **argv)
{
  ros::init (argc, argv, "pgr_camera");

  try
  {
    ros::NodeHandle nh ("camera");
    PGRCameraNode pn(nh);

    ros::spin ();

  } catch (std::runtime_error & e)
  {
    ROS_FATAL ("Uncaught exception: '%s', aborting.", e.what ());
    ROS_BREAK ();
  }

  return 0;

}
