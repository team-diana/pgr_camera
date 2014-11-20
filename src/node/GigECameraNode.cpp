#include "GigECameraNode.h"

#include "stdlib.h"

GigECameraNode::GigECameraNode(const ros::NodeHandle& nodeHandle, std::unique_ptr<flycapcam::FlycapCameraGigE>&& flycapCamera) :
 CameraNode(nodeHandle),
 dynamicReconfigureServerGigECamera(nodeHandle)
{
  this->flycapCamera = std::move(flycapCamera);
}


void GigECameraNode::configureGigE(pgr_camera::PgrGigECameraConfig& config, uint32_t level)
{
  flycapCamera->setPacketDelay(config.packet_delay);
  flycapCamera->setPacketSize(config.packet_size);
}

flycapcam::FlycapCamera* GigECameraNode::getFlycapCamera() const
{
  return flycapCamera.get();
}

void GigECameraNode::initImpl()
{
  DynamicGigEReconfigureServer::CallbackType f = boost::bind(&GigECameraNode::configureGigE, this, _1, _2);
  dynamicReconfigureServerGigECamera.setCallback(f);
}
