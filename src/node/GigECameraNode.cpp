#include "GigECameraNode.h"

#include "stdlib.h"

GigECameraNode::GigECameraNode(const ros::NodeHandle& nodeHandle, std::unique_ptr<flycapcam::FlycapCameraGigE>&& flycapCamera) :
 CameraNode(nodeHandle)
{
  this->flycapCamera = std::move(flycapCamera);
}


void GigECameraNode::configure ( pgr_camera::PgrCameraConfig &config, uint32_t level )  {

}

void GigECameraNode::gigeConfigure(pgr_camera::PgrCameraConfig &config, uint32_t level ) {
}

flycapcam::FlycapCamera* GigECameraNode::getFlycapCamera() const
{
  return flycapCamera.get();
}
