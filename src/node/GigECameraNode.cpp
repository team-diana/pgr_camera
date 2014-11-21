#include "GigECameraNode.h"

#include "stdlib.h"

#include <team_diana_lib/logging/logging.h>
#include <team_diana_lib/strings/strings.h>

using namespace Td;
using namespace std;
using namespace FlyCapture2;
using namespace flycapcam;

GigECameraNode::GigECameraNode(const ros::NodeHandle& nodeHandle, std::unique_ptr<flycapcam::FlycapCameraGigE>&& flycapCamera) :
 CameraNode(nodeHandle),
 gigeNodeHandle(toString("GigECamera", flycapCamera->getSerialNumber())),
 dynamicReconfigureServerGigECamera(dynamicReconfigureGigEMutex, gigeNodeHandle)
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

  updateDynamicReconfigureServerGigECamera();
}

void GigECameraNode::updateDynamicReconfigureServerGigECamera()
{
  pgr_camera::PgrGigECameraConfig config;
  FlycapResult result;

  unsigned int packet_delay;
  if(result = flycapCamera->getPacketDelay(packet_delay)) {
    config.packet_delay = static_cast<unsigned int>(packet_delay);
  } else {
    ros_error(result.getErrorDescription());
  }

  unsigned int packet_size;
  if(result = flycapCamera->getPacketSize(packet_size)) {
    config.packet_size = static_cast<unsigned int>(packet_size);
  } else {
    ros_error(result.getErrorDescription());
  }

  std::lock_guard<boost::recursive_mutex> lock(dynamicReconfigureGigEMutex);
  dynamicReconfigureServerGigECamera.updateConfig(config);
}

