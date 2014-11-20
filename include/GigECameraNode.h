#ifndef GIG_E_CAMERA_NODE_H
#define GIG_E_CAMERA_NODE_H

#include "CameraNode.h"
#include "libflycapcam/FlycapCameraGigE.h"

#include <ros/node_handle.h>

class GigECameraNode : public CameraNode {

public:
  GigECameraNode(const ros::NodeHandle& nodeHandle, std::unique_ptr<flycapcam::FlycapCameraGigE>&& pgrCamera);
  virtual void configure(pgr_camera::PgrCameraConfig &config, uint32_t level) ;
  flycapcam::FlycapCamera* getFlycapCamera() const override;

private:
  void gigeConfigure(pgr_camera::PgrCameraConfig &config, uint32_t level) ;
  std::unique_ptr<flycapcam::FlycapCameraGigE> flycapCamera;
  DynamicReconfigureServer dynamicReconfigureServerGigECamera;
};

#endif
