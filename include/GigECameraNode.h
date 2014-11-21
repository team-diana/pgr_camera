#ifndef GIG_E_CAMERA_NODE_H
#define GIG_E_CAMERA_NODE_H

#include "CameraNode.h"
#include "libflycapcam/FlycapCameraGigE.h"
#include "pgr_camera/PgrGigECameraConfig.h"

#include <ros/node_handle.h>

typedef dynamic_reconfigure::Server < pgr_camera::PgrGigECameraConfig > DynamicGigEReconfigureServer;

class GigECameraNode : public CameraNode {

public:
  GigECameraNode(const ros::NodeHandle& nodeHandle, std::unique_ptr<flycapcam::FlycapCameraGigE>&& pgrCamera);
  flycapcam::FlycapCamera* getFlycapCamera() const override;
  void printGigEInfo();
  void printCameraStats();

private:
  ros::NodeHandle gigeNodeHandle;
  std::unique_ptr<flycapcam::FlycapCameraGigE> flycapCamera;
  boost::recursive_mutex camGigEReconfigureMutex;
  DynamicGigEReconfigureServer camGigEReconfigureServer;
  virtual void initImpl();
  void configureGigE(pgr_camera::PgrGigECameraConfig &config, uint32_t level) ;
  void updateDynamicReconfigureServerGigECamera();

};

#endif
