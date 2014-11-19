#ifndef FLYCAP_CAMERA_MANAGER_H
#define FLYCAP_CAMERA_MANAGER_H

#include "FlycapCameraGigE.h"

#include "libflycapcam/FlycapTypes.h"


class FlycapCameraManager
{
public:
  FlycapCameraManager();
  virtual ~FlycapCameraManager() {}

  int getNumOfAvailableCameras();
  std::unique_ptr<flycapcam::FlycapCameraGigE> createGigECamera(flycapcam::SerialNumber serialNumber);

private:
  void printNumOfAvaliableCameras();
  std::unique_ptr<flycapcam::FlycapCameraGigE> createGigECameraFromGuid(FlyCapture2::PGRGuid guid, flycapcam::SerialNumber serialNumber);

};

#endif                                                      // PGRCAMERAFACTORY_H
