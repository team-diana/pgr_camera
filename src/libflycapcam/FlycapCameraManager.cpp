#include "libflycapcam/FlycapCameraManager.h"

using namespace FlyCapture2;
using namespace std;
using namespace flycapcam;

FlycapCameraManager::FlycapCameraManager()
{
  printNumOfAvaliableCameras();
}

int FlycapCameraManager::getNumOfAvailableCameras()
{
  Error error;
  BusManager busMgr;
  unsigned int numOfCameras = 0;

  if((error = busMgr.GetNumOfCameras(&numOfCameras)) != PGRERROR_OK) {
    std::cerr << "Unable to get number of cameras: " << error.GetDescription() << std::endl;
  }

  return numOfCameras;
}


unique_ptr< FlycapCameraGigE > FlycapCameraManager::createGigECameraFromGuid(PGRGuid guid, SerialNumber serialNumber)
{
  Error error;
  unique_ptr<GigECamera>  gigeCamera(new GigECamera());

  if((error = gigeCamera->Connect(&guid)) != PGRERROR_OK) {
    std::cerr << Td::toString("Unable to connect to GigE camera with guid %s", guidToString(guid));
    return nullptr;
  } else {
    return unique_ptr<FlycapCameraGigE> (
             new FlycapCameraGigE(move(gigeCamera), guid, serialNumber, INTERFACE_GIGE)
           );
  }
}

unique_ptr<FlycapCameraGigE> FlycapCameraManager::createGigECamera(SerialNumber serialNumber)
{ Error error;
  BusManager busMgr;
  PGRGuid cameraGuid;

  if((error = busMgr.GetCameraFromSerialNumber(serialNumber,  &cameraGuid)) != PGRERROR_OK) {
    std::cerr << "Unable to retrieve camera with serial number " <<  serialNumber;
    return nullptr;
  } else {
    return createGigECameraFromGuid(cameraGuid, serialNumber);
  }
}

void FlycapCameraManager::printNumOfAvaliableCameras()
{
  int numOfCameras = getNumOfAvailableCameras();
  std::cout << Td::toString("Found ",  numOfCameras, " cameras");
}
