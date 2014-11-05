#include "pgr_camera/PgrCameraFactory.h"

using namespace FlyCapture2;

PgrCameraFactory::PgrCameraFactory()
{
     printNumOfAvaliableCameras();
}

int PgrCameraFactory::getNumOfAvailableCameras()
{
     Error error;
     BusManager busMgr;
     unsigned int numOfCameras = 0;
     if ( ( error = busMgr.GetNumOfCameras ( &numOfCameras ) ) != PGRERROR_OK ) {
          PRINT_ERROR ( error );
          ROS_ERROR ( "Unable to retrieve number of available cameras from bus manager" );
     }
     return numOfCameras;
}


shared_ptr<PgrCamera> PgrCameraFactory::getCameraFromSerialNumber ( unsigned int serialNumber )
{
     Error error;
     BusManager busMgr;
     PGRGuid cameraGuid;
     shared_ptr<PgrCamera> pgrCamera;

     if ( ( error = busMgr.GetCameraFromSerialNumber ( serialNumber,  &cameraGuid ) ) != PGRERROR_OK ) {
          PRINT_ERROR ( error );
          ROS_ERROR ( "Unable to retrieve camera with serial number %ud",  serialNumber );
          return pgrCamera;
     }

     InterfaceType interfaceType;

     if ( ( error = busMgr.GetInterfaceTypeFromGuid ( &cameraGuid,  &interfaceType ) ) != PGRERROR_OK ) {
          PRINT_ERROR ( error );
          ROS_ERROR ( "Unable to retrieve interface type. Reverting to generic camera type",  serialNumber );
          interfaceType = FlyCapture2::INTERFACE_UNKNOWN;
     }

     switch ( interfaceType ) {
     case FlyCapture2::INTERFACE_GIGE:
          pgrCamera = createGigECamera(cameraGuid,  serialNumber);
          break;
     default:
          ROS_ERROR ( "Camera type not supported. To be implemented" );
          break;
     }

     return pgrCamera;
}


shared_ptr<PgrCamera> PgrCameraFactory::createGigECamera ( PGRGuid cameraGuid,  unsigned int serialNumber)
{
     Error error;
     shared_ptr<PgrCamera> pgrCamera;
     boost::shared_ptr<GigECamera>  gigeCamera ( new GigECamera() );
     if ( ( error = gigeCamera->Connect ( &cameraGuid ) ) != PGRERROR_OK ) {
          PRINT_ERROR ( error );
          ROS_ERROR ( "Unable to connect to GigE camera with guid %s", guidToString ( cameraGuid ).c_str() );
          return pgrCamera;
     }
     boost::shared_ptr<CameraBase> baseCamera = static_pointer_cast<CameraBase>(gigeCamera);

     pgrCamera = shared_ptr<PgrCamera> ( new PgrCamera (baseCamera,  cameraGuid, serialNumber,  FlyCapture2::INTERFACE_GIGE) );
     return pgrCamera;
}

std::string PgrCameraFactory::guidToString ( PGRGuid guid )
{
     return (boost::format ( "%1%:%2%:%3%:%4%" ) % guid.value[0] % guid.value[1] % guid.value[2] % guid.value[3]).str();
}

void PgrCameraFactory::printNumOfAvaliableCameras()
{
     int numOfCameras = getNumOfAvailableCameras();
     ROS_INFO ( "Found %d camera",  numOfCameras );
}
