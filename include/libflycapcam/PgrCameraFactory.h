#ifndef PGRCAMERAFACTORY_H
#define PGRCAMERAFACTORY_H

#include "PgrCamera.h"

using namespace FlyCapture2;
using namespace std;


class PgrCameraFactory
{
public:
    PgrCameraFactory ();

    int getNumOfAvailableCameras();

    shared_ptr<pgr_camera::PgrCamera> getCameraFromSerialNumber ( unsigned int serialNumber );
    shared_ptr<pgr_camera::PgrCamera> createGigECamera ( PGRGuid cameraGuid, unsigned int serialNumber );

    virtual ~PgrCameraFactory() {}

    static std::string guidToString ( PGRGuid guid );

private:
    void printNumOfAvaliableCameras();
};

#endif                                                      // PGRCAMERAFACTORY_H
