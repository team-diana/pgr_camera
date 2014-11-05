#ifndef PGRGIGECAMERANODE_H
# define PGRGIGECAMERANODE_H

#include "pgr_camera/PgrCamera.h"
# include "PgrCameraNode.h"


class PgrGigECameraNode : public PgrCameraNode {

public:
     PgrGigECameraNode ( const ros::NodeHandle& nodeHandle, shared_ptr< PgrCamera > pgrCamera ) : PgrCameraNode(nodeHandle,  pgrCamera) {
       
    }
     virtual void configure ( pgr_camera::PGRCameraConfig &config, uint32_t level ) ;

private:
     void gigeConfigure(pgr_camera::PGRCameraConfig &config, uint32_t level ) ;
};

#endif                                                      // PGRGIGECAMERANODE_H
