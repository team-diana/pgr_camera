#include "CameraSynchronizer.h"


// TODO: change camerasToSync to reference if possible.
CameraSynchronizer::CameraSynchronizer ( std::vector< boost::shared_ptr<PgrCameraNode> > camerasToSync )
{
     for ( std::vector<boost::shared_ptr<PgrCameraNode> >::iterator i = camerasToSync.begin(); i != camerasToSync.end(); i++ ) {
          shared_ptr<PgrCameraNode> node = *i;
          node->overrideFrameCallback ( boost::bind ( &publishSyncImage, this, _1, _2 ) );
          cameraNodesMap.insert ( std::pair<unsigned int, shared_ptr<PgrCameraNode> > ( node->getCameraIndex(), node ) );
          cameraDidPublish.push_back ( false );
     }
}

CameraSynchronizer::~CameraSynchronizer()
{

}


void CameraSynchronizer::publishSyncImage ( FlyCapture2::Image *frame, int camIndex )
{
     ros::Time timestamp;


     masterTimestampMutex.lock();
     if ( cameraDidPublish[camIndex] || (camIndex != 0 && !cameraDidPublish[0])) {
	  masterTimestampMutex.unlock();
          return;
     }
     if ( camIndex == 0 ) {
          timestamp = masterTimestamp = ros::Time::now();
     } else {
          timestamp = masterTimestamp;
     }
     masterTimestampMutex.unlock();

     shared_ptr<PgrCameraNode> node = cameraNodesMap.find ( camIndex )->second;
     node->publishImageWithTimestamp ( frame,  camIndex,  timestamp );

     masterTimestampMutex.lock();
     cameraDidPublish[camIndex] = true;
     if ( allDidPublish() ) {
          cameraDidPublish.flip();
     }
     masterTimestampMutex.unlock();
}

