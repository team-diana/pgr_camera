#ifndef CAMERASYNCHRONIZER
#define CAMERASYNCHRONIZER
#include <vector>

# include <boost/function.hpp>

# include "PgrCameraNode.h"

using namespace FlyCapture2;

class CameraSynchronizer {
public:
     CameraSynchronizer ( vector< std::shared_ptr< PgrCameraNode > > camerasToSync );
     virtual ~CameraSynchronizer();

private:
     std::mutex masterTimestampMutex;
     ros::Time masterTimestamp;
     void publishSyncImage ( Image* frame, int camIndex );
     std::map<unsigned int,  std::shared_ptr<PgrCameraNode> > cameraNodesMap;
     std::vector<bool> cameraDidPublish;

     bool allDidPublish() {
          for ( std::vector<bool>::iterator it = cameraDidPublish.begin(); it != cameraDidPublish.end(); it++ ) {
             if (!*it) {
               return false;
            }
          }
          return true;
     }

};


#endif // CAMERASYNCHRONIZER