#ifndef PGRCAMERANODE_H
# define PGRCAMERANODE_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/fill_image.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <image_transport/image_transport.h>
#include <camera_calibration_parsers/parse_ini.h>
#include <std_msgs/String.h>
#include <polled_camera/publication_server.h>
#include <pgr_camera/PgrCamera.h>
#include <pgr_camera/boolean.h>
#include <pgr_camera/published.h>
#include <pgr_camera/oneshot.h>
#include <camera_info_manager/camera_info_manager.h>

// Dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <driver_base/SensorLevels.h>
#include "pgr_camera/PGRCameraConfig.h"
#include "pgr_camera/PgrCameraFactory.h"

#include <XmlRpcValue.h>

// Standard libs
#include <boost/scoped_ptr.hpp>
#include <boost/bind.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/format.hpp>
#include <vector>
#include <sstream>
#include <fstream>
#include <sys/stat.h>
#include <cstdlib>
#include <unistd.h>

typedef dynamic_reconfigure::Server < pgr_camera::PGRCameraConfig > DynamicReconfigureServer;

class PgrCameraNode {

public:
     PgrCameraNode ( const ros::NodeHandle &nodeHandle, shared_ptr<PgrCamera> pgrCamera );
     ~PgrCameraNode ();

     bool isSetupDone();

     DynamicReconfigureServer& getDynamicReconfigureServer();

     void publishImage ( FlyCapture2::Image *frame, int camIndex );
     void publishImageWithTimestamp ( FlyCapture2::Image *frame, int camIndex,  ros::Time timestamp );
     void overrideFrameCallback ( boost::function < void ( FlyCapture2::Image *, unsigned int )  > callback );
     bool enableStreamCallback( pgr_camera::booleanRequest& request, pgr_camera::booleanResponse& response );
     bool enableOneShot( pgr_camera::oneshotRequest& request, pgr_camera::oneshotResponse& response );

     virtual void configure ( pgr_camera::PGRCameraConfig &config, uint32_t level ) ;

     unsigned int getCameraIndex() {
          return pgrCamera->getCamIndex();
     }

     void setup();
     void start ();
     void stop ();

     void enablePublishing(bool enable) {
       pgrCamera->enableCallback(enable);
    }

     void enableOneShot(bool enable, int oneshotCountMax) {
       oneshotPublished = false;
       this->oneshotCount = 0;
       this->oneshotCountMax = oneshotCountMax;
       pgrCamera->enableCallback(enable);
       oneshotEnabled = enable;
     }

protected:
     static bool frameToImage ( FlyCapture2::Image *frame, sensor_msgs::Image &image );
     bool processFrame ( FlyCapture2::Image *frame, sensor_msgs::Image &img, sensor_msgs::CameraInfo &cam_info,  ros::Time timestamp );
     void loadIntrinsics ( string inifile, unsigned int cameraSerialNumber );
     void setupConfigure();
     void baseSetupConfigure(pgr_camera::PGRCameraConfig& min, pgr_camera::PGRCameraConfig& max);
     void baseConfigure(pgr_camera::PGRCameraConfig& config,  uint32_t level);
     PropertyInfo getPropertyInfo(PropertyType propertyType);

protected:
     ros::NodeHandle nodeHandler;
     ros::Publisher cameraDidPublishPublisher;
     image_transport::ImageTransport imageTransport;
     image_transport::CameraPublisher cameraPublisher;
     polled_camera::PublicationServer publicationServer;
     ros::ServiceServer streamEnabledService;
     ros::ServiceServer oneShotService;
     camera_info_manager::CameraInfoManager cameraInfoManager;
     DynamicReconfigureServer dynamicReconfigureServer;
     pgr_camera::PGRCameraConfig currentConfig;
     bool oneshotEnabled;

     // Camera
     boost::shared_ptr<PgrCamera > pgrCamera;
     bool setupDone;
     bool running;

     // ROS messages
     sensor_msgs::Image sensorImage;
     sensor_msgs::CameraInfo cameraInfo;

     // Diagnostics
     int frameCount;
     int oneshotPublished;
     int publishedCount;
     int oneshotCount;
     int oneshotCountMax;
};


#endif                                                      // PGRCAMERANODE_H
