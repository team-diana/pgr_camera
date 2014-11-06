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
#include <PgrCamera.h>
#include <pgr_camera/boolean.h>
#include <pgr_camera/published.h>
#include <pgr_camera/oneshot.h>
#include <camera_info_manager/camera_info_manager.h>

// Dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <driver_base/SensorLevels.h>
#include "pgr_camera/PgrCameraConfig.h"
#include "PgrCameraFactory.h"

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

typedef dynamic_reconfigure::Server < pgr_camera::PgrCameraConfig > DynamicReconfigureServer;

class PgrCameraNode
{

    struct DiagnosticsData {

        DiagnosticsData() :
          frameCount(0),
          oneshotPublished(false),
          publishedCount(0),
          oneshotCount(0),
          oneshotCountMax(0)
        {}

        // Diagnostics
        int frameCount;
        int oneshotPublished;
        int publishedCount;
        int oneshotCount;
        int oneshotCountMax;
    };


    enum State {
        SETTING_UP,
        RUNNING,
        STOP,
        ONE_SHOT
    };


public:
    PgrCameraNode ( const ros::NodeHandle &nodeHandle, shared_ptr<pgr_camera::PgrCamera> pgrCamera );
    ~PgrCameraNode ();

    bool isSetupDone();

    DynamicReconfigureServer& getDynamicReconfigureServer();

    void publishImage ( FlyCapture2::Image *frame, int camIndex );
    void publishImageWithTimestamp ( FlyCapture2::Image *frame, int camIndex,  ros::Time timestamp );
    void overrideFrameCallback ( std::function <void ( FlyCapture2::Image *, unsigned int ) > callback );
    bool enableStreamCallback ( pgr_camera::booleanRequest& request, pgr_camera::booleanResponse& response );
    bool enableOneShot ( pgr_camera::oneshotRequest& request, pgr_camera::oneshotResponse& response );

    virtual void configure ( pgr_camera::PgrCameraConfig &config, uint32_t level ) ;

    unsigned int getCameraIndex() {
        return pgrCamera->getCamIndex();
    }

    void setup();
    void start ();
    void stop ();

    void enablePublishing ( bool enable ) {
        pgrCamera->enableCallback ( enable );
    }

    void enableOneShot ( bool enable, int oneshotCountMax ) {
        diagnosticsData.oneshotPublished = false;
        diagnosticsData.oneshotCount = 0;
        diagnosticsData.oneshotCountMax = oneshotCountMax;
        pgrCamera->enableCallback ( enable );
        state = ONE_SHOT;
    }

protected:
    bool frameToImage ( FlyCapture2::Image *frame, sensor_msgs::Image &image );
    bool processFrame ( FlyCapture2::Image *frame, sensor_msgs::Image &img, sensor_msgs::CameraInfo &cam_info,  ros::Time timestamp );
    void loadIntrinsics ( string inifile, unsigned int cameraSerialNumber );
    void setupConfigure();
    void baseSetupConfigure ( pgr_camera::PgrCameraConfig& min, pgr_camera::PgrCameraConfig& max );
    void baseConfigure ( pgr_camera::PgrCameraConfig& config,  uint32_t level );
    PropertyInfo getPropertyInfo ( PropertyType propertyType );

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

    pgr_camera::PgrCameraConfig currentConfig;

    std::shared_ptr<pgr_camera::PgrCamera> pgrCamera;
    sensor_msgs::Image sensorImage;
    sensor_msgs::CameraInfo cameraInfo;
    DiagnosticsData diagnosticsData;
    static std::mutex globalPublishMutex;
    State state;
};
#endif                                                      // PGRCAMERANODE_H
