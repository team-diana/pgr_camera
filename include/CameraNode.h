#ifndef CAMERA_NODE_H
#define CAMERA_NODE_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/fill_image.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <image_transport/image_transport.h>
#include <camera_calibration_parsers/parse_ini.h>
#include <std_msgs/String.h>
#include <polled_camera/publication_server.h>
#include <pgr_camera/boolean.h>
#include <pgr_camera/published.h>
#include <pgr_camera/oneshot.h>
#include <camera_info_manager/camera_info_manager.h>

#include <dynamic_reconfigure/server.h>
#include <driver_base/SensorLevels.h>
#include "pgr_camera/PgrCameraConfig.h"
#include "libflycapcam/FlycapCameraGigE.h"

#include <XmlRpcValue.h>

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
#include <boost/date_time.hpp>

typedef dynamic_reconfigure::Server < pgr_camera::PgrCameraConfig > DynamicReconfigureServer;

class CameraNode
{

public:
  CameraNode(const ros::NodeHandle &nodeHandle);
  virtual ~CameraNode() = 0;

  void retrieveAndPublishFrame(ros::Time timestamp);
  void publishImage(FlyCapture2::Image& frame, ros::Time timestamp);
  void configure(pgr_camera::PgrCameraConfig &config, uint32_t level);

  void init();
  void start();
  void stop();

  void setStartAndStopEnabled(bool enabled);

private:
  void baseInit();
  void updateReconfigureServer();

protected:
  void retrieveAndPublishFrameImpl(ros::Time timestamp);
  void retrieveAndPublishFrameStartAndStop(ros::Time timestamp);
  bool frameToImage(FlyCapture2::Image *frame, sensor_msgs::Image &image);
  bool processFrame(FlyCapture2::Image *frame, sensor_msgs::Image &img, sensor_msgs::CameraInfo &cam_info,  ros::Time timestamp);
  void loadIntrinsics(std::string inifile, unsigned int cameraSerialNumber);
  void dynamicReconfigureCameraCallback(pgr_camera::PgrCameraConfig& config,  uint32_t level);
  void printResultErrorMessageIfAny(const flycapcam::FlycapResult& result, std::string msg = "");
  virtual flycapcam::FlycapCamera* getFlycapCamera() const = 0;
  virtual void initImpl() {};

protected:
  ros::NodeHandle nodeHandler;
  image_transport::ImageTransport imageTransport;
  image_transport::CameraPublisher cameraPublisher;
  polled_camera::PublicationServer publicationServer;
  camera_info_manager::CameraInfoManager cameraInfoManager;
  boost::recursive_mutex camReconfigureMutex;
  DynamicReconfigureServer camReconfigureServer;
  sensor_msgs::Image sensorImage;
  sensor_msgs::CameraInfo cameraInfo;
  bool startAndStopEnabled;
};

#endif
