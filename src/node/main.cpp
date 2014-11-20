#include "pgr_camera/PgrCameraConfig.h"
#include "libflycapcam/FlycapCameraManager.h"
#include "GigECameraNode.h"

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/fill_image.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <image_transport/image_transport.h>
#include <camera_calibration_parsers/parse_ini.h>
#include <std_msgs/String.h>
#include <polled_camera/publication_server.h>
#include <camera_info_manager/camera_info_manager.h>
#include <team_diana_lib/logging/logging.h>
#include <team_diana_lib/strings/strings.h>

#include <dynamic_reconfigure/server.h>
#include <driver_base/SensorLevels.h>
#include <XmlRpcValue.h>

#include <mutex>
#include <vector>
#include <sstream>
#include <fstream>
#include <sys/stat.h>
#include <cstdlib>
#include <thread>
#include <unistd.h>
#include <boost/program_options.hpp>
#include <unistd.h>

using namespace Td;
using namespace std;

std::shared_ptr<CameraNode> createCameraNode(ros::NodeHandle handle, std::unique_ptr<flycapcam::FlycapCameraGigE>&& camera)
{
  return std::shared_ptr<CameraNode>(new GigECameraNode(handle, std::move(camera)));
}

bool parseCommandLine(int argc, char** argv, std::vector<unsigned int>& serialsToStart,
                      std::vector<unsigned int>& serialsToSync)
{
  using namespace boost::program_options;
  try {
    /** Define and parse the program options
     */
    options_description desc("Options");
    desc.add_options()
    ("help, h", "Print help messages")
    ("serials, s", value<std::vector<unsigned int> >()->multitoken(), "list of serials of the cameras to start")
    ("sync-serials, S", value<std::vector<unsigned int> >()->multitoken(), "list of serials of the cameras to sync");

    variables_map varsMap;
    try {
      store(parse_command_line(argc, argv, desc), varsMap);

      if(varsMap.count("help")) {
        ros_error("Usage: \n" \
                  "--serials {list of serials of the cameras to start} \n"\
                  "--sync-serials {list of serials of the cameras to start in sync}");
        return false;
      }

      if(!varsMap["serials"].empty()) {
        std::vector<unsigned int> serialsToStartArgs;
        serialsToStartArgs = varsMap["serials"].as<vector<unsigned int> >();
        serialsToStart.insert(serialsToStart.end(),  serialsToStartArgs.begin(),  serialsToStartArgs.end());
      }
      else  {
        ros_error("No serials specified. Use help to see usage");
        return false;
      }

      if(!varsMap["sync-serials"].empty()) {
        std::vector<unsigned int> serialsToSyncArgs;
        serialsToSyncArgs = varsMap["sync-serials"].as<vector<unsigned int> >();
        serialsToSync.insert(serialsToSync.end(),  serialsToSyncArgs.begin(),  serialsToSyncArgs.end());
      }

      notify(varsMap);
    }
    catch
      (error& e) {
      ROS_ERROR("Unable to parse description: %s",  e.what());
      return false;
    }

  }
  catch
    (std::exception& e) {
    ROS_ERROR("Unhandled Exception reached the top of main: %s \n application will now exit",  e.what());
    return false;
  }
  return true;
}


int main(int argc, char **argv)
{
  std::string nodeName = "pgr_camera";
  std::string masterNodeName = "master_pgr_camera";

  ros::init(argc, argv, nodeName);
  ros::NodeHandle masterNodeHandle(masterNodeName);

  std::vector<std::shared_ptr<CameraNode>> cameraNodes;

  std::vector<unsigned int> cameraSerialToStart;
  std::vector<unsigned int> cameraSerialToSync;

  if(!parseCommandLine(argc,  argv,  cameraSerialToStart,  cameraSerialToSync)) {
    ROS_ERROR("Error while parsing the command line arguments");
    return -1;
  }

  FlycapCameraManager cameraManager;

  try {
    std::vector<shared_ptr<CameraNode>> camerasToSync;
    int cameraIndex = 0;
    for(auto serialNumber : cameraSerialToStart) {
      string cameraNodeName = toString("camera", serialNumber);

      unique_ptr<flycapcam::FlycapCameraGigE> flycapCamera = cameraManager.createGigECamera(serialNumber);

      ros::NodeHandle nh(cameraNodeName);
      std::shared_ptr<CameraNode> pn(createCameraNode(nh,  std::move(flycapCamera)));
      cameraNodes.push_back(pn);
      pn->init();

      if(std::find(cameraSerialToSync.begin(),
                   cameraSerialToSync.end(), serialNumber) != cameraSerialToSync.end()) {
        camerasToSync.push_back(pn);
      }

      pn->start();
      cameraIndex++;
    }


    ros_info("All camera initialized");
  }
  catch
    (std::runtime_error &e) {
    ROS_FATAL("Uncaught exception: '%s', aborting.", e.what());
    ROS_BREAK();
  }

  while(true) {
    ros::Time timestamp = ros::Time::now();
    for(std::shared_ptr<CameraNode>& cam : cameraNodes) {
      usleep(3000);
      cam->retrieveAndPublishFrame(timestamp);
    }
  }

  return 0;
}
