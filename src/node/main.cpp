#include "pgr_camera/PgrCameraConfig.h"
#include "libflycapcam/FlycapCameraManager.h"
#include "GigECameraNode.h"
#include "CommandLineParsing.h"

#include <ros/ros.h>
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

std::shared_ptr<GigECameraNode> createGigECameraNode(unsigned int serialNumber,
                                             FlycapCameraManager& cameraManager)
{
  string cameraNodeName = toString("camera", serialNumber);
  unique_ptr<flycapcam::FlycapCameraGigE> flycapCamera = cameraManager.createGigECamera(serialNumber);

  ros::NodeHandle nh(cameraNodeName);
  std::shared_ptr<GigECameraNode> pn(new GigECameraNode(nh, std::move(flycapCamera)));

  return pn;
}

void printGigEInfo(const std::vector<unsigned int>& serials) {
  FlycapCameraManager cameraManager;
  for(auto serial : serials) {
    auto camNode = createGigECameraNode(serial, cameraManager);
    camNode->printGigEInfo();
  }
}

bool running;

void sigintHandler(int) {
  running = false;
}

int main(int argc, char **argv)
{
  std::string nodeName = "pgr_camera";
  std::string masterNodeName = "master_pgr_camera";

  signal(SIGINT, sigintHandler);

  ros::init(argc, argv, nodeName);
  ros::NodeHandle masterNodeHandle(masterNodeName);

  std::vector<std::shared_ptr<GigECameraNode>> cameraNodes;

  std::vector<unsigned int> cameraSerialToStart;
  std::vector<unsigned int> cameraSerialToSync;
  bool showGigEInfo;

  if(!parseCommandLine(argc,  argv,  cameraSerialToStart,  cameraSerialToSync, showGigEInfo)) {
    ROS_ERROR("Error while parsing the command line arguments");
    return -1;
  }

  try {
    FlycapCameraManager cameraManager;

    for(auto serialNumber : cameraSerialToStart) {

      std::shared_ptr<GigECameraNode> pn = createGigECameraNode(serialNumber, cameraManager);
      cameraNodes.push_back(pn);
      pn->init();

//       if(std::find(cameraSerialToSync.begin(),
//                    cameraSerialToSync.end(), serialNumber) != cameraSerialToSync.end()) {
//         camerasToSync.push_back(pn);
//       }

      pn->start();
//       cameraIndex++;
    }


    ros_info("All camera initialized");
  }
  catch
    (std::runtime_error &e) {
    ROS_FATAL("Uncaught exception: '%s', aborting.", e.what());
    ROS_BREAK();
  }

  running = true;

  unsigned long cycle = 0;
  while(running) {
    ros::Time timestamp = ros::Time::now();
    for(std::shared_ptr<GigECameraNode>& cam : cameraNodes) {
      usleep(3000);
      cam->retrieveAndPublishFrame(timestamp);

      if(cycle % 20) {
        cam->printGigEInfo();
        cam->printCameraStats();
      }
    }


    cycle++;
  }

  for(std::shared_ptr<GigECameraNode>& cam : cameraNodes) {
    cam->stop();
  }

  return 0;
}
