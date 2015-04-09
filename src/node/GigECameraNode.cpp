#include "GigECameraNode.h"

#include "stdlib.h"

#include <team_diana_lib/logging/logging.h>
#include <team_diana_lib/strings/strings.h>
#include <team_diana_lib/strings/iterables.h>

#include <ros/ros.h>

using namespace Td;
using namespace std;
using namespace FlyCapture2;
using namespace flycapcam;

GigECameraNode::GigECameraNode(const ros::NodeHandle& nodeHandle, std::unique_ptr<flycapcam::FlycapCameraGigE>&& flycapCamera) :
 CameraNode(nodeHandle),
 gigeNodeHandle(toString("GigECamera", flycapCamera->getSerialNumber())),
 camGigEReconfigureServer(camGigEReconfigureMutex, gigeNodeHandle)
{
  this->flycapCamera = std::move(flycapCamera);
}


void GigECameraNode::configureGigE(pgr_camera::PgrGigECameraConfig& config, uint32_t level)
{
  std::lock_guard<boost::recursive_mutex> lock(camGigEReconfigureMutex);
  FlycapResult result;

  result = flycapCamera->setPacketDelay(config.packet_delay);
  printResultErrorMessageIfAny(result, "Unable to set packet delay");

  result = flycapCamera->setPacketSize(config.packet_size);
  printResultErrorMessageIfAny(result, "Unable to set packet size");

  result = flycapCamera->setPacketResendEnabled(config.packet_resend);
  printResultErrorMessageIfAny(result, "Unable to set packet resend");
}

flycapcam::FlycapCamera* GigECameraNode::getFlycapCamera() const
{
  return flycapCamera.get();
}

void GigECameraNode::printGigEInfo()
{
  FlycapResult result;
  unsigned int availablePacketSize;

  cout << "GigE informations for camera " << flycapCamera->getSerialNumber() << std::endl;
  if(result = flycapCamera->getAvailableGigEPacketSize(availablePacketSize)) {
    cout << "\tAvailable packet size (i.e. maximum mtu): " << availablePacketSize << std::endl;
  } else {
    cerr << "\tUnable to get available packet size: " << result.getErrorDescription() << std::endl;
  }

  vector<GigEStreamChannel> channelsInfo;

  if(result = flycapCamera->getGigEChannelsInfo(channelsInfo)) {
    cout << "\tnumber of channels: " << channelsInfo.size();
    for(const GigEStreamChannel& chanInfo : channelsInfo) {
      cout << "\tChannel Info: " << endl;
      //cout << "\t\t destination ip: " << iterableToString(chanInfo.destinationIpAddress.octets, ":");
      cout << "\t\tsourcePort: " << chanInfo.sourcePort << endl;
      //cout << "\t\thost port: " << chanInfo.hostPort << endl;
      cout << "\t\tinterPacketDelay: " << chanInfo.interPacketDelay << endl;
      cout << "\t\tpacketSize: " << chanInfo.packetSize << endl;
      cout << "\t\tnetworkInterfaceIndex: " << chanInfo.networkInterfaceIndex << endl;
    }
  } else {
    cerr << "Unable to get channels info: " << result.getErrorDescription() << endl;
  }

  cout << "Print GigE info done" << endl;
}

void GigECameraNode::printCameraStats()
{
  FlycapResult result;
  CameraStats stats;

  auto& out = std::cerr;

  if(result = flycapCamera->getCameraStats(stats)) {
    out << "Camera Stats:" << endl;
    out << "\t N. currupted images: " << stats.imageCorrupt << endl;
    out << "\t N. image dropped in driver: " << stats.imageDriverDropped << endl;
    out << "\t N. image dropped: " << stats.imageDropped << endl;
    out << "\t N. packet-resend received: " << stats.numResendPacketsReceived << endl;
    out << "\t N. packet-resend requested: " << stats.numResendPacketsRequested << endl;
    out << "\t N. failed image transmission: " << (stats.imageXmitFailed - (unsigned int)(-1))<< endl;
    out << "\t N. port errors: " << stats.portErrors << endl;
    out << "\t uptime (seconds): " << stats.timeSinceInitialization << endl;
    out << "\t seconds from last bus reset: " << stats.timeSinceBusReset << endl;
    out << "\t N. register read failed: " << stats.regReadFailed << endl;
    out << "\t N. register write failed: " << stats.regWriteFailed << endl;
    out << "\t Camera is powered up : " << stats.cameraPowerUp << endl;
  } else {
    cerr << "Unable to get camera stats: " << result.getErrorDescription() << std::endl;
  }

}

void GigECameraNode::setPrintOnNewFrame(bool enabled)
{
  flycapCamera->printOnNewFrame = enabled;
}

void GigECameraNode::initImpl()
{
  DynamicGigEReconfigureServer::CallbackType f = boost::bind(&GigECameraNode::configureGigE, this, _1, _2);
  camGigEReconfigureServer.setCallback(f);

  showInfoSubscriber = nodeHandler.subscribe<std_msgs::String>("show_info", 1000, [this](const std_msgs::String::ConstPtr& ) {
    this->printGigEInfo();
    this->printCameraStats();
  });

  updateDynamicReconfigureServerGigECamera();
}

void GigECameraNode::updateDynamicReconfigureServerGigECamera()
{
  pgr_camera::PgrGigECameraConfig config;
  FlycapResult result;

  unsigned int packet_delay;
  if(result = flycapCamera->getPacketDelay(packet_delay)) {
    config.packet_delay = static_cast<unsigned int>(packet_delay);
  } else {
    ros_error(result.getErrorDescription());
  }

  unsigned int packet_size;
  if(result = flycapCamera->getPacketSize(packet_size)) {
    config.packet_size = static_cast<unsigned int>(packet_size);
  } else {
    ros_error(result.getErrorDescription());
  }

  bool packet_resend_enabled;
  if(result = flycapCamera->isPacketResendEnabled(packet_resend_enabled)) {
    config.packet_resend = packet_resend_enabled;
  } else {
    ros_error(result.getErrorDescription());
  }

  std::lock_guard<boost::recursive_mutex> lock(camGigEReconfigureMutex);
  camGigEReconfigureServer.updateConfig(config);
}

