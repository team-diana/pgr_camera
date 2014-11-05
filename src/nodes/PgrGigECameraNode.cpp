#include "PgrGigECameraNode.h"
#include "stdlib.h"

#define SLEEP_TIME 1

void PgrGigECameraNode::configure ( pgr_camera::PGRCameraConfig &config, uint32_t level )  {
  baseConfigure(config,  level);
  if (config.packet_size != currentConfig.packet_size || config.packet_delay != currentConfig.packet_delay) {
    sleep(SLEEP_TIME);
    gigeConfigure(config,  level);
    sleep(SLEEP_TIME);
  }
  currentConfig = config;
}

void PgrGigECameraNode::gigeConfigure(pgr_camera::PGRCameraConfig &config, uint32_t level ) {
  pgrCamera->SetGigESettings ( config.packet_size,  config.packet_delay );
}
