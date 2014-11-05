

// ROS communication
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
#include <boost/program_options.hpp>
#include <vector>
#include <sstream>
#include <fstream>
#include <sys/stat.h>
#include <cstdlib>
#include <unistd.h>

#include "PgrCameraNode.h"
#include "PgrGigECameraNode.h"
#include "CameraSynchronizer.h"

PgrCameraNode* createCameraNode ( ros::NodeHandle handle, boost::shared_ptr<PgrCamera> camera )
{
     if ( camera->getInterfaceType() == FlyCapture2::INTERFACE_GIGE ) {
          return new PgrGigECameraNode ( handle,  camera );
     } else {
          return new PgrCameraNode ( handle,  camera );
     }
}

bool parseCommandLine ( int argc,  char** argv,  std::vector<unsigned int>& serialsToStart, std::vector<unsigned int>& serialsToSync )
{
     using namespace boost::program_options;
     try {
          /** Define and parse the program options
           */
          options_description desc ( "Options" );
          desc.add_options()
          ( "help, h", "Print help messages" )
          ( "serials, s", value<std::vector<unsigned int> >()->multitoken(), "list of serials of the cameras to start" )
          ( "sync-serials, S", value<std::vector<unsigned int> >()->multitoken(), "list of serials of the cameras to sync" );

          variables_map vm;
          try {
               store ( parse_command_line ( argc, argv, desc ),
                       vm ); // can throw

               if ( vm.count ( "help" ) ) {
                    ROS_ERROR ( "Usage: \n" \
                                "--serials {list of serials of the cameras to start} \n"\
                                "--sync-serials {list of serials of the cameras to start in sync}" );
                    return false;
               }

               if ( !vm["serials"].empty() ) {
                    std::vector<unsigned int> serialsToStartArgs;
                    serialsToStartArgs = vm["serials"].as<vector<unsigned int> >();
                    serialsToStart.insert ( serialsToStart.end(),  serialsToStartArgs.begin(),  serialsToStartArgs.end() );
               } else  {
                    ROS_ERROR ( "No serials specified. Use help to see usage" );
                    return false;
               }

               if ( !vm["sync-serials"].empty() ) {
                    std::vector<unsigned int> serialsToSyncArgs;
                    serialsToSyncArgs = vm["sync-serials"].as<vector<unsigned int> >();
                    serialsToSync.insert ( serialsToSync.end(),  serialsToSyncArgs.begin(),  serialsToSyncArgs.end() );
               }

               notify ( vm ); // throws on error, so do after help in case
               // there are any problems
          } catch ( error& e ) {
               ROS_ERROR ( "Unable to parse description: %s",  e.what() );
               return false;
          }

     } catch ( std::exception& e ) {
          ROS_ERROR ( "Unhandled Exception reached the top of main: %s \n application will now exit",  e.what() );
          return false;
     }
     return true;
}


int main ( int argc, char **argv )
{
     std::string nodeName = "pgr_camera";
     std::string masterNodeName= "master_pgr_camera";

     ros::init ( argc, argv, nodeName );
     ros::NodeHandle masterNodeHandle ( masterNodeName );

     std::vector<boost::shared_ptr<PgrCameraNode> > cameraNodes;

     std::vector<unsigned int> cameraSerialToStart;
     std::vector<unsigned int> cameraSerialToSync;

     if ( !parseCommandLine ( argc,  argv,  cameraSerialToStart,  cameraSerialToSync ) ) {
          ROS_ERROR ( "Error while parsing the command line arguments" );
          return -1;
     }

     PgrCameraFactory pgrCameraFactory;

     try {
          std::vector<shared_ptr<PgrCameraNode> > camerasToSync;
          int cameraIndex = 0;
          for ( std::vector<unsigned int>::iterator i = cameraSerialToStart.begin(); i != cameraSerialToStart.end(); i++ ) {
               unsigned int serialNumber = *i;
               string cameraNodeName = ( boost::format ( "camera%1%" ) % cameraIndex ).str();

               shared_ptr<PgrCamera> pgrCamera = pgrCameraFactory.getCameraFromSerialNumber ( serialNumber );
               pgrCamera->setCamIndex ( cameraIndex );
               ros::NodeHandle nh ( cameraNodeName );
               boost::shared_ptr<PgrCameraNode> pn (createCameraNode(nh,  pgrCamera));
               cameraNodes.push_back ( pn );
               pn->setup();

               DynamicReconfigureServer::CallbackType f = boost::bind ( &PgrCameraNode::configure, pn, _1, _2 );
               pn->getDynamicReconfigureServer().setCallback ( f );
               cameraIndex++;

               if ( std::find ( cameraSerialToSync.begin(),  cameraSerialToSync.end(), serialNumber ) != cameraSerialToSync.end() ) {
                    camerasToSync.push_back ( pn );
               }
               pn->start();
          }


          ROS_INFO ( "All camera initialized" );

          CameraSynchronizer cameraSynchronizer ( camerasToSync );

          ros::spin ();

     } catch ( std::runtime_error &e ) {
          ROS_FATAL ( "Uncaught exception: '%s', aborting.", e.what () );
          ROS_BREAK ();
     }

     return 0;
}
