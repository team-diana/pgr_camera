#include <CommandLineParsing.h>

#include <ros/ros.h>
#include <boost/program_options.hpp>
#include <team_diana_lib/logging/logging.h>
#include <team_diana_lib/strings/strings.h>

using namespace Td;
using namespace std;

bool parseCommandLine(int argc, char* const * argv, std::vector<unsigned int>& serialsToStart,
                      std::vector<unsigned int>& serialsToSync, bool& startAndStop,
                      bool& printGigEInfo, bool& printDebugInfo, bool& enablePublish, bool& printOnNewFrame)
{
  using namespace boost::program_options;
  try {
    /** Define and parse the program options
     */
    options_description desc("Options");
    desc.add_options()
    ("help,h", "Print help messages")
    ("serials,s", value<std::vector<unsigned int> >()->multitoken(), "list of serials of the cameras to start")
//     ("sync-serials, S", value<std::vector<unsigned int> >()->multitoken(), "list of serials of the cameras to sync");
    ("start-and-stop,P", value<bool>()->default_value(false)->implicit_value(true), "start and stop each camera for every frame")
    ("print-gige-info,G", value<bool>()->default_value(false)->implicit_value(true), "reports information about the GigE network")
    ("disable-publishing,D", value<bool>()->default_value(false)->implicit_value(true), "disable image publishing for debug purpose")
    ("print-on-new-frame,F", value<bool>()->default_value(false)->implicit_value(true), "print a msg for each new frame")
    ("debug,d", value<bool>()->default_value(false)->implicit_value(true), "print debug information during execution");

    variables_map varsMap;
    try {
      store(parse_command_line(argc, argv, desc), varsMap);

      if(varsMap.count("help")) {
        cout << desc << endl;
//         cout << "Usage: " << endl <<
//                   "--serials,-s {list of serials of the cameras to start} " << endl <<
// //                   "--sync-serials {list of serials of the cameras to start in sync}"
//                   "--start-and-stop,-P\t use " << std::endl;
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

//       if(!varsMap["sync-serials"].empty()) {
//         std::vector<unsigned int> serialsToSyncArgs;
//         serialsToSyncArgs = varsMap["sync-serials"].as<vector<unsigned int> >();
//         serialsToSync.insert(serialsToSync.end(),  serialsToSyncArgs.begin(),  serialsToSyncArgs.end());
//       }

      printGigEInfo = boost::any_cast<bool>(varsMap["print-gige-info"].value());
      printDebugInfo = boost::any_cast<bool>(varsMap["debug"].value());
      startAndStop = boost::any_cast<bool>(varsMap["start-and-stop"].value());
      printOnNewFrame = boost::any_cast<bool>(varsMap["print-on-new-frame"].value());
      enablePublish = !boost::any_cast<bool>(varsMap["disable-publishing"].value());

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
