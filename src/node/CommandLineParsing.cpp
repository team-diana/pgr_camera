#include <ros/ros.h>
#include <boost/program_options.hpp>
#include <team_diana_lib/logging/logging.h>
#include <team_diana_lib/strings/strings.h>

using namespace Td;
using namespace std;

bool parseCommandLine(int argc, char* const * argv, std::vector<unsigned int>& serialsToStart,
                      std::vector<unsigned int>& serialsToSync, bool& showGigEInfo)
{
  using namespace boost::program_options;
  try {
    /** Define and parse the program options
     */
    options_description desc("Options");
    desc.add_options()
    ("help, h", "Print help messages")
    ("serials, s", value<std::vector<unsigned int> >()->multitoken(), "list of serials of the cameras to start")
//     ("sync-serials, S", value<std::vector<unsigned int> >()->multitoken(), "list of serials of the cameras to sync");
    ("show-gige-info, N", value<bool>()->default_value(false)->implicit_value(true), "reports information about the GigE network");

    variables_map varsMap;
    try {
      store(parse_command_line(argc, argv, desc), varsMap);

      if(varsMap.count("help")) {
        ros_error("Usage: \n" \
                  "--serials {list of serials of the cameras to start} \n"\
//                   "--sync-serials {list of serials of the cameras to start in sync}"
                  ""
                 );
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

      showGigEInfo = boost::any_cast<bool>(varsMap["show-gige-info"].value());

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
