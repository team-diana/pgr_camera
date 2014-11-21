#ifndef COMMAND_LINE_PARSING_H
#define COMMAND_LINE_PARSING_H

bool parseCommandLine(int argc, char* const * argv, std::vector<unsigned int>& serialsToStart,
                      std::vector<unsigned int>& serialsToSync, bool& showGigEInfo);

#endif
