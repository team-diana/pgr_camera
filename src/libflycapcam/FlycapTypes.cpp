#include "libflycapcam/FlycapTypes.h"

#include <cstdio>
#include <cstring>

namespace flycapcam {

std::string guidToString(const FlyCapture2::PGRGuid& guid) {
  char guidString[30];
  snprintf(guidString, sizeof(guidString), "%X:%X:%X:%X", guid.value[0], guid.value[1], guid.value[2], guid.value[3]);
  return std::string(guidString);
}

}
