#ifndef PGR_TYPES
#define PGR_TYPES

#include <flycapture/Error.h>

#include <team_diana_lib/strings/strings.h>

#include <string>

#include <cstring>
#include <cstdio>

namespace flycapcam {

typedef unsigned int SerialNumber;

struct Binning {
    unsigned int x;
    unsigned int y;
};

struct GigEPacketParams {
    unsigned int size;
    unsigned int delay;
};


class FlycapResult {

public:
  explicit FlycapResult(const FlyCapture2::Error& error) {
    this->error = error;
  }

  FlyCapture2::Error getError() const {
    return error;
  }

  operator bool() const
  {
      return error == FlyCapture2::PGRERROR_OK;
  }

private:
  FlyCapture2::Error error;
};

static std::string guidToString(FlyCapture2::PGRGuid guid) {
  char guidString[30];
  snprintf(guidString, sizeof(guidString), "%X:%X:%X:%X", guid.value[0], guid.value[1], guid.value[2], guid.value[3]);
  return std::string(guidString);
}

}

#endif // PGR_TYPES
