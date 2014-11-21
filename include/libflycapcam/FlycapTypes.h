#ifndef PGR_TYPES
#define PGR_TYPES

#include <flycapture/Error.h>
#include <flycapture/FlyCapture2.h>

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
  FlycapResult() = default;

  FlycapResult(const FlyCapture2::Error& error) {
    this->error = error;
  }

  FlyCapture2::Error getError() const {
    return error;
  }

  std::string getErrorDescription() const {
    return error.GetDescription();
  }

  operator bool() const
  {
      return error == FlyCapture2::PGRERROR_OK;
  }

  friend bool operator! (const FlycapResult& result);

private:
  FlyCapture2::Error error;
};

std::string guidToString(const FlyCapture2::PGRGuid& guid);

}

#endif // PGR_TYPES
