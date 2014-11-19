#ifndef PGR_TYPES
#define PGR_TYPES

#include <flycapture/Error.h>

namespace flycapcam {

typedef unsigned int SerialNumber;

struct Binning {
    unsigned int x;
    unsigned int y;
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

}

#endif // PGR_TYPES
