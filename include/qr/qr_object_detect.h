#ifndef QR_OBJECT_DETECT_H_
#define QR_OBJECT_DETECT_H_
#include "log.h"
#include "qr_detect.h"

namespace qr {
class Kalman2DTracker {
 public:
  Kalman2DTracker();
  virtual ~Kalman2DTracker();
};

class QrFeatureDetector {
 public:
  QrFeatureDetector();
};

class QrTracker : public Kalman2DTracker {
 public:
  QrTracker();
  virtual ~QrTracker();
};
}  // namespace qr
#endif  // QR_OBJECT_DETECT_H_
