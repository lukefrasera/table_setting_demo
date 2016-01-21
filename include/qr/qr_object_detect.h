#ifndef QR_OBJECT_DETECT_H_
#define QR_OBJECT_DETECT_H_
#include "log.h"
#include "qr_detect.h"
#include <opencv2/features2d.hpp>
#include <opencv2/opencv.hpp>

namespace qr {
class Kalman2DTracker {
 public:
  Kalman2DTracker();
  virtual ~Kalman2DTracker();
 private:
  cv::KalmanFilter position_filter;
  cv::KalmanFilter bounding_filter;
};

class QrFeatureDetector {
 public:
  QrFeatureDetector();
  virtual ~QrFeatureDetector();
};

class QrTracker : public Kalman2DTracker {
 public:
  QrTracker();
  virtual ~QrTracker();
};
}  // namespace qr
#endif  // QR_OBJECT_DETECT_H_
