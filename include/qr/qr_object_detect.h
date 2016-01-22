#ifndef QR_OBJECT_DETECT_H_
#define QR_OBJECT_DETECT_H_
#include "log.h"
#include "qr_detect.h"
#include <opencv2/features2d.hpp>
#include <opencv2/opencv.hpp>

namespace qr {
typedef enum TrackingState {
  TRACK = 0,
  PREDICT,
} TrackingState_e;

typedef struct TrackerState {
  uint32_t tracking_state;
  bool valid;
} TrackerState_t;

class Kalman2DTracker {
 public:
  Kalman2DTracker();
  virtual ~Kalman2DTracker();

  void MeasurementUpdate(cv::Point2f position, cv::Rect window);
  void GetStatePrediction(cv::Point2f *position, cv::Rect *window);
  void GetStateEstimate(cv::Point2f *position, cv::Rect *window);
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

class QrObjectsTrack {

};
}  // namespace qr
#endif  // QR_OBJECT_DETECT_H_
