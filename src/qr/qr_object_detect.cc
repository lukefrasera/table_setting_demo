#include "qr/qr_object_detect.h"
namespace qr {
////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////// Tracker ///////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
Tracker::Tracker(
    cv::Ptr<cv::Feature2D> detector_,
    cv::Ptr<cv::DescriptorMatcher> matcher_) : 
      detector(detector_),
      matcher(matcher_) {
  // Constructor for Tracker Class
}

void Tracker::InitializeTracker(
    const cv::Mat &frame,
    std::vector<cv::Point> bbox,
    std::string object_id) {

  bounding.x = bbox[0].x;
  bounding.y = bbox[0].y;
  bounding.width  = bbox[0].x - bbox[1].x;
  bounding.height = bbox[0].y - bbox[1].y;

  // Initialize filter
  filter.InitializeFilter(bounding);

  // crop image for orb detector frame
  cv::Mat roi = frame(
    cv::Range(bbox[0].y, bbox[1].y),
    cv::Range(bbox[0].x, bbox[1].x));



  detector->detect(roi, key_points);
  detector->compute(roi, key_points, descriptor);
  cv::Mat img = roi.clone();
  cv::drawKeypoints(roi, key_points, img);
  imshow("Original", img);
}
void Tracker::ProcessFrame(const cv::Mat &image) {
  const double nn_match_ratio = 0.8f;
  std::vector<cv::KeyPoint> img_keypoints;
  cv::Mat img_descriptor;

  detector->detect(image, img_keypoints);
  detector->compute(image, img_keypoints, img_descriptor);

  std::vector< std::vector<cv::DMatch> > matches;
  std::vector<cv::KeyPoint> matched_sample, matched_image;

  matcher->knnMatch(descriptor, img_descriptor, matches, 2);

  for (int i = 0; i <matches.size(); ++i) {
    if (matches[i][0].distance < nn_match_ratio * matches[i][1].distance) {
      matched_image.push_back(key_points[matches[i][0].queryIdx]);
      matched_sample.push_back(img_keypoints[matches[i][0].trainIdx]);
    }
  }

  cv::Mat img2 = image.clone();
  cv::drawKeypoints(image, matched_sample, img2);
  cv::imshow("Matches", img2);
  cv::waitKey(10);
}
////////////////////////////////////////////////////////////////////////////////
//////////////////////////////// Kalman2DTracker ///////////////////////////////
////////////////////////////////////////////////////////////////////////////////
Kalman2DTracker::Kalman2DTracker() {}
Kalman2DTracker::~Kalman2DTracker() {}
void Kalman2DTracker::InitializeFilter(const cv::Rect &measurement) {
  position_filter.init(4, 2, 0);
  bounding_filter.init(4, 2, 0);

  // Set Transition Matrices
  position_filter.transitionMatrix = *(cv::Mat_<float>(4, 4) <<
    1, 0, 1, 0,
    0, 1, 0, 1,
    0, 0, 1, 0,
    0, 0, 0, 1);
  bounding_filter.transitionMatrix = *(cv::Mat_<float>(4, 4) <<
    1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1, 0,
    0, 0, 0, 1);

  // set measurement Matrices
  cv::setIdentity(position_filter.measurementMatrix, cv::Scalar(1));
  cv::setIdentity(bounding_filter.measurementMatrix, cv::Scalar(1));

  // set Process Noise Cov
  cv::setIdentity(position_filter.processNoiseCov, cv::Scalar(0.001));
  cv::setIdentity(bounding_filter.processNoiseCov, cv::Scalar(0.0001));

  // set Measurement Noise Covariance
  cv::setIdentity(position_filter.measurementNoiseCov, cv::Scalar(0.1));
  cv::setIdentity(bounding_filter.measurementNoiseCov, cv::Scalar(0.01));

  // set Error Covariance Post
  cv::setIdentity(position_filter.errorCovPost, cv::Scalar(.1));
  cv::setIdentity(bounding_filter.errorCovPost, cv::Scalar(.1));

  // Initialize state
  position_filter.statePre.at<float>(0) = measurement.x + measurement.width/2;
  position_filter.statePre.at<float>(1) = measurement.y + measurement.height/2;
  position_filter.statePre.at<float>(2) = 0;
  position_filter.statePre.at<float>(3) = 0;

  bounding_filter.statePre.at<float>(0) = measurement.width;
  bounding_filter.statePre.at<float>(1) = measurement.height;
  bounding_filter.statePre.at<float>(2) = 0;
  bounding_filter.statePre.at<float>(3) = 0;

  // Initial predict
  state_position_prediction = position_filter.predict();
  state_bounding_prediction = bounding_filter.predict();
}

void Kalman2DTracker::MeasurementUpdate(const cv::Rect &measurement) {
  cv::Mat_<float> position(2, 1);
  position.at<float>(0) = measurement.x + measurement.width  / 2;
  position.at<float>(1) = measurement.y + measurement.height / 2;

  cv::Mat_<float> bounding(2, 1);
  bounding.at<float>(0) = measurement.width;
  bounding.at<float>(1) = measurement.height;

  state_position_estimate = position_filter.correct(position);
  state_bounding_estimate = bounding_filter.correct(bounding);

  state_position_prediction = position_filter.predict(position);
  state_bounding_prediction = bounding_filter.predict(bounding);
}

void Kalman2DTracker::GetStatePrediction(cv::Rect *measurement) {
  measurement->x = state_position_prediction.at<float>(0) -
                   state_bounding_prediction.at<float>(0)/2;
  measurement->y = state_position_prediction.at<float>(1) -
                   state_bounding_prediction.at<float>(1)/2;

  measurement->width  = state_bounding_prediction.at<float>(0);
  measurement->height = state_bounding_prediction.at<float>(1);
}

void Kalman2DTracker::GetStateEstimate(cv::Rect *measurement) {
  measurement->x = state_position_estimate.at<float>(0) -
                   state_bounding_estimate.at<float>(0)/2;
  measurement->y = state_position_estimate.at<float>(1) -
                   state_bounding_estimate.at<float>(1)/2;

  measurement->width  = state_bounding_estimate.at<float>(0);
  measurement->height = state_bounding_estimate.at<float>(1);
}
////////////////////////////////////////////////////////////////////////////////
/////////////////////////////// QrFeatureDetector //////////////////////////////
////////////////////////////////////////////////////////////////////////////////
QrFeatureDetector::QrFeatureDetector() {}
QrFeatureDetector::~QrFeatureDetector() {}
////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////// QrTracker //////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
QrTracker::QrTracker() {}
QrTracker::~QrTracker() {}
////////////////////////////////////////////////////////////////////////////////
//////////////////////////////// QrObjectsTrack ////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
QrObjectsTrack::QrObjectsTrack() {}
QrObjectsTrack::~QrObjectsTrack() {}

bool QrObjectsTrack::GetObject(std::string object, std::string &object_id) {
  return false;
}
bool QrObjectsTrack::ObjectInView(std::string object) {
  return false;
}
}  // namespace qr