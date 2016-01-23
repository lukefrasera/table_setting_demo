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
    // std::vector<cv::Point2f> bbox,
    std::string object_id) {

  detector->detect(frame, key_points);
  detector->compute(frame, key_points, descriptor);
  cv::Mat img = frame.clone();
  cv::drawKeypoints(frame, key_points, img);
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