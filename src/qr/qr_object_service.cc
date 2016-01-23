#include "log.h"
#include "stdlib.h"
#include "stdint.h"
#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "table_setting_demo/object_request.h"
#include "table_setting_demo/object_position.h"
#include "table_setting_demo/pick_and_place.h"
#include "table_setting_demo/table_setting_demo_types.h"
#include "qr/qr_object_detect.h"
#include "qr_detect.h"


class QrObjectService {
 public:
  QrObjectService(ros::NodeHandle *nh);
  virtual ~QrObjectService();

  bool QrInView(
    table_setting_demo::qr_inview::Request &req,
    table_setting_demo::qr_inview::Response &res);
  bool GetQrObject(
    table_setting_demo::object_request::Request &req,
    table_setting_demo::object_request::Response &res);
  bool GetQrPosition(
    table_setting_demo::object_position::Request &req,
    table_setting_demo::object_position::Response &res);
  void CameraImageCallback(const sensor_msgs::ImageConstPtr &msg);

  bool QrDetectionProcess(const cv::Mat &image);
 private:
  qr::QrObjectsTrack object_detector;

  image_transport::Subscriber image_subscriber;
  image_transport::ImageTransport it;
  cv::Ptr<qr::Tracker> orb_tracker;
};
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

QrObjectService::QrObjectService(ros::NodeHandle *nh) : it(*nh) {
  ros::NodeHandle local_nh("~");
  std::string camera_topic;
  if  (!local_nh.getParam("camera_topic", camera_topic)) {
    LOG_INFO("ERROR: [camera_topic] not intiialized!");
  }
  // Subscribe to image topic
  image_subscriber = it.subscribe(
    camera_topic.c_str(),
    1,
    &QrObjectService::CameraImageCallback,
    this);

  // load first frame into akaze tracker
  cv::Mat image = cv::imread("/home/luke/Pictures/qr_code_sample_.jpg");
  if (!image.data) {
    ROS_ERROR("Image didn't load");
  }

  double orb_samples = 1500;
  cv::Ptr<cv::ORB> orb = new cv::ORB(orb_samples);
  cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
  orb_tracker = new qr::Tracker(orb, matcher);

  orb_tracker->InitializeTracker(image, "test_object");
}

QrObjectService::~QrObjectService() {}

void QrObjectService::CameraImageCallback(
    const sensor_msgs::ImageConstPtr &msg) {
  cv_bridge::CvImagePtr image_ptr;
  try {
    image_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  QrDetectionProcess(image_ptr->image);
}

bool QrObjectService::QrDetectionProcess(const cv::Mat &image) {
  // qr::Contour_t corners;
  // qr::QRDetectIdentifiers(image, &corners);
  // cv::drawContours(image, corners, -1, cv::Scalar(0,255,50));
  // cv::imshow("Detection", image);
  // cv::waitKey(10);

  orb_tracker->ProcessFrame(image);
  return true;
}

bool QrObjectService::QrInView(
    table_setting_demo::qr_inview::Request &req,
    table_setting_demo::qr_inview::Response &res) {
  if (object_detector.ObjectInView(req.object)) {
    res.success = true;
  } else {
    res.success = false;
  }
  return true;
}

bool QrObjectService::GetQrObject(
    table_setting_demo::object_request::Request &req,
    table_setting_demo::object_request::Response &res) {
  std::string object_id;
  if (object_detector.GetObject(req.object, object_id)) {
    res.object_id = object_id;
    res.in_scene = true;
  } else {
    res.object_id = "";
    res.in_scene = false;
  }
  return true;
}

bool QrObjectService::GetQrPosition(
    table_setting_demo::object_position::Request &req,
    table_setting_demo::object_position::Response &res) {
  return true;
}
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

int main(int argc, char *argv[]) {
  // Initialize node environment
  ros::init(argc, argv, "qr_object_detection_service");
  ros::NodeHandle nh;
  QrObjectService qr_service(&nh);

  ros::spin();
  return 0;
}