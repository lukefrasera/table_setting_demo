#include "log.h"
#include "stdlib.h"
#include "stdint.h"
#include <iostream>
#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <boost/filesystem.hpp>
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
  std::string image_filename;
  if (!local_nh.getParam("image_file", image_filename)) {
    ROS_ERROR("Image File parameter missing!");
  }
  cv::Mat image = cv::imread(image_filename.c_str());
  if (!image.data) {
    ROS_ERROR("Image: [%s] didn't load", image_filename.c_str());
  }

  // Split image file into parts
  boost::filesystem::path image_path = image_filename;
  printf("Filename: %s, extension: %s\n",
    image_path.stem().string().c_str(),
    image_path.extension().string().c_str());

  std::vector<int> bounding;
  if (!local_nh.getParam(image_path.stem().string(), bounding)) {
    ROS_ERROR("Image [%s]: Bounds parameters not found!",
      image_path.stem().string().c_str());
  }

  std::vector<cv::Point> bbox(bounding.size()/2);
  for (int i = 0; i < bbox.size(); ++i) {
    bbox[i].x = bounding[2*i];
    bbox[i].y = bounding[2*i+1];
    std::cout <<bbox[i] <<std::endl;
  }

  double orb_samples = 1000;
  cv::Ptr<cv::ORB> orb = new cv::ORB(orb_samples);
  cv::Ptr<cv::DescriptorMatcher> matcher =
    cv::DescriptorMatcher::create("BruteForce-Hamming");
  orb_tracker = new qr::Tracker(orb, matcher);

  orb_tracker->InitializeTracker(image, bbox, "test_object");
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
  qr::Contour_t corners;
  qr::QRDetectIdentifiers(image, &corners);
  cv::drawContours(image, corners, -1, cv::Scalar(0,255,50));
  cv::imshow("Detection", image);
  cv::waitKey(10);

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
