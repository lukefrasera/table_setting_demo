#include "log.h"
#include "stdlib.h"
#include "stdint.h"
#include <iostream>
#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <boost/filesystem.hpp>
#include <cv_bridge/cv_bridge.h>
#include "table_setting_demo/object_request.h"
#include <cv_bridge/cv_bridge.h>
#include "table_setting_demo/object_request.h"
#include "table_setting_demo/object_position.h"
#include "table_setting_demo/pick_and_place.h"
#include "table_setting_demo/table_setting_demo_types.h"
#include "qr/qr_object_detect.h"
#include "qr_detect.h"
#include <quirc.h>


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
  std::string folder_str;
  if (!local_nh.getParam("folder", folder_str)) {
    ROS_ERROR("No image Folder!");
  }

  cv::Mat image = cv::imread((folder_str +"/"+ image_filename).c_str());
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
    std::cout << bbox[i] <<std::endl;
  }

  // double orb_samples = 1000;
  // cv::Ptr<cv::ORB> orb = cv::ORB::create();
  // orb->setMaxFeatures(orb_samples);
  // cv::Ptr<cv::DescriptorMatcher> matcher =
  //   cv::DescriptorMatcher::create("BruteForce-Hamming");
  // orb_tracker = new qr::Tracker(orb, matcher);

  // orb_tracker->InitializeTracker(image, bbox, "test_object");

  // // Initialize tracking system
  object_detector.Init();
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

float CvPointDist(cv::Point2f A, cv::Point2f B) {
  return std::sqrt(std::pow((A.x - B.x), 2) + std::pow((A.y - B.y), 2));
}

float PointLinePerpendicularDistance(
    cv::Point2f L,
    cv::Point2f M,
    cv::Point2f J) {
  float a,b,c,pdist;
  a = -((M.y - L.y) / (M.x - L.x));
  b = 1.0;
  c = (((M.y - L.y) /(M.x - L.x)) * L.x) - L.y;
  
  // Now that we have a, b, c from the equation ax + by + c, time to substitute (x,y) by values from the Point J

  pdist = (a * J.x + (b * J.y) + c) / sqrt((a * a) + (b * b));
  return pdist;
}

float LineSlope(cv::Point2f L, cv::Point2f M, int *align) {
  float dx, dy;
  dx = M.x - L.x;
  dy = M.y - L.y;

  if (dy != 0) {
    *align = 1;
    return (dy/dx);
  } else {
    *align = 0;
    return 0.0;
  }
}
// Output QR points in order: Top, Right, Bottom
std::vector<cv::Point2f> CvOrderQrPoints(std::vector<cv::Point2f> points) {
  float ab = CvPointDist(points[0], points[1]);
  float bc = CvPointDist(points[1], points[2]);
  float ca = CvPointDist(points[2], points[0]);
  ROS_INFO("A: %f,%f - distAB: %f",points[0].x, points[0].y, ab);
  ROS_INFO("B: %f,%f - distBC: %f",points[1].x, points[1].y, bc);
  ROS_INFO("C: %f,%f - distCA: %f",points[2].x, points[2].y, ca);

  int top, right, bottom, median1, median2;
  int A = 0, B = 1, C = 2;
  if        (ab > bc && ab > ca) {
    top = C; median1 = A; median2 = B;
  } else if (ca > ab && ca > bc) {
    top = B; median1 = A; median2 = C;
  } else if (bc > ab && bc > ca) {
    top = A; median1 = B; median2 = C;
  }

  // Determine bottom and right
  float dist = PointLinePerpendicularDistance(
    points[median1],
    points[median2],
    points[top]);
  int align;
  float slope = LineSlope(points[median1], points[median2], &align);

  if (align == 0) {
    bottom = median1;
    right = median2;
  } else if (slope < 0 && dist < 0) {
    bottom = median1;
    right = median2;
  } else if (slope > 0 && dist < 0) {
    right = median1;
    bottom = median2;
  } else if (slope < 0 && dist > 0) {
    right = median1;
    bottom = median2;
  } else if (slope > 0 && dist > 0) {
    bottom = median1;
    right = median2;
  }
  std::vector<cv::Point2f> output;
  output.push_back(points[top]);
  output.push_back(points[bottom]);
  output.push_back(points[right]);
  return output;
}

bool QrObjectService::QrDetectionProcess(const cv::Mat &image) {
  cv::Rect roi;
  cv::Mat qr_image;
  cv::Mat img;
  if (qr::QRDetectIdentifiers(image, &roi, qr_image)) {
    img = image.clone();
    cv::rectangle(img, roi, cv::Scalar(0,255,100), 3);
    cv::imshow("OVERLAY", img);
    cv::imshow("qr_image", qr_image);
    cv::waitKey(10);
  }
  object_detector.UpdateFrame(image);
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
