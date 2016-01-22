#include "log.h"
#include "stdlib.h"
#include "stdint.h"
#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>


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

  bool QrDetectionProcess();
 private:
  QrObjectsTrack object_detector;

  image_transport::Subscriber image_subscriber;
  image_transport::ImageTransport it;
};

QrObjectService::QrObjectService(ros::NodeHandle *nh) : it(*nh) {
  ros::NodeHandle local_nh("~");
  
  image_subscriber = it.subscribe()
}

int main(int argc, char *argv[]) {
  // Initialize node environment
  ros::init(argc, argv, "qr_object_detection_service");
  ros::NodeHandle nh;
  QrObjectService qr_service(&nh);

  ros::spin();
  return 0;
}