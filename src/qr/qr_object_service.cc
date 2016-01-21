#include "log.h"
#include "stdlib.h"
#include "stdint.h"
#include "ros/ros.h"


int main(int argc, char *argv[]) {
  // Initialize node environment
  ros::init(argc, argv, "qr_object_detection_service");
  ros::NodeHandle nh;

  ros::spin();
  return 0;
}