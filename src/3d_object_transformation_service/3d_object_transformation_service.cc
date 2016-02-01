#include "3d_object_transformation_service/3d_object_transformation_service.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <limits>

namespace task_net {
void StateUpdate(ObjectTransService *service) {
  while (true) {
    printf("Polling for table state\n");
    service->GetState();
    boost::this_thread::sleep(boost::posix_time::millisec(5000));
  }
}

ObjectTransService::ObjectTransService(ros::NodeHandle *nh) {
  // Start tabletop thread
  state_thread = new boost::thread(&StateUpdate, this);
}

ObjectTransService::~ObjectTransService() {}


void ObjectTransService::GetState() {
  tabletop_segmenter::TabletopSegmentation msg;
  if (ros::service::call("/tabletop_segmentation", msg)) {
    environment_state = msg.response;
  } else {
    ROS_ERROR("Tabletop Segmentation Service not responding");
  }
}

float PointDistance(cv::Point2f A, cv::Point2f B) {
  return std::sqrt(std::pow(A.x - B.x,2) + std::pow(A.y - B.y, 2));
}

// Function:    GetObjectPosition
// Description: Responsible for  taking the current state of the tabletop
//              and determine which object is represented by the image track
//              bounding box and return a tf transform from the space into world
bool ObjectTransService::GetObjectTransformation(
    table_setting_demo::ObjectTransformation::Request &req,
    table_setting_demo::ObjectTransformation::Response &res) {
  // Get point cloud data


  // for each cluster we need to compare it to the the objects
  cv::Mat mask;
  std::vector<cv::Point2f> object_centers(environment_state.masks.size());
  for (int i = 0; i < environment_state.masks.size(); ++i) {
    try {
      mask = cv_bridge::toCvCopy(environment_state.masks[i], "mono8")->image;
    } catch (cv_bridge::Exception &e) {
      ROS_ERROR("could not convert from '%s' to 'mono8'.", environment_state.masks[i].encoding.c_str());
    }

    // find the centroid of each image objects
    int pixel_count = 0;
    float x_sum = 0;
    float y_sum = 0;
    uint8_t *data = (uint8_t*)mask.data;
    for (int y = 0; y < mask.rows; ++y) {
      for (int x = 0; x < mask.cols; ++x) {
        if (data[y*mask.cols + x] > 0) {
          x_sum += x;
          y_sum += y;
          pixel_count++;
        }
      }
    }
    float xmean = x_sum / (float)pixel_count;
    float ymean = y_sum / (float)pixel_count;
    printf("X: %f Y: %f\n", xmean, ymean);
    object_centers[i] = cv::Point2f(xmean, ymean);
  }

  // find the centroid of the bounding box messaged here
  float obj_meanx = req.x + req.w / 2.0;
  float obj_meany = req.y + req.h / 2.0;
  cv::Point2f object_centroid(obj_meanx, obj_meany);
  float minimum_dist = std::numeric_limits<float>::infinity();
  int index;
  // compare to the 
  std::vector<float> distances(object_centers.size());
  for (int i = 0; i < object_centers.size(); ++i) {
    distances[i] = PointDistance(object_centroid, object_centers[i]);
    printf("Distance %f\n", distances[i]);
    if (distances[i] < minimum_dist) {
      index = i;
      minimum_dist = distances[i];
    }
  }

  // we know the group that it matches now as index

  // compute the orientation and position in 3D Space

  // compute first moment to get position

  // compute covariance matrix

  // obtain eigen vectors of covariance matrix
  
  // generate rotation matrix from object space into world

  // combine position and rotation matrix to generate transformation matrix

  return true;
}
}