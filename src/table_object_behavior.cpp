/*
baxter_demos
Copyright (C) 2015  Luke Fraser

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#include <cmath>
#include "log.h"
#include "table_setting_demo/table_object_behavior.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/JointState.h"
#include "table_setting_demo/table_setting_demo_types.h"
#include "table_setting_demo/pick_and_place.h"
#include "table_setting_demo/object_request.h"
#include "table_setting_demo/pick_and_place_state.h"
#include "table_setting_demo/object_position.h"

namespace pr2 {
typedef enum STATE {
  APPROACHING = 0,
  PICKING,
  PICKED,
  PLACING,
  PLACED,
  NEUTRAL,
  IDLE
} STATE_t;
}  // namespace pr2

namespace task_net {

static const char *dynamic_object_str[] = {
  // "cup",
  // "bowl",
  // "soda",
  "fork",
  "spoon",
  "knife"
};
static const char *static_object_str[] = {
  "cup",
  "bowl",
  "soda",
  "neutral",
  "placemat",
  "wineglass",
  "plate"
};

TableObject::TableObject() {}
TableObject::TableObject(NodeId_t name, NodeList peers, NodeList children,
    NodeId_t parent,
    State_t state,
    std::string mutex_topic,
    std::string object,
    std::vector<float> pos,
    std::vector<float> neutral_pos,
    bool use_local_callback_queue,
    boost::posix_time::millisec mtime) : Behavior(name,
      peers,
      children,
      parent,
      state), mut(name.topic.c_str(), mutex_topic) {
  object_ = object;
  object_pos = pos;
  neutral_object_pos = neutral_pos;
  object_id_ = "";

  // check if dynamic object
  std::vector<std::string> static_objects_ = std::vector<std::string>(
    static_object_str,
    static_object_str + sizeof(static_object_str) / sizeof(char*));
  dynamic_object = true;
  for (int i = 0; i < static_objects_.size(); ++i) {
    if (object == static_objects_[i]) {
      dynamic_object = false;
      break;
    }
  }
  // set object_id
  if (!dynamic_object) {
    object_id_ = object;
  }
}
TableObject::~TableObject() {}

void TableObject::UpdateActivationPotential() {
  float dist;
  // Get object neutral position and object position 
  //   from service call potentially
  float x = pow(neutral_object_pos[0] - object_pos[0], 2);
  float y = pow(neutral_object_pos[1] - object_pos[1], 2);
  float z = pow(neutral_object_pos[2] - object_pos[2], 2);
  dist = sqrt(x + y + z);
  state_.activation_potential = 1.0f / dist;
}
void TableObject::PickAndPlace(std::string object) {
  table_setting_demo::pick_and_place msg;
  msg.request.object = object;
  if (ros::service::call("pick_and_place_object", msg)) {

  }
}
bool TableObject::Precondition() {
  // check if dynamic object
  if (dynamic_object) {
    table_setting_demo::object_request msg;
    table_setting_demo::object_position pos_msg;
    msg.request.object = object_;
    // Check if object available in scene
    if (ros::service::call("qr_get_object", msg)) {
      if (msg.response.in_scene) {
        object_id_ = msg.response.object_id;
        pos_msg.request.object_id = object_id_;
        if (ros::service::call("qr_get_object_position", pos_msg)) {
          object_pos = pos_msg.response.position;
        } else {
          LOG_INFO("SERVICE: [%s] - Not responding!", "qr_get_object_position");
        }
        return true;
      }
    }
    return false;
  } else {
    return true;
  }
}
bool TableObject::ActivationPrecondition() {
  return mut.Lock(state_.activation_potential);
}

bool TableObject::PickAndPlaceDone() {
  table_setting_demo::pick_and_place msg;
  msg.request.object = object_;
  ros::service::call("pick_and_place_check", msg);
  return msg.response.success;
}

void TableObject::Work() {
  PickAndPlace(object_id_.c_str());
  while (!PickAndPlaceDone()) {
    boost::this_thread::sleep(boost::posix_time::millisec(500));
  }
  // Check if succeeded and try again
  mut.Release();
}
float CalcPositionDistance(std::vector<float> pos_a, std::vector<float> pos_b) {
  float x = pow(pos_a[0] - pos_b[0], 2);
  float y = pow(pos_a[1] - pos_b[1], 2);
  float z = pow(pos_a[2] - pos_b[2], 2);
  return sqrt(x + y + z);
}

bool TableObject::CheckWork() {
  table_setting_demo::pick_and_place_state msg;
  table_setting_demo::pick_and_place view_msg;
  table_setting_demo::object_position pos_msg;
  float distance_thresh = 50;
  float dist;
  if(ros::service::call("pick_and_place_state", msg)) {
    if (msg.response.state == pr2::APPROACHING) {
      LOG_INFO("Approaching object: %s - Checking Object Availability",
        object_.c_str());
      // Check if the object is still in view
      // TODO: consider self intersection with objects blocking the view.
      view_msg.request.object = object_id_.c_str();
      if (true) {
      // if (ros::service::call("qr_object_inview", view_msg)) {
        LOG_INFO("OBJECT IN VIEW!!!");
        if (view_msg.response.success) {
          // Check if the object is in the same place with in reason
          pos_msg.request.object_id = object_id_;
          if (ros::service::call("qr_get_object_position", pos_msg)) {
            dist = CalcPositionDistance(pos_msg.response.position, object_pos);
            if (dist < distance_thresh) {
              return true;
            }
          } else {
            LOG_INFO("SERVICE - [%s] NOT AVAILABLE", "qr_get_object_position");
          }
        } else {
          ROS_INFO("RESETING  BEHAVIOR!!!!!!!!!!!!!!1");
          return false;
        }
      } else {
        LOG_INFO("SERVICE - [%s] NOT AVILABLE", "qr_object_inview");
      }
    }
  } else {
    LOG_INFO("SERVICE - [%s] NOT AVAILABLE", "pick_and_place_state");
  }
  return true;
}

void TableObject::UndoWork() {
  LOG_INFO("UNDOING WORK!!");
  table_setting_demo::pick_and_place_stop msg;
  ros::service::call("pick_and_place_stop", msg);
  mut.Release();
}
}  // namespace task_net