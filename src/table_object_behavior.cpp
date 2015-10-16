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
#include "table_setting/table_object.h"
#include "baxter_interface/VelocityJointTrajectoryActionServerConfig.h"
#include "baxter_interface/PositionJointTrajectoryActionServerConfig.h"
#include "baxter_interface/GripperActionServerConfig.h"
#include "baxter_core_msgs/SolvePositionIK.h"
#include "baxter_core_msgs/SolvePositionIKRequest.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/JointState.h"
#include "baxter_demos/pick_and_place.h"

namespace task_net {
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
    }
TableObject::~TableObject() {}

void TableObject::UpdateActivationPotential() {
  float dist;
  float x = pow(neutral_object_pos[0] - object_pos[0], 2);
  float y = pow(neutral_object_pos[1] - object_pos[1], 2);
  float z = pow(neutral_object_pos[2] - object_pos[2], 2);
  dist = sqrt(x + y + z);
  state_.activation_potential = 1.0f / dist;
}
void TableObject::PickAndPlace(std::string object) {
  baxter_demos::pick_and_place msg;
  msg.request.object = object;
  if (ros::service::call("pick_and_place_object", msg)) {

  }
}
bool TableObject::Precondition() {
  return true;
}
bool TableObject::ActivationPrecondition() {
  return mut.Lock(state_.activation_potential);
}
void TableObject::Work() {
  PickAndPlace(object_.c_str());
  mut.Release();
}
}