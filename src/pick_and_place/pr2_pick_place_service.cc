#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <arm_navigation_msgs/MoveArmAction.h>
#include <pr2_controllers_msgs/Pr2GripperCommandAction.h>
#include <tf/transform_listener.h>
#include <pr2_arm_navigation_luke/pick_and_place.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string>
#include <iostream>
#include <fstream>

//Open the gripper
void Gripper::Open(){
  pr2_controllers_msgs::Pr2GripperCommandGoal open;
  open.command.position = 0.08;
  open.command.max_effort = 80.0;  // Do not limit effort (negative)
  
  ROS_INFO("Sending open goal");
  gripper_client_->sendGoal(open);
  gripper_client_->waitForResult();
  if(gripper_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("The gripper opened!");
  else
    ROS_INFO("The gripper failed to open.");
}

//Close the gripper
void Gripper::Close(){
  pr2_controllers_msgs::Pr2GripperCommandGoal squeeze;
  squeeze.command.position = 0.0;
  squeeze.command.max_effort = 30.0;  // Close gently
  
  ROS_INFO("Sending squeeze goal");
  gripper_client_->sendGoal(squeeze);
  gripper_client_->waitForResult();
  if(gripper_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("The gripper closed!");
  else
    ROS_INFO("The gripper failed to close.");
}

PickPlace::PickPlace(std::string arm) : move_arm_("move_right_arm",true) {
  arm_ = arm;
  const char *object_str[] = {
    "neutral",
    "placemat",
    "cup",
    "plate",
    "fork",
    "spoon",
    "knife",
    "bowl",
    "soda",
    "wineglass"
  };
  objects_ = std::vector<std::string>(object_str,
    object_str + sizeof(object_str) / sizeof(char*));
  for (uint32_t i = 0; i < objects_.size(); ++i) {
    printf("Object: %s\n", objects_[i].c_str());
  }
}
bool PickPlace::PickAndPlaceObject(
    pr2_arm_navigation_luke::pick_and_place::Request &req,
    pr2_arm_navigation_luke::pick_and_place::Response &res) {
  printf("Picking up Object: %s\n", req.object.c_str());
  r_gripper_.Open();
  // Move to Neutral Start
  object_goal_map_["neutral"].pick_pose.motion_plan_request.goal_constraints.position_constraints[0].header.stamp = ros::Time::now();
  if (!SendGoal(object_goal_map_["neutral"].pick_pose)) {
    res.success = false;
    return true;
  }
  // Move to Object Pick location
  object_goal_map_[req.object.c_str()].pick_pose.motion_plan_request.goal_constraints.position_constraints[0].header.stamp = ros::Time::now();
  if (!SendGoal(object_goal_map_[req.object.c_str()].pick_pose)) {
    res.success = false;
    return true;
  }
  r_gripper_.Close();

  // Move to Neutral start
  object_goal_map_["neutral"].pick_pose.motion_plan_request.goal_constraints.position_constraints[0].header.stamp = ros::Time::now();
  if (!SendGoal(object_goal_map_["neutral"].pick_pose)) {
    res.success = false;
    return true;
  }
  object_goal_map_["neutral"].place_pose.motion_plan_request.goal_constraints.position_constraints[0].header.stamp = ros::Time::now();
  if (!SendGoal(object_goal_map_["neutral"].place_pose)) {
    res.success = false;
    return true;
  }

  // obejct place
  object_goal_map_[req.object.c_str()].place_pose.motion_plan_request.goal_constraints.position_constraints[0].header.stamp = ros::Time::now();
  if (!SendGoal(object_goal_map_[req.object.c_str()].place_pose)) {
    res.success = false;
    return true;
  }
  r_gripper_.Open();

   object_goal_map_["neutral"].place_pose.motion_plan_request.goal_constraints.position_constraints[0].header.stamp = ros::Time::now();
  if (!SendGoal(object_goal_map_["neutral"].place_pose)) {
    res.success = false;
    return true;
  }

  res.success = true;
  return true;
}

void PickPlace::PostParameters() {
  std::vector<double> positions;
  std::string topic = "/ObjectPositions/";
  for (std::map<std::string, PickPlaceGoal>::iterator it = object_goal_map_.begin();
      it != object_goal_map_.end();
      ++it) {
    positions.push_back(it->second.pick_pose.motion_plan_request.goal_constraints.position_constraints[0].position.x);
    positions.push_back(it->second.pick_pose.motion_plan_request.goal_constraints.position_constraints[0].position.y);
    positions.push_back(it->second.pick_pose.motion_plan_request.goal_constraints.position_constraints[0].position.z);
    ros::param::set((topic + it->first).c_str(), positions);
    positions.clear();
  }
}
void PickPlace::CalibrateObjects() {
  char c;
  r_gripper_.Open();
  for (uint32_t i = 0; i < objects_.size(); ++i) {
    printf(
      "Move %s limb to: %s picking location and Press Any Key and Enter\n",
      arm_.c_str(),
      objects_[i].c_str());
    std::cin >> c;
    object_goal_map_[objects_[i]].pick_pose = GetArmPoseGoal();
    r_gripper_.Close();
    printf(
      "Move %s limb to: %s Placeing location and Press Any Key and Enter\n",
      arm_.c_str(),
      objects_[i].c_str());
    std::cin >> c;
    object_goal_map_[objects_[i]].place_pose = GetArmPoseGoal();
    r_gripper_.Open();
  }
}
void PickPlace::ReadCalibration(std::string filename) {
  std::ifstream fin;
  fin.open(filename.c_str(), std::ifstream::binary);
  char header[256];
  char frame_id_str[256],link_str[256]; 
  std::string key;
  Point_t position, orientation;
  // Read header
  fin.read(header, 256);
  std::string frame_id, link;
  sscanf(header,"%s\n%s\n", frame_id_str, link_str);
  frame_id = frame_id_str;
  link = link_str;
  // read in
  while (fin.peek() != EOF) {
    fin.read(header, 128);
    key = header;
    fin.read(reinterpret_cast<char*>(&position), sizeof(Point_t));
    printf("Position: x: %f, y: %f, z: %f, \n", position.x, position.y, position.z);
    fin.read(reinterpret_cast<char*>(&orientation), sizeof(Point_t));
    printf("Orientation: x: %f, y: %f, z: %f, w: %f\n", orientation.x, orientation.y, orientation.z, orientation.w);
    object_goal_map_[key].pick_pose = GetArmPoseFromPoints(frame_id, link, position, orientation);
    fin.read(reinterpret_cast<char*>(&position), sizeof(Point_t));
    printf("Position: x: %f, y: %f, z: %f, \n", position.x, position.y, position.z);
    fin.read(reinterpret_cast<char*>(&orientation), sizeof(Point_t));
    printf("Orientation: x: %f, y: %f, z: %f, w: %f\n", orientation.x, orientation.y, orientation.z, orientation.w);
    object_goal_map_[key].place_pose = GetArmPoseFromPoints(frame_id, link, position, orientation);
  }
  fin.close();
}
arm_navigation_msgs::MoveArmGoal PickPlace::GetArmPoseFromPoints(std::string frame_id, std::string link, Point_t position, Point_t orientation) {
  arm_navigation_msgs::MoveArmGoal goal;

  goal.motion_plan_request.group_name = arm_.c_str();
  goal.motion_plan_request.num_planning_attempts = 5;
  goal.motion_plan_request.allowed_planning_time = ros::Duration(5.0);

  nh_.param<std::string>(
    "planner_id",
    goal.motion_plan_request.planner_id,
    std::string(""));
  nh_.param<std::string>(
    "planner_service_name",
    goal.planner_service_name,
    std::string("ompl_planning/plan_kinematic_path"));

  // Setup position of Joint
  goal.motion_plan_request.goal_constraints.position_constraints.resize(1);
  goal.motion_plan_request.goal_constraints.position_constraints[0].header.stamp = ros::Time::now();
  goal.motion_plan_request.goal_constraints.position_constraints[0].header.frame_id = frame_id.c_str();
  goal.motion_plan_request.goal_constraints.position_constraints[0].link_name = link.c_str();
  goal.motion_plan_request.goal_constraints.position_constraints[0].position.x = position.x;
  goal.motion_plan_request.goal_constraints.position_constraints[0].position.y = position.y;
  goal.motion_plan_request.goal_constraints.position_constraints[0].position.z = position.z;

  goal.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_shape.type = arm_navigation_msgs::Shape::BOX;
  goal.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_shape.dimensions.push_back(0.05);
  goal.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_shape.dimensions.push_back(0.05);
  goal.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_shape.dimensions.push_back(0.05);
  goal.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_orientation.w = 1.0;
  goal.motion_plan_request.goal_constraints.position_constraints[0].weight = 0.8;

  // Setup Orientation
  goal.motion_plan_request.goal_constraints.orientation_constraints.resize(1);
  goal.motion_plan_request.goal_constraints.orientation_constraints[0].header.stamp = ros::Time::now();
  goal.motion_plan_request.goal_constraints.orientation_constraints[0].header.frame_id = frame_id.c_str();
  goal.motion_plan_request.goal_constraints.orientation_constraints[0].link_name = link.c_str();
  goal.motion_plan_request.goal_constraints.orientation_constraints[0].orientation.x = orientation.x;
  goal.motion_plan_request.goal_constraints.orientation_constraints[0].orientation.y = orientation.y;
  goal.motion_plan_request.goal_constraints.orientation_constraints[0].orientation.z = orientation.z;
  goal.motion_plan_request.goal_constraints.orientation_constraints[0].orientation.w = orientation.w;
  goal.motion_plan_request.goal_constraints.orientation_constraints[0].absolute_roll_tolerance = 0.08;
  goal.motion_plan_request.goal_constraints.orientation_constraints[0].absolute_pitch_tolerance = 0.08;
  goal.motion_plan_request.goal_constraints.orientation_constraints[0].absolute_yaw_tolerance = 0.08;
  goal.motion_plan_request.goal_constraints.orientation_constraints[0].weight = 0.8;

  return goal;
}
void PickPlace::SaveCalibration(std::string filename) {
  std::ofstream fout;
  fout.open(filename.c_str(), std::ofstream::binary);
  char header[256];
  // Save links to first 
  snprintf(header, 256, "%s\n%s\n", "torso_lift_link", "r_wrist_roll_link");
  fout.write(header, 256);
  // save positions
  Point_t point;
  point.w = 1;
  for (std::map<std::string, PickPlaceGoal>::iterator it = object_goal_map_.begin();
      it != object_goal_map_.end();
      ++it) {
    snprintf(header, 128, "%s", it->first.c_str());
    fout.write(header, 128);
    printf("Key:%s\n", header);
    point.x = it->second.pick_pose.motion_plan_request.goal_constraints.position_constraints[0].position.x;
    point.y = it->second.pick_pose.motion_plan_request.goal_constraints.position_constraints[0].position.y;
    point.z = it->second.pick_pose.motion_plan_request.goal_constraints.position_constraints[0].position.z;
    printf("Position: x: %f, y: %f, z: %f\n", point.x, point.y, point.z);
    fout.write(reinterpret_cast<char*>(&point), sizeof(point));
    point.x = it->second.pick_pose.motion_plan_request.goal_constraints.orientation_constraints[0].orientation.x;
    point.y = it->second.pick_pose.motion_plan_request.goal_constraints.orientation_constraints[0].orientation.y;
    point.z = it->second.pick_pose.motion_plan_request.goal_constraints.orientation_constraints[0].orientation.z;
    point.w = it->second.pick_pose.motion_plan_request.goal_constraints.orientation_constraints[0].orientation.w;
    printf("Orientation: x: %f, y: %f, z: %f, w: %f\n", point.x, point.y, point.z, point.w);
    fout.write(reinterpret_cast<char*>(&point), sizeof(point));
    point.x = it->second.place_pose.motion_plan_request.goal_constraints.position_constraints[0].position.x;
    point.y = it->second.place_pose.motion_plan_request.goal_constraints.position_constraints[0].position.y;
    point.z = it->second.place_pose.motion_plan_request.goal_constraints.position_constraints[0].position.z;
    printf("Position: x: %f, y: %f, z: %f\n", point.x, point.y, point.z);
    fout.write(reinterpret_cast<char*>(&point), sizeof(point));
    point.x = it->second.place_pose.motion_plan_request.goal_constraints.orientation_constraints[0].orientation.x;
    point.y = it->second.place_pose.motion_plan_request.goal_constraints.orientation_constraints[0].orientation.y;
    point.z = it->second.place_pose.motion_plan_request.goal_constraints.orientation_constraints[0].orientation.z;
    point.w = it->second.place_pose.motion_plan_request.goal_constraints.orientation_constraints[0].orientation.w;
    printf("Orientation: x: %f, y: %f, z: %f, w: %f\n", point.x, point.y, point.z, point.w);
    fout.write(reinterpret_cast<char*>(&point), sizeof(point));
  }
}

bool PickPlace::SendGoal(arm_navigation_msgs::MoveArmGoal goal) {
  if (nh_.ok()) {
    bool finished_within_time = false;
    ROS_INFO("Sending Goal");
    move_arm_.sendGoal(goal);
    finished_within_time = move_arm_.waitForResult(ros::Duration(45.0));
    if (!finished_within_time) {
      move_arm_.cancelGoal();
      ROS_INFO("Timed out achieving Goal");
    } else {
      actionlib::SimpleClientGoalState state = move_arm_.getState();
      bool success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
      if (success) {
        ROS_INFO("Action finished: %s",state.toString().c_str());
        return true;
      } else {
        ROS_INFO("Action failed: %s",state.toString().c_str());
        return false;
      }
    }
  }
  return false;
}
arm_navigation_msgs::MoveArmGoal PickPlace::GetArmPoseGoal() {
  arm_navigation_msgs::MoveArmGoal goal;
  tf::TransformListener listener;
  tf::StampedTransform transform;
  // Get Current Arm Pose
  listener.waitForTransform("torso_lift_link", "r_wrist_roll_link",
    ros::Time(0), ros::Duration(3.0));
  listener.lookupTransform("torso_lift_link", "r_wrist_roll_link",
    ros::Time(0), transform);
  goal.motion_plan_request.group_name = arm_.c_str();
  goal.motion_plan_request.num_planning_attempts = 5;
  goal.motion_plan_request.allowed_planning_time = ros::Duration(5.0);

  nh_.param<std::string>(
    "planner_id",
    goal.motion_plan_request.planner_id,
    std::string(""));
  nh_.param<std::string>(
    "planner_service_name",
    goal.planner_service_name,
    std::string("ompl_planning/plan_kinematic_path"));

  // Setup position of Joint
  goal.motion_plan_request.goal_constraints.position_constraints.resize(1);
  goal.motion_plan_request.goal_constraints.position_constraints[0].header.stamp = ros::Time::now();
  goal.motion_plan_request.goal_constraints.position_constraints[0].header.frame_id = "torso_lift_link";
  goal.motion_plan_request.goal_constraints.position_constraints[0].link_name = "r_wrist_roll_link";
  goal.motion_plan_request.goal_constraints.position_constraints[0].position.x = transform.getOrigin().x();
  goal.motion_plan_request.goal_constraints.position_constraints[0].position.y = transform.getOrigin().y();
  goal.motion_plan_request.goal_constraints.position_constraints[0].position.z = transform.getOrigin().z();

  goal.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_shape.type = arm_navigation_msgs::Shape::BOX;
  goal.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_shape.dimensions.push_back(0.05);
  goal.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_shape.dimensions.push_back(0.05);
  goal.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_shape.dimensions.push_back(0.05);
  goal.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_orientation.w = 1.0;
  goal.motion_plan_request.goal_constraints.position_constraints[0].weight = 0.8;

  // Setup Orientation
  goal.motion_plan_request.goal_constraints.orientation_constraints.resize(1);
  goal.motion_plan_request.goal_constraints.orientation_constraints[0].header.stamp = ros::Time::now();
  goal.motion_plan_request.goal_constraints.orientation_constraints[0].header.frame_id = "torso_lift_link";    
  goal.motion_plan_request.goal_constraints.orientation_constraints[0].link_name = "r_wrist_roll_link";
  goal.motion_plan_request.goal_constraints.orientation_constraints[0].orientation.x = transform.getRotation().x();
  goal.motion_plan_request.goal_constraints.orientation_constraints[0].orientation.y = transform.getRotation().y();
  goal.motion_plan_request.goal_constraints.orientation_constraints[0].orientation.z = transform.getRotation().z();
  goal.motion_plan_request.goal_constraints.orientation_constraints[0].orientation.w = transform.getRotation().w();
  goal.motion_plan_request.goal_constraints.orientation_constraints[0].absolute_roll_tolerance = 0.08;
  goal.motion_plan_request.goal_constraints.orientation_constraints[0].absolute_pitch_tolerance = 0.08;
  goal.motion_plan_request.goal_constraints.orientation_constraints[0].absolute_yaw_tolerance = 0.08;
  goal.motion_plan_request.goal_constraints.orientation_constraints[0].weight = 0.8;

  return goal;
}
  ros::NodeHandle nh_;
  std::vector<std::string> objects_;
  std::string arm_;
  std::map<std::string, PickPlaceGoal> object_goal_map_;
  actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> move_arm_;
  Gripper r_gripper_;
};
