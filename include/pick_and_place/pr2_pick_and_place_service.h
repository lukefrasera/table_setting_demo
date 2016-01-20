#ifndef PICK_AND_PLACE_PR2_PICK_AND_PLACE_SERVICE
#define PICK_AND_PLACE_PR2_PICK_AND_PLACE_SERVICE

#include "table_setting_demo/pick_and_place.h"
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <arm_navigation_msgs/MoveArmAction.h>
#include <pr2_controllers_msgs/Pr2GripperCommandAction.h>
#include <tf/transform_listener.h>
#include <table_setting_demo/pick_and_place.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string>
#include <iostream>
#include <stdint.h>
#include <fstream>

namespace pr2 {

typedef enum STATE {
  APPROACHING = 0,
  PICKED,
  PLACING,
  PLACED,
  NEUTRAL
} STATE_t;

struct PickPlaceGoal {
  arm_navigation_msgs::MoveArmGoal pick_pose;
  arm_navigation_msgs::MoveArmGoal place_pose;
};

typedef struct Point {
  double x,y,z,w;
}__attribute__((packed)) Point_t;

typedef actionlib::SimpleActionClient<pr2_controllers_msgs::Pr2GripperCommandAction> GripperClient;
typedef arm_navigation_msgs::MoveArmGoal MoveArmGoal_t;

class Gripper {
 public:
  Gripper();
  virtual ~Gripper();

  virtual void Open();
  virtual void Close();
 private:
  GripperClient *gripper_client_;
};

class PickPlace {
 public:
  PickPlace(std::string arm);
  virtual ~PickPlace();

  bool PickAndPlaceObject(
    table_setting_demo::pick_and_place::Request &req,
    table_setting_demo::pick_and_place::Response &res);
  bool PickAndPlacecheck(
    table_setting_demo::pick_and_place::Request &req,
    table_setting_demo::pick_and_place::Response &res);
  bool PickAndPlaceState(
    table_setting_demo::pick_and_place::Request &req,
    table_setting_demo::pick_and_place::Response &res);
  void PostParameters();
  void CalibrateObjects();
  void ReadCalibration(std::string filename);
  MoveArmGoal_t GetArmPoseFromPoints(
    std::string frame_id,
    std::string link,
    Point_t position,
    Point_t orientation);
  void SaveCalibration(std::string filename);

 private:
  bool SendGoal(MoveArmGoal_t goal);
  MoveArmGoal_t GetArmPoseGoal();

  ros::NodeHandle nh_;
  std::vector<std::string> objects_;
  std::string arm_;
  std::map<std::string, PickPlaceGoal> object_goal_map_;
  actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> move_arm_;
  Gripper r_gripper_;
  uint32_t state_;
};
}
#endif  // PICK_AND_PLACE_PR2_PICK_AND_PLACE_SERVICE