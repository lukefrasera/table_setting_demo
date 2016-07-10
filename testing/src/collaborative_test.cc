/*
table_setting_demo
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
#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <string>
#include <boost/thread/thread.hpp>
#include <boost/assert.hpp>
#include "robotics_task_tree_eval/behavior.h"
#include "table_setting_demo/table_object_behavior.h"
#include "log.h"

class CollaborativeTest {
 public:
  ros::NodeHandle nh_;
  boost::shared_ptr<task_net::Node> net_robot_1;
  boost::shared_ptr<task_net::Node> net_robot_2;
  std::vector<std::string> nodes;

  CollaborativeTest() : nh_("~") {
    // Get parameters for test
    if (!nh_.getParam("NodeList", nodes)) {
      LOG_INFO("Error: Node Parameter NOTFound!");
      BOOST_ASSERT(false);
    }
    for (std::vector<std::string>::iterator it = nodes.begin(); it != nodes.end(); ++it) {
      std::cout << *it << std::endl;
    }
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "collaborative_test");

  CollaborativeTest test;
  // ros::spin();
  return 0;
}
