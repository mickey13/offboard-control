#include <offboard_control/offboard_control.h>

#include <ros/ros.h>
#include <boost/algorithm/string.hpp>

int main(int argc, char** argv) {
  ros::init(argc, argv, "offboard_control");

  ros::NodeHandle rosNode;
  float rate = 30.0;
  std::string odometryTopic = "mavros/local_position/odom";
  std::string autopilot;
  rosNode.param(ros::this_node::getName() + "/rate", rate, rate);
  rosNode.param(ros::this_node::getName() + "/odometry_topic", odometryTopic, odometryTopic);
  rosNode.param(ros::this_node::getName() + "/autopilot", autopilot, autopilot);
  ros::Rate rosRate(rate);
  MavrosAdapter::Autopilot mavrosAutopilot = MavrosAdapter::Autopilot::UNDEFINED;
  boost::algorithm::to_lower(autopilot);
  if (autopilot == "px4") {
    mavrosAutopilot = MavrosAdapter::Autopilot::PX4;
  }
  else if (autopilot == "apm") {
    mavrosAutopilot = MavrosAdapter::Autopilot::APM;
  }
  else {
    ROS_ERROR("Autopilot not configured.");
    return 1;
  }
  OffboardControl offboardControl(rosNode, rosRate, odometryTopic, mavrosAutopilot);
  offboardControl.initializeMavros();

  while (rosNode.ok()) {
    ros::spinOnce();
    rosRate.sleep();
  }
  return 0;
}
