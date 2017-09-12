#include <offboard_control/offboard_control.h>

#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "offboard_control");
  ros::NodeHandle rosNode;
  float frequency = 30.0;
  float takeoffHeight = 10.0;
  std::string odometryTopic = "mavros/local_position/odom";
  rosNode.param(ros::this_node::getName() + "/frequency", frequency, frequency);
  rosNode.param(ros::this_node::getName() + "/takeoff_height", takeoffHeight, takeoffHeight);
  rosNode.param(ros::this_node::getName() + "/odometry_topic", odometryTopic, odometryTopic);
  ros::Rate rosRate(frequency);

  OffboardControl offboardControl(rosNode, rosRate, odometryTopic, takeoffHeight);
  offboardControl.initializeMavros();

  while (rosNode.ok()) {
    ros::spinOnce();
    rosRate.sleep();
  }
  return 0;
}
