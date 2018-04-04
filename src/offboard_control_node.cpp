#include <offboard_control/offboard_control.h>

#include <ros/ros.h>
#include <csignal>

std::atomic<bool> isRunning(true);

void signalHandler(int signum) {
  ROS_INFO("Offboard control node shutting down.");
  isRunning = false;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "offboard_control");
  ros::NodeHandle rosNode;
  ros::Rate rosRate(20.0);
  float takeoffHeight = 2.5;
  std::string odometryTopic = "mavros/local_position/odom";

  rosNode.param(ros::this_node::getName() + "/takeoff_height", takeoffHeight, takeoffHeight);
  rosNode.param(ros::this_node::getName() + "/odometry_topic", odometryTopic, odometryTopic);

  std::signal(SIGINT, signalHandler);

  OffboardControl offboardControl(rosNode, odometryTopic, takeoffHeight);
  offboardControl.initializeMavros();

  while (rosNode.ok() && isRunning) {
    ros::spinOnce();
    rosRate.sleep();
  }
  return 0;
}
