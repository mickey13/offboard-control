#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "offboard_control");
  ros::NodeHandle rosNode;
  ros::Rate rosRate(30.0);
  while (rosNode.ok()) {
    ros::spinOnce();
    rosRate.sleep();
  }
  return 0;
}
