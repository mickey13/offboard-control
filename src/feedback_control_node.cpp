#include <offboard_control/feedback_control.h>

#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "feedback_control");
  ros::NodeHandle rosNode;
  float frequency = 2.0;
  float maxDescentSpeed = -1.0;
  float maxLateralSpeed = 2.0;
  rosNode.param(ros::this_node::getName() + "/frequency", frequency, frequency);
  rosNode.param(ros::this_node::getName() + "/max_descent_speed", maxDescentSpeed, maxDescentSpeed);
  rosNode.param(ros::this_node::getName() + "/max_lateral_speed", maxLateralSpeed, maxLateralSpeed);
  ros::Rate rosRate(frequency);

  FeedbackControl feedbackControl(rosNode, maxDescentSpeed, maxLateralSpeed);

  while (rosNode.ok()) {
    ros::spinOnce();
    rosRate.sleep();
  }
  return 0;
}
