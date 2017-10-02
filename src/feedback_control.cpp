#include <offboard_control/feedback_control.h>

#include <sstream>
#include <algorithm>

FeedbackControl::FeedbackControl(
  ros::NodeHandle &rosNode,
  double maxDescentSpeed,
  double maxLateralSpeed
) {
  this->mRosNode = &rosNode;
  this->mEnableService = this->mRosNode->advertiseService("feedback_control/enable", &FeedbackControl::enableService, this);
  this->mVelocityService = this->mRosNode->serviceClient<offboard_control::Twist>("offboard_control/velocity");
  this->mFeedbackSubscriber = this->mRosNode->subscribe<geometry_msgs::Pose>("feedback_control/feedback", 1, &FeedbackControl::feedbackCallback, this);
  this->mIsEnabled = false;
  this->mMaxDescentSpeed = -1.0 * fabs(maxDescentSpeed);
  this->mMaxLateralSpeed = fabs(maxLateralSpeed);
}

bool FeedbackControl::enableService(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response) {
  this->mIsEnabled = request.data;
  std::stringstream ss;
  ss << "Feedback control has been " << (this->mIsEnabled ? "enabled" : "disabled") << ".";
  response.success = true;
  response.message = ss.str();
  return true;
}

void FeedbackControl::feedbackCallback(const geometry_msgs::Pose::ConstPtr& msg) {
  if (this->mIsEnabled) {
    offboard_control::Twist twist = this->calculateVelocity(*msg);
    this->mVelocityService.call(twist);
    ROS_INFO_STREAM(twist.response.message);
  }
  else {
    ROS_WARN("Feedback control is disabled.");
  }
}

offboard_control::Twist FeedbackControl::calculateVelocity(const geometry_msgs::Pose& pose) const {
  offboard_control::Twist twist;
  double descent = (-1.0 * fabs(pose.position.z) / 4.0) - 0.1;
  twist.request.linear.x = std::min(pose.position.x, this->mMaxLateralSpeed);
  twist.request.linear.y = std::min(pose.position.y, this->mMaxLateralSpeed);
  twist.request.linear.z = std::max(descent, this->mMaxDescentSpeed);
  return twist;
}
