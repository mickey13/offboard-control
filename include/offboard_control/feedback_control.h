#ifndef FEEDBACK_CONTROL_H
#define FEEDBACK_CONTROL_H

#include <offboard_control/Twist.h>

#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <geometry_msgs/Pose.h>

class FeedbackControl {
public:
  FeedbackControl(
    ros::NodeHandle &rosNode,
    double maxDescentSpeed,
    double maxLateralSpeed
  );

private:
  FeedbackControl();
  bool enableService(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response);
  void feedbackCallback(const geometry_msgs::Pose::ConstPtr& msg);
  offboard_control::Twist calculateVelocity(const geometry_msgs::Pose& pose) const;
  double capSpeed(double speed) const;

  ros::NodeHandle* mRosNode;
  ros::ServiceServer mEnableService;
  ros::ServiceClient mVelocityService;
  ros::Subscriber mFeedbackSubscriber;

  bool mIsEnabled;
  double mMaxDescentSpeed;
  double mMaxLateralSpeed;
};

#endif
