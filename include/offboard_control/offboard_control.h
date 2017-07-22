#ifndef OFFBOARD_CONTROL_H
#define OFFBOARD_CONTROL_H

#include <offboard_control/mavros_adapter.h>

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

class OffboardControl
{
public:
  OffboardControl(
    ros::NodeHandle &rosNode,
    ros::Rate rosRate,
    std::string odometryTopic,
    MavrosAdapter::Autopilot autopilot
  );
  void initializeMavros();
  void armingEvent();
  void localPoseEvent(geometry_msgs::Pose pose);
  void controlEffortEvent(geometry_msgs::Twist twist);

private:
  OffboardControl();

  void initializeParameters();
  bool takeoffService(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response);
  bool landService(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response);
  bool rtlService(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response);
  void waypointCallback(const geometry_msgs::Pose::ConstPtr& msg);
  void velocityCallback(const geometry_msgs::Twist::ConstPtr& msg);
  void resumeMissionCallback(const std_msgs::Empty::ConstPtr& msg);
  void stopCallback(const std_msgs::Empty::ConstPtr& msg);
  void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void gripperCallback(const std_msgs::Bool::ConstPtr& msg);
  void setWaypoint(const geometry_msgs::Pose pose);
  void logMessage(std::string message, bool isError = false) const;
  geometry_msgs::Pose transformGlobalPoseToLocalPose(const geometry_msgs::Pose& pose) const;
  geometry_msgs::Pose rotateAboutZ(const geometry_msgs::Pose& pose) const;
  std::string getNamespace() const;
  void publishState() const;

  MavrosAdapter mMavrosAdapter;
  ros::ServiceServer mTakeoffService;
  ros::ServiceServer mLandService;
  ros::ServiceServer mRtlService;
  ros::Subscriber mWaypointSubscriber;
  ros::Subscriber mVelocitySubscriber;
  ros::Subscriber mResumeMissionSubscriber;
  ros::Subscriber mStopSubscriber;
  ros::Subscriber mOdometrySubscriber;
  ros::Subscriber mGripperSubscriber;
  ros::Publisher mWaypointArrivedPublisher;
  ros::Publisher mVelocityAlertPublisher;
  ros::Publisher mStatePublisher;
  ros::Publisher mLoggerPublisher;

  ros::NodeHandle* mNodeHandle;
  geometry_msgs::Pose mGlobalWaypoint;
  geometry_msgs::Pose mLocalWaypoint;
  nav_msgs::Odometry mCurrentOdometry;
  float mAngleOffset;
  bool mEnRouteToWaypoint;
  bool mReceivedOdometry;
  bool mIsSimulation;
};

#endif
