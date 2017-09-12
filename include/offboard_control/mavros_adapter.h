#ifndef MAVROS_ADAPTER_H
#define MAVROS_ADAPTER_H

#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <thread>

class OffboardControl;

class MavrosAdapter {
public:
  MavrosAdapter(
    ros::NodeHandle &rosNode,
    ros::Rate rosRate
  );
  void initialize();
  bool arm(bool doArm);
  bool takeoff(float takeoffHeight);
  bool land();
  void waypoint(geometry_msgs::Pose waypoint);
  bool isFcuConnected() const;
  bool isFcuArmed() const;
  sensor_msgs::NavSatFix getGlobalPosition() const;

private:
  enum OffboardMode {
    NONE,
    WAYPOINT,
    VELOCITY
  };
  MavrosAdapter();
  void stateCallback(const mavros_msgs::State::ConstPtr& msg);
  void localOdometryCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void globalPositionCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);
  void publishWaypoint() const;
  void connectToFlightController();
  void configureOffboardMode();
  void threadLoop();

  ros::NodeHandle* mRosNode;
  ros::Rate mRosRate;
  ros::ServiceClient mArmingService;
  ros::ServiceClient mTakeoffService;
  ros::ServiceClient mLandService;
  ros::ServiceClient mSetModeService;
  ros::Subscriber mStateSubscriber;
  ros::Subscriber mLocalOdometrySubscriber;
  ros::Subscriber mGlobalPositionSubscriber;
  ros::Publisher mWaypointPublisher;
  ros::Time mLastRequest;

  std::thread* mMavrosThread;
  mavros_msgs::State mFcuState;
  nav_msgs::Odometry mLocalOdometry;
  sensor_msgs::NavSatFix mGlobalPosition;
  geometry_msgs::Pose mOffboardWaypoint;
  OffboardMode mOffboardMode;
};

#endif
