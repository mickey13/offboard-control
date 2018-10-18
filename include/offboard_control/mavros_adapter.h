#ifndef MAVROS_ADAPTER_H
#define MAVROS_ADAPTER_H

#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/RCIn.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <thread>
#include <atomic>

class OffboardControl;

class MavrosAdapter {
public:
  MavrosAdapter(ros::NodeHandle &rosNode);
  ~MavrosAdapter();
  void initialize();
  bool arm(bool doArm);
  void waypoint(geometry_msgs::Pose pose);
  void velocity(geometry_msgs::Twist twist);
  void setEnabled(bool isEnabled);
  bool isFcuConnected() const;
  bool isFcuArmed() const;
  bool isEnabled() const;

private:
  enum OffboardMode {
    WAYPOINT,
    VELOCITY
  };
  MavrosAdapter();
  void stateCallback(const mavros_msgs::State::ConstPtr& msg);
  void rcInCallback(const mavros_msgs::RCIn::ConstPtr& msg);
  void publishWaypoint() const;
  void publishVelocity() const;
  void connectToFlightController();
  void configureOffboardMode();
  void threadLoop();

  ros::NodeHandle* mRosNode;
  ros::ServiceClient mArmingService;
  ros::ServiceClient mSetModeService;
  ros::Subscriber mStateSubscriber;
  ros::Subscriber mRcInSubscriber;
  ros::Publisher mWaypointPublisher;
  ros::Publisher mVelocityPublisher;
  ros::Time mLastRequest;

  std::thread* mMavrosThread;
  std::atomic<bool> mIsRunning;
  std::atomic<bool> mIsEnabled;

  mavros_msgs::State mFcuState;
  geometry_msgs::Pose mOffboardWaypoint;
  geometry_msgs::Twist mOffboardVelocity;
  OffboardMode mOffboardMode;
  unsigned int mRcFlightModePulseValue;
  bool mIsRcInterrupt;
};

#endif
