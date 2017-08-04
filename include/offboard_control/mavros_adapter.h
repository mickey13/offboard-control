#ifndef MAVROS_ADAPTER_H
#define MAVROS_ADAPTER_H

#include <offboard_control/landing_gear.h>

#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/RCIn.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <thread>
#include <string>

class OffboardControl;

class MavrosAdapter {
public:
  enum Autopilot {
    UNDEFINED,
    PX4,
    APM
  };

  MavrosAdapter(
    ros::NodeHandle &rosNode,
    ros::Rate rosRate,
    Autopilot autopilot,
    float takeoffHeight,
    OffboardControl* uavControl,
    void (OffboardControl::*armingCallback)(),
    void (OffboardControl::*localPoseCallback)(geometry_msgs::Pose pose)
  );
  ~MavrosAdapter();
  void initialize();
  void executeTakeoffSequence(float yaw);
  void executeLandingSequence();
  void executeRtlSequence();
  void executeMoveWithVelocity(geometry_msgs::Twist velocity);
  void executeMoveWithWaypoint(geometry_msgs::Pose waypoint);
  void executeMissionResume();
  void enableVelocityMode(bool doPublishVelocity);
  void enableGripper(bool enable);
  bool isArmed() const;
  bool isInVelocityMode() const;

private:
  enum Action {
    NONE,
    TAKEOFF,
    LAND,
    RTL,
    VELOCITY,
    WAYPOINT
  };

  MavrosAdapter();
  void configureAutopilot(Autopilot autopilot);
  void stateCallback(const mavros_msgs::State::ConstPtr& msg);
  void globalPositionCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);
  void localPositionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void rcInCallback(const mavros_msgs::RCIn::ConstPtr& msg);
  void connectToFlightController();
  void configureFlightMode(const std::string& flightMode);
  void armFlightController();
  void callTakeoffService();
  void setRcChannelStreamRate();
  void nudgeThrottle(bool enable) const;
  void releaseAllChannels() const;
  void completeTakeoff();
  void tryLandingGearRetract();
  void threadLoop(ros::Rate rosRate);
  void publishVelocity() const;
  void publishWaypoint() const;
  void logMessage(std::string message) const;
  bool isTakingOff() const;

  const float TAKEOFF_HEIGHT;

  ros::Subscriber mStateSubscriber;
  ros::Subscriber mGlobalPositionSubscriber;
  ros::Subscriber mLocalPositionSubscriber;
  ros::Subscriber mRcInSubscriber;
  ros::Publisher mWaypointPublisher;
  ros::Publisher mVelocityPublisher;
  ros::Publisher mRcOverridePublisher;
  ros::Publisher mLoggerPublisher;
  ros::ServiceClient mArmingService;
  ros::ServiceClient mTakeoffService;
  ros::ServiceClient mSetModeService;
  ros::ServiceClient mSetStreamRateService;

  LandingGear mLandingGear;
  std::thread* mMavrosThread;
  ros::NodeHandle* mNodeHandle;
  ros::Rate mRosRate;
  ros::Time mLastRequest;
  ros::Time mTakeoffStart;
  mavros_msgs::State mCurrentState;
  sensor_msgs::NavSatFix mGlobalPosition;
  geometry_msgs::Twist mVelocity;
  geometry_msgs::Pose mWaypoint;
  geometry_msgs::Pose mLocalPose;
  geometry_msgs::Pose mPreArmPose;
  Action mAction;
  bool mIsRcInterrupt;
  bool mIsTakingOff;
  bool mTakeoffServiceResponse;
  unsigned int mRcFlightModePulseValue;
  float mFixedYaw;
  int mLandingGearRetractAttempts;
  std::string mFcuModeOffboard;
  std::string mFcuModeLand;
  std::string mFcuModeRtl;
  OffboardControl* mOffboardControl;
  void (OffboardControl::*mEventCallback)();
  void (OffboardControl::*mLocalPoseCallback)(geometry_msgs::Pose pose);
};

#endif
