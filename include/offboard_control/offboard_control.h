#ifndef OFFBOARD_CONTROL_H
#define OFFBOARD_CONTROL_H

#include <offboard_control/mavros_adapter.h>
#include <offboard_control/Pose.h>
#include <offboard_control/Twist.h>

#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <thread>
#include <atomic>

class OffboardControl {
public:
  OffboardControl(
    ros::NodeHandle &rosNode,
    ros::Rate rosRate,
    std::string odometryTopic,
    float takeoffHeight
  );
  ~OffboardControl();
  void initializeMavros();

private:
  enum Mode {
    IDLE,
    TAKEOFF,
    LAND,
    WAYPOINT,
    VELOCITY
  };

  OffboardControl();
  bool enableService(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response);
  bool disableService(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response);
  bool takeoffService(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response);
  bool landService(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response);
  bool waypointService(offboard_control::Pose::Request& request, offboard_control::Pose::Response& response);
  bool velocityService(offboard_control::Twist::Request& request, offboard_control::Twist::Response& response);
  void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void completeTakeoff();
  void completeLand();
  void completeWaypoint();
  void publishEvent(unsigned int controlEvent) const;
  void publishState() const;
  void threadLoop();
  std::string getStringFromEnum(Mode mode) const;

  MavrosAdapter mMavrosAdapter;
  ros::NodeHandle* mRosNode;
  ros::ServiceServer mTakeoffService;
  ros::ServiceServer mLandService;
  ros::ServiceServer mWaypointService;
  ros::ServiceServer mVelocityService;
  ros::ServiceServer mEnableService;
  ros::ServiceServer mDisableService;
  ros::Subscriber mOdometrySubscriber;
  ros::Publisher mEventPublisher;
  ros::Publisher mStatePublisher;

  std::thread* mStateThread;
  std::atomic<bool> mIsRunning;
  std::atomic<Mode> mMode;

  nav_msgs::Odometry mInitialOdometry;
  nav_msgs::Odometry mCurrentOdometry;
  geometry_msgs::Pose mLocalWaypoint;
  float mTakeoffHeight;
};

#endif
