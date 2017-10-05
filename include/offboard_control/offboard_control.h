#ifndef OFFBOARD_CONTROL_H
#define OFFBOARD_CONTROL_H

#include <offboard_control/mavros_adapter.h>
#include <offboard_control/Pose.h>
#include <offboard_control/Twist.h>

#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <nav_msgs/Odometry.h>

class OffboardControl
{
public:
  OffboardControl(
    ros::NodeHandle &rosNode,
    ros::Rate rosRate,
    std::string odometryTopic,
    float takeoffHeight
  );
  void initializeMavros();

private:
  enum State {
    IDLE,
    TAKEOFF,
    LAND,
    WAYPOINT,
    VELOCITY
  };

  OffboardControl();
  bool takeoffService(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response);
  bool landService(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response);
  bool waypointService(offboard_control::Pose::Request& request, offboard_control::Pose::Response& response);
  bool velocityService(offboard_control::Twist::Request& request, offboard_control::Twist::Response& response);
  void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void completeTakeoff();
  void completeLand();
  void completeWaypoint();
  void publishEvent(unsigned int controlEvent) const;

  MavrosAdapter mMavrosAdapter;
  ros::NodeHandle* mRosNode;
  ros::ServiceServer mTakeoffService;
  ros::ServiceServer mLandService;
  ros::ServiceServer mWaypointService;
  ros::ServiceServer mVelocityService;
  ros::Subscriber mOdometrySubscriber;
  ros::Publisher mEventPublisher;

  State mState;
  nav_msgs::Odometry mInitialOdometry;
  nav_msgs::Odometry mCurrentOdometry;
  float mTakeoffHeight;
};

#endif
