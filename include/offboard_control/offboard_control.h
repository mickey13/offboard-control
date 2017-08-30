#ifndef OFFBOARD_CONTROL_H
#define OFFBOARD_CONTROL_H

#include <offboard_control/mavros_adapter.h>
#include <offboard_control/gimbal.h>
#include <offboard_control/Pose.h>

#include <ros/ros.h>
#include <std_srvs/Trigger.h>

class OffboardControl
{
public:
  OffboardControl(
    ros::NodeHandle &rosNode,
    ros::Rate rosRate,
    std::string odometryTopic,
    float takeoffHeight
  );

private:
  OffboardControl();
  bool takeoffService(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response);
  bool landService(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response);

  MavrosAdapter mMavrosAdapter;
  ros::NodeHandle* mRosNode;
  ros::ServiceServer mTakeoffService;
  ros::ServiceServer mLandService;
  ros::ServiceServer mRtlService;
  float mTakeoffHeight;
};


// #include <ros/ros.h>
// #include <std_msgs/Empty.h>
// #include <std_msgs/Bool.h>
// #include <std_srvs/Trigger.h>
// #include <geometry_msgs/Pose.h>
// #include <geometry_msgs/Twist.h>
// #include <nav_msgs/Odometry.h>
//
// class OffboardControl
// {
// public:
//   OffboardControl(
//     ros::NodeHandle &rosNode,
//     ros::Rate rosRate,
//     std::string odometryTopic,
//     MavrosAdapter::Autopilot autopilot
//   );
//   void initializeMavros();
//   void armingEvent();
//   void localPoseEvent(geometry_msgs::Pose pose);
//   void controlEffortEvent(geometry_msgs::Twist twist);
//
// private:
//   enum State {
//     IDLE,
//     TAKEOFF,
//     LAND,
//     RTL,
//     WAYPOINT,
//     VELOCITY
//   };
//
//   OffboardControl();
//   void initializeParameters();
//   bool takeoffService(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response);
//   bool landService(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response);
//   bool rtlService(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response);
//   void waypointCallback(const offboard_control::Pose::ConstPtr& msg);
//   void velocityCallback(const geometry_msgs::Twist::ConstPtr& msg);
//   void resumeMissionCallback(const std_msgs::Empty::ConstPtr& msg);
//   void stopCallback(const std_msgs::Empty::ConstPtr& msg);
//   void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg);
//   void gripperCallback(const std_msgs::Bool::ConstPtr& msg);
//   void completeTakeoff();
//   void completeWaypoint();
//   void setWaypoint(const geometry_msgs::Pose pose);
//   void logMessage(std::string message, bool isError = false) const;
//   geometry_msgs::Pose transformGlobalPoseToLocalPose(const geometry_msgs::Pose& pose) const;
//   geometry_msgs::Pose rotateAboutZ(const geometry_msgs::Pose& pose) const;
//   std::string getNamespace() const;
//   std::string getStateString() const;
//   void publishState() const;
//
//   MavrosAdapter mMavrosAdapter;
//   Gimbal mGimbal;
//   ros::ServiceServer mTakeoffService;
//   ros::ServiceServer mLandService;
//   ros::ServiceServer mRtlService;
//   ros::Subscriber mWaypointSubscriber;
//   ros::Subscriber mVelocitySubscriber;
//   ros::Subscriber mResumeMissionSubscriber;
//   ros::Subscriber mStopSubscriber;
//   ros::Subscriber mOdometrySubscriber;
//   ros::Subscriber mGripperSubscriber;
//   ros::Publisher mEventPublisher;
//   ros::Publisher mVelocityAlertPublisher;
//   ros::Publisher mStatePublisher;
//   ros::Publisher mLoggerPublisher;
//
//   ros::NodeHandle* mNodeHandle;
//   geometry_msgs::Pose mLocalWaypoint;
//   nav_msgs::Odometry mCurrentOdometry;
//   State mState;
//   float mAngleOffset;
//   bool mReceivedOdometry;
// };

#endif
