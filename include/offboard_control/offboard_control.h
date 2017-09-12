#ifndef OFFBOARD_CONTROL_H
#define OFFBOARD_CONTROL_H

#include <offboard_control/mavros_adapter.h>
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
  void initializeMavros();

private:
  OffboardControl();
  bool takeoffService(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response);
  bool landService(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response);
  bool waypointService(offboard_control::Pose::Request& request, offboard_control::Pose::Response& response);

  MavrosAdapter mMavrosAdapter;
  ros::NodeHandle* mRosNode;
  ros::ServiceServer mTakeoffService;
  ros::ServiceServer mLandService;
  ros::ServiceServer mWaypointService;
  float mTakeoffHeight;
};

#endif
