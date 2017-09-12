#include <offboard_control/offboard_control.h>
#include <offboard_control/State.h>

#include <sstream>

OffboardControl::OffboardControl(
  ros::NodeHandle &rosNode,
  ros::Rate rosRate,
  std::string odometryTopic,
  float takeoffHeight
) : mMavrosAdapter(rosNode, rosRate) {
  this->mRosNode = &rosNode;
  this->mTakeoffService = this->mRosNode->advertiseService("offboard_control/takeoff", &OffboardControl::takeoffService, this);
  this->mLandService = this->mRosNode->advertiseService("offboard_control/land", &OffboardControl::landService, this);
  this->mWaypointService = this->mRosNode->advertiseService("offboard_control/waypoint", &OffboardControl::waypointService, this);
  this->mTakeoffHeight = takeoffHeight;
}

void OffboardControl::initializeMavros() {
  this->mMavrosAdapter.initialize();
}

bool OffboardControl::takeoffService(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response) {
  if (this->mMavrosAdapter.arm(true)) {
    if (this->mMavrosAdapter.takeoff(this->mTakeoffHeight)) {
      std::stringstream ss;
      ss << "Taking off to " << this->mTakeoffHeight << " meters.";
      response.success = true;
      response.message = ss.str();
    }
    else {
      response.success = false;
      response.message = "Failed to initiate takeoff.";
    }
  }
  else {
    response.success = false;
    response.message = "Failed to arm.";
  }
  return true;
}

bool OffboardControl::landService(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response) {
  if (this->mMavrosAdapter.land()) {
    std::stringstream ss;
    double latitude = this->mMavrosAdapter.getGlobalPosition().latitude;
    double longitude = this->mMavrosAdapter.getGlobalPosition().longitude;
    ss << "Landing at " << latitude << " latitude, " << longitude << " longitude.";
    response.success = true;
    response.message = ss.str();
  }
  else {
    response.success = false;
    response.message = "Failed to initiate landing.";
  }
  return true;
}

bool OffboardControl::waypointService(offboard_control::Pose::Request& request, offboard_control::Pose::Response& response) {
  geometry_msgs::Pose waypoint;
  std::stringstream ss;
  waypoint.position = request.position;
  ss << "Moving to (" << waypoint.position.x << ", " << waypoint.position.y << ", " << waypoint.position.z << ").";
  this->mMavrosAdapter.waypoint(waypoint);
  response.success = true;
  response.message = ss.str();
  return true;
}
