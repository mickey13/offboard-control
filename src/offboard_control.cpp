#include <offboard_control/offboard_control.h>
#include <offboard_control/State.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/UInt16.h>
#include <sstream>

static const unsigned int CONTROL_EVENT_TAKEOFF_COMPLETE = 0;
static const unsigned int CONTROL_EVENT_LAND_COMPLETE = 1;
static const unsigned int CONTROL_EVENT_WAYPOINT_COMPLETE = 2;
static const float CLOSE_ENOUGH = 2.0;

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
  this->mVelocityService = this->mRosNode->advertiseService("offboard_control/velocity", &OffboardControl::velocityService, this);
  this->mOdometrySubscriber = this->mRosNode->subscribe<nav_msgs::Odometry>(odometryTopic, 1, &OffboardControl::odometryCallback, this);
  this->mEventPublisher = this->mRosNode->advertise<std_msgs::UInt16>("offboard_control/event", 1);
  this->mState = State::IDLE;
  this->mTakeoffHeight = takeoffHeight;
}

void OffboardControl::initializeMavros() {
  this->mMavrosAdapter.initialize();
}

bool OffboardControl::takeoffService(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response) {
  if (this->mMavrosAdapter.arm(true)) {
    if (this->mMavrosAdapter.takeoff(this->mTakeoffHeight)) {
      this->mState = State::TAKEOFF;
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
  ss << "Moving to local position (" << waypoint.position.x << ", " << waypoint.position.y << ", " << waypoint.position.z << ").";
  this->mMavrosAdapter.waypoint(waypoint);
  response.success = true;
  response.message = ss.str();
  return true;
}

bool OffboardControl::velocityService(offboard_control::Twist::Request& request, offboard_control::Twist::Response& response) {
  geometry_msgs::Twist velocity;
  std::stringstream ss;
  velocity.linear = request.linear;
  ss << "Moving with velocity (" << velocity.linear.x << ", " << velocity.linear.y << ", " << velocity.linear.z << ").";
  this->mMavrosAdapter.velocity(velocity);
  response.success = true;
  response.message = ss.str();
  return true;
}

void OffboardControl::odometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  if (this->mMavrosAdapter.isFcuArmed()) {
    this->mCurrentOdometry = *msg;
  }
  else {
    this->mInitialOdometry = *msg;
  }
  switch (this->mState) {
  case State::TAKEOFF:
    if (fabs(this->mCurrentOdometry.pose.pose.position.z - this->mInitialOdometry.pose.pose.position.z) > CLOSE_ENOUGH) {
      this->completeTakeoff();
    }
    break;
  case State::LAND:
    break;
  case State::WAYPOINT:
    break;
  case State::VELOCITY:
    break;
  default:
    break;
  }
}

void OffboardControl::completeTakeoff() {
  this->publishEvent(CONTROL_EVENT_TAKEOFF_COMPLETE);
  this->mState = State::IDLE;
}

void OffboardControl::completeLand() {
  this->publishEvent(CONTROL_EVENT_LAND_COMPLETE);
  this->mState = State::IDLE;
}

void OffboardControl::completeWaypoint() {
  this->publishEvent(CONTROL_EVENT_WAYPOINT_COMPLETE);
  this->mState = State::IDLE;
}

void OffboardControl::publishEvent(unsigned int controlEvent) const {
  std_msgs::UInt16 msg;
  msg.data = controlEvent;
  this->mEventPublisher.publish(msg);
}
