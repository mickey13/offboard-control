#include <offboard_control/offboard_control.h>
#include <offboard_control/State.h>

#include <tf/transform_datatypes.h>
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
  if (this->mMavrosAdapter.isFcuArmed()) {
    response.success = false;
    response.message = "Takeoff command failed. FCU should not be already armed.";
  }
  else if (this->mMavrosAdapter.arm(true)) {
    this->mState = State::TAKEOFF;
    this->mLocalWaypoint = this->mInitialOdometry.pose.pose;
    this->mLocalWaypoint.position.z += this->mTakeoffHeight;
    this->mMavrosAdapter.waypoint(this->mLocalWaypoint);
    std::stringstream ss;
    ss << "Taking off to " << this->mTakeoffHeight << " meters.";
    response.success = true;
    response.message = ss.str();
  }
  else {
    response.success = false;
    response.message = "Failed to arm FCU via MAVROS.";
  }
  return true;
}

bool OffboardControl::landService(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response) {
  if (this->mMavrosAdapter.isFcuArmed()) {
    geometry_msgs::Twist velocity;
    std::stringstream ss;
    this->mState = State::LAND;
    velocity.linear.z = -1.0;
    ss << "Landing with velocity: (" << velocity.linear.x << ", " << velocity.linear.y << ", " << velocity.linear.z << "), yaw: " << velocity.angular.z << ".";
    this->mMavrosAdapter.velocity(velocity);
    response.success = true;
    response.message = ss.str();
  }
  else {
    response.success = false;
    response.message = "Land command failed. FCU should be armed.";
  }
  return true;
}

bool OffboardControl::waypointService(offboard_control::Pose::Request& request, offboard_control::Pose::Response& response) {
  std::stringstream ss;
  this->mState = State::WAYPOINT;
  this->mLocalWaypoint.position = request.position;
  this->mLocalWaypoint.orientation = tf::createQuaternionMsgFromYaw(request.yaw);
  ss << "Moving to local position: (" << this->mLocalWaypoint.position.x << ", " << this->mLocalWaypoint.position.y << ", " << this->mLocalWaypoint.position.z << "), yaw: " << request.yaw << ".";
  this->mMavrosAdapter.waypoint(this->mLocalWaypoint);
  response.success = true;
  response.message = ss.str();
  return true;
}

bool OffboardControl::velocityService(offboard_control::Twist::Request& request, offboard_control::Twist::Response& response) {
  geometry_msgs::Twist velocity;
  std::stringstream ss;
  this->mState = State::VELOCITY;
  velocity.linear = request.linear;
  velocity.angular.z = request.yaw;
  ss << "Moving with velocity: (" << velocity.linear.x << ", " << velocity.linear.y << ", " << velocity.linear.z << "), yaw: " << velocity.angular.z << ".";
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
    if (fabs(this->mCurrentOdometry.pose.pose.position.x - this->mLocalWaypoint.position.x) < CLOSE_ENOUGH &&
        fabs(this->mCurrentOdometry.pose.pose.position.y - this->mLocalWaypoint.position.y) < CLOSE_ENOUGH &&
        fabs(this->mCurrentOdometry.pose.pose.position.z - this->mLocalWaypoint.position.z) < CLOSE_ENOUGH) {
      this->completeWaypoint();
    }
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
