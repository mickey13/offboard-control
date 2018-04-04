#include <offboard_control/offboard_control.h>
#include <offboard_control/State.h>

#include <tf/transform_datatypes.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/UInt16.h>
#include <sstream>

static const unsigned int CONTROL_EVENT_TAKEOFF_COMPLETE = 0;
static const unsigned int CONTROL_EVENT_LAND_COMPLETE = 1;
static const unsigned int CONTROL_EVENT_WAYPOINT_COMPLETE = 2;
static const float CLOSE_ENOUGH = 1.5;

OffboardControl::OffboardControl(
  ros::NodeHandle &rosNode,
  std::string odometryTopic,
  float takeoffHeight
) : mMavrosAdapter(rosNode) {
  this->mRosNode = &rosNode;
  this->mEnableService = this->mRosNode->advertiseService("offboard_control/enable", &OffboardControl::enableService, this);
  this->mDisableService = this->mRosNode->advertiseService("offboard_control/disable", &OffboardControl::disableService, this);
  this->mTakeoffService = this->mRosNode->advertiseService("offboard_control/takeoff", &OffboardControl::takeoffService, this);
  this->mLandService = this->mRosNode->advertiseService("offboard_control/land", &OffboardControl::landService, this);
  this->mWaypointService = this->mRosNode->advertiseService("offboard_control/waypoint", &OffboardControl::waypointService, this);
  this->mVelocityService = this->mRosNode->advertiseService("offboard_control/velocity", &OffboardControl::velocityService, this);
  this->mOdometrySubscriber = this->mRosNode->subscribe<nav_msgs::Odometry>(odometryTopic, 1, &OffboardControl::odometryCallback, this);
  this->mEventPublisher = this->mRosNode->advertise<std_msgs::UInt16>("offboard_control/event", 1);
  this->mStatePublisher = this->mRosNode->advertise<offboard_control::State>("offboard_control/state", 1);
  this->mTakeoffHeight = takeoffHeight;
  this->mIsRunning = false;
  this->mMode = Mode::IDLE;
  this->mStateThread = new std::thread(&OffboardControl::threadLoop, this);
}

OffboardControl::~OffboardControl() {
  if (this->mStateThread != NULL) {
    this->mIsRunning = false;
    this->mStateThread->join();
    delete this->mStateThread;
  }
}

void OffboardControl::initializeMavros() {
  this->mMavrosAdapter.initialize();
  this->mLocalWaypoint = this->mInitialOdometry.pose.pose;
  this->mMavrosAdapter.waypoint(this->mLocalWaypoint);
}

bool OffboardControl::enableService(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response) {
  if (this->mMavrosAdapter.isEnabled()) {
    response.success = false;
    response.message = "Offboard control already enabled.";
  }
  else {
    this->mLocalWaypoint = this->mInitialOdometry.pose.pose;
    this->mMavrosAdapter.waypoint(this->mLocalWaypoint);
    this->mMavrosAdapter.setEnabled(true);
    response.success = true;
    response.message = "Offboard control is now enabled.";
  }
  return true;
}

bool OffboardControl::disableService(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response) {
  if (this->mMavrosAdapter.isEnabled()) {
    this->mMavrosAdapter.setEnabled(false);
    response.success = true;
    response.message = "Offboard control is now disabled.";
  }
  else {
    response.success = false;
    response.message = "Offboard control already disabled.";
  }
  return true;
}

bool OffboardControl::takeoffService(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response) {
  if (this->mMavrosAdapter.isFcuArmed()) {
    response.success = false;
    response.message = "Takeoff command failed. FCU should not be already armed.";
  }
  else if (this->mMavrosAdapter.arm(true)) {
    this->mMode = Mode::TAKEOFF;
    ros::Duration(1.0).sleep();
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
    this->mMode = Mode::LAND;
    velocity.linear.z = -0.5;
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
  this->mMode = Mode::WAYPOINT;
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
  this->mMode = Mode::VELOCITY;
  velocity.linear = request.linear;
  velocity.angular.z = request.yaw;
  ss << "Moving with velocity: (" << velocity.linear.x << ", " << velocity.linear.y << ", " << velocity.linear.z << "), yaw: " << velocity.angular.z << ".";
  this->mMavrosAdapter.velocity(velocity);
  response.success = true;
  response.message = ss.str();
  return true;
}

void OffboardControl::odometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  this->mCurrentOdometry = *msg;
  if (!this->mMavrosAdapter.isFcuArmed() || !this->mMavrosAdapter.isEnabled()) {
    this->mInitialOdometry = *msg;
  }
  switch (this->mMode) {
  case Mode::TAKEOFF:
    if (fabs(this->mCurrentOdometry.pose.pose.position.z - this->mInitialOdometry.pose.pose.position.z) > CLOSE_ENOUGH) {
      this->completeTakeoff();
    }
    break;
  case Mode::LAND:
    if (!this->mMavrosAdapter.isFcuArmed()) {
      this->completeLand();
    }
    break;
  case Mode::WAYPOINT:
    if (fabs(this->mCurrentOdometry.pose.pose.position.x - this->mLocalWaypoint.position.x) < CLOSE_ENOUGH &&
        fabs(this->mCurrentOdometry.pose.pose.position.y - this->mLocalWaypoint.position.y) < CLOSE_ENOUGH &&
        fabs(this->mCurrentOdometry.pose.pose.position.z - this->mLocalWaypoint.position.z) < CLOSE_ENOUGH) {
      this->completeWaypoint();
    }
    break;
  case Mode::VELOCITY:
    break;
  default:
    break;
  }
}

void OffboardControl::completeTakeoff() {
  this->publishEvent(CONTROL_EVENT_TAKEOFF_COMPLETE);
  this->mMode = Mode::IDLE;
}

void OffboardControl::completeLand() {
  this->publishEvent(CONTROL_EVENT_LAND_COMPLETE);
  this->mMode = Mode::IDLE;
}

void OffboardControl::completeWaypoint() {
  this->publishEvent(CONTROL_EVENT_WAYPOINT_COMPLETE);
  this->mMode = Mode::IDLE;
}

void OffboardControl::publishEvent(unsigned int controlEvent) const {
  std_msgs::UInt16 msg;
  msg.data = controlEvent;
  this->mEventPublisher.publish(msg);
}

void OffboardControl::publishState() const {
  offboard_control::State msg;
  msg.enabled = this->mMavrosAdapter.isEnabled();
  msg.mode = this->getStringFromEnum(this->mMode);
  this->mStatePublisher.publish(msg);
}

void OffboardControl::threadLoop() {
  ros::Rate rosRate(1.0);
  this->mIsRunning = true;
  while (this->mRosNode->ok() && this->mIsRunning) {
    this->publishState();
    ros::spinOnce();
    rosRate.sleep();
  }
}

std::string OffboardControl::getStringFromEnum(Mode mode) const {
  switch (mode) {
    case Mode::IDLE: return "IDLE";
    case Mode::TAKEOFF: return "TAKEOFF";
    case Mode::LAND: return "LAND";
    case Mode::WAYPOINT: return "WAYPOINT";
    case Mode::VELOCITY: return "VELOCITY";
    default: return "UNKNOWN";
  }
}
