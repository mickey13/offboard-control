#include <offboard_control/offboard_control.h>
#include <offboard_control/StateMsg.h>

#include <std_msgs/String.h>
#include <sstream>

static const int QUEUE_SIZE = 1;
static const std::string TOPIC_CONTROL_TAKEOFF = "offboard_control/takeoff";
static const std::string TOPIC_CONTROL_LAND = "offboard_control/land";
static const std::string TOPIC_CONTROL_RTL = "offboard_control/rtl";
static const std::string TOPIC_CONTROL_WAYPOINT = "offboard_control/waypoint";
static const std::string TOPIC_CONTROL_WAYPOINT_ARRIVED = "offboard_control/waypoint/arrived";
static const std::string TOPIC_CONTROL_VELOCITY = "offboard_control/velocity";
static const std::string TOPIC_CONTROL_VELOCITY_ALERT = "offboard_control/velocity/alert";
static const std::string TOPIC_CONTROL_RESUME_MISSION = "offboard_control/resume_mission";
static const std::string TOPIC_CONTROL_STOP = "offboard_control/stop";
static const std::string TOPIC_CONTROL_STATE = "offboard_control/state";
static const std::string TOPIC_CONTROL_GRIPPER = "offboard_control/gripper";
static const std::string TOPIC_LOGGER = "/ocs/log_message";
static const float TAKEOFF_HEIGHT = 10.0;
static const float CLOSE_ENOUGH = 2.0;
static const float VELOCITY_ALERT_HEIGHT = 3.0;
static const float PI = 3.14159265359;

OffboardControl::OffboardControl(
  ros::NodeHandle &rosNode,
  ros::Rate rosRate,
  std::string odometryTopic,
  MavrosAdapter::Autopilot autopilot
) : mMavrosAdapter(rosNode, rosRate, autopilot, TAKEOFF_HEIGHT, this, &OffboardControl::armingEvent, &OffboardControl::localPoseEvent) {
  this->mNodeHandle = &rosNode;
  this->mTakeoffService = this->mNodeHandle->advertiseService(TOPIC_CONTROL_TAKEOFF, &OffboardControl::takeoffService, this);
  this->mLandService = this->mNodeHandle->advertiseService(TOPIC_CONTROL_LAND, &OffboardControl::landService, this);
  this->mRtlService = this->mNodeHandle->advertiseService(TOPIC_CONTROL_RTL, &OffboardControl::rtlService, this);
  this->mWaypointSubscriber = this->mNodeHandle->subscribe<geometry_msgs::Pose>(TOPIC_CONTROL_WAYPOINT, QUEUE_SIZE, &OffboardControl::waypointCallback, this);
  this->mVelocitySubscriber = this->mNodeHandle->subscribe<geometry_msgs::Twist>(TOPIC_CONTROL_VELOCITY, QUEUE_SIZE, &OffboardControl::velocityCallback, this);
  this->mResumeMissionSubscriber = this->mNodeHandle->subscribe<std_msgs::Empty>(TOPIC_CONTROL_RESUME_MISSION, QUEUE_SIZE, &OffboardControl::resumeMissionCallback, this);
  this->mStopSubscriber = this->mNodeHandle->subscribe<std_msgs::Empty>(TOPIC_CONTROL_STOP, QUEUE_SIZE, &OffboardControl::stopCallback, this);
  this->mOdometrySubscriber = this->mNodeHandle->subscribe<nav_msgs::Odometry>(odometryTopic, QUEUE_SIZE, &OffboardControl::odometryCallback, this);
  this->mGripperSubscriber = this->mNodeHandle->subscribe<std_msgs::Bool>(TOPIC_CONTROL_GRIPPER, QUEUE_SIZE, &OffboardControl::gripperCallback, this);
  this->mWaypointArrivedPublisher = this->mNodeHandle->advertise<std_msgs::Empty>(TOPIC_CONTROL_WAYPOINT_ARRIVED, QUEUE_SIZE);
  this->mVelocityAlertPublisher = this->mNodeHandle->advertise<geometry_msgs::Pose>(TOPIC_CONTROL_VELOCITY_ALERT, QUEUE_SIZE);
  this->mStatePublisher = this->mNodeHandle->advertise<offboard_control::StateMsg>(TOPIC_CONTROL_STATE, QUEUE_SIZE);
  this->mLoggerPublisher = this->mNodeHandle->advertise<std_msgs::String>(TOPIC_LOGGER, QUEUE_SIZE);
  this->mEnRouteToWaypoint = false;
  this->mReceivedOdometry = false;
  this->initializeParameters();
}

void OffboardControl::initializeMavros() {
  this->mMavrosAdapter.initialize();
  this->logMessage("Initialized");
}

void OffboardControl::armingEvent() {
  this->mGlobalWaypoint = this->mCurrentOdometry.pose.pose;
  this->mGlobalWaypoint.position.z += TAKEOFF_HEIGHT;
  this->setWaypoint(this->mGlobalWaypoint);
  std::stringstream ss;
  ss << "Armed at (" << this->mCurrentOdometry.pose.pose.position.x << ", " << this->mCurrentOdometry.pose.pose.position.y << ", " << this->mCurrentOdometry.pose.pose.position.z << ")";
  this->logMessage(ss.str());
}

void OffboardControl::localPoseEvent(geometry_msgs::Pose pose) {
  this->publishState();
}

void OffboardControl::controlEffortEvent(geometry_msgs::Twist twist) {
  if (this->mMavrosAdapter.isInVelocityMode()) {
    this->mMavrosAdapter.executeMoveWithVelocity(twist);
  }
}

void OffboardControl::initializeParameters() {
  if (this->mNodeHandle->getParam("/arena/angle", this->mAngleOffset)) {
    std::stringstream ss;
    ss << "Angular offset set to \"" << this->mAngleOffset << "\".";
    this->logMessage(ss.str());
  }
  else {
    this->mAngleOffset = 0.0;
    this->logMessage("Angular offset parameter for arena not configured.", true);
  }
}

bool OffboardControl::takeoffService(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response) {
  if (this->mReceivedOdometry) {
    std::stringstream ss;
    ss << "Takeing off to " << TAKEOFF_HEIGHT << " meters.";
    response.success = true;
    response.message = ss.str();
    this->mMavrosAdapter.executeTakeoffSequence(this->mAngleOffset);
  }
  else {
    response.success = false;
    response.message = "Takeoff request denied - no odometry has been received.";
  }
  return true;
}

bool OffboardControl::landService(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response) {
  std::stringstream ss;
  ss << "Landing at (" << this->mCurrentOdometry.pose.pose.position.x << ", " << this->mCurrentOdometry.pose.pose.position.y << ")";
  response.success = true;
  response.message = ss.str();
  this->mMavrosAdapter.executeLandingSequence();
  return true;
}

bool OffboardControl::rtlService(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response) {
  response.success = true;
  response.message = "Returning to launch.";
  this->mMavrosAdapter.executeRtlSequence();
  return true;
}

void OffboardControl::waypointCallback(const geometry_msgs::Pose::ConstPtr& msg) {
  this->setWaypoint(*msg);
}

void OffboardControl::velocityCallback(const geometry_msgs::Twist::ConstPtr& msg) {
  geometry_msgs::Twist twist;
  float sinTheta = sin(this->mAngleOffset);
  float cosTheta = cos(this->mAngleOffset);
  twist.linear.x = (msg->linear.x * cosTheta) - (msg->linear.y * sinTheta);
  twist.linear.y = (msg->linear.y * cosTheta) + (msg->linear.x * sinTheta);
  twist.linear.z = msg->linear.z;
  this->mMavrosAdapter.executeMoveWithVelocity(twist);
}

void OffboardControl::resumeMissionCallback(const std_msgs::Empty::ConstPtr& msg) {
  this->mEnRouteToWaypoint = false;
  // Set waypoint to current position when returning to mission.
  this->mGlobalWaypoint = this->mCurrentOdometry.pose.pose;
  this->setWaypoint(this->mGlobalWaypoint);
  this->mMavrosAdapter.executeMissionResume();
}

void OffboardControl::stopCallback(const std_msgs::Empty::ConstPtr& msg) {
  this->mGlobalWaypoint = this->mCurrentOdometry.pose.pose;
  this->setWaypoint(this->mGlobalWaypoint);
}

void OffboardControl::odometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  this->mCurrentOdometry = *msg;
  this->mReceivedOdometry = true;
  if (this->mEnRouteToWaypoint &&
      fabs(msg->pose.pose.position.x - this->mGlobalWaypoint.position.x) < CLOSE_ENOUGH &&
      fabs(msg->pose.pose.position.y - this->mGlobalWaypoint.position.y) < CLOSE_ENOUGH &&
      fabs(msg->pose.pose.position.z - this->mGlobalWaypoint.position.z) < CLOSE_ENOUGH) {
    this->mWaypointArrivedPublisher.publish(std_msgs::Empty());
    this->mEnRouteToWaypoint = false;
  }
  else if (this->mMavrosAdapter.isInVelocityMode() && msg->pose.pose.position.z < VELOCITY_ALERT_HEIGHT) {
    this->mVelocityAlertPublisher.publish(msg->pose.pose);
  }
}

void OffboardControl::gripperCallback(const std_msgs::Bool::ConstPtr& msg) {
  this->mMavrosAdapter.enableGripper(msg->data);
  this->logMessage("Magnetic gripper: " + msg->data);
}

void OffboardControl::setWaypoint(const geometry_msgs::Pose pose) {
  this->mMavrosAdapter.executeMoveWithWaypoint(pose);
}

void OffboardControl::logMessage(std::string message, bool isError) const {
  std_msgs::String logMessage;
  if (isError) {
    logMessage.data = "[ERROR] ";
  }
  logMessage.data += "[" + this->getNamespace() + "] [OffboardControl] " + message;
  this->mLoggerPublisher.publish(logMessage);
}

std::string OffboardControl::getNamespace() const {
  std::string ns = this->mNodeHandle->getNamespace();
  ns.erase(0, 2);
  return ns;
}

void OffboardControl::publishState() const {
  offboard_control::StateMsg stateMsg;
  stateMsg.mode = this->mMavrosAdapter.getActionString();
  stateMsg.en_route_to_waypoint = this->mEnRouteToWaypoint;
  stateMsg.global_waypoint = this->mGlobalWaypoint;
  stateMsg.local_waypoint = this->mLocalWaypoint;
  this->mStatePublisher.publish(stateMsg);
}
