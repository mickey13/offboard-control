#include <offboard_control/mavros_adapter.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <geometry_msgs/PoseStamped.h>

static const std::string PX4_MODE_OFFBOARD = "OFFBOARD";
static const float REQUEST_INTERVAL = 2.0;

MavrosAdapter::MavrosAdapter(
  ros::NodeHandle &rosNode,
  ros::Rate rate
) : mRosRate(rate) {
  this->mRosNode = &rosNode;
  this->mArmingService = this->mRosNode->serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  this->mTakeoffService = this->mRosNode->serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/takeoff");
  this->mLandService = this->mRosNode->serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");
  this->mSetModeService = this->mRosNode->serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
  this->mStateSubscriber = this->mRosNode->subscribe<mavros_msgs::State>("mavros/state", 1, &MavrosAdapter::stateCallback, this);
  this->mLocalOdometrySubscriber = this->mRosNode->subscribe<nav_msgs::Odometry>("mavros/local_position/odom", 1, &MavrosAdapter::localOdometryCallback, this);
  this->mGlobalPositionSubscriber = this->mRosNode->subscribe<sensor_msgs::NavSatFix>("mavros/global_position/global", 1, &MavrosAdapter::globalPositionCallback, this);
  this->mWaypointPublisher = this->mRosNode->advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 1);
  this->mLastRequest = ros::Time::now();
  this->mOffboardMode = OffboardMode::NONE;
}

void MavrosAdapter::initialize() {
  this->connectToFlightController();
  this->mMavrosThread = new std::thread(&MavrosAdapter::threadLoop, this);
}

bool MavrosAdapter::arm(bool doArm) {
  mavros_msgs::CommandBool armMsg;
  armMsg.request.value = doArm;
  return this->mArmingService.call(armMsg);
}

bool MavrosAdapter::takeoff(float takeoffHeight) {
  this->mOffboardMode = OffboardMode::NONE;
  this->mOffboardWaypoint.position.z = takeoffHeight;
  mavros_msgs::CommandTOL takeoffMsg;
  takeoffMsg.request.latitude = this->mGlobalPosition.latitude;
  takeoffMsg.request.longitude = this->mGlobalPosition.longitude;
  takeoffMsg.request.altitude = this->mGlobalPosition.altitude + takeoffHeight;
  return this->mTakeoffService.call(takeoffMsg);
}

bool MavrosAdapter::land() {
  this->mOffboardMode = OffboardMode::NONE;
  mavros_msgs::CommandTOL landMsg;
  landMsg.request.latitude = this->mGlobalPosition.latitude;
  landMsg.request.longitude = this->mGlobalPosition.longitude;
  return this->mLandService.call(landMsg);
}

void MavrosAdapter::waypoint(geometry_msgs::Pose waypoint) {
  this->mOffboardWaypoint = waypoint;
  this->mOffboardMode = OffboardMode::WAYPOINT;
}

bool MavrosAdapter::isFcuConnected() const {
  return this->mFcuState.connected;
}

bool MavrosAdapter::isFcuArmed() const {
  return this->mFcuState.armed;
}

sensor_msgs::NavSatFix MavrosAdapter::getGlobalPosition() const {
  return this->mGlobalPosition;
}

void MavrosAdapter::stateCallback(const mavros_msgs::State::ConstPtr& msg) {
  this->mFcuState = *msg;
}

void MavrosAdapter::localOdometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  this->mLocalOdometry = *msg;
}

void MavrosAdapter::globalPositionCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
  this->mGlobalPosition = *msg;
}

void MavrosAdapter::publishWaypoint() const {
  geometry_msgs::PoseStamped waypointMsg;
  waypointMsg.pose = this->mOffboardWaypoint;
  this->mWaypointPublisher.publish(waypointMsg);
}

void MavrosAdapter::connectToFlightController() {
  // Before publishing anything, we wait for the connection to be established between mavros and the autopilot.
  // This loop should exit as soon as a heartbeat message is received.
  while (ros::ok() && !this->mFcuState.connected) {
    ROS_INFO("Connecting to flight control unit.");
    ros::spinOnce();
    this->mRosRate.sleep();
  }
}

void MavrosAdapter::configureOffboardMode() {
  if (this->mFcuState.mode != PX4_MODE_OFFBOARD && (ros::Time::now() - this->mLastRequest > ros::Duration(REQUEST_INTERVAL))) {
    // Before entering offboard mode, you must have already started streaming setpoints otherwise the mode switch will be rejected.
    for (int i = 50; ros::ok() && i > 0; --i) {
      this->publishWaypoint();
      ros::spinOnce();
      this->mRosRate.sleep();
    }
    mavros_msgs::SetMode setModeCommand;
    setModeCommand.request.custom_mode = PX4_MODE_OFFBOARD;
    if (this->mSetModeService.call(setModeCommand)) {
    }
    this->mLastRequest = ros::Time::now();
  }
}

void MavrosAdapter::threadLoop() {
  while (this->mRosNode->ok()) {
    switch (this->mOffboardMode) {
    case OffboardMode::WAYPOINT:
      this->configureOffboardMode();
      this->publishWaypoint();
      break;
    default:
      break;
    }
    ros::spinOnce();
    this->mRosRate.sleep();
  }
}
