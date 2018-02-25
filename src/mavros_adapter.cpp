#include <offboard_control/mavros_adapter.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

static const std::string PX4_MODE_OFFBOARD = "OFFBOARD";
static const float REQUEST_INTERVAL = 2.0;

MavrosAdapter::MavrosAdapter(
  ros::NodeHandle &rosNode,
  ros::Rate rate
) : mRosRate(rate) {
  this->mRosNode = &rosNode;
  this->mArmingService = this->mRosNode->serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  this->mSetModeService = this->mRosNode->serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
  this->mStateSubscriber = this->mRosNode->subscribe<mavros_msgs::State>("mavros/state", 1, &MavrosAdapter::stateCallback, this);
  this->mWaypointPublisher = this->mRosNode->advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 1);
  this->mVelocityPublisher = this->mRosNode->advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 1);
  this->mLastRequest = ros::Time::now();
  this->mOffboardMode = OffboardMode::WAYPOINT;
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

void MavrosAdapter::waypoint(geometry_msgs::Pose pose) {
  this->mOffboardWaypoint = pose;
  this->mOffboardMode = OffboardMode::WAYPOINT;
}

void MavrosAdapter::velocity(geometry_msgs::Twist twist) {
  this->mOffboardVelocity = twist;
  this->mOffboardMode = OffboardMode::VELOCITY;
}

bool MavrosAdapter::isFcuConnected() const {
  return this->mFcuState.connected;
}

bool MavrosAdapter::isFcuArmed() const {
  return this->mFcuState.armed;
}

void MavrosAdapter::stateCallback(const mavros_msgs::State::ConstPtr& msg) {
  this->mFcuState = *msg;
}

void MavrosAdapter::publishWaypoint() const {
  geometry_msgs::PoseStamped poseMsg;
  poseMsg.pose = this->mOffboardWaypoint;
  this->mWaypointPublisher.publish(poseMsg);
}

void MavrosAdapter::publishVelocity() const {
  geometry_msgs::TwistStamped twistMsg;
  twistMsg.twist = this->mOffboardVelocity;
  this->mVelocityPublisher.publish(twistMsg);
}

void MavrosAdapter::connectToFlightController() {
  // Before publishing anything, we wait for the connection to be established between mavros and the autopilot.
  // This loop should exit as soon as a heartbeat message is received.
  ROS_INFO("Connecting to flight control unit.");
  while (ros::ok() && !this->mFcuState.connected) {
    ros::spinOnce();
    this->mRosRate.sleep();
  }
  ROS_INFO("Connection between MAVROS and flight control unit established.");
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
      ROS_INFO_STREAM("Setting flight mode to " + PX4_MODE_OFFBOARD + ".");
    }
    this->mLastRequest = ros::Time::now();
  }
}

void MavrosAdapter::threadLoop() {
  while (this->mRosNode->ok()) {
    this->configureOffboardMode();
    switch (this->mOffboardMode) {
    case OffboardMode::WAYPOINT:
      this->publishWaypoint();
      break;
    case OffboardMode::VELOCITY:
      this->publishVelocity();
      break;
    default:
      break;
    }
    ros::spinOnce();
    this->mRosRate.sleep();
  }
}
