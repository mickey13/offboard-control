#include <offboard_control/gimbal.h>

#include <mavros_msgs/CommandLong.h>

static const unsigned int MAV_CMD_DO_MOUNT_CONFIGURE = 204;
static const unsigned int MAV_CMD_DO_MOUNT_CONTROL = 205;
static const unsigned int MAV_MOUNT_MODE_MAVLINK_TARGETING = 2;
static const float PI = 3.14159265359;

Gimbal::Gimbal(ros::NodeHandle &rosNode) {
  this->mNodeHandle = &rosNode;
  this->mGimbalSubscriber = this->mNodeHandle->subscribe<geometry_msgs::Vector3>("offboard_control/gimbal", 1, &Gimbal::gimbalCallback, this);
  this->mCommandClient = this->mNodeHandle->serviceClient<mavros_msgs::CommandLong>("mavros/cmd/command");
  this->mGimbalThread = NULL;
}

Gimbal::~Gimbal() {
  if (this->mGimbalThread != NULL) {
    this->mGimbalThread->join();
    delete this->mGimbalThread;
  }
}

void Gimbal::initialize() {
  if (this->configureGimbal()) {
    this->mGimbalThread = new std::thread(&Gimbal::threadLoop, this, 1.0);
  }
}

void Gimbal::updateRobotPose(const geometry_msgs::Pose& pose) {
  this->mMutex.lock();
  this->mRobotPose = pose;
  this->mMutex.unlock();
}

geometry_msgs::Vector3 Gimbal::getOrientation() const {
  return this->mGimbalOrientation;
}

void Gimbal::gimbalCallback(const geometry_msgs::Vector3::ConstPtr& msg) {
  this->mGimbalOrientation = *msg;
}

bool Gimbal::configureGimbal() {
  mavros_msgs::CommandLong commandMsg;
  commandMsg.request.command = MAV_CMD_DO_MOUNT_CONFIGURE;
  commandMsg.request.param1 = MAV_MOUNT_MODE_MAVLINK_TARGETING;
  commandMsg.request.param2 = 1;
  commandMsg.request.param3 = 1;
  commandMsg.request.param4 = 1;
  return this->mCommandClient.call(commandMsg);
}

bool Gimbal::actuateGimbal() {
  this->mMutex.lock();
  geometry_msgs::Pose robotPose = this->mRobotPose;
  this->mMutex.unlock();
  mavros_msgs::CommandLong commandMsg;
  commandMsg.request.command = MAV_CMD_DO_MOUNT_CONTROL;
  commandMsg.request.param1 = this->radiansToDegrees(this->mGimbalOrientation.y);
  commandMsg.request.param2 = this->radiansToDegrees(this->mGimbalOrientation.x);
  commandMsg.request.param3 = this->radiansToDegrees(this->mGimbalOrientation.z);
  commandMsg.request.param7 = MAV_MOUNT_MODE_MAVLINK_TARGETING;
  return this->mCommandClient.call(commandMsg);
}

void Gimbal::threadLoop(ros::Rate rosRate) {
  while (this->mNodeHandle->ok()) {
    this->actuateGimbal();
    ros::spinOnce();
    rosRate.sleep();
  }
}

double Gimbal::radiansToDegrees(double radians) {
  return 180.0 * radians / PI;
}
