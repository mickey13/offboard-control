#ifndef GIMBAL_H
#define GIMBAL_H

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <thread>
#include <mutex>

class Gimbal {
public:
  Gimbal(ros::NodeHandle &rosNode);
  ~Gimbal();
  void initialize();
  void updateRobotPose(const geometry_msgs::Pose& pose);
  geometry_msgs::Vector3 getOrientation() const;

private:
  Gimbal();
  void gimbalCallback(const geometry_msgs::Vector3::ConstPtr& msg);
  bool configureGimbal();
  bool actuateGimbal();
  void threadLoop(ros::Rate rosRate);
  static double radiansToDegrees(double radians);

  std::thread* mGimbalThread;
  std::mutex mMutex;
  ros::NodeHandle* mNodeHandle;
  ros::Subscriber mGimbalSubscriber;
  ros::ServiceClient mCommandClient;
  geometry_msgs::Pose mRobotPose;
  geometry_msgs::Vector3 mGimbalOrientation;
};

#endif
