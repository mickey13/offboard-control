# Offboard Control

Offboard control package for MAVROS compatible autopilot firmware.

## Commands

### Takeoff

    rosservice call offboard_control/takeoff

### Land

    rosservice call offboard_control/land

### Return to Launch

    rosservice call offboard_control/rtl

### Waypoint

    rostopic pub offboard_control/waypoint offboard_control/Pose '{ position: { x: 0.0, y: 10.0, z: 10.0 }, yaw: 0.0 }'

### Velocity

    rostopic pub offboard_control/velocity geometry_msgs/Twist '{ linear: { x: 0.0, y: 0.0, z: 0.0 }, angular: { x: 0.0, y: 0.0, z: 0.0 } }'

### State

    rostopic echo offboard_control/state
