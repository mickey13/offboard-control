# Offboard Control

Offboard control package for MAVROS compatible autopilot firmware.

## Commands

### Takeoff

    rosservice call /hexacopter/offboard_control/takeoff

### Land

    rosservice call /hexacopter/offboard_control/land

### Return to Launch

    rosservice call /hexacopter/offboard_control/rtl

### Waypoint

    rostopic pub /hexacopter/offboard_control/waypoint geometry_msgs/Pose '{ position: { x: 6.0, y: 0.0, z: 10.0 }, orientation: { x: 0.0, y: 0.0, z: 0.0, w: 0.0 } }'

### Velocity

    rostopic pub /hexacopter/offboard_control/velocity geometry_msgs/Twist '{ linear: { x: 0.0, y: 0.0, z: 0.0 }, angular: { x: 0.0, y: 0.0, z: 0.0 } }'

### State

    rostopic echo /hexacopter/offboard_control/state
