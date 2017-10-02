# Offboard Control

Offboard control package for PX4 flight stack using MAVROS.

## Commands

### Takeoff

    rosservice call offboard_control/takeoff "{}"

### Land

    rosservice call offboard_control/land "{}"

### Waypoint

    rosservice call offboard_control/waypoint "{ position: { x: 0.0, y: 0.0, z: 0.0 }, yaw: 0.0 }"

### Velocity

    rosservice call offboard_control/velocity "{ linear: { x: 0.0, y: 0.0, z: 0.0 }, yaw: 0.0 }"

### Feedback Control

    rosservice call feedback_control/enable "data: true"
    rostopic pub feedback_control/feedback geometry_msgs/Pose "{ position: { x: 0.0, y: 0.0, z: 0.0 }, orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 } }"
    rosservice call feedback_control/enable "data: false"

<!-- ### Gimbal

    rostopic pub offboard_control/gimbal geometry_msgs/Vector3 '{ x: 0.0, y: 0.0, z: 0.0 }'

### State

    rostopic echo offboard_control/state -->
