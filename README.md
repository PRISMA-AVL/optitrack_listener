# optitrack_listener
This package contains a ROS node to convert and re-publish the feedback position and orientation of a rigid body streamed from an optitrack system.

### The node is tested with ros noetic

This package works with [natnet_ros_cpp](https://github.com/PRISMA-AVL/natnet_ros_cpp.git). 
It is needed to convert the actual streamed pose from the optitrack frame to the ENU frame. 
The converted pose is then published in the "/mavros/vision_pose/pose" needed by mavros to allow switching to position flight mode.

### NB: The actual transformation considers the fixed frame defined by the optitrack calibration "CAL 2024-01-18 14.53.21 (3DErr 0.743 mm)"

## The node bb_creator just take pose of the obstacle and create a bounding box around it
### Added velocity estimation in optitrack_listener
