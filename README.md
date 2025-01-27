# Navigate to Goal with Obstacle Avoidance

This project is a part of the larger Autonomous Vehicle initiative. BB-8, affectionately known as "Beebee-Ate," is a lovable and intelligent droid character from the __Star Wars__ franchise. Inspired by this iconic character, our robot BB-8 is designed to navigate and interact with its environment, showcasing advanced capabilities in object tracking and autonomous navigation.

## Overview

This ROS2 package allows a robot to autonomously navigate through pre-defined waypoints, starting from a known global position (0m, 0m), while avoiding static and dynamic obstacles. The navigation is implemented using two key nodes: __getObjectRange__ and __goToGoal__. The system uses onboard odometry for position tracking, and the goal is to stop the robot within a defined radius of each waypoint.

## Dependencies

- ROS2 Humble
- rclpy
- sensor_msgs
- std_msgs
- geometry_msgs

## Features

- Obstacle detection and localization using LIDAR
- Accurate navigation through pre-defined waypoints
- Controllers to adjust robot speed and orientation for obstacle avoidance
- Configurable waypoints for flexible path planning

## Nodes
**1. getObjectRange**

This node processes LIDAR data to detect the range and orientation of obstacles in the robot's local coordinate frame.

**2. goToGoal**

This node drives the robot through the sequence of waypoints. It incorporates obstacle data from getObjectRange and uses onboard odometry for accurate global positioning.

## Usage
1. Ensure all dependencies are installed.
2. Clone this package into your ROS2 workspace.
3. Build the package using `colcon build`.
4. Source your workspace.
5. Run the nodes using:
```
ros2 run bb8_chase_object getObjectRange
ros2 run bb8_chase_object goToGoal
```

## Tips
- Use the `tf2` package to transform between local and global frames if necessary.
- Debug the robot's odometry data to ensure accurate navigation.
- Visualize the navigation and obstacle detection in `RViz2` for better insights.
- Tune controllers for optimal performance (e.g., smooth trajectory and faster goal-reaching).

## Troubleshooting
- Odometry Issues: Restart the TurtleBot3 bringup process if odometry drifts or is misaligned.
- LIDAR Noise: Filter irrelevant obstacles (e.g., walls or stray objects) in the LIDAR data.
