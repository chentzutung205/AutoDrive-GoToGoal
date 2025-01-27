# Navigate to Goal with Obstacle Avoidance

This project is a part of the larger Autonomous Vehicle initiative. BB-8, affectionately known as "Beebee-Ate," is a lovable and intelligent droid character from the __Star Wars__ franchise. Inspired by this iconic character, our robot BB-8 is designed to navigate and interact with its environment, showcasing advanced capabilities in object tracking and autonomous navigation.

## Overview

This ROS2 package allows a robot to autonomously navigate through pre-defined waypoints, starting from a known global position (0m, 0m), while avoiding static and dynamic obstacles. The navigation is implemented using two key nodes: __getObjectRange__ and __goToGoal__. The system uses onboard odometry for position tracking, and the goal is to stop the robot within a defined radius of each waypoint.

## Dependencies
