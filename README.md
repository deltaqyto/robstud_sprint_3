# ROS2 Lidar Cylinder Detection

## Overview

This ROS2 project is designed to monitor lidar scans from a TurtleBot and identify cylindrical objects. The system supports detection of a range of diameters, and defaults to 0.3m. The detected cylinders are marked and published as a PoseArray.

## Features

- Processes lidar scan data from a TurtleBot
- Supports detection of cylinders within a specified range of diameters (default: 0.3m)
- Publishes detected cylinders as a PoseArray

## Requirements

- ROS2 Humble
- OpenCV
- TurtleBot3 (or compatible robot with lidar)

## Node Details

- Subscribed Topics:
  - /scan: Lidar scan data from the TurtleBot
  - /odom: To correctly offset the markers in map frame. This is not applicable if the result is required in robot frame, and the code can be toggled.

## Configuration

You can modify the cylinder detection parameters in the source, the class contains static upper and lower limits on dimension.
