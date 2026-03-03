# Pickn_place
PyBullet Vision-Based Pick and Place (Franka Panda)

This project implements a simple vision-based pick-and-place pipeline in PyBullet using the Franka Panda robot.

The system:

Spawns multiple cubes on a table

Captures RGB-D images from an overhead camera

Detects a red cube in image space

Converts pixel coordinates to 3D world coordinates

Executes a grasp and lift using inverse kinematics
