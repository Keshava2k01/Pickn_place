# PyBullet Vision-Based Pick and Place (Franka Panda)

This project implements a simple vision-based pick-and-place pipeline in PyBullet using the Franka Panda robot.

The system:
- Spawns multiple cubes on a table
- Captures RGB-D images from an overhead camera
- Detects a red cube in image space
- Converts pixel coordinates to 3D world coordinates
- Executes a grasp and lift using inverse kinematics

---


## Requirements

- Python 3.8+
- pybullet
- numpy

Install dependencies:

```bash
pip install pybullet numpy
```

---

## Run

```bash
python main.py
```

The GUI will open, detect a cube, and execute a grasp and lift motion.

---

## Notes

- Cube detection uses simple color thresholding.
- 3D position is computed from RGB-D camera matrices.
- Motion control uses PyBullet inverse kinematics.
- This is a minimal demonstration and does not include full motion planning or collision-aware trajectories.
