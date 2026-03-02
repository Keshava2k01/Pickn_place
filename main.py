import pybullet as p
import pybullet_data
import time
import numpy as np

from scene import setup_scene
from robot import PandaRobot
from vision import OverheadCamera
from planner import PickPlacePlanner
import config


def detect_cube_center(rgb):

    red_mask = (
        (rgb[:, :, 0] > 150) &
        (rgb[:, :, 1] < 100) &
        (rgb[:, :, 2] < 100)
    )

    ys, xs = np.where(red_mask)

    if len(xs) == 0:
        return None

    # Image center
    cx = rgb.shape[1] // 2
    cy = rgb.shape[0] // 2

    # Choose red pixel closest to image center
    distances = (xs - cx)**2 + (ys - cy)**2
    idx = np.argmin(distances)

    u = xs[idx]
    v = ys[idx]

    print("Selected pixel:", u, v)

    return u, v


def run(gui=True):

    if gui:
        p.connect(p.GUI)
    else:
        p.connect(p.DIRECT)

    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, config.GRAVITY)

    plane_id, cube_ids = setup_scene()

    robot = PandaRobot()
    camera = OverheadCamera()
    planner = PickPlacePlanner(robot)

    # Let simulation settle
    for _ in range(240):
        p.stepSimulation()
        time.sleep(1.0 / 240.0)

    rgb, depth = camera.capture()
    result = detect_cube_center(rgb)

    if result is None:
        print("No cube detected.")
        return

    u, v = result


    world_point = camera.pixel_to_world(u, v)

    print("Detected world point:", world_point)

    # Debug: compare with true cube position
    for i, cid in enumerate(cube_ids):
        pos = p.getBasePositionAndOrientation(cid)[0]
        print(f"Cube {i} position:", pos)


    planner.grasp_point_world(world_point)

    print("Pick and lift complete.")
    while p.isConnected():
        p.stepSimulation()
        time.sleep(1.0 / 240.0)


if __name__ == "__main__":
    run(gui=True)