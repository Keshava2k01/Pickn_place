import pybullet as p
import pybullet_data
import random
import config


def setup_scene():
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    plane_id = p.loadURDF("plane.urdf")

    p.loadURDF(
        "table/table.urdf",
        basePosition=config.TABLE_POSITION
    )

    cube_ids = []  # IMPORTANT: this must be a list

    for _ in range(config.NUM_CUBES):
        x = random.uniform(0.4, 0.6)
        y = random.uniform(-0.2, 0.2)
        z = 0.65

        cube_id = p.loadURDF(
            "cube_small.urdf",
            basePosition=[x, y, z]
        )

        # Make cube red for detection
        p.changeVisualShape(cube_id, -1,
                            rgbaColor=[1, 0, 0, 1])

        cube_ids.append(cube_id)

    return plane_id, cube_ids