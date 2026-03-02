import pybullet as p
import numpy as np
import config


class OverheadCamera:
    def __init__(self):
        print("Initializing OverheadCamera...")

        self.width = config.CAMERA_WIDTH
        self.height = config.CAMERA_HEIGHT
        self.fov = config.CAMERA_FOV
        self.near = config.CAMERA_NEAR
        self.far = config.CAMERA_FAR

        self.camera_pos = config.CAMERA_POSITION
        self.target_pos = config.CAMERA_TARGET
        self.up_vector = config.CAMERA_UP

        # View matrix
        self.view_matrix = p.computeViewMatrix(
            cameraEyePosition=self.camera_pos,
            cameraTargetPosition=self.target_pos,
            cameraUpVector=self.up_vector
        )

        # Projection matrix
        self.projection_matrix = p.computeProjectionMatrixFOV(
            fov=self.fov,
            aspect=self.width / self.height,
            nearVal=self.near,
            farVal=self.far
        )

        print("Projection matrix created successfully.")

    def capture(self):
        print("Capturing image...")

        img = p.getCameraImage(
            width=self.width,
            height=self.height,
            viewMatrix=self.view_matrix,
            projectionMatrix=self.projection_matrix
        )

        rgb = np.reshape(img[2], (self.height, self.width, 4))
        depth = np.reshape(img[3], (self.height, self.width))

        return rgb[:, :, :3], depth

    def depth_buffer_to_meters(self, depth_buffer):
        near = self.near
        far = self.far

        depth = depth_buffer
        z = 2.0 * near * far / (
                far + near - (2.0 * depth - 1.0) * (far - near)
        )
        return z

    def pixel_to_world(self, u, v):
        # Convert pixel to normalized coordinates
        x = (2.0 * u / self.width) - 1.0
        y = 1.0 - (2.0 * v / self.height)

        proj = np.array(self.projection_matrix).reshape(4, 4).T
        view = np.array(self.view_matrix).reshape(4, 4).T
        inv = np.linalg.inv(proj @ view)

        clip_near = np.array([x, y, -1.0, 1.0])
        clip_far = np.array([x, y, 1.0, 1.0])

        world_near = inv @ clip_near
        world_far = inv @ clip_far

        world_near /= world_near[3]
        world_far /= world_far[3]

        result = p.rayTest(world_near[:3], world_far[:3])
        hit_position = result[0][3]

        return np.array(hit_position)