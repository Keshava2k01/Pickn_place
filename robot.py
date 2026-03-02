import pybullet as p
import pybullet_data
import numpy as np

class PandaRobot:
    def __init__(self):
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        self.robot_id = p.loadURDF(
            "franka_panda/panda.urdf",
            basePosition=[0.3, 0, 0.62],
            useFixedBase=True
        )

        self.ee_link = 8
        self.gripper_joints = [9, 10]

    def move_ee(self, target_pos):

        joint_poses = p.calculateInverseKinematics(
            bodyUniqueId=self.robot_id,
            endEffectorLinkIndex=self.ee_link,
            targetPosition=target_pos,
            targetOrientation=p.getQuaternionFromEuler([3.14, 0, 0]),
            lowerLimits=[-2.9] * 7,
            upperLimits=[2.9] * 7,
            jointRanges=[5.8] * 7,
            restPoses=[0, -0.5, 0, -2.0, 0, 2.0, 0.8],
            maxNumIterations=200,
            residualThreshold=1e-4
        )
        # Apply ONLY arm joints (first 7)
        for i in range(7):
            p.setJointMotorControl2(
                bodyUniqueId=self.robot_id,
                jointIndex=i,
                controlMode=p.POSITION_CONTROL,
                targetPosition=joint_poses[i],
                force=500
            )

        # Let simulation settle
        for _ in range(240):
            p.stepSimulation()

    def control_gripper(self, open=True):
        position = 0.04 if open else 0.0

        for joint in self.gripper_joints:
            p.setJointMotorControl2(
                bodyUniqueId=self.robot_id,
                jointIndex=joint,
                controlMode=p.POSITION_CONTROL,
                targetPosition=position,
                force=100
            )