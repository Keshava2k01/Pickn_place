import time
import pybullet as p
import config

class PickPlacePlanner:
    def __init__(self, robot):
        self.robot = robot

    def wait(self, steps=240):
        for _ in range(steps):
            p.stepSimulation()
            time.sleep(1./240.)

    def move_above(self, point):
        target = [
            point[0],
            point[1],
            point[2] + config.APPROACH_HEIGHT
        ]
        self.robot.move_ee(target)
        self.wait(240)

    def descend(self, point):
        grasp_point = [
            point[0],
            point[1],
            point[2] + 0.02 #small safety offset
        ]
        self.robot.move_ee(grasp_point)
        self.wait(240)

    def lift(self, point):
        target = [
            point[0],
            point[1],
            point[2] + config.LIFT_HEIGHT
        ]
        self.robot.move_ee(target)
        self.wait(240)

    def grasp_point_world(self, point):
        #open gripper first
        self.robot.control_gripper(open=True)
        self.wait(120)

        #approach
        self.move_above(point)

        #Descend
        self.descend(point)

        #close gripper
        self.robot.control_gripper(open=False)
        self.wait(240)

        #Lift
        self.lift(point)