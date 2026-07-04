"""Simple test script for AsyncRobot class using SimController.
"""
import numpy as np

from cri.robot import SyncRobot
from cri.controller import SimController
from tactile_sim.pybullet_env import pybullet_env

np.set_printoptions(precision=2, suppress=True)


def main():
    work_frame = (350, 0, 200, -180, 0, 0)  # x->back,  y->left,  z->down, rz->clockwise

    embodiment = pybullet_env(arm_type='mg400')
    with SyncRobot(SimController(embodiment.arm)) as robot:

        # Set TCP and coordinate frame
        robot.coord_frame = work_frame
        robot.tcp = (0, 0, 0, 0, 0, 0)

        # Display robot info
        print("Robot info: {}".format(robot.info))

        # Display initial joint angles
        print("Initial joint angles: {}".format(robot.joint_angles))

        # Display initial pose in work frame
        print("Initial pose in work frame: {}".format(robot.pose))

        # Move to origin of work frame
        print("Moving to origin of work frame ...")
        robot.move_linear((0, 0, 0, 0, 0, 0))
        print("Target origin pose in work frame: {}".format(robot.target_pose))
        print("Origin pose in work frame: {}".format(robot.pose))

        # Move backward and forward
        print("Moving backward and forward ...")
        robot.move_linear((50, 0, 0, 0, 0, 0)) 
        print("Target pose in work frame: {}".format(robot.target_pose))
        robot.move_linear((0, 0, 0, 0, 0, 0))

        # Move right and left
        print("Moving right and left ...")
        robot.move_linear((0, 50, 0, 0, 0, 0))
        print("Target pose in work frame: {}".format(robot.target_pose))
        robot.move_linear((0, 0, 0, 0, 0, 0))

        # Move down and up
        print("Moving down and up ...")
        robot.move_linear((0, 0, 50, 0, 0, 0))
        print("Target pose in work frame: {}".format(robot.target_pose))
        robot.move_linear((0, 0, 0, 0, 0, 0))

        # Turn clockwise and anticlockwise around work frame z-axis
        print("Turning clockwise and anticlockwise around work frame z-axis ...")
        robot.move_linear((0, 0, 0, 0, 0, 90))
        print("Target pose in work frame: {}".format(robot.target_pose))
        robot.move_linear((0, 0, 0, 0, 0, 0))

        # Finish
        print("Moving to origin of work frame ...")
        robot.coord_frame = work_frame
        robot.move_linear((0, 0, 0, 0, 0, 0))
        print("Final target pose in work frame: {}".format(robot.target_pose))
        print("Final pose in work frame: {}".format(robot.pose))


if __name__ == '__main__':
    main()
