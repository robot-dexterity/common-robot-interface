"""Simple test script for SyncRobot class using MG400 Controller.
Ethernet TCP/IPv4: 192.168.1.10, ip: 192.168.1.6 """

import numpy as np

from cri.robot import SyncRobot
from cri.controller import MG400Controller as Controller

np.set_printoptions(precision=2, suppress=True)


def main():
    base_frame = (0, 0, 0, 0, 0, 0)  # base frame: x->front, y->left, z->up, rz->anticlockwise
    work_frame = (300, 0, 0, -180, 0, 0) 

    with SyncRobot(Controller()) as robot:

        # Set TCP, linear speed,  angular speed and coordinate frame
        robot.coord_frame = work_frame
        robot.tcp = (0, 0, -50, 0, 0, 0)  # (0, -38, 0, 0, 0, 0) # right angle mount
        robot.speed = 20

        # zero last joint
        robot.controller.tolerance = 0.1 
        robot.move_joints([*robot.joint_angles[:-1], 0])
        robot.controller.tolerance = None

        # Display robot info
        print("speed: ", robot.speed) # delayed until next run
        print("Robot info: {}".format(robot.info))

        # Display initial joint angles and inital pose in work frame
        print("Initial joint angles: {}".format(robot.joint_angles))
        print("Initial pose in work frame: {}".format(robot.pose))
        
        # Move to origin of work frame
        print("Moving to origin of work frame ...")
        robot.move_linear((0, 0, 0, 0, 0, 0))
        print("Target origin pose in work frame: {}".format(robot.target_pose))
        print("Origin pose in work frame: {}".format(robot.pose))

        # Increase and decrease all joint angles 
        print("Increasing and decreasing all joint angles ...")
        robot.move_joints(robot.joint_angles - (10,)*4)   
        print("Target joint angles: {}".format(robot.target_joint_angles))
        print("Joint angles: {}".format(robot.joint_angles))
        robot.move_joints(robot.joint_angles + (10,)*4)  
        print("Target joint angles: {}".format(robot.target_joint_angles))
        print("Joint angles: {}".format(robot.joint_angles))
        
        # Move backward and forward
        print("Moving backward and forward ...")        
        robot.move_linear((50, 0, 0, 0, 0, 0))
        robot.move_linear((0, 0, 0, 0, 0, 0))

        # Move right and left
        print("Moving right and left ...")  
        robot.move_linear((0, 50, 0, 0, 0, 0))
        robot.move_linear((0, 0, 0, 0, 0, 0))

        # Move down and up
        print("Moving down and up ...")  
        robot.move_linear((0, 0, 50, 0, 0, 0))
        robot.move_linear((0, 0, 0, 0, 0, 0))

        # Turn clockwise and anticlockwise around work frame z-axis
        print("Turning clockwise and anticlockwise around work frame z-axis ...")        
        robot.move_linear((0, 0, 0, 0, 0, 30))
        robot.move_linear((0, 0, 0, 0, 0, 0))

        # Servo control
        robot.move_linear((0, 0, 0, 0, 0, 0))
        robot.controller.servo_mode = True
        robot.controller.servo_delay = 1
        for i in range(6):
            robot.move_linear((0, - 10*(i%2), 10*i, 0, 0, 0))
        robot.controller.servo_mode = False

        # # Make a circular move down/up, via a point on the right/left
        # print("Making a circular move down and up, via a point ...")
        # robot.move_linear((0, 0, 0, 0, 0, 0))
        # robot.move_circular((0, 0, 10, 0, 0, 0), (0, 0, 0, 0, 0, 0))
        # # robot.move_circular((0, -5, 5, 0, 0, 0), (0, 0, 0, 0, 0, 0))  

        # # Move to offset pose then tap down and up in sensor frame
        # print("Moving to 50 mm/ 30 deg offset in pose ...")         
        # robot.move_linear((50, 50, 50, 0, 0, 30))
        # print("Target pose after offset move: {}".format(robot.target_pose))
        # print("Pose after offset move: {}".format(robot.pose))
        # print("Tapping down and up ...")
        # robot.coord_frame = base_frame
        # robot.coord_frame = robot.target_pose
        # robot.move_linear((0, 0, -50, 0, 0, 0))
        # robot.move_linear((0, 0, 0, 0, 0, 0))

        # Finish
        print("Moving to origin of work frame ...")
        robot.coord_frame = work_frame
        robot.move_linear((0, 0, 0, 0, 0, 0))
        print("Final target pose in work frame: {}".format(robot.target_pose))
        print("Final pose in work frame: {}".format(robot.pose))

        # zero last joint
        robot.controller.tolerance = 0.1 
        robot.move_joints([*robot.joint_angles[:-1], 0])

if __name__ == '__main__':
    main()

