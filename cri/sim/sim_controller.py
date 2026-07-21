"""Robot controller interface/implementations provide a common, low-level
interface to various robot arms.
"""

import warnings
import numpy as np

from cri.sim.sim_client import SimClient
from cri.controller import RobotController


class SimController(RobotController):
    """Simulated controller class implements common interface robot arms.

    Poses and coordinate frames are specified using 3D Cartesian positions
    and a quaternion rotations.  This format makes it easy to perform
    coordinate transformations.
    """

    def __init__(self, robot_arm):
        self._client = SimClient(robot_arm)
        try:
            self._commanded_joint_angles = None
            self._commanded_pose = None
            # TCP speeds default to None = "unset": move_linear then behaves exactly as before
            # (the arm is called without any speed argument). A backend whose arm honours a
            # velocity -- e.g. the IsaacLab CriDriveEnv, which maps it to the blocking-move
            # interpolation step -- picks these up once set.
            self._linear_speed = None
            self._angular_speed = None
        except:
            self._client.close()
            raise

    @property
    def info(self):
        """Returns a unique robot identifier string.
        """
        return "info: {}".format(self._client.get_info())

    @property
    def tcp(self):
        """Returns the tool center point (TCP) of the robot.
        """
        return None

    @tcp.setter
    def tcp(self, tcp):
        """Sets the tool center point (TCP) of the robot.
        """
        pass

    @property
    def linear_speed(self):
        """Returns the linear speed of the robot TCP (None if unset).
        """
        return self._linear_speed

    @linear_speed.setter
    def linear_speed(self, speed):
        """Sets the linear speed of the robot TCP.

        Passed to the arm's move_linear on subsequent moves. Whether it takes effect depends on
        the backend arm: the IsaacLab CriDriveEnv converts it to a per-step interpolation
        displacement (velocity = step / dt); arms that ignore speed are unaffected.
        """
        self._linear_speed = speed

    @property
    def angular_speed(self):
        """Returns the angular speed of the robot TCP (None if unset).
        """
        return self._angular_speed

    @angular_speed.setter
    def angular_speed(self, speed):
        """Sets the angular speed of the robot TCP (see linear_speed for semantics).
        """
        self._angular_speed = speed

    @property
    def speed(self):
        """Returns the speed of the robot TCP.
        """
        warnings.warn("speed property not implemented in Simulated Controller")
        return None

    @speed.setter
    def speed(self, speed):
        """Sets the speed of the robot TCP.
        """
        warnings.warn("speed property not implemented in Simulated Controller")

    @property
    def blend_radius(self):
        """Returns the robot blend radius (mm).
        """
        warnings.warn("blend_radius property not implemented in Simulated Controller")
        pass

    @blend_radius.setter
    def blend_radius(self, blend_radius):
        """Sets the robot blend radius (mm).
        """
        warnings.warn("blend_radius property not implemented in Simulated Controller")
        pass

    @property
    def joint_angles(self):
        """Returns the current joint angles.
        """
        return self._client.get_joint_angles()

    @property
    def commanded_joint_angles(self):
        """ Returns the commanded joint angles.
        """
        return self._commanded_joint_angles

    @property
    def pose(self):
        """Returns the current TCP pose.
        """
        return self._client.get_pose()

    @property
    def commanded_pose(self):
        """Returns the commanded TCP pose.
        """
        return self._commanded_pose

    @property
    def elbow(self):
        """Returns the current elbow angle.
        """
        warnings.warn("elbow property not implemented in Simulated controller")
        return None

    @property
    def commanded_elbow(self):
        """Returns the commanded elbow angle.
        """
        warnings.warn("elbow property not implemented in Simulated controller")
        return None

    def move_joints(self, joint_angles):
        """Executes an immediate move to the specified joint angles.
        """
        joint_angles = np.array(joint_angles)
        self._commanded_joint_angles = joint_angles
        self._client.move_joints(joint_angles)

    def move_linear(self, pose, elbow=None):
        """Executes a linear/cartesian move from the current base frame pose to
        the specified pose.
        """
        if elbow is not None:
            warnings.warn("elbow property not implemented in Simulated controller")
        self._commanded_pose = pose
        self._client.move_linear(pose, linear_speed=self._linear_speed, angular_speed=self._angular_speed)

    def move_circular(self, via_pose, end_pose, elbow=None):
        """Executes a movement in a circular path from the current base frame
        pose, through via_pose, to end_pose.
        """
        if elbow is not None:
            warnings.warn("elbow property not implemented in Simulated controller")
        self._commanded_pose = end_pose
        self._client.move_circular(via_pose, end_pose)

    def close(self):
        """Releases any resources held by the controller (e.g., sockets).
        """
        self._client.close()
