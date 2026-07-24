"""Robot controller interface/implementations provide a common, low-level
interface to various robot arms.
"""

from abc import ABC, abstractmethod


class RobotController(ABC):
    """Robot controller class provides a common interface to various robot arms.
    
    Poses and coordinate frames are specified using 3D Cartesian positions
    and quaternion rotations.  This makes it easy to perform coordinate
    transformations using quaternion operations.    
    """
    def __repr__(self):      
        return "{} ({})".format(self.__class__.__name__, self.info)

    def __str__(self):
        return self.__repr__()

    def __enter__(self):
        return self
        
    def __exit__(self, exc_type, exc_value, traceback):
        self.close()
 
    @property       
    @abstractmethod
    def info(self):
        """Returns a unique robot identifier string.
        """
        pass

    @property    
    @abstractmethod
    def tcp(self):
        """Returns the tool center point (TCP) of the robot.
        
        tool = (x, y, z, qw, qx, qy, qz)
        x, y, z specify a Cartesian position (mm)
        qw, qx, qy, qz specify a quaternion rotation
        """
        pass

    @tcp.setter
    @abstractmethod
    def tcp(self, tcp):
        """Sets the tool center point (TCP) of the robot.
        
        tcp = (x, y, z, qw, qx, qy, qz)
        x, y, z specify a Cartesian position (mm)
        qw, qx, qy, qz specify a quaternion rotation
        """
        pass

    @property
    @abstractmethod
    def linear_speed(self):
        """Returns the linear speed of the robot TCP (mm/s).
        """
        pass

    @linear_speed.setter
    @abstractmethod
    def linear_speed(self, speed):
        """Sets the linear speed of the robot TCP (mm/s).
        """
        pass

    @property
    @abstractmethod
    def angular_speed(self):
        """Returns the joint speed of the robot TCP (deg/s).
        """
        pass

    @angular_speed.setter    
    @abstractmethod
    def angular_speed(self, speed):
        """Sets the joint speed of the robot TCP (deg/s).
        """
        pass

    @property    
    @abstractmethod
    def blend_radius(self):
        """Returns the robot blend radius (mm).
        """
        pass

    @blend_radius.setter    
    @abstractmethod
    def blend_radius(self, blend_radius):
        """Sets the robot blend radius (mm).
        """
        pass

    @property
    @abstractmethod
    def joint_angles(self):
        """Returns the current joint angles.
        
        joint angles = (j0, j1, j2, j3, j4, j5, [j6])
        j0, j1, j2, j3, j4, j5, [j6] are numbered from base to end effector and are
        measured in degrees
        """
        pass

    @property
    @abstractmethod
    def commanded_joint_angles(self):
        """Returns the commanded joint angles.

        joint angles = (j0, j1, j2, j3, j4, j5, [j6])
        j0, j1, j2, j3, j4, j5, [j6] are numbered from base to end effector and are
        measured in degrees
        """
        pass

    @property    
    @abstractmethod
    def pose(self):
        """Returns the current TCP pose in the reference coordinate frame.
        
        pose = (x, y, z, qw, qx, qy, qz)
        x, y, z specify a Cartesian position (mm)
        qw, qx, qy, qz specify a quaternion rotation
        """
        pass

    @property
    @abstractmethod
    def commanded_pose(self):
        """Returns the commanded TCP pose in the reference coordinate frame.

        pose = (x, y, z, qw, qx, qy, qz)
        x, y, z specify a Cartesian position (mm)
        qw, qx, qy, qz specify a quaternion rotation
        """
        pass

    @property
    @abstractmethod
    def elbow(self):
        """Returns the current elbow angle.
        """
        pass

    @property
    @abstractmethod
    def commanded_elbow(self):
        """Returns the commanded elbow angle.
        """
        pass

    @abstractmethod
    def move_joints(self, joint_angles):
        """Executes an immediate move to the specified joint angles.
        
        joint_angles = (j0, j1, j2, j3, j4, j5)
        j0, j1, j2, j3, j4, j5 are numbered from base to end effector and are
        measured in degrees
        """
        pass

    @abstractmethod
    def move_linear(self, pose, elbow):
        """Executes a linear/cartesian move from the current base frame pose to
        the specified pose.
        
        pose = (x, y, z, qw, qx, qy, qz)
        x, y, z specify a Cartesian position (mm)
        qw, qx, qy, qz specify a quaternion rotation
        elbow = target elbow angle for 7-DOF robot arm (optional)
        """
        pass

    @abstractmethod
    def move_circular(self, via_pose, end_pose, elbow):
        """Executes a movement in a circular path from the current base frame
        pose, through via_pose, to end_pose.
        
        via_pose, end_pose = (x, y, z, qw, qx, qy, qz)
        x, y, z specify a Cartesian position (mm)
        qw, qx, qy, qz specify a quaternion rotation
        elbow = target elbow angle for 7-DOF robot arm (optional)
        """
        pass

    @abstractmethod        
    def close(self):
        """Releases any resources held by the controller (e.g., sockets).
        """
        pass


def get_controller(controller_type: str):
    """Lazy-load controller implementations to avoid optional import failures."""
    if controller_type == 'abb':
        from cri.abb.abb_controller import ABBController
        return ABBController
    elif controller_type == 'dummy':
        from cri.dummy.dummy_controller import DummyController
        return DummyController
    elif controller_type == 'magician':
        from cri.dobot.magician_controller import MagicianController
        return MagicianController
    elif controller_type == 'mg400':
        from cri.dobot.mg400_controller import MG400Controller
        return MG400Controller
    elif controller_type in ('cr', 'nova2'):
        # The Dobot Nova2 speaks the same TCP-IP protocol as the CR series, so it is driven by
        # CRController. 'nova2' is accepted as a readable alias for 'cr'.
        from cri.dobot.cr_controller import CRController
        return CRController
    elif controller_type == 'ur':
        from cri.ur.rtde_controller import RTDEController
        return RTDEController
    elif controller_type == 'sim':
        from cri.sim.sim_controller import SimController
        return SimController
    elif controller_type == 'franka':
        from cri.franka.pyfranka_controller import PyfrankaController
        return PyfrankaController
    else:
        raise ValueError(f"Unknown controller type: {controller_type}")


class _ControllerDict(dict):
    """Dictionary-like controller registry that imports implementations on demand."""
    _controller_types = {'abb', 'dummy', 'magician', 'mg400', 'cr', 'nova2', 'ur', 'sim', 'franka'}

    def __getitem__(self, key):
        return get_controller(key)

    def __contains__(self, key):
        return key in self._controller_types

    def get(self, key, default=None):
        try:
            return self[key]
        except ValueError:
            return default


Controller = _ControllerDict()
