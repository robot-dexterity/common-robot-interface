"""Python client interface for Dobot MG400 using sockets
Firmware version 1.7.0.3
Uses https://github.com/Dobot-Arm/TCP-IP-4Axis-Python
API modified to have ServoP command
"""

import threading
import numpy as np
from time import sleep

from cri.dobot.mg400.dobot_api import DobotApiDashboard, DobotApi, DobotApiMove, MyType
from cri.transforms import euler2quat, quat2euler


class MG400Client:
    """Python client interface for Dobot MG400 """

    class CommandFailed(RuntimeError):
        pass

    class InvalidZone(ValueError):
        pass

    def __init__(self, ip="192.168.1.6"):
        self.set_units('millimeters', 'degrees')     
        self.joint_angles_actual, self.pose_actual, self.speed_actual = None, None, None
        self.connect(ip)

    def __repr__(self):
        return "{} (<{}>)".format(self.__class__.__name__, self.get_info())

    def __str__(self):
        return self.__repr__()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()        

    def _wait_pose(self, pose, tol=0.1):
        """Waits for the robot to reach the specified pose within a tolerance."""
        while True:
            self._globalLock.acquire()
            if self.pose_actual is not None:
                if np.all(np.abs(self.pose_actual[:4] - pose[:4]) <= tol):
                    self._globalLock.release()
                    return
            self._globalLock.release()
            sleep(0.001)

    def _wait_joint_angles(self, joint_angles, tol=0.1):
        """Waits for the robot to reach the specified joint angles within a tolerance."""
        while True:
            self._globalLock.acquire()
            if self.joint_angles_actual is not None:
                if np.all(np.abs(self.joint_angles_actual[:4] - joint_angles[:4]) <= tol):
                    self._globalLock.release()
                    return
            self._globalLock.release()
            sleep(0.001)
        
    def set_units(self, linear, angular):
        """Sets linear and angular units """
        units_l = {'millimeters' : 1.0, 'meters' : 1000.0, 'inches' : 25.4}
        units_a = {'degrees' : 1.0, 'radians' : 57.2957795}
        self._scale_linear = units_l[linear]
        self._scale_angle = units_a[angular]

    def connect(self, ip):
        """Connects to Dobot MG400 """
        self._dashboard = DobotApiDashboard(ip, 29999)
        self._move = DobotApiMove(ip, 30003)
        self._feed = DobotApi(ip, 30004)

        self._dashboard.ClearError()
        self._dashboard.EnableRobot()
        self._dashboard.User(0)
        self._dashboard.Tool(0)

        feed_thread = threading.Thread(target=self.get_feed, args=(self._feed,))
        feed_thread.setDaemon(True)
        feed_thread.start()
        self._globalLock = threading.Lock()

    def get_feed(self, feed: DobotApi):
        hasRead = 0
        while True:
            data = bytes()
            while hasRead < 1440:
                temp = feed.socket_dobot.recv(1440 - hasRead)
                if len(temp) > 0:
                    hasRead += len(temp)
                    data += temp
            hasRead = 0
            feedInfo = np.frombuffer(data, dtype=MyType)
            if hex((feedInfo['test_value'][0])) == '0x123456789abcdef':
                self._globalLock.acquire()
                self.joint_angles_actual = feedInfo["q_actual"][0]
                self.pose_actual = feedInfo["tool_vector_actual"][0]
                self.speed_actual = feedInfo["speed_scaling"][0]
                self._globalLock.release()
            sleep(0.001)

    def get_info(self):
        """Returns a unique robot identifier string """
        pass
    
    def set_servo_delay(self, servo_delay):
        """Sets time delay for servo move commands """
        if servo_delay is not None:
            if isinstance(servo_delay, (float, int)):
                servo_delay = (servo_delay, 0.02)
        self._servo_delay = servo_delay

    def get_servo_delay(self):
        """Gets time delay for servo move commands """
        return self._servo_delay

    def set_tolerance(self, tolerance):
        """Sets the tolerance for move commands """
        self._tolerance = tolerance

    def get_tolerance(self):
        """Gets the tolerance for move commands """
        return self._tolerance

    def set_linear_speed(self, linear_speed):
        """Sets the linear speed (% of maximum) """
        if linear_speed < 1 or linear_speed > 100: 
            raise Exception("Linear speed value outside range of 1-100%")
        self._dashboard.SpeedL(round(linear_speed))
        self._move.Sync()

    def set_angular_speed(self, angular_speed):
        """Sets the angular speed (% of maximum) """
        if angular_speed < 1 or angular_speed > 100: 
            raise Exception("Angular speed value outside range of 1-100%")
        self._dashboard.SpeedJ(round(angular_speed))
        self._move.Sync()
   
    def set_speed(self, speed):
        """Sets the angular speed (% of maximum) """
        if speed < 1 or speed > 100: 
            raise Exception("Speed value outside range of 1-100%")
        self._dashboard.SpeedFactor(round(speed))
        self._move.Sync()

    def get_speed(self):
        """Gets the speed (% of maximum) """
        return self.speed_actual

    def get_joint_angles(self):
        """returns the robot joint angles.
        joint_angles = (j0, j1, j2, rz)
        j0, j1, j2, rz numbered from base to end effector(default degrees) """
        return self.joint_angles_actual[:4] / self._scale_angle

    def get_pose(self):
        """returns the TCP pose in the reference coordinate frame.
        pose = (x, y, z, 0, 0, rz)
        x, y, z specify a Euclidean position (default mm)
        rz rotation of end effector (default degrees) """
        pose = self.pose_actual
        pose = np.array([*pose[:3], 0, 0, pose[3]], dtype=np.float32)
        pose = euler2quat(pose, 'sxyz')
        pose[:3] /= self._scale_linear        
        pose[3:] /= self._scale_angle        
        return pose

    def move_joints(self, joint_angles):
        """Executes an immediate move to the specified joint angles.
        joint_angles = (j0, j1, j2, rz)
        j0, j1, j2, rz numbered from base to end effector (default degres) """       
        joint_angles = np.array(joint_angles, dtype=np.float32).ravel()
        joint_angles *= self._scale_angle 
        if self._servo_delay is not None:
            self._move.ServoJ(*joint_angles, self._servo_delay[0] + self._servo_delay[1])
            sleep(self._servo_delay[0])
        else:
            self._move.JointMovJ(*joint_angles)
            self._wait_joint_angles(joint_angles, self._tolerance) if self._tolerance else self._move.Sync()

    def move_linear(self, pose):
        """Executes a linear/cartesian move from current base frame pose to specified pose.
        pose = (x, y, z, 0, 0, rz)
        x, y, z specify a Euclidean position (default mm)
        rz rotation of end effector (default degrees) """
        pose = np.array(pose, dtype=np.float32).ravel()
        pose = quat2euler(pose, 'sxyz')[[0, 1, 2, 5]]
        pose[:3] *= self._scale_linear
        pose[3] *= self._scale_angle
        pose = self.unwrap_pose_angle(pose)
        if self._servo_delay is not None:
            self._move.ServoP(*pose, self._servo_delay[0] + self._servo_delay[1])
            sleep(self._servo_delay[0])
        else: 
            self._move.MovL(*pose)
            self._wait_pose(pose, self._tolerance) if self._tolerance else self._move.Sync()

    def move_circular(self, via_pose, end_pose):
        """Executes a movement in a circular path from current base frame pose, through via_pose, to end_pose.
        via_pose, end_pose = (x, y, z, qw, qx, qy, qz)
        x, y, z specify a Cartesian position (default mm)
        qw, qx, qy, qz specify a quaternion rotation
        """     
        via_pose = np.array(via_pose, dtype=np.float32).ravel()
        via_pose = quat2euler(via_pose, 'sxyz')[[0, 1, 2, 5]]
        via_pose[:3] *= self._scale_linear
        via_pose[3] *= self._scale_angle
        end_pose = np.array(end_pose, dtype=np.float32).ravel()
        end_pose = quat2euler(end_pose, 'sxyz')[[0, 1, 2, 5]]
        end_pose[:3] *= self._scale_linear   
        end_pose[3] *= self._scale_angle    
        self._move.Circle(*via_pose, *end_pose, 1) # doesn't work
        self.wait_pose(end_pose)   

    def unwrap_pose_angle(self, pose):
        # Ensure angle is continuous with previous value to avoid jumps (e.g. wrapping)
        if not hasattr(self, '_pose_prev'):
            self._pose_prev = pose[-1]
        pose[-1] = min((pose[-1] + d for d in (-360, 0, 360)), key=lambda x: abs(x - self._pose_prev))
        self._pose_prev = pose[-1]
        return pose

    def close(self):
        """Releases any resources held by the controller (e.g., sockets). Disconnects Dobot """
        self._move.Sync()
        self._dashboard.ClearError()
        self._dashboard.DisableRobot()
        self._dashboard.close()
        print("Disconnecting Dobot...")
