import importlib
import sys
import unittest
from unittest.mock import patch


class ControllerRegistryTests(unittest.TestCase):
    def test_controller_import_does_not_eagerly_import_backends(self):
        sys.modules.pop("cri.controller", None)
        for module_name in (
            "cri.abb.abb_controller",
            "cri.dobot.mg400_controller",
            "cri.franka.pyfranka_controller",
            "cri.sim.sim_controller",
        ):
            sys.modules.pop(module_name, None)

        controller_module = importlib.import_module("cri.controller")

        self.assertIn("sim", controller_module.Controller)
        self.assertNotIn("cri.franka.pyfranka_controller", sys.modules)
        self.assertEqual(controller_module.Controller.get("missing", "fallback"), "fallback")


class FakeMG400Client:
    def __init__(self, ip):
        self.ip = ip
        self.servo_delay = "unset"
        self.tolerance = "unset"
        self.linear_speed = None
        self.closed = False

    def set_linear_speed(self, speed):
        self.linear_speed = speed

    def set_servo_delay(self, servo_delay):
        if servo_delay is not None and isinstance(servo_delay, (float, int)):
            servo_delay = (servo_delay, 0.02)
        self.servo_delay = servo_delay

    def get_servo_delay(self):
        return self.servo_delay

    def set_tolerance(self, tolerance):
        self.tolerance = tolerance

    def get_tolerance(self):
        return self.tolerance

    def close(self):
        self.closed = True


class MG400ControllerTests(unittest.TestCase):
    def test_mg400_exposes_servo_delay_and_tolerance(self):
        try:
            import numpy  # noqa: F401
        except ModuleNotFoundError as exc:
            self.skipTest(f"runtime dependency unavailable: {exc.name}")

        from cri.dobot import mg400_controller

        with patch.object(mg400_controller, "MG400Client", FakeMG400Client):
            controller = mg400_controller.MG400Controller(ip="127.0.0.1")

        self.assertEqual(controller._client.linear_speed, 20)
        self.assertIsNone(controller.servo_delay)
        self.assertIsNone(controller.tolerance)

        controller.servo_delay = 0.25
        controller.tolerance = 0.1

        self.assertEqual(controller.servo_delay, (0.25, 0.02))
        self.assertEqual(controller.tolerance, 0.1)


class RecordingController:
    def __init__(self):
        self.circular_moves = []

    def move_circular(self, via_pose, end_pose, elbow=None):
        self.circular_moves.append((via_pose, end_pose, elbow))

    def close(self):
        pass


class SyncRobotCircularMoveTests(unittest.TestCase):
    def test_move_circular_uses_distinct_via_and_end_poses(self):
        try:
            import numpy as np
        except ModuleNotFoundError as exc:
            self.skipTest(f"runtime dependency unavailable: {exc.name}")

        from cri.robot import SyncRobot
        from cri.transforms import euler2quat

        controller = RecordingController()
        robot = SyncRobot(controller)

        via_pose = (10, 20, 30, 1, 2, 3)
        end_pose = (40, 50, 60, 4, 5, 6)
        robot.move_circular(via_pose, end_pose, elbow=7)

        via_pose_q, end_pose_q, elbow = controller.circular_moves[0]

        np.testing.assert_allclose(via_pose_q, euler2quat(via_pose, "sxyz"))
        np.testing.assert_allclose(end_pose_q, euler2quat(end_pose, "sxyz"))
        self.assertEqual(elbow, 7)


if __name__ == "__main__":
    unittest.main()
