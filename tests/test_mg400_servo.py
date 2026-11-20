"""The MG400 client's servo mode, offline: its sockets stubbed, each move recorded and the robot's reported pose
set to it at once, as the controller would report it once there.

    python -m pytest tests/test_mg400_servo.py
"""
import threading
import unittest
from unittest.mock import patch

try:
    import numpy as np
except ModuleNotFoundError:  # pragma: no cover
    np = None


class Recorder:
    """The move socket: each command recorded; a move reported reached at once, unless held."""

    def __init__(self, client):
        self.client, self.sent, self.hold = client, [], False

    def _reach(self, x, y, z, r):
        if not self.hold:
            self.client.pose_actual = np.array([x, y, z, r], dtype=float)

    def MovL(self, x, y, z, r, *rest):
        self.sent.append(("MovL", (x, y, z, r)))
        self._reach(x, y, z, r)

    def ServoP(self, x, y, z, r, *rest):
        self.sent.append(("ServoP", (x, y, z, r), rest))
        self._reach(x, y, z, r)

    def JointMovJ(self, *joints):
        self.sent.append(("JointMovJ", joints))

    def ServoJ(self, *joints):
        self.sent.append(("ServoJ", joints))

    def Sync(self):
        self.sent.append(("Sync",))


class Stub:
    def __getattr__(self, name):
        return lambda *args, **kwargs: None


def connect(self, ip):
    self._dashboard, self._move, self._feed = Stub(), Recorder(self), None
    self._globalLock = threading.Lock()
    self.pose_actual = np.array([300.0, 0.0, 100.0, 0.0])


@unittest.skipIf(np is None, "numpy unavailable")
class MG400ServoModeTests(unittest.TestCase):
    def setUp(self):
        from cri.dobot import mg400_client
        from cri.transforms import euler2quat
        self.mg400, self.quat = mg400_client, lambda *pose: euler2quat(pose, "sxyz")
        patcher = patch.object(mg400_client.MG400Client, "connect", connect)
        patcher.start()
        self.addCleanup(patcher.stop)
        self.sleeps = []
        sleeper = patch.object(mg400_client, "sleep", self.sleeps.append)
        sleeper.start()
        self.addCleanup(sleeper.stop)
        self.client = mg400_client.MG400Client()

    def test_moves_are_waited_for_out_of_servo_mode_and_streamed_in_it(self):
        self.client.move_linear(self.quat(300, 0, 90, 0, 0, 0))
        self.assertEqual([command[0] for command in self.client._move.sent], ["MovL", "Sync"])
        self.client.set_servo_delay(0.05)
        self.assertEqual([command[0] for command in self.client._move.sent], ["MovL", "Sync"])  # Not on its own.
        self.client.set_servo_mode(True)
        self.client.move_linear(self.quat(301, 0, 90, 0, 0, 0))
        name, pose, rest = self.client._move.sent[-1]
        self.assertEqual(name, "ServoP")
        np.testing.assert_allclose(pose, (301, 0, 90, 0), atol=1e-4)
        self.assertAlmostEqual(rest[0], 0.07)  # Its delay and the overlap: the time ServoP has to reach it.
        self.assertEqual(self.sleeps[-1], 0.05)  # Then the delay.
        self.client.set_servo_mode(False)
        self.client.move_linear(self.quat(300, 0, 90, 0, 0, 0))
        self.assertEqual([command[0] for command in self.client._move.sent[-2:]], ["MovL", "Sync"])

    def test_servo_mode_without_a_delay_streams_at_the_default(self):
        self.client.set_servo_mode(True)
        self.client.move_linear(self.quat(300, 1, 100, 0, 0, 0))
        self.assertAlmostEqual(self.client._move.sent[-1][2][0], sum(self.mg400.MG400Client.DEFAULT_SERVO_DELAY))
        self.assertEqual(self.sleeps[-1], self.mg400.MG400Client.DEFAULT_SERVO_DELAY[0])

    def test_a_streamed_target_too_far_from_the_last_is_refused_unsent(self):
        self.client.set_servo_mode(True)
        self.client.move_linear(self.quat(304, 0, 100, 0, 0, 0))  # 4 mm from where the robot is: within 5.
        sent = len(self.client._move.sent)
        with self.assertRaises(self.mg400.MG400Client.ServoJump):  # 6 mm from the last target.
            self.client.move_linear(self.quat(310, 0, 100, 0, 0, 0))
        with self.assertRaises(ValueError):  # 20 degrees: a ValueError, as a move out of reach is.
            self.client.move_linear(self.quat(304, 0, 100, 0, 0, 20))
        self.assertEqual(len(self.client._move.sent), sent)
        self.client.set_servo_step_limit((10, 30))
        self.client.move_linear(self.quat(310, 0, 100, 0, 0, 20))
        self.assertEqual(self.client._move.sent[-1][0], "ServoP")

    def test_leaving_servo_mode_waits_for_the_last_target_and_fails_if_never_reached(self):
        self.client.set_servo_mode(True)
        self.client._move.hold = True  # The robot stays where it was.
        self.client.move_linear(self.quat(301, 0, 100, 0, 0, 0))
        with patch.object(self.mg400, "monotonic", side_effect=[0, 0, 10]):
            with self.assertRaises(self.mg400.MG400Client.CommandFailed):
                self.client.set_servo_mode(False)
        self.client._move.hold = False
        self.client.set_servo_mode(True)
        self.client.move_linear(self.quat(302, 0, 100, 0, 0, 0))
        self.client.set_servo_mode(False)  # Reached at once: no wait.
        self.assertFalse(self.client.get_servo_mode())


if __name__ == "__main__":
    unittest.main()
