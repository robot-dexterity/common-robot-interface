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
            self.client.joint_angles_actual = np.array([0, 0, 0, r, 0, 0], dtype=float)  # The wrist's turn: J1 + J4.

    def MovL(self, x, y, z, r, *rest):
        self.sent.append(("MovL", (x, y, z, r)))
        self._reach(x, y, z, r)

    def ServoP(self, x, y, z, r, *rest):
        self.sent.append(("ServoP", (x, y, z, r), rest))
        self._reach(x, y, z, r)

    def JointMovJ(self, *joints):
        self.sent.append(("JointMovJ", joints))
        self.client.joint_angles_actual = np.array([*joints[:4], 0, 0], dtype=float)

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
    self.joint_angles_actual = np.zeros(6)


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

    def run_paced(self, delay, caller_times):
        """Moves in servo mode on a fake clock: each sleep advances it, as does the caller's own time between moves;
        each move's time given (ServoP's t) and the waits."""
        clock, sleeps = [0.0], []

        def sleep(seconds):
            sleeps.append(seconds)
            clock[0] += seconds
        self.client.set_servo_delay(delay)
        self.client.set_servo_mode(True)
        with patch.object(self.mg400, "perf_counter", lambda: clock[0]), patch.object(self.mg400, "sleep", sleep):
            for k, caller in enumerate([0.0] + list(caller_times)):
                clock[0] += caller
                self.client.move_linear(self.quat(301 + k, 0, 100, 0, 0, 0))
        times = [rest[0] for name, _, rest in (c for c in self.client._move.sent if c[0] == "ServoP")]
        return times, sleeps

    def test_each_servo_move_is_given_the_longest_recent_time_between_moves(self):
        # The caller taking longer than the step (0.05 s): 0.2 s between the first two moves, then 0.15, then 0.11.
        times, _ = self.run_paced((0.05, 0.02), [0.15, 0.15, 0.11, 0.11, 0.11, 0.11, 0.11])
        # The first: its delay and overlap; then the longest of the last five gaps and 0.02 more: the 0.2 s while
        # one of them, then the 0.15, then the 0.11.
        np.testing.assert_allclose(times, [0.07, 0.22, 0.22, 0.22, 0.22, 0.22, 0.17, 0.13])

    def test_the_wait_after_a_servo_move_is_what_is_left_of_its_step_after_the_callers_own_time(self):
        _, sleeps = self.run_paced((0.15, 0.02), [0.03, 0.08, 0.2])  # The last longer than the step: no wait
        np.testing.assert_allclose(sleeps, [0.15, 0.12, 0.07, 0.0], atol=1e-9)

    def test_a_robot_reading_far_behind_its_last_target_has_stopped_and_no_more_are_sent(self):
        self.client.set_servo_mode(True)
        self.client._move.hold = True  # The robot stays where it was: stopped.
        for x in range(301, 307):  # Each 1 mm on: the robot up to 5 mm behind the last, as far as allowed,
            self.client.move_linear(self.quat(x, 0, 100, 0, 0, 0))
        sent = len(self.client._move.sent)
        with self.assertRaises(self.mg400.MG400Client.ServoStopped):  # then 6 mm: no more.
            self.client.move_linear(self.quat(307, 0, 100, 0, 0, 0))
        self.assertEqual(len(self.client._move.sent), sent)
        self.assertTrue(issubclass(self.mg400.MG400Client.ServoStopped, ValueError))  # As a move out of reach.

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
        self.assertFalse(self.client.get_servo_mode())  # Left all the same: the next move is not streamed.
        self.client.move_linear(self.quat(301, 0, 100, 0, 0, 0))
        self.assertEqual(self.client._move.sent[-2][0], "MovL")
        self.client._move.hold = False
        self.client.set_servo_mode(True)
        self.client.move_linear(self.quat(302, 0, 100, 0, 0, 0))
        self.client.set_servo_mode(False)  # Reached at once: no wait.
        self.assertFalse(self.client.get_servo_mode())

    def test_a_linear_move_takes_the_turn_nearest_the_wrist_as_its_joints_read_whatever_moved_it_last(self):
        self.client.move_linear(self.quat(300, 0, 100, 0, 0, 170))
        self.client.move_linear(self.quat(300, 0, 100, 0, 0, -170))  # On round: unwrapped to 190, a turn on.
        self.assertAlmostEqual(self.client._move.sent[-2][1][3], 190, places=3)
        self.client.move_joints((0, 0, 0, 0))  # The wrist unwound by its joint (as a step starts, at home).
        self.client.move_linear(self.quat(300, 0, 100, 0, 0, -176))  # Its start: -176, not 184
        self.assertAlmostEqual(self.client._move.sent[-2][1][3], -176, places=3)  # (nearest the 190 before it).
        self.client.move_linear(self.quat(300, 0, 100, 0, 0, 175))  # Round the other way, to -185,
        self.client.joint_angles_actual = np.array([0, 0, 0, 0, 0, 0])  # then turned back by hand (or a stop):
        self.client.move_linear(self.quat(300, 0, 100, 0, 0, 1))  # 1, nearest the wrist, not 361 or -359.
        self.assertAlmostEqual(self.client._move.sent[-2][1][3], 1, places=3)

    def test_in_servo_mode_each_target_takes_the_turn_nearest_the_last_streamed(self):
        self.client.move_linear(self.quat(300, 0, 100, 0, 0, 178))
        self.client.set_servo_mode(True)
        self.client._move.hold = True  # The robot behind its targets: its joints still at 178,
        self.client.move_linear(self.quat(300, 0, 100, 0, 0, -179))  # 181,
        self.client.move_linear(self.quat(300, 0, 100, 0, 0, -176))  # 184: from the last sent, not the joints.
        self.assertAlmostEqual(self.client._move.sent[-1][1][3], 184, places=3)


    def test_connecting_waits_until_the_robot_reports_itself_enabled(self):
        class Dashboard:  # Enabling after an alarm: two polls in error, then enabled.
            def __init__(self):
                self.modes = ["0,{9},RobotMode();", "0,{9},RobotMode();", "0,{5},RobotMode();"]

            def RobotMode(self):
                return self.modes.pop(0) if len(self.modes) > 1 else self.modes[0]

        self.client._dashboard, self.sleeps[:] = Dashboard(), []
        self.client._wait_enabled(timeout=10, poll=0.1)
        self.assertEqual(self.sleeps, [0.1, 0.1])  # Polled until enabled, then on.

    def test_a_robot_that_never_enables_fails_to_connect_naming_its_mode(self):
        class Dashboard:
            def RobotMode(self):
                return "0,{9},RobotMode();"

        self.client._dashboard = Dashboard()
        with patch.object(self.mg400, "monotonic", side_effect=[0.0, 0.5, 11.0]):
            with self.assertRaisesRegex(self.mg400.MG400Client.CommandFailed, "RobotMode 9"):
                self.client._wait_enabled(timeout=10)


if __name__ == "__main__":
    unittest.main()
