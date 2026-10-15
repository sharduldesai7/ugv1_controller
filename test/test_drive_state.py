"""Tests for the manual-drive rules (no ROS needed)."""

import unittest

from ugv1_controller.web.drive_state import DriveController


class FakeClock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t

    def advance(self, dt):
        self.t += dt


class DriveStateTest(unittest.TestCase):

    def setUp(self):
        self.clock = FakeClock()
        self.sent = []
        self.dc = DriveController(
            publish=lambda lin, ang: self.sent.append((lin, ang)),
            clock=self.clock)

    def feed_imu(self, seconds, still=True, rate=100):
        """Feed IMU samples for `seconds`; ticking the controller as time passes."""
        for i in range(int(seconds * rate)):
            self.clock.advance(1.0 / rate)
            if still:
                self.dc.imu_sample(0.01, 0.0, 0.0, 0.0, 0.0, 9.81)
            else:
                swing = 0.5 if i % 2 else -0.5
                self.dc.imu_sample(swing, 0.0, 0.0, 0.0, swing * 4, 9.81)
            if self.dc.snapshot(None)['enabled']:
                self.dc.tick()

    # ── Ownership ────────────────────────────────────────────────────────────

    def test_one_driver_at_a_time(self):
        self.assertEqual(self.dc.enable('a'), (True, None))
        self.assertEqual(self.dc.enable('b'), (False, 'locked'))
        self.assertEqual(self.dc.enable('a'), (True, None))     # same client again
        self.assertEqual(self.dc.command('b', 1, 0), (False, 'not_driver'))
        self.assertEqual(self.dc.disable('b'), (False, 'not_driver'))
        snap_b = self.dc.snapshot('b')
        self.assertTrue(snap_b['locked'])
        self.assertFalse(snap_b['you_drive'])

    def test_commands_refused_until_enabled(self):
        self.assertEqual(self.dc.command('a', 1, 0), (False, 'not_driver'))

    # ── Publishing ───────────────────────────────────────────────────────────

    def test_nothing_published_while_disabled(self):
        self.dc.tick()
        self.assertEqual(self.sent, [])

    def test_command_scaled_by_speed_limits(self):
        self.dc.enable('a')
        self.dc.command('a', 1, -1)
        self.dc.tick()
        self.assertEqual(self.sent[-1], (0.5, -1.0))     # defaults 0.5 m/s, 1.0 rad/s

    def test_joystick_values_clamped(self):
        self.dc.enable('a')
        self.dc.command('a', 9, -9)
        self.dc.tick()
        self.assertEqual(self.sent[-1], (0.5, -1.0))

    def test_stale_command_becomes_zero(self):
        self.dc.enable('a')
        self.dc.command('a', 1, 0)
        self.dc.tick()
        self.assertEqual(self.sent[-1], (0.5, 0.0))
        self.clock.advance(0.35)                          # cmd_timeout is 0.3 s
        self.dc.tick()
        self.assertEqual(self.sent[-1], (0.0, 0.0))

    def test_one_final_stop_after_release_then_silence(self):
        self.dc.enable('a')
        self.dc.command('a', 1, 0)
        self.dc.tick()
        self.dc.disable('a')                              # refused: moving
        self.clock.advance(0.35)
        self.dc.tick()                                    # stale -> zero
        self.feed_imu(0.7)
        ok, _ = self.dc.disable('a')
        self.assertTrue(ok)
        self.dc.tick()
        self.assertEqual(self.sent[-1], (0.0, 0.0))
        count = len(self.sent)
        self.dc.tick()
        self.dc.tick()
        self.assertEqual(len(self.sent), count)           # silent: Nav2 may publish

    # ── Link loss ────────────────────────────────────────────────────────────

    def test_silent_driver_is_released_and_stopped(self):
        self.dc.enable('a')
        self.dc.command('a', 1, 0)
        self.dc.tick()
        self.clock.advance(2.5)                           # link_timeout is 2 s
        self.dc.tick()
        self.assertEqual(self.sent[-1], (0.0, 0.0))
        snap = self.dc.snapshot('a')
        self.assertFalse(snap['enabled'])
        self.assertEqual(self.dc.enable('b'), (True, None))

    def test_polling_keeps_driver_alive(self):
        self.dc.enable('a')
        for _ in range(30):                               # 6 s of 200 ms polls
            self.clock.advance(0.2)
            self.dc.snapshot('a')
            self.dc.tick()
        self.assertTrue(self.dc.snapshot('a')['you_drive'])

    # ── Switching off ────────────────────────────────────────────────────────

    def test_cannot_disable_while_command_active(self):
        self.dc.enable('a')
        self.dc.command('a', 1, 0)
        self.assertEqual(self.dc.disable('a'), (False, 'moving'))

    def test_cannot_disable_just_after_motion(self):
        self.dc.enable('a')
        self.feed_imu(1.0)
        self.dc.command('a', 1, 0)
        self.dc.command('a', 0, 0)
        self.feed_imu(0.2)
        self.assertEqual(self.dc.disable('a'), (False, 'settling'))

    def test_refuses_without_imu_data(self):
        self.dc.enable('a')
        self.clock.advance(1.0)
        self.dc.snapshot('a')
        self.assertEqual(self.dc.disable('a'), (False, 'imu_missing'))

    def test_refuses_when_imu_data_stops(self):
        self.dc.enable('a')
        self.feed_imu(1.0)
        self.assertEqual(self.dc.disable('a')[1], None)   # sanity: would be allowed
        self.dc.enable('a')
        self.clock.advance(0.8)                           # IMU silent
        self.assertEqual(self.dc.disable('a'), (False, 'imu_missing'))

    def test_refuses_while_imu_shows_motion(self):
        self.dc.enable('a')
        self.feed_imu(1.0, still=False)
        self.assertEqual(self.dc.disable('a'), (False, 'imu_moving'))

    def test_disable_allowed_when_still(self):
        self.dc.enable('a')
        self.feed_imu(1.0)
        self.assertEqual(self.dc.disable('a'), (True, None))
        self.assertFalse(self.dc.snapshot('a')['enabled'])

    def test_snapshot_reports_reason_for_driver_only(self):
        self.dc.enable('a')
        self.feed_imu(1.0, still=False)
        self.assertEqual(self.dc.snapshot('a')['disable_reason'], 'imu_moving')
        self.assertIsNone(self.dc.snapshot('b')['disable_reason'])

    # ── Speed ────────────────────────────────────────────────────────────────

    def test_speed_steps_like_teleop(self):
        self.dc.enable('a')
        self.dc.adjust_speed('a', 'both', 1)
        self.assertAlmostEqual(self.dc.lin, 0.55)
        self.assertAlmostEqual(self.dc.turn, 1.1)
        self.dc.adjust_speed('a', 'lin', -1)
        self.assertAlmostEqual(self.dc.lin, 0.5)
        self.assertAlmostEqual(self.dc.turn, 1.1)
        self.dc.adjust_speed('a', 'turn', -1)
        self.assertAlmostEqual(self.dc.turn, 1.0)

    def test_speed_limits_hold(self):
        self.dc.enable('a')
        for _ in range(200):
            self.dc.adjust_speed('a', 'both', 1)
        self.assertLessEqual(self.dc.lin, 5.0)
        for _ in range(400):
            self.dc.adjust_speed('a', 'both', -1)
        self.assertGreaterEqual(self.dc.lin, 0.05)

    def test_only_driver_changes_speed(self):
        self.dc.enable('a')
        self.assertEqual(self.dc.adjust_speed('b', 'both', 1), (False, 'not_driver'))


if __name__ == '__main__':
    unittest.main()
