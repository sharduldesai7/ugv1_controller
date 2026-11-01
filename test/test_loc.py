"""Tests for the standard localisation stack that need no ROS."""

import math
import os
import re
import unittest

import numpy as np
import yaml

from ugv1_controller.loc import conditioning as cond
from ugv1_controller.loc import loc_eval

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _read(*parts):
    with open(os.path.join(ROOT, *parts)) as f:
        return f.read()


class ConditioningTest(unittest.TestCase):

    def test_covariance_shapes(self):
        c = cond.wheel_twist_covariance(0.0025, 0.01)
        self.assertEqual(len(c), 36)
        self.assertEqual(c[0], 0.0025)
        self.assertEqual(c[35], 0.01)
        self.assertEqual(len(cond.imu_angular_covariance(2.5e-5)), 9)

    def test_gyro_bias_from_stationary_window(self):
        g = cond.GyroBias(2.0)
        for k in range(300):
            g.update(k * 0.01, 0.003, True)
        self.assertTrue(g.done)
        self.assertAlmostEqual(g.bias, 0.003, places=6)
        self.assertAlmostEqual(g.corrected(0.5), 0.497, places=6)

    def test_motion_restarts_window(self):
        g = cond.GyroBias(2.0)
        for k in range(150):
            g.update(k * 0.01, 0.003, True)
        g.update(1.5, 0.2, False)
        for k in range(150):
            g.update(1.6 + k * 0.01, 0.004, True)
        self.assertFalse(g.done)

    def test_disabled(self):
        g = cond.GyroBias(0.0)
        self.assertTrue(g.done)
        self.assertEqual(g.corrected(0.1), 0.1)

    def test_stationary(self):
        self.assertTrue(cond.is_stationary(0.0, 0.0))
        self.assertFalse(cond.is_stationary(0.2, 0.0))


class LocEvalTest(unittest.TestCase):

    def _path(self, n=600):
        t = np.linspace(0, 60, n)
        p = np.column_stack([5 * np.sin(t / 6), 3 * np.sin(t / 4) + 0.05 * t, np.zeros(n)])
        return t, p

    def test_pure_frame_offset_gives_zero_error(self):
        t, gt = self._path()
        th = math.radians(40)
        r = np.array([[math.cos(th), -math.sin(th), 0], [math.sin(th), math.cos(th), 0], [0, 0, 1]])
        est = (gt - [1.0, 2.0, 0.0]) @ r
        st = loc_eval.error_stats(loc_eval.error_timeline(t, est, t, gt))
        self.assertLess(st['rmse_m'], 1e-6)
        self.assertGreater(st['distance_m'], 10)

    def test_drift_is_reported(self):
        t, gt = self._path()
        est = gt.copy()
        est[:, 0] += np.linspace(0, 0.5, len(t))
        tl = loc_eval.error_timeline(t, est, t, gt)
        st = loc_eval.error_stats(tl)
        self.assertGreater(st['max_m'], 0.1)
        self.assertLess(st['max_m'], 0.6)

    def test_too_few_matches(self):
        t, gt = self._path()
        with self.assertRaises(ValueError):
            loc_eval.error_timeline(t + 100.0, gt, t, gt)


class ConfigTest(unittest.TestCase):

    def test_ekf_topics_match_conditioner(self):
        cfg = yaml.safe_load(_read('config', 'loc', 'ekf.yaml'))['ekf_filter_node']['ros__parameters']
        self.assertEqual(cfg['odom0'], cond.WHEEL_OUT)
        self.assertEqual(cfg['imu0'], cond.IMU_OUT)
        self.assertEqual(len(cfg['odom0_config']), 15)
        self.assertEqual(len(cfg['imu0_config']), 15)
        # wheel: vx and vyaw; gyro: vyaw only
        self.assertEqual([i for i, v in enumerate(cfg['odom0_config']) if v], [6, 11])
        self.assertEqual([i for i, v in enumerate(cfg['imu0_config']) if v], [11])

    def test_slam_frames_match_ekf(self):
        ekf = yaml.safe_load(_read('config', 'loc', 'ekf.yaml'))['ekf_filter_node']['ros__parameters']
        slam = yaml.safe_load(_read('config', 'loc', 'slam_toolbox_mapping.yaml'))['slam_toolbox']['ros__parameters']
        self.assertEqual(slam['odom_frame'], ekf['odom_frame'])
        self.assertEqual(slam['base_frame'], ekf['base_link_frame'])
        self.assertEqual(slam['scan_topic'], '/scan')

    def test_entry_points_and_files_exist(self):
        setup = _read('setup.py')
        for name, mod in (('sensor_conditioner_node', 'sensor_conditioner_node'),
                          ('map_pose_node', 'map_pose_node'), ('loc_eval', 'loc_eval')):
            self.assertIn(f'ugv1_controller.loc.{mod}:main', setup)
            self.assertTrue(os.path.isfile(os.path.join(ROOT, 'ugv1_controller', 'loc', mod + '.py')))
        self.assertIn("'config', 'loc'", setup)

    def test_launch_file_is_valid_python_and_uses_known_executables(self):
        src = _read('launch', 'loc.launch.py')
        compile(src, 'loc.launch.py', 'exec')
        setup = _read('setup.py')
        for exe in re.findall(r"package='ugv1_controller', executable='(\w+)'", src):
            self.assertIn(f'{exe} =', setup)


if __name__ == '__main__':
    unittest.main()
