"""Tests for the VIO scoring maths (no ROS needed)."""

import math
import unittest

import numpy as np

from ugv1_controller import vio_eval as ve


def lawnmower(n=600):
    t = np.linspace(0, 60, n)
    x = 5 * np.sin(t / 6)
    y = 3 * np.sin(t / 4) + 0.05 * t
    z = 0.02 * np.sin(t)
    return t, np.column_stack([x, y, z])


class VioEvalTest(unittest.TestCase):

    def test_recovers_rotation_translation_and_scale(self):
        t, gt = lawnmower()
        th = math.radians(37.0)
        r = np.array([[math.cos(th), -math.sin(th), 0], [math.sin(th), math.cos(th), 0], [0, 0, 1]])
        # estimate = gt seen from a rotated, shifted frame at half scale
        est = ((gt - [1.0, -2.0, 0.1]) @ r) * 0.5
        res = ve.score(t, est, t, gt)
        self.assertAlmostEqual(res['scale'], 2.0, places=3)
        self.assertLess(res['ate_rmse_scaled_m'], 1e-6)
        self.assertAlmostEqual(res['path_ratio'], 0.5, places=3)
        self.assertGreater(res['ate_rmse_m'], 0.5)       # unscaled fit is poor for a wrong scale

    def test_perfect_estimate_scores_zero(self):
        t, gt = lawnmower()
        res = ve.score(t, gt + [10, 5, 0], t, gt)
        self.assertLess(res['ate_rmse_m'], 1e-9)
        self.assertAlmostEqual(res['scale'], 1.0, places=6)

    def test_noise_gives_small_error(self):
        t, gt = lawnmower()
        rng = np.random.default_rng(1)
        res = ve.score(t, gt + rng.normal(0, 0.02, gt.shape), t, gt)
        self.assertLess(res['ate_rmse_m'], 0.05)

    def test_association_respects_tolerance(self):
        ie, ig = ve.associate([1.0, 2.0, 3.0], [1.005, 2.5, 3.01], tol=0.02)
        self.assertEqual(list(ie), [0, 2])
        self.assertEqual(list(ig), [0, 2])

    def test_too_few_matches_is_an_error(self):
        t, gt = lawnmower(50)
        with self.assertRaises(ValueError):
            ve.score(t[:5] + 1000, gt[:5], t, gt)

    def test_latency_uses_matching_imu_stamp(self):
        imu_s = np.arange(0, 1, 0.01)
        imu_r = imu_s + 100.0                   # imu arrives on time
        lat = ve.latency([0.50, 0.80], [100.55, 100.90], imu_s, imu_r)
        np.testing.assert_allclose(lat, [0.05, 0.10], atol=1e-9)


if __name__ == '__main__':
    unittest.main()
