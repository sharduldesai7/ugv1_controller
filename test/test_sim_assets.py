"""Checks that the generated simulation matches the real rover's calibration."""

import importlib.util
import math
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import yaml

REPO = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('gen', REPO / 'sim/tools/make_sim_assets.py')
gen = importlib.util.module_from_spec(spec)
spec.loader.exec_module(gen)


def read_T(path):
    d = yaml.safe_load(Path(path).read_text().replace('%YAML:1.0', ''))
    return np.array(d['cam0']['T_imu_cam'], float), d['cam0']


class SimAssetsTest(unittest.TestCase):

    def setUp(self):
        self.rover = ET.parse(REPO / 'sim/models/ugv1_rover/model.sdf').getroot()

    def sensor_pose(self, name):
        s = [e for e in self.rover.iter('sensor') if e.get('name') == name][0]
        return [float(v) for v in s.find('pose').text.split()]

    def test_camera_in_sdf_reproduces_real_extrinsics(self):
        """Rebuild T_imu_cam from the SDF poses and compare with the Kalibr result."""
        T_real, _ = read_T(REPO / 'config/kalibr_imucam_chain.yaml')
        x, y, z, r, p, yw = self.sensor_pose('camera')
        R_base_gz = gen.matrix_from_rpy(r, p, yw)
        imu = self.sensor_pose('imu')
        R_base_imu = gen.matrix_from_rpy(*imu[3:])
        R_imu_gz = R_base_imu.T @ R_base_gz
        p_imu = R_base_imu.T @ (np.array([x, y, z]) - np.array(imu[:3]))
        # Gazebo camera (x fwd, y left, z up) -> optical (x right, y down, z fwd)
        x_c, y_c, z_c = -R_imu_gz[:, 1], -R_imu_gz[:, 2], R_imu_gz[:, 0]
        R_imu_cam = np.column_stack([x_c, y_c, z_c])
        np.testing.assert_allclose(R_imu_cam, gen.orthonormalize(T_real[:3, :3]), atol=1e-5)
        np.testing.assert_allclose(p_imu, T_real[:3, 3], atol=1e-5)

    def test_camera_looks_forward_and_slightly_up(self):
        _, p, _ = self.sensor_pose('camera')[3:]
        self.assertLess(p, 0)                       # negative pitch = nose up in SDF
        self.assertAlmostEqual(math.degrees(-p), 7.9, delta=0.2)

    def test_sim_openvins_camera_matches_sensor(self):
        T_sim, cam = read_T(REPO / 'config/sim/kalibr_imucam_chain.yaml')
        T_real, _ = read_T(REPO / 'config/kalibr_imucam_chain.yaml')
        np.testing.assert_allclose(T_sim[:3, 3], T_real[:3, 3], atol=1e-8)
        fx = cam['intrinsics'][0]
        fov = float(self.rover.find('.//horizontal_fov').text)
        self.assertAlmostEqual(2 * math.atan(320 / fx), fov, places=4)
        self.assertEqual(cam['distortion_coeffs'], [0.0, 0.0, 0.0, 0.0])
        self.assertEqual(cam['timeshift_cam_imu'], 0.0)

    def test_imu_is_turned_pi_about_z(self):
        self.assertAlmostEqual(self.sensor_pose('imu')[5], math.pi, places=4)

    def test_lidar_matches_launch_file(self):
        pose = self.sensor_pose('lidar')
        self.assertEqual(pose[:3], [-0.0835, -0.0063, 0.0947])
        self.assertAlmostEqual(pose[5], 3.14159, places=5)

    def test_imu_noise_follows_kalibr(self):
        n = gen.imu_noise()
        self.assertAlmostEqual(n['gyro_sigma'], 1.134e-3, places=6)
        self.assertAlmostEqual(n['accel_sigma'], 7.5e-3, places=6)
        self.assertEqual(gen.imu_noise(0.0)['gyro_sigma'], 0.0)

    def test_sensor_topics_match_the_real_stack(self):
        topics = {e.get('name'): e.find('topic').text for e in self.rover.iter('sensor')}
        self.assertEqual(topics, {'imu': '/imu/data_raw', 'camera': '/video_raw_raw', 'lidar': '/scan'})

    def test_every_bridged_topic_is_declared(self):
        bridge = yaml.safe_load((REPO / 'config/sim/bridge.yaml').read_text())
        names = {b['ros_topic_name'] for b in bridge}
        self.assertTrue({'/clock', '/imu/data_raw', '/video_raw_raw', '/scan', '/cmd_vel',
                         '/ground_truth/odom', '/wheel_odom'} <= names)

    def test_route_stays_inside_aisles(self):
        """Every route segment must clear the racks (rack footprints +/- rover half width)."""
        from ugv1_controller.sim_route import DEFAULT_ROUTE as R
        pts = [(R[i], R[i + 1]) for i in range(0, len(R), 2)]
        pts = [(-6.0, -3.8)] + pts
        for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
            for k in np.linspace(0, 1, 50):
                x, y = x0 + k * (x1 - x0), y0 + k * (y1 - y0)
                for cy in gen.RACK_Y:
                    in_x = abs(x) < gen.RACK_LEN / 2 + 0.2
                    in_y = abs(y - cy) < gen.RACK_DEPTH / 2 + 0.2
                    self.assertFalse(in_x and in_y, f'route hits rack at {x:.1f},{y:.1f}')
                self.assertLess(abs(x), gen.ROOM_X / 2 - 0.3)
                self.assertLess(abs(y), gen.ROOM_Y / 2 - 0.3)


if __name__ == '__main__':
    unittest.main()
