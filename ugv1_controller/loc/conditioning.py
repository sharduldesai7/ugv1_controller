"""Sensor conditioning for the standard localisation stack (no ROS needed).

The EKF trusts each message's own covariance, and Gazebo's wheel odometry and IMU
messages carry none, so the conditioner node sets explicit values. It also removes
the gyro's turn-rate bias, which the EKF does not estimate, using a short window at
start-up while the wheels say the rover is standing still.
"""

# Topic names shared by the nodes, the EKF config and the tests.
WHEEL_IN = '/wheel_odom'
WHEEL_OUT = '/wheel_odom/conditioned'
IMU_IN = '/imu/data_raw'
IMU_OUT = '/imu/conditioned'

UNKNOWN = 1.0e6          # variance for quantities nobody measures


def diag36(values):
    """6x6 covariance (row-major list of 36) with the given diagonal."""
    out = [0.0] * 36
    for i, v in enumerate(values):
        out[i * 6 + i] = float(v)
    return out


def wheel_twist_covariance(vx_var, vyaw_var):
    """Twist covariance order: vx, vy, vz, wx, wy, wz."""
    return diag36([vx_var, UNKNOWN, UNKNOWN, UNKNOWN, UNKNOWN, vyaw_var])


def wheel_pose_covariance(xy_var=0.1, yaw_var=0.1):
    """Pose covariance order: x, y, z, roll, pitch, yaw. The EKF does not use it."""
    return diag36([xy_var, xy_var, UNKNOWN, UNKNOWN, UNKNOWN, yaw_var])


def imu_angular_covariance(gyro_var):
    """3x3 (row-major list of 9) with the same variance on every axis."""
    out = [0.0] * 9
    for i in range(3):
        out[i * 4] = float(gyro_var)
    return out


def is_stationary(vx, wz, vx_max=0.01, wz_max=0.01):
    return abs(vx) < vx_max and abs(wz) < wz_max


class GyroBias:
    """Estimate the gyro z bias from a window of samples taken while stationary.

    update(t, gz, stationary) is called for every IMU sample with its time stamp (s).
    Samples count only while `stationary`; any moving sample restarts the window. After
    `duration_s` of continuous stationary samples the mean is the bias and `done` is
    True. duration_s = 0 disables the correction (bias 0, done at once).
    """

    MIN_SAMPLES = 20

    def __init__(self, duration_s):
        self.duration_s = float(duration_s)
        self.bias = 0.0
        self.done = self.duration_s <= 0.0
        self._start = None
        self._sum = 0.0
        self._n = 0

    def update(self, t, gz, stationary):
        if self.done:
            return
        if not stationary:
            self._start, self._sum, self._n = None, 0.0, 0
            return
        if self._start is None:
            self._start = t
        self._sum += gz
        self._n += 1
        if t - self._start >= self.duration_s and self._n >= self.MIN_SAMPLES:
            self.bias = self._sum / self._n
            self.done = True

    def corrected(self, gz):
        return gz - self.bias if self.done else gz
