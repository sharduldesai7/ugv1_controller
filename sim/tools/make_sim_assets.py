#!/usr/bin/env python3
"""Generate the Gazebo assets for the UGV1 simulation.

Writes, relative to the repository root:

  sim/models/ugv1_warehouse/   textured warehouse (floor, walls, racks, boxes)
  sim/models/ugv1_rover/       differential-drive rover with camera, IMU and LiDAR
  sim/worlds/warehouse.sdf     world that includes both
  config/sim/                  OpenVINS configuration matching the simulated sensors

Everything is derived from the real rover's calibration (config/kalibr_*.yaml)
and the numbers in lidar.launch.py, so the simulated sensors sit where the real
ones do. Run it again after changing any number below; the output is
deterministic.

  python3 sim/tools/make_sim_assets.py [--camera-format L8|R8G8B8]
                                       [--imu-noise-scale 1.0]

Needs numpy and Pillow (not ROS).
"""

import argparse
import math
import shutil
from pathlib import Path

import numpy as np
import yaml

REPO = Path(__file__).resolve().parents[2]
SEED = 20260101

# ── Rover geometry (metres). Wheel and chassis sizes are assumptions: the real
#    chassis is being replaced. Sensor poses come from the real calibration. ──
WHEEL_RADIUS = 0.0325            # 65 mm wheels (Plan A hardware)
TRACK = 0.22                     # distance between left and right wheel centres
BASE_HEIGHT = 0.10               # base_link (the IMU) height above the floor
CHASSIS = (0.26, 0.20, 0.05)     # x, y, z size
CHASSIS_Z = -0.045               # chassis centre relative to base_link
CHASSIS_MASS = 1.6
WHEEL_MASS = 0.08

# LiDAR pose in base_link: same defaults as launch/lidar.launch.py
LIDAR_XYZ = (-0.0835, -0.0063, 0.0947)
LIDAR_YAW = 3.14159

# Camera: ideal pinhole with the focal length of the real camera, centred
CAM_W, CAM_H, CAM_FPS = 640, 480, 15
CAM_F = 396.6                    # mean of the calibrated fx 397.15 and fy 396.03
CAM_HFOV = 2.0 * math.atan(CAM_W / 2.0 / CAM_F)

IMU_RATE = 100.0

# ── World layout ──────────────────────────────────────────────────────────────
ROOM_X, ROOM_Y = 14.0, 10.0
WALL_H = 2.5
RACK_LEN, RACK_DEPTH, RACK_H = 10.0, 0.6, 1.9
RACK_Y = (-2.4, 0.0, 2.4)
SHELF_Z = (0.10, 0.80, 1.50)
SPAWN = (-6.0, -3.8, 0.0)        # x, y, yaw


# ── Small math helpers ────────────────────────────────────────────────────────
def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def orthonormalize(R):
    u, _, vt = np.linalg.svd(np.asarray(R, dtype=float))
    out = u @ vt
    if np.linalg.det(out) < 0:
        u[:, -1] *= -1
        out = u @ vt
    return out


def rpy_from_matrix(R):
    """SDF roll/pitch/yaw (R = Rz(yaw) Ry(pitch) Rx(roll))."""
    pitch = -math.asin(max(-1.0, min(1.0, R[2, 0])))
    roll = math.atan2(R[2, 1], R[2, 2])
    yaw = math.atan2(R[1, 0], R[0, 0])
    return roll, pitch, yaw


def matrix_from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def camera_pose_in_base(T_imu_cam):
    """Pose of a Gazebo camera in base_link, from the Kalibr/OpenVINS T_imu_cam.

    OpenVINS camera frame: x right, y down, z forward (optical axis).
    Gazebo camera frame:   x forward, y left, z up.
    base_link: the IMU frame turned 180 deg about z (IMU x points backward).
    Returns (xyz, (roll, pitch, yaw)) of the Gazebo camera in base_link and the
    orthonormalised rotation actually used for T_imu_cam.
    """
    T = np.asarray(T_imu_cam, dtype=float)
    R_imu_cam = orthonormalize(T[:3, :3])
    p_imu = T[:3, 3]
    x_g, y_g, z_g = R_imu_cam[:, 2], -R_imu_cam[:, 0], -R_imu_cam[:, 1]
    R_imu_gz = np.column_stack([x_g, y_g, z_g])
    flip = rot_z(math.pi)
    R_base_gz = flip @ R_imu_gz
    p_base = flip @ p_imu
    return tuple(p_base), rpy_from_matrix(R_base_gz), R_imu_cam


def imu_noise(noise_scale=1.0):
    """Continuous-time Kalibr noise -> per-sample Gazebo noise at IMU_RATE.

    Gazebo has white noise per sample plus a first-order Gauss-Markov bias. A
    random walk of density rw over correlation time tau is matched by a bias
    standard deviation of rw * sqrt(tau / 2).
    """
    tau = 300.0
    dt_root = math.sqrt(IMU_RATE)
    return {
        'gyro_sigma': 1.134e-4 * dt_root * noise_scale,
        'gyro_bias_sigma': 3.0e-5 * math.sqrt(tau / 2.0) * noise_scale,
        'accel_sigma': 7.5e-4 * dt_root * noise_scale,
        'accel_bias_sigma': 8.0e-4 * math.sqrt(tau / 2.0) * noise_scale,
        'tau': tau,
    }


def f(v):
    return f'{v:.6g}'


def pose(x=0.0, y=0.0, z=0.0, r=0.0, p=0.0, yw=0.0):
    return f'{f(x)} {f(y)} {f(z)} {f(r)} {f(p)} {f(yw)}'


def box_inertia(m, x, y, z):
    return (m / 12.0 * (y * y + z * z), m / 12.0 * (x * x + z * z), m / 12.0 * (x * x + y * y))


# ── Textures ──────────────────────────────────────────────────────────────────
def _smooth_noise(rng, h, w, cells):
    """Value noise: random grid of `cells` upsampled bilinearly to (h, w)."""
    from PIL import Image
    grid = rng.random((cells, cells)).astype(np.float32)
    img = Image.fromarray((grid * 255).astype(np.uint8)).resize((w, h), Image.BICUBIC)
    return np.asarray(img, dtype=np.float32) / 255.0


def _fractal(rng, h, w, octaves=(4, 8, 16, 32, 64)):
    out = np.zeros((h, w), np.float32)
    amp = 1.0
    total = 0.0
    for cells in octaves:
        out += amp * _smooth_noise(rng, h, w, cells)
        total += amp
        amp *= 0.6
    return out / total


def _save(arr, path):
    from PIL import Image
    arr = np.clip(arr, 0, 255).astype(np.uint8)
    Image.fromarray(arr).save(path, optimize=True)


def make_floor(rng, path):
    n = 512
    base = 0.55 + 0.35 * (_fractal(rng, n, n) - 0.5)
    grain = rng.normal(0.0, 0.035, (n, n)).astype(np.float32)
    img = (base + grain) * 255.0
    rgb = np.stack([img * 0.98, img * 0.99, img * 1.0], axis=2)
    from PIL import Image, ImageDraw
    pil = Image.fromarray(np.clip(rgb, 0, 255).astype(np.uint8))
    d = ImageDraw.Draw(pil, 'RGBA')
    for _ in range(int(rng.integers(1, 4))):            # oil stains
        cx, cy = rng.integers(40, n - 40, 2)
        r = int(rng.integers(12, 45))
        d.ellipse([cx - r, cy - r * 0.7, cx + r, cy + r * 0.7], fill=(30, 30, 32, int(rng.integers(40, 90))))
    for _ in range(int(rng.integers(1, 3))):            # cracks
        x, y = float(rng.integers(0, n)), float(rng.integers(0, n))
        pts = [(x, y)]
        for _ in range(int(rng.integers(6, 14))):
            x += float(rng.normal(0, 22))
            y += float(rng.normal(0, 22))
            pts.append((x, y))
        d.line(pts, fill=(25, 25, 25, 190), width=2)
    d.rectangle([0, 0, n - 1, n - 1], outline=(40, 40, 42, 255), width=3)  # slab joint
    for _ in range(int(rng.integers(0, 3))):            # painted marks
        x, y = rng.integers(60, n - 120, 2)
        d.rectangle([x, y, x + int(rng.integers(40, 110)), y + 10], fill=(215, 190, 40, 230))
    pil.save(path, optimize=True)


def make_wall(rng, path):
    from PIL import Image, ImageDraw
    w, h = 512, 640                        # a 2.0 m x 2.5 m panel
    base = 0.62 + 0.25 * (_fractal(rng, h, w, (3, 6, 12, 24)) - 0.5)
    grain = rng.normal(0.0, 0.03, (h, w)).astype(np.float32)
    img = np.clip((base + grain) * 255.0, 0, 255)
    rgb = np.stack([img, img * 0.97, img * 0.92], axis=2).astype(np.uint8)
    pil = Image.fromarray(rgb)
    d = ImageDraw.Draw(pil, 'RGBA')
    bh = 32                                # block courses
    for row in range(0, h, bh):
        d.line([(0, row), (w, row)], fill=(60, 55, 50, 170), width=2)
        off = 0 if (row // bh) % 2 == 0 else 64
        for col in range(off, w, 128):
            d.line([(col, row), (col, row + bh)], fill=(60, 55, 50, 170), width=2)
    for k in range(0, w, 64):              # hazard stripe along the bottom
        d.polygon([(k, h - 40), (k + 32, h - 40), (k + 64, h), (k + 32, h)], fill=(230, 195, 30, 255))
    d.rectangle([0, h - 44, w, h - 40], fill=(20, 20, 20, 255))
    for _ in range(int(rng.integers(1, 3))):             # bay signs
        x, y = int(rng.integers(30, w - 160)), int(rng.integers(120, 380))
        sw, sh = int(rng.integers(90, 140)), int(rng.integers(60, 90))
        d.rectangle([x, y, x + sw, y + sh], fill=(25, 70, 140, 255), outline=(240, 240, 240, 255), width=3)
        d.text((x + 12, y + 14), f'BAY {int(rng.integers(1, 99)):02d}', fill=(250, 250, 250, 255), font=_font(26))
    pil.save(path, optimize=True)


_FONT = {}


def _font(size):
    from PIL import ImageFont
    if size not in _FONT:
        _FONT[size] = ImageFont.load_default(size=size)
    return _FONT[size]


def make_box(rng, path, idx):
    from PIL import Image, ImageDraw
    n = 256
    palette = [(176, 140, 98), (160, 125, 85), (190, 160, 120), (205, 205, 200), (150, 150, 155),
               (120, 150, 175), (170, 120, 110), (195, 180, 140)]
    col = np.array(palette[idx % len(palette)], dtype=np.float32)
    shade = 0.82 + 0.3 * _fractal(rng, n, n, (4, 10, 24, 48))
    grain = rng.normal(0.0, 5.0, (n, n)).astype(np.float32)
    rgb = np.clip(col[None, None, :] * shade[:, :, None] + grain[:, :, None], 0, 255).astype(np.uint8)
    pil = Image.fromarray(rgb)
    d = ImageDraw.Draw(pil, 'RGBA')
    ty = int(rng.integers(90, 160))
    d.rectangle([0, ty, n, ty + 22], fill=(215, 190, 150, 200))     # tape
    lx, ly = int(rng.integers(12, 70)), int(rng.integers(12, 60))
    d.rectangle([lx, ly, lx + 130, ly + 70], fill=(245, 245, 245, 255), outline=(40, 40, 40, 255), width=2)
    x = lx + 8
    while x < lx + 120:                                             # barcode
        bw = int(rng.integers(1, 5))
        d.rectangle([x, ly + 34, x + bw, ly + 62], fill=(15, 15, 15, 255))
        x += bw + int(rng.integers(2, 5))
    d.text((lx + 8, ly + 6), f'SKU {int(rng.integers(10000, 99999))}', fill=(10, 10, 10, 255), font=_font(18))
    for _ in range(int(rng.integers(2, 5))):                        # printed marks
        x0, y0 = int(rng.integers(150, 230)), int(rng.integers(170, 235))
        c = tuple(int(v) for v in rng.integers(30, 220, 3)) + (230,)
        if rng.random() < 0.5:
            d.ellipse([x0 - 12, y0 - 12, x0 + 12, y0 + 12], fill=c)
        else:
            d.rectangle([x0 - 14, y0 - 10, x0 + 14, y0 + 10], fill=c)
    d.text((14, 185), f'{"ABCDEFGH"[idx % 8]}-{int(rng.integers(100, 999))}', fill=(30, 30, 30, 255), font=_font(34))
    d.polygon([(220, 8), (248, 8), (248, 36)], fill=(180, 30, 30, 255))
    pil.save(path, optimize=True)


# ── Warehouse model ───────────────────────────────────────────────────────────
N_FLOOR, N_WALL, N_BOX = 4, 3, 12
TEX_URI = 'model://ugv1_warehouse/textures/'


def visual(name, size, xyz, tex=None, rgba=None, yaw=0.0):
    sx, sy, sz = size
    if tex:
        mat = (f'<material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse><specular>0 0 0 1</specular>'
               f'<pbr><metal><albedo_map>{TEX_URI}{tex}</albedo_map><roughness>1</roughness>'
               f'<metalness>0</metalness></metal></pbr></material>')
    else:
        r, g, b = rgba
        mat = (f'<material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse>'
               f'<specular>0.05 0.05 0.05 1</specular></material>')
    return (f'    <visual name="{name}"><pose>{pose(*xyz, 0, 0, yaw)}</pose>'
            f'<geometry><box><size>{f(sx)} {f(sy)} {f(sz)}</size></box></geometry>{mat}</visual>\n')


def collision(name, size, xyz):
    sx, sy, sz = size
    return (f'    <collision name="{name}"><pose>{pose(*xyz)}</pose>'
            f'<geometry><box><size>{f(sx)} {f(sy)} {f(sz)}</size></box></geometry></collision>\n')


def build_warehouse(rng):
    out = ['<?xml version="1.0"?>\n<!-- Generated by sim/tools/make_sim_assets.py. Do not edit by hand. -->\n',
           '<sdf version="1.9">\n<model name="ugv1_warehouse">\n  <static>true</static>\n  <link name="structure">\n']
    # Floor: 2 m slabs, four texture variants
    tile = 2.0
    nx, ny = int(ROOM_X / tile), int(ROOM_Y / tile)
    for i in range(nx):
        for j in range(ny):
            x = -ROOM_X / 2 + tile * (i + 0.5)
            y = -ROOM_Y / 2 + tile * (j + 0.5)
            out.append(visual(f'floor_{i}_{j}', (tile, tile, 0.04), (x, y, -0.02),
                              tex=f'floor_{int(rng.integers(0, N_FLOOR))}.png'))
    out.append(collision('floor', (ROOM_X + 1, ROOM_Y + 1, 0.04), (0, 0, -0.02)))
    # Walls: 2 m panels
    t = 0.1
    for i in range(int(ROOM_X / tile)):
        x = -ROOM_X / 2 + tile * (i + 0.5)
        for sgn, tag in ((1, 'n'), (-1, 's')):
            out.append(visual(f'wall_{tag}_{i}', (tile, t, WALL_H), (x, sgn * (ROOM_Y / 2 + t / 2), WALL_H / 2),
                              tex=f'wall_{int(rng.integers(0, N_WALL))}.png'))
    for j in range(int(ROOM_Y / tile)):
        y = -ROOM_Y / 2 + tile * (j + 0.5)
        for sgn, tag in ((1, 'e'), (-1, 'w')):
            out.append(visual(f'wall_{tag}_{j}', (t, tile, WALL_H), (sgn * (ROOM_X / 2 + t / 2), y, WALL_H / 2),
                              tex=f'wall_{int(rng.integers(0, N_WALL))}.png'))
    out.append(collision('wall_n', (ROOM_X + 0.4, t, WALL_H), (0, ROOM_Y / 2 + t / 2, WALL_H / 2)))
    out.append(collision('wall_s', (ROOM_X + 0.4, t, WALL_H), (0, -ROOM_Y / 2 - t / 2, WALL_H / 2)))
    out.append(collision('wall_e', (t, ROOM_Y, WALL_H), (ROOM_X / 2 + t / 2, 0, WALL_H / 2)))
    out.append(collision('wall_w', (t, ROOM_Y, WALL_H), (-ROOM_X / 2 - t / 2, 0, WALL_H / 2)))
    # Racks
    orange, grey = (0.85, 0.35, 0.08), (0.45, 0.47, 0.5)
    nbox = 0
    for ri, cy in enumerate(RACK_Y):
        for k, ux in enumerate(np.linspace(-RACK_LEN / 2, RACK_LEN / 2, 5)):
            for side in (-1, 1):
                out.append(visual(f'rack{ri}_post_{k}_{side}', (0.05, 0.05, RACK_H),
                                  (ux, cy + side * (RACK_DEPTH / 2 - 0.03), RACK_H / 2), rgba=orange))
        for li, sz in enumerate(SHELF_Z):
            out.append(visual(f'rack{ri}_board_{li}', (RACK_LEN, RACK_DEPTH, 0.04), (0, cy, sz - 0.02), rgba=grey))
            x = -RACK_LEN / 2 + 0.12
            while x < RACK_LEN / 2 - 0.5:
                w = float(rng.uniform(0.40, 0.90))
                if x + w > RACK_LEN / 2 - 0.1:
                    break
                if rng.random() > 0.12:                      # some bays stay empty
                    h = float(rng.uniform(0.25, 0.52))
                    d = float(rng.uniform(0.40, 0.56))
                    yo = float(rng.uniform(-1, 1)) * (RACK_DEPTH / 2 - d / 2 - 0.03)
                    out.append(visual(f'box_{nbox}', (w, d, h), (x + w / 2, cy + yo, sz + h / 2),
                                      tex=f'box_{int(rng.integers(0, N_BOX))}.png'))
                    nbox += 1
                x += w + float(rng.uniform(0.02, 0.18))
        out.append(collision(f'rack{ri}', (RACK_LEN, RACK_DEPTH, RACK_H), (0, cy, RACK_H / 2)))
    out.append('  </link>\n</model>\n</sdf>\n')
    return ''.join(out), nbox


MODEL_CONFIG = """<?xml version="1.0"?>
<model>
  <name>{name}</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <description>{desc}</description>
</model>
"""


# ── Rover model ───────────────────────────────────────────────────────────────
def build_rover(cam_xyz, cam_rpy, camera_format, noise_scale):
    n = imu_noise(noise_scale)
    cx, cy, cz = CHASSIS
    ixx, iyy, izz = box_inertia(CHASSIS_MASS, cx, cy, cz)
    wr = WHEEL_RADIUS
    wixx = WHEEL_MASS * (3 * wr * wr + 0.02 ** 2) / 12.0
    wizz = WHEEL_MASS * wr * wr / 2.0
    axle_z = WHEEL_RADIUS - BASE_HEIGHT           # wheel centre relative to base_link
    caster_x = 0.10

    def wheel(name, side):
        return f"""  <link name="{name}_link">
    <pose>{pose(0, side * TRACK / 2, axle_z, -math.pi / 2)}</pose>
    <inertial><mass>{WHEEL_MASS}</mass><inertia><ixx>{f(wixx)}</ixx><iyy>{f(wixx)}</iyy><izz>{f(wizz)}</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
    <collision name="collision"><geometry><cylinder><radius>{wr}</radius><length>0.025</length></cylinder></geometry>
      <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface></collision>
    <visual name="visual"><geometry><cylinder><radius>{wr}</radius><length>0.025</length></cylinder></geometry>
      <material><ambient>0.08 0.08 0.08 1</ambient><diffuse>0.1 0.1 0.1 1</diffuse></material></visual>
  </link>
  <joint name="{name}_joint" type="revolute">
    <parent>base_link</parent><child>{name}_link</child>
    <axis><xyz>0 0 1</xyz><limit><lower>-1e16</lower><upper>1e16</upper></limit></axis>
  </joint>
"""

    caster = lambda tag, sx: (  # noqa: E731
        f'    <collision name="caster_{tag}"><pose>{pose(sx * caster_x, 0, axle_z)}</pose>'
        f'<geometry><sphere><radius>{wr}</radius></sphere></geometry>'
        f'<surface><friction><ode><mu>0.0</mu><mu2>0.0</mu2></ode></friction></surface></collision>\n'
        f'    <visual name="caster_{tag}"><pose>{pose(sx * caster_x, 0, axle_z)}</pose>'
        f'<geometry><sphere><radius>{wr}</radius></sphere></geometry>'
        f'<material><ambient>0.5 0.5 0.5 1</ambient><diffuse>0.5 0.5 0.5 1</diffuse></material></visual>\n')

    xml = f"""<?xml version="1.0"?>
<!-- Generated by sim/tools/make_sim_assets.py. Do not edit by hand.
     base_link is the IMU position. IMU frame = base_link turned 180 deg about z,
     exactly as on the real rover. Camera and LiDAR poses come from the real
     calibration (config/kalibr_imucam_chain.yaml, launch/lidar.launch.py). -->
<sdf version="1.9">
<model name="ugv1_rover">
  <link name="base_link">
    <inertial><pose>{pose(0, 0, CHASSIS_Z)}</pose><mass>{CHASSIS_MASS}</mass>
      <inertia><ixx>{f(ixx)}</ixx><iyy>{f(iyy)}</iyy><izz>{f(izz)}</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
    <collision name="chassis"><pose>{pose(0, 0, CHASSIS_Z)}</pose><geometry><box><size>{cx} {cy} {cz}</size></box></geometry></collision>
    <visual name="chassis"><pose>{pose(0, 0, CHASSIS_Z)}</pose><geometry><box><size>{cx} {cy} {cz}</size></box></geometry>
      <material><ambient>0.15 0.35 0.6 1</ambient><diffuse>0.2 0.45 0.75 1</diffuse></material></visual>
{caster('front', 1)}{caster('rear', -1)}    <visual name="lidar_housing"><pose>{pose(*LIDAR_XYZ)}</pose><geometry><cylinder><radius>0.035</radius><length>0.04</length></cylinder></geometry>
      <material><ambient>0.05 0.05 0.05 1</ambient><diffuse>0.08 0.08 0.08 1</diffuse></material></visual>

    <sensor name="imu" type="imu">
      <pose>{pose(0, 0, 0, 0, 0, math.pi)}</pose>
      <always_on>1</always_on><update_rate>{IMU_RATE:g}</update_rate><visualize>false</visualize>
      <topic>/imu/data_raw</topic>
      <gz_frame_id>imu_link</gz_frame_id>
      <imu>
        <angular_velocity>
{''.join(_axis_noise(a, n['gyro_sigma'], n['gyro_bias_sigma'], n['tau']) for a in 'xyz')}        </angular_velocity>
        <linear_acceleration>
{''.join(_axis_noise(a, n['accel_sigma'], n['accel_bias_sigma'], n['tau']) for a in 'xyz')}        </linear_acceleration>
      </imu>
    </sensor>

    <sensor name="camera" type="camera">
      <pose>{pose(*cam_xyz, *cam_rpy)}</pose>
      <always_on>1</always_on><update_rate>{CAM_FPS}</update_rate><visualize>false</visualize>
      <topic>/video_raw_raw</topic>
      <gz_frame_id>camera_frame</gz_frame_id>
      <camera>
        <horizontal_fov>{CAM_HFOV:.6f}</horizontal_fov>
        <image><width>{CAM_W}</width><height>{CAM_H}</height><format>{camera_format}</format></image>
        <clip><near>0.05</near><far>40</far></clip>
        <noise><type>gaussian</type><mean>0.0</mean><stddev>0.007</stddev></noise>
      </camera>
    </sensor>

    <sensor name="lidar" type="gpu_lidar">
      <pose>{pose(*LIDAR_XYZ, 0, 0, LIDAR_YAW)}</pose>
      <always_on>1</always_on><update_rate>7.4</update_rate><visualize>false</visualize>
      <topic>/scan</topic>
      <gz_frame_id>laser</gz_frame_id>
      <lidar>
        <scan><horizontal><samples>720</samples><resolution>1</resolution>
          <min_angle>-3.14159</min_angle><max_angle>3.14159</max_angle></horizontal></scan>
        <range><min>0.15</min><max>12.0</max><resolution>0.01</resolution></range>
        <noise><type>gaussian</type><mean>0.0</mean><stddev>0.005</stddev></noise>
      </lidar>
    </sensor>
  </link>

{wheel('left_wheel', 1)}{wheel('right_wheel', -1)}
  <plugin filename="gz-sim-diff-drive-system" name="gz::sim::systems::DiffDrive">
    <left_joint>left_wheel_joint</left_joint>
    <right_joint>right_wheel_joint</right_joint>
    <wheel_separation>{TRACK}</wheel_separation>
    <wheel_radius>{WHEEL_RADIUS}</wheel_radius>
    <topic>/cmd_vel</topic>
    <odom_topic>/wheel_odom</odom_topic>
    <tf_topic>/sim/wheel_odom_tf</tf_topic>
    <frame_id>odom_wheel</frame_id>
    <child_frame_id>base_link</child_frame_id>
    <odom_publish_frequency>20</odom_publish_frequency>
    <max_linear_acceleration>1.0</max_linear_acceleration>
    <min_linear_acceleration>-1.0</min_linear_acceleration>
    <max_angular_acceleration>3.0</max_angular_acceleration>
    <min_angular_acceleration>-3.0</min_angular_acceleration>
  </plugin>

  <!-- Ground truth for scoring the estimator: pose of base_link in the world -->
  <plugin filename="gz-sim-odometry-publisher-system" name="gz::sim::systems::OdometryPublisher">
    <odom_frame>world</odom_frame>
    <robot_base_frame>base_link</robot_base_frame>
    <odom_topic>/ground_truth/odom</odom_topic>
    <tf_topic>/sim/ground_truth_tf</tf_topic>
    <odom_publish_frequency>50</odom_publish_frequency>
    <dimensions>3</dimensions>
  </plugin>
</model>
</sdf>
"""
    return xml


def _axis_noise(axis, sigma, bias_sigma, tau):
    return (f'          <{axis}><noise type="gaussian"><mean>0.0</mean><stddev>{sigma:.6g}</stddev>'
            f'<bias_mean>0.0</bias_mean><bias_stddev>0.0</bias_stddev>'
            f'<dynamic_bias_stddev>{bias_sigma:.6g}</dynamic_bias_stddev>'
            f'<dynamic_bias_correlation_time>{tau:g}</dynamic_bias_correlation_time></noise></{axis}>\n')


# ── World ─────────────────────────────────────────────────────────────────────
def build_world():
    sx, sy, syaw = SPAWN
    return f"""<?xml version="1.0"?>
<!-- Generated by sim/tools/make_sim_assets.py. Do not edit by hand. -->
<sdf version="1.9">
<world name="warehouse">
  <physics name="step" type="ignored"><max_step_size>0.002</max_step_size><real_time_factor>1.0</real_time_factor></physics>
  <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
  <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
  <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
  <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
  <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
  <gravity>0 0 -9.81</gravity>
  <scene><ambient>0.7 0.7 0.7 1</ambient><background>0.15 0.16 0.18 1</background><shadows>false</shadows></scene>
  <light type="directional" name="sun">
    <cast_shadows>false</cast_shadows><pose>0 0 10 0 0 0</pose>
    <diffuse>0.8 0.8 0.78 1</diffuse><specular>0 0 0 1</specular><direction>-0.3 0.2 -1</direction>
  </light>
  <include><uri>model://ugv1_warehouse</uri><name>warehouse</name></include>
  <include><uri>model://ugv1_rover</uri><name>ugv1_rover</name>
    <pose>{pose(sx, sy, BASE_HEIGHT, 0, 0, syaw)}</pose></include>
</world>
</sdf>
"""


# ── OpenVINS configuration for the simulated sensors ──────────────────────────
def write_openvins_configs(T_imu_cam_ortho, p_imu_cam, out_dir):
    out_dir.mkdir(parents=True, exist_ok=True)
    cfg = REPO / 'config'
    est = (cfg / 'estimator_config.yaml').read_text()
    est = est.replace('verbosity: "INFO"', 'verbosity: "INFO"', 1)
    header = ('# Simulation copy of config/estimator_config.yaml. Same tuning as the real rover;\n'
              '# only the camera/IMU chain files (next to this file) differ.\n')
    (out_dir / 'estimator_config.yaml').write_text(header + est)
    shutil.copy(cfg / 'kalibr_imu_chain.yaml', out_dir / 'kalibr_imu_chain.yaml')

    R = T_imu_cam_ortho
    rows = [[*map(float, R[i]), float(p_imu_cam[i])] for i in range(3)] + [[0.0, 0.0, 0.0, 1.0]]
    text = '%YAML:1.0\n\n' + ('# Simulated camera: ideal pinhole (no distortion), no time offset, pose from\n'
                              '# the real calibration. Generated by sim/tools/make_sim_assets.py.\n') + 'cam0:\n'
    text += '  T_imu_cam:\n' + ''.join('    - [' + ', '.join(f'{v:.9f}' for v in r) + ']\n' for r in rows)
    text += (f'  cam_overlaps: []\n  camera_model: pinhole\n  distortion_coeffs: [0.0, 0.0, 0.0, 0.0]\n'
             f'  distortion_model: radtan\n  intrinsics: [{CAM_F}, {CAM_F}, {CAM_W / 2}, {CAM_H / 2}]\n'
             f'  resolution: [{CAM_W}, {CAM_H}]\n  rostopic: /video_raw_raw\n  timeshift_cam_imu: 0.0\n')
    (out_dir / 'kalibr_imucam_chain.yaml').write_text(text)


# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--camera-format', default='L8', choices=['L8', 'R8G8B8'],
                    help='L8 (mono8) is what OpenVINS consumes and is 3x smaller; R8G8B8 (rgb8) if the bridge rejects L8')
    ap.add_argument('--imu-noise-scale', type=float, default=1.0,
                    help='multiply the Kalibr IMU noise (0 = noiseless IMU)')
    args = ap.parse_args()

    kal = yaml.safe_load((REPO / 'config' / 'kalibr_imucam_chain.yaml').read_text().replace('%YAML:1.0', ''))
    T = np.array(kal['cam0']['T_imu_cam'], dtype=float)
    cam_xyz, cam_rpy, R_ortho = camera_pose_in_base(T)
    cam_xyz = tuple(float(v) for v in cam_xyz)

    rng = np.random.default_rng(SEED)
    wh = REPO / 'sim' / 'models' / 'ugv1_warehouse'
    tex = wh / 'textures'
    tex.mkdir(parents=True, exist_ok=True)
    for i in range(N_FLOOR):
        make_floor(rng, tex / f'floor_{i}.png')
    for i in range(N_WALL):
        make_wall(rng, tex / f'wall_{i}.png')
    for i in range(N_BOX):
        make_box(rng, tex / f'box_{i}.png', i)
    sdf, nbox = build_warehouse(rng)
    (wh / 'model.sdf').write_text(sdf)
    (wh / 'model.config').write_text(MODEL_CONFIG.format(name='ugv1_warehouse', desc='Textured warehouse aisles for VIO testing'))

    rv = REPO / 'sim' / 'models' / 'ugv1_rover'
    rv.mkdir(parents=True, exist_ok=True)
    (rv / 'model.sdf').write_text(build_rover(cam_xyz, cam_rpy, args.camera_format, args.imu_noise_scale))
    (rv / 'model.config').write_text(MODEL_CONFIG.format(name='ugv1_rover', desc='UGV1 differential-drive rover with camera, IMU and LiDAR'))

    (REPO / 'sim' / 'worlds').mkdir(parents=True, exist_ok=True)
    (REPO / 'sim' / 'worlds' / 'warehouse.sdf').write_text(build_world())
    write_openvins_configs(R_ortho, T[:3, 3], REPO / 'config' / 'sim')

    print(f'warehouse: {nbox} boxes, textures in {tex.relative_to(REPO)}')
    print(f'camera in base_link: xyz={tuple(round(v, 4) for v in cam_xyz)} rpy={tuple(round(v, 4) for v in cam_rpy)}'
          f' hfov={CAM_HFOV:.4f} rad')
    print(f'camera format {args.camera_format}, IMU noise scale {args.imu_noise_scale}')


if __name__ == '__main__':
    main()
