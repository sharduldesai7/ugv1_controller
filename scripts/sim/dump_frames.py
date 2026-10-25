#!/usr/bin/env python3
"""Contact sheet of camera frames from a recorded sim bag, with a corner count per frame.

Usage: python3 scripts/sim/dump_frames.py BAG START_S END_S [OUT.png] [COUNT=12]

START_S and END_S are seconds since the first camera frame in the bag. Frames are picked evenly
in that window. Each tile is labelled with its time and the number of FAST corners (threshold 20,
like the tracker's fast_threshold) so a textureless or repetitive view stands out. raw is the image as recorded; eq is after histogram
equalisation, which is what OpenVINS tracks on.
Keep the window short (30 s or less): only frames inside it are held in memory.
"""
import sys
import numpy as np


def corner_count(gray, threshold=20, equalize=False):
    """FAST corners. equalize=True first applies histogram equalisation, as OpenVINS does with
    histogram_method HISTOGRAM, which is what the tracker really sees."""
    import cv2
    if equalize:
        gray = cv2.equalizeHist(gray)
    return len(cv2.FastFeatureDetector_create(threshold=threshold, nonmaxSuppression=True).detect(gray))


def make_sheet(tiles, cols=4, scale=0.5):
    """tiles: list of (label, gray image). Returns one BGR image."""
    import cv2
    small = []
    for label, g in tiles:
        im = cv2.cvtColor(cv2.resize(g, None, fx=scale, fy=scale), cv2.COLOR_GRAY2BGR)
        cv2.rectangle(im, (0, 0), (im.shape[1], 18), (0, 0, 0), -1)
        cv2.putText(im, label, (4, 13), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)
        small.append(im)
    h, w = small[0].shape[:2]
    rows = (len(small) + cols - 1) // cols
    sheet = np.zeros((rows * h, cols * w, 3), np.uint8)
    for k, im in enumerate(small):
        r, c = divmod(k, cols)
        sheet[r * h:(r + 1) * h, c * w:(c + 1) * w] = im
    return sheet


def read_frames(path, t0, t1, count):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import Image
    meta = rosbag2_py.Info().read_metadata(path, '')
    reader = rosbag2_py.SequentialCompressionReader() if meta.compression_format else rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id=''), rosbag2_py.ConverterOptions('', ''))
    reader.set_filter(rosbag2_py.StorageFilter(topics=['/video_raw_raw']))
    # Header stamps sit at fixed offsets after the 4-byte CDR header: sec (int32), nanosec (uint32).
    # Only frames inside the window are kept, so memory stays small.
    base, kept = None, []
    while reader.has_next():
        _, data, _ = reader.read_next()
        t = int.from_bytes(data[4:8], 'little', signed=True) + int.from_bytes(data[8:12], 'little') * 1e-9
        if base is None:
            base = t
        if t0 <= t - base <= t1:
            kept.append((t - base, data))
    if not kept:
        raise SystemExit(f'no frames between {t0} and {t1} s after the first frame')
    pick = np.linspace(0, len(kept) - 1, min(count, len(kept))).astype(int)
    out = []
    for i in pick:
        m = deserialize_message(kept[i][1], Image)
        img = np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.width, -1)[:, :, 0]
        out.append((kept[i][0], img))
    return out


def main():
    import cv2
    path, t0, t1 = sys.argv[1], float(sys.argv[2]), float(sys.argv[3])
    out = sys.argv[4] if len(sys.argv) > 4 else 'frames.png'
    count = int(sys.argv[5]) if len(sys.argv) > 5 else 12
    tiles = [(f't={t:5.1f}s raw={corner_count(g)} eq={corner_count(g, equalize=True)}', g) for t, g in read_frames(path, t0, t1, count)]
    cv2.imwrite(out, make_sheet(tiles))
    print(f'wrote {out} with {len(tiles)} frames')
    for label, _ in tiles:
        print(label)


if __name__ == '__main__':
    main()
