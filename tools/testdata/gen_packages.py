#!/usr/bin/env python3
"""
点云任务包测试数据生成器（只依赖 numpy）。

生成符合 docs/10 §7《任务包目录规范》的合成多帧任务包：
自车沿道路行驶，场景含地面、人行道、路沿、建筑、树、杆、停放车辆、行驶车辆和行人；
每帧一个雷达点云（雷达坐标系，binary PCD），两路针孔相机图片（PNG），以及标定与位姿。

输出：
  <out>/<packageId>/package.json
  <out>/<packageId>/lidar/<frameId>.pcd
  <out>/<packageId>/camera/<camera>/<frameId>.png
  <out>/<packageId>/truth/<frameId>.label   每点真值类别（uint16，类别编号见 classes.json）
  <out>/<packageId>/truth/tracks.json       运动目标逐帧真值框（自车坐标）
  <out>/import.csv                          平台“OSS 键清单”导入文件（每包一行）
  <out>/classes.json                        测试规范类别（名称、颜色、编号）

用法：
  python3 gen_packages.py --out ./out --packages 2 --frames 20 --prefix labeling/pc-test
"""
import argparse
import json
import math
import os
import struct
import zlib

import numpy as np

CLASSES = [
    # (编号, 名称, 英文, 颜色)
    (1, '地面', 'road', '#a0a0a0'),
    (2, '人行道', 'sidewalk', '#d08770'),
    (3, '路沿', 'curb', '#ebcb8b'),
    (4, '建筑物', 'building', '#5e81ac'),
    (5, '植被', 'vegetation', '#a3be8c'),
    (6, '杆', 'pole', '#b48ead'),
    (7, '乘用车', 'car', '#bf616a'),
    (8, '行人', 'pedestrian', '#e5c07b'),
]
GROUND, SIDEWALK, CURB, BUILDING, VEG, POLE, CAR, PED = range(1, 9)

LIDAR_HEIGHT = 1.8
LIDAR_RANGE = 45.0


# ---------------------------------------------------------------- 几何采样

def sample_box_surface(rng, center, size, yaw, density):
    """立方体外表面均匀采样（不含底面），返回世界坐标 Nx3。"""
    l, w, h = size
    faces = [
        (w * h, lambda n: np.c_[np.full(n, l / 2), rng.uniform(-w / 2, w / 2, n), rng.uniform(0, h, n)]),
        (w * h, lambda n: np.c_[np.full(n, -l / 2), rng.uniform(-w / 2, w / 2, n), rng.uniform(0, h, n)]),
        (l * h, lambda n: np.c_[rng.uniform(-l / 2, l / 2, n), np.full(n, w / 2), rng.uniform(0, h, n)]),
        (l * h, lambda n: np.c_[rng.uniform(-l / 2, l / 2, n), np.full(n, -w / 2), rng.uniform(0, h, n)]),
        (l * w, lambda n: np.c_[rng.uniform(-l / 2, l / 2, n), rng.uniform(-w / 2, w / 2, n), np.full(n, h)]),
    ]
    parts = []
    for area, fn in faces:
        n = max(1, int(area * density))
        parts.append(fn(n))
    p = np.vstack(parts)
    c, s = math.cos(yaw), math.sin(yaw)
    x = p[:, 0] * c - p[:, 1] * s + center[0]
    y = p[:, 0] * s + p[:, 1] * c + center[1]
    z = p[:, 2] + center[2] - h / 2
    return np.c_[x, y, z]


def sample_cylinder(rng, x, y, z0, radius, height, density):
    n = max(1, int(2 * math.pi * radius * height * density))
    a = rng.uniform(0, 2 * math.pi, n)
    return np.c_[x + radius * np.cos(a), y + radius * np.sin(a), z0 + rng.uniform(0, height, n)]


def sample_sphere(rng, cx, cy, cz, r, density):
    n = max(1, int(4 * math.pi * r * r * density))
    v = rng.normal(size=(n, 3))
    v /= np.linalg.norm(v, axis=1, keepdims=True)
    v *= r * rng.uniform(0.85, 1.0, (n, 1))
    return v + np.array([cx, cy, cz])


# ---------------------------------------------------------------- 场景

class Scene:
    def __init__(self, rng, length):
        self.static_pts = []
        self.static_lbl = []
        x0, x1 = -40.0, length + 40.0
        # 路面：y ∈ [-7, 7]
        self._plane(rng, x0, x1, -7.0, 7.0, 0.0, GROUND, 3.0)
        # 路沿：y = ±7 竖直面 0.15 m
        for side in (-1, 1):
            n = int((x1 - x0) * 120)
            xs = rng.uniform(x0, x1, n)
            self._add(np.c_[xs, np.full(n, 7.0 * side), rng.uniform(0, 0.15, n)], CURB)
        # 人行道：y ∈ ±[7, 11]，高 0.15
        self._plane(rng, x0, x1, 7.0, 11.0, 0.15, SIDEWALK, 3.0)
        self._plane(rng, x0, x1, -11.0, -7.0, 0.15, SIDEWALK, 3.0)
        # 建筑：两侧
        x = x0
        while x < x1:
            ln = rng.uniform(10, 22)
            for side in (-1, 1):
                h = rng.uniform(6, 15)
                d = rng.uniform(6, 10)
                cy = side * (12.0 + d / 2)
                self._add(sample_box_surface(rng, (x + ln / 2, cy, 0.15 + h / 2), (ln, d, h), 0.0, 2.5), BUILDING)
            x += ln + rng.uniform(2, 6)
        # 树与杆：人行道上
        x = x0 + 5
        while x < x1:
            side = 1 if rng.random() < 0.5 else -1
            y = side * rng.uniform(8.0, 10.0)
            if rng.random() < 0.6:
                self._add(sample_cylinder(rng, x, y, 0.15, 0.18, 2.5, 25), VEG)
                self._add(sample_sphere(rng, x, y, 3.6, rng.uniform(1.2, 2.0), 6), VEG)
            else:
                self._add(sample_cylinder(rng, x, y, 0.15, 0.08, rng.uniform(5, 7), 40), POLE)
            x += rng.uniform(8, 14)
        # 停放车辆：路边
        self.parked = []
        x = x0 + 10
        while x < x1:
            if rng.random() < 0.5:
                y = rng.choice([-5.6, 5.6])
                size = (rng.uniform(4.2, 4.8), rng.uniform(1.75, 1.9), rng.uniform(1.4, 1.6))
                box = ((x, y, size[2] / 2), size, 0.0 if rng.random() < 0.5 else math.pi)
                self.parked.append(box)
                self._add(sample_box_surface(rng, box[0], size, box[2], 30), CAR)
            x += rng.uniform(9, 16)
        self.static_pts = np.vstack(self.static_pts).astype(np.float64)
        self.static_lbl = np.concatenate(self.static_lbl).astype(np.uint16)

    def _plane(self, rng, x0, x1, y0, y1, z, label, density):
        n = int((x1 - x0) * (y1 - y0) * density)
        self._add(np.c_[rng.uniform(x0, x1, n), rng.uniform(y0, y1, n), np.full(n, z) + rng.normal(0, 0.01, n)], label)

    def _add(self, pts, label):
        self.static_pts.append(pts)
        self.static_lbl.append(np.full(len(pts), label, dtype=np.uint16))


def moving_agents(rng, length):
    """行驶车辆与行人：返回 [(trackId, label, size, fn(t)->(x, y, yaw))]"""
    agents = []
    agents.append(('1', CAR, (4.6, 1.85, 1.5), lambda t, v=10.5: (12.0 + v * t, -3.5, 0.0)))
    agents.append(('2', CAR, (4.4, 1.8, 1.45), lambda t, v=9.0: (length + 20 - v * t, 3.5, math.pi)))
    agents.append(('3', PED, (0.6, 0.6, 1.75), lambda t: (20.0 + 1.3 * t, 9.0, 0.0)))
    return agents


# ---------------------------------------------------------------- 写文件

def write_pcd(path, xyz, intensity):
    n = len(xyz)
    header = (
        '# .PCD v0.7 - synthetic\nVERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n'
        f'WIDTH {n}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {n}\nDATA binary\n'
    )
    data = np.c_[xyz, intensity].astype('<f4').tobytes()
    with open(path, 'wb') as f:
        f.write(header.encode('ascii'))
        f.write(data)


def write_png(path, rgb):
    h, w, _ = rgb.shape
    raw = b''.join(b'\x00' + rgb[y].tobytes() for y in range(h))

    def chunk(tag, data):
        c = struct.pack('>I', len(data)) + tag + data
        return c + struct.pack('>I', zlib.crc32(tag + data) & 0xFFFFFFFF)

    png = b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack('>IIBBBBB', w, h, 8, 2, 0, 0, 0))
    png += chunk(b'IDAT', zlib.compress(raw, 6)) + chunk(b'IEND', b'')
    with open(path, 'wb') as f:
        f.write(png)


def quat_from_yaw(yaw):
    return [math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)]


def quat_to_mat(q):
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


CAMERAS = [
    # 相机→自车：相机 z 朝前（自车 +x），x 朝右（自车 -y），y 朝下（自车 -z）
    {'name': 'camera_front', 'label': '前视', 'yaw': 0.0, 't': [1.6, 0.0, 1.5]},
    {'name': 'camera_rear', 'label': '后视', 'yaw': math.pi, 't': [-1.0, 0.0, 1.5]},
]
IMG_W, IMG_H = 640, 360
FX = FY = 380.0
CX, CY = IMG_W / 2, IMG_H / 2


def camera_extrinsic(cam):
    base = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], dtype=float)  # 相机轴在自车中的方向（列）
    c, s = math.cos(cam['yaw']), math.sin(cam['yaw'])
    rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    r = rz @ base
    # 旋转矩阵 → 四元数 [w, x, y, z]
    tr = r.trace()
    if tr > 0:
        k = math.sqrt(tr + 1.0) * 2
        q = [0.25 * k, (r[2, 1] - r[1, 2]) / k, (r[0, 2] - r[2, 0]) / k, (r[1, 0] - r[0, 1]) / k]
    else:
        i = int(np.argmax(np.diag(r)))
        j, kk = (i + 1) % 3, (i + 2) % 3
        k = math.sqrt(1.0 + r[i, i] - r[j, j] - r[kk, kk]) * 2
        v = [0.0, 0.0, 0.0]
        v[i] = 0.25 * k
        v[j] = (r[j, i] + r[i, j]) / k
        v[kk] = (r[kk, i] + r[i, kk]) / k
        q = [(r[kk, j] - r[j, kk]) / k] + v
    return r, [round(float(a), 9) for a in q]


def render_camera(ego_pts, labels, cam):
    r, _ = camera_extrinsic(cam)
    pc = (ego_pts - np.array(cam['t'])) @ r  # 自车 → 相机（r 正交）
    m = pc[:, 2] > 0.5
    pc, lab = pc[m], labels[m]
    u = (FX * pc[:, 0] / pc[:, 2] + CX).astype(int)
    v = (FY * pc[:, 1] / pc[:, 2] + CY).astype(int)
    ok = (u >= 0) & (u < IMG_W) & (v >= 0) & (v < IMG_H)
    u, v, d, lab = u[ok], v[ok], pc[ok, 2], lab[ok]
    img = np.zeros((IMG_H, IMG_W, 3), dtype=np.uint8)
    img[:] = (24, 28, 36)
    img[: IMG_H // 2] = (60, 78, 104)
    palette = np.array([[0, 0, 0]] + [[int(c[3][i:i + 2], 16) for i in (1, 3, 5)] for c in CLASSES], dtype=float)
    order = np.argsort(-d)  # 远到近，近处覆盖
    shade = np.clip(1.15 - d / 60.0, 0.35, 1.0)
    col = (palette[lab] * shade[:, None]).astype(np.uint8)
    for dy in (0, 1):
        for dx in (0, 1):
            img[np.clip(v[order] + dy, 0, IMG_H - 1), np.clip(u[order] + dx, 0, IMG_W - 1)] = col[order]
    return img


# ---------------------------------------------------------------- 主流程

def make_package(out_dir, prefix, pkg_index, frames, seed):
    rng = np.random.default_rng(seed)
    package_id = f'PCTEST_{pkg_index:04d}'
    root = os.path.join(out_dir, package_id)
    for sub in ('lidar', 'truth') + tuple(f'camera/{c["name"]}' for c in CAMERAS):
        os.makedirs(os.path.join(root, sub), exist_ok=True)
    dt = 0.1
    speed = 8.0
    length = speed * dt * frames + 30
    scene = Scene(rng, length)
    agents = moving_agents(rng, length)
    t0 = 1773044875900000 + pkg_index * 100000000
    lidar_t = np.array([0.0, 0.0, LIDAR_HEIGHT])
    frame_rows = []
    tracks = []
    for fi in range(frames):
        t = fi * dt
        ego_x = speed * t
        ego_yaw = 0.02 * math.sin(t * 0.8)
        ts = t0 + int(t * 1e6)
        fid = str(ts)
        # 动态目标（世界坐标）
        dyn_pts, dyn_lbl, frame_boxes = [], [], []
        for track_id, lbl, size, fn in agents:
            x, y, yaw = fn(t)
            center = (x, y, size[2] / 2)
            dyn_pts.append(sample_box_surface(rng, center, size, yaw, 35 if lbl == CAR else 120))
            dyn_lbl.append(np.full(len(dyn_pts[-1]), lbl, dtype=np.uint16))
            frame_boxes.append((track_id, lbl, center, size, yaw))
        world = np.vstack([scene.static_pts] + dyn_pts)
        labels = np.concatenate([scene.static_lbl] + dyn_lbl)
        # 世界 → 自车
        c, s = math.cos(ego_yaw), math.sin(ego_yaw)
        rel = world - np.array([ego_x, 0.0, 0.0])
        ego = np.c_[rel[:, 0] * c + rel[:, 1] * s, -rel[:, 0] * s + rel[:, 1] * c, rel[:, 2]]
        # 量程裁剪 + 抖动 + 稀疏化（远处更稀）
        dist = np.hypot(ego[:, 0], ego[:, 1])
        keep = (dist < LIDAR_RANGE) & (rng.random(len(ego)) < np.clip(1.4 - dist / 40.0, 0.25, 1.0))
        ego, labels = ego[keep], labels[keep]
        ego = ego + rng.normal(0, 0.012, ego.shape)
        # 按方位角排序，模拟扫描顺序（标签空间连续）
        order = np.argsort(np.arctan2(ego[:, 1], ego[:, 0]), kind='stable')
        ego, labels = ego[order], labels[order]
        lidar_pts = ego - lidar_t
        intensity = np.where(labels == GROUND, 12, np.where(labels == CAR, 80, 40)) + rng.uniform(0, 20, len(ego))
        write_pcd(os.path.join(root, 'lidar', f'{fid}.pcd'), lidar_pts, intensity)
        labels.astype('<u2').tofile(os.path.join(root, 'truth', f'{fid}.label'))
        cams = {}
        for cam in CAMERAS:
            key = f'camera/{cam["name"]}/{fid}.png'
            write_png(os.path.join(root, key), render_camera(ego, labels, cam))
            cams[cam['name']] = {'key': key, 'timestampUs': ts}
        frame_rows.append({
            'id': fid,
            'timestampUs': ts,
            'lidar': {'top': f'lidar/{fid}.pcd'},
            'cameras': cams,
            'pose': {'translation': [round(ego_x, 6), 0.0, 0.0], 'rotation': [round(v, 9) for v in quat_from_yaw(ego_yaw)]},
        })
        for track_id, lbl, center, size, yaw in frame_boxes:
            rx, ry = center[0] - ego_x, center[1]
            tracks.append({
                'frameIndex': fi, 'trackId': track_id, 'classNo': int(lbl),
                'center': [round(rx * c + ry * s, 4), round(-rx * s + ry * c, 4), round(center[2] - LIDAR_HEIGHT, 4)],
                'size': [round(v, 4) for v in size], 'yawDeg': round(math.degrees(yaw - ego_yaw) % 360, 3),
            })
    sensors = {
        'lidars': [{'name': 'top', 'extrinsic': {'translation': [0.0, 0.0, LIDAR_HEIGHT], 'rotation': [1.0, 0.0, 0.0, 0.0]}}],
        'cameras': [],
    }
    for cam in CAMERAS:
        _, q = camera_extrinsic(cam)
        sensors['cameras'].append({
            'name': cam['name'], 'label': cam['label'], 'model': 'pinhole', 'width': IMG_W, 'height': IMG_H,
            'intrinsic': [FX, FY, CX, CY], 'distortion': [0, 0, 0, 0, 0],
            'extrinsic': {'translation': cam['t'], 'rotation': q},
        })
    package = {
        'version': 'pc-package-v1',
        'packageId': package_id,
        'coordinate': {'unit': 'm', 'up': 'z'},
        'sensors': sensors,
        'frames': frame_rows,
    }
    with open(os.path.join(root, 'package.json'), 'w', encoding='utf-8') as f:
        json.dump(package, f, ensure_ascii=False, indent=1)
    with open(os.path.join(root, 'truth', 'tracks.json'), 'w', encoding='utf-8') as f:
        json.dump({'note': '框为雷达坐标系（与点云文件一致）', 'boxes': tracks}, f, ensure_ascii=False, indent=1)
    first_key = f'{prefix.rstrip("/")}/{package_id}/lidar/{frame_rows[0]["id"]}.pcd' if prefix else f'{package_id}/lidar/{frame_rows[0]["id"]}.pcd'
    return package_id, first_key


def main():
    ap = argparse.ArgumentParser(description='生成点云任务包测试数据')
    ap.add_argument('--out', default='./out')
    ap.add_argument('--packages', type=int, default=2)
    ap.add_argument('--frames', type=int, default=20)
    ap.add_argument('--prefix', default='labeling/pc-test', help='上传到对象存储时的目录前缀（写入 import.csv）')
    ap.add_argument('--seed', type=int, default=7)
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    rows = ['object_key,manifest_code,media_type']
    for i in range(1, args.packages + 1):
        pid, key = make_package(args.out, args.prefix, i, args.frames, args.seed + i)
        rows.append(f'{key},{pid},lidar')
        print('生成', pid)
    with open(os.path.join(args.out, 'import.csv'), 'w', encoding='utf-8') as f:
        f.write('\n'.join(rows) + '\n')
    with open(os.path.join(args.out, 'classes.json'), 'w', encoding='utf-8') as f:
        json.dump([{'categoryId': c[0], 'name': c[1], 'englishName': c[2], 'color': c[3]} for c in CLASSES],
                  f, ensure_ascii=False, indent=1)
    print('完成：', os.path.abspath(args.out))


if __name__ == '__main__':
    main()
