#!/usr/bin/env python3
"""
重投影误差分析脚本
用于验证poses.json + tracks.isat_tracks的三角化质量
"""
import json, struct, numpy as np

# ---- 读取 incremental_sfm 输出的 tracks ----
track_file = '/home/jones/Data/insight_data/iat/765-new3/incremental_sfm/tracks.isat_tracks'
with open(track_file, 'rb') as f:
    magic = f.read(4)
    version = struct.unpack('<I', f.read(4))[0]
    json_size = struct.unpack('<Q', f.read(8))[0]
    json_data = f.read(json_size)
    header = json.loads(json_data)
    pos = 16 + json_size
    pad = (8 - pos % 8) % 8
    f.read(pad)
    blobs = {}
    for b in header['blobs']:
        data = f.read(b['size'])
        arr = np.frombuffer(data, dtype=b['dtype']).reshape(b['shape'])
        blobs[b['name']] = arr

track_xyz = blobs['track_xyz'].astype(np.float64)       # [N, 3]
track_flags = blobs['track_flags']                       # [N]
track_obs_offset = blobs['track_obs_offset']             # [N+1]
obs_image_index = blobs['obs_image_index']               # [M]
obs_u = blobs['obs_u'].astype(np.float64)                # [M]
obs_v = blobs['obs_v'].astype(np.float64)                # [M]
obs_flags = blobs['obs_flags']                           # [M]

# ---- 读取 poses ----
with open('/home/jones/Data/insight_data/iat/765-new3/incremental_sfm/poses.json') as f:
    poses_data = json.load(f)

cam = poses_data['cameras'][0]
fx, fy = cam['fx'], cam['fy']
cx, cy = cam['cx'], cam['cy']
k1, k2, k3 = cam['k1'], cam['k2'], cam['k3']
p1, p2 = cam['p1'], cam['p2']

# image_index -> (R, C)
pose_map = {}
for p in poses_data['poses']:
    img_idx = p['image_index']
    R = np.array(p['R']).reshape(3, 3)
    C = np.array(p['C'])
    pose_map[img_idx] = (R, C)

print(f"Registered images: {len(pose_map)}")
print(f"Total tracks: {len(track_xyz)}")

# ---- 畸变投影函数 ----
def project_with_distortion(X_world, R, C, fx, fy, cx, cy, k1, k2, k3, p1, p2):
    """
    X_world: [3] world point
    R: [3,3] rotation (row-major, X_cam = R @ (X_world - C))
    C: [3] camera center in world
    """
    Xc = R @ (X_world - C)
    if Xc[2] <= 0:
        return None
    xn = Xc[0] / Xc[2]
    yn = Xc[1] / Xc[2]
    r2 = xn*xn + yn*yn
    r4 = r2*r2
    r6 = r4*r2
    radial = 1 + k1*r2 + k2*r4 + k3*r6
    xd = xn*radial + 2*p1*xn*yn + p2*(r2 + 2*xn*xn)
    yd = yn*radial + p1*(r2 + 2*yn*yn) + 2*p2*xn*yn
    u = fx*xd + cx
    v = fy*yd + cy
    return u, v

# ---- 批量计算重投影误差 ----
errors = []
errors_per_image = {}
n_valid_tracks = 0
n_behind_camera = 0
n_skipped_no_pose = 0

for tid in range(len(track_xyz)):
    if track_flags[tid] == 0:
        continue
    X = track_xyz[tid]
    if np.all(X == 0):
        continue
    n_valid_tracks += 1
    obs_start = int(track_obs_offset[tid])
    obs_end = int(track_obs_offset[tid + 1])
    for oi in range(obs_start, obs_end):
        img_idx = int(obs_image_index[oi])
        if img_idx not in pose_map:
            n_skipped_no_pose += 1
            continue
        R, C = pose_map[img_idx]
        uv = project_with_distortion(X, R, C, fx, fy, cx, cy, k1, k2, k3, p1, p2)
        if uv is None:
            n_behind_camera += 1
            continue
        u_proj, v_proj = uv
        err = np.sqrt((u_proj - obs_u[oi])**2 + (v_proj - obs_v[oi])**2)
        errors.append(err)
        if img_idx not in errors_per_image:
            errors_per_image[img_idx] = []
        errors_per_image[img_idx].append(err)

errors = np.array(errors)
print(f"\n=== 重投影误差统计 ===")
print(f"有效tracks: {n_valid_tracks}")
print(f"总观测数: {len(errors)}")
print(f"无pose跳过: {n_skipped_no_pose}")
print(f"相机后方点: {n_behind_camera}")
print(f"Mean:   {errors.mean():.4f} px")
print(f"Median: {np.median(errors):.4f} px")
print(f"RMSE:   {np.sqrt(np.mean(errors**2)):.4f} px")
print(f"P90:    {np.percentile(errors, 90):.4f} px")
print(f"P95:    {np.percentile(errors, 95):.4f} px")
print(f"P99:    {np.percentile(errors, 99):.4f} px")
print(f"Max:    {errors.max():.4f} px")
print(f"< 1px:  {100*np.mean(errors < 1):.1f}%")
print(f"< 2px:  {100*np.mean(errors < 2):.1f}%")
print(f"< 5px:  {100*np.mean(errors < 5):.1f}%")
print(f"> 10px: {100*np.mean(errors > 10):.1f}%")

# 直方图
bins = [0, 0.5, 1, 2, 3, 5, 10, 20, 50, 1e9]
labels = ['0-0.5', '0.5-1', '1-2', '2-3', '3-5', '5-10', '10-20', '20-50', '>50']
print("\n误差分布直方图:")
for i, label in enumerate(labels):
    cnt = np.sum((errors >= bins[i]) & (errors < bins[i+1]))
    pct = 100 * cnt / len(errors)
    bar = '#' * int(pct / 2)
    print(f"  {label:8s}: {cnt:8d} ({pct:5.1f}%) {bar}")

# 每张图的平均误差
per_img_mean = {k: np.mean(v) for k, v in errors_per_image.items()}
img_means = np.array(list(per_img_mean.values()))
print(f"\n每张图平均误差: mean={img_means.mean():.4f} median={np.median(img_means):.4f} max={img_means.max():.4f}")

worst = sorted(per_img_mean.items(), key=lambda x: -x[1])[:10]
print("最差10张图:")
for img_idx, mean_err in worst:
    print(f"  image {img_idx}: mean_err={mean_err:.4f} px, n_obs={len(errors_per_image[img_idx])}")

# ---- 额外：检查R矩阵是否正交 ----
print("\n=== R矩阵正交性检查（前5个pose）===")
for p in poses_data['poses'][:5]:
    R = np.array(p['R']).reshape(3, 3)
    err_orth = np.max(np.abs(R @ R.T - np.eye(3)))
    det = np.linalg.det(R)
    print(f"  image {p['image_index']}: |R*R^T - I|_max={err_orth:.2e}, det(R)={det:.6f}")
