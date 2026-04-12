#!/usr/bin/env python3
"""
深入分析：区分已三角化的track vs 未三角化的track，
以及分析误差大的track的特征
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

track_xyz = blobs['track_xyz'].astype(np.float64)
track_flags = blobs['track_flags']
track_obs_offset = blobs['track_obs_offset']
obs_image_index = blobs['obs_image_index']
obs_u = blobs['obs_u'].astype(np.float64)
obs_v = blobs['obs_v'].astype(np.float64)
obs_flags = blobs['obs_flags']

# ---- 读取 poses ----
with open('/home/jones/Data/insight_data/iat/765-new3/incremental_sfm/poses.json') as f:
    poses_data = json.load(f)

cam = poses_data['cameras'][0]
fx, fy = cam['fx'], cam['fy']
cx, cy = cam['cx'], cam['cy']
k1, k2, k3 = cam['k1'], cam['k2'], cam['k3']
p1, p2 = cam['p1'], cam['p2']

pose_map = {}
for p in poses_data['poses']:
    img_idx = p['image_index']
    R = np.array(p['R']).reshape(3, 3)
    C = np.array(p['C'])
    pose_map[img_idx] = (R, C)

registered_set = set(pose_map.keys())

# ---- 统计track状态 ----
# track_xyz全零 = 未三角化
xyz_nonzero = np.any(track_xyz != 0, axis=1)
print(f"=== Track状态统计 ===")
print(f"总tracks: {len(track_xyz)}")
print(f"有xyz(非零): {xyz_nonzero.sum()}")
print(f"xyz全零(未三角化): {(~xyz_nonzero).sum()}")

# 对于有xyz的track，统计有多少obs在已注册图像中
print(f"\n=== 分析有xyz的track的obs覆盖 ===")
n_tri_tracks = 0
n_obs_in_registered = 0
n_obs_total = 0
track_obs_counts = []  # (n_total_obs, n_registered_obs)

for tid in range(len(track_xyz)):
    if not xyz_nonzero[tid]:
        continue
    n_tri_tracks += 1
    obs_start = int(track_obs_offset[tid])
    obs_end = int(track_obs_offset[tid + 1])
    n_total = obs_end - obs_start
    n_reg = sum(1 for oi in range(obs_start, obs_end) if int(obs_image_index[oi]) in registered_set)
    n_obs_total += n_total
    n_obs_in_registered += n_reg
    track_obs_counts.append((n_total, n_reg))

track_obs_counts = np.array(track_obs_counts)
print(f"有xyz的tracks: {n_tri_tracks}")
print(f"这些track的总obs: {n_obs_total}")
print(f"其中在已注册图像中的obs: {n_obs_in_registered} ({100*n_obs_in_registered/n_obs_total:.1f}%)")
print(f"每个track平均obs数: {track_obs_counts[:,0].mean():.2f}")
print(f"每个track平均已注册obs数: {track_obs_counts[:,1].mean():.2f}")

# ---- 重投影误差，只用已注册图像 ----
def project_with_distortion(X_world, R, C):
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
    return fx*xd + cx, fy*yd + cy

# 按track收集误差，分析误差大的track
track_errors = []  # (tid, mean_err, max_err, n_obs)
all_errors = []

for tid in range(len(track_xyz)):
    if not xyz_nonzero[tid]:
        continue
    X = track_xyz[tid]
    obs_start = int(track_obs_offset[tid])
    obs_end = int(track_obs_offset[tid + 1])
    errs = []
    for oi in range(obs_start, obs_end):
        img_idx = int(obs_image_index[oi])
        if img_idx not in pose_map:
            continue
        R, C = pose_map[img_idx]
        uv = project_with_distortion(X, R, C)
        if uv is None:
            continue
        err = np.sqrt((uv[0] - obs_u[oi])**2 + (uv[1] - obs_v[oi])**2)
        errs.append(err)
        all_errors.append(err)
    if errs:
        track_errors.append((tid, np.mean(errs), np.max(errs), len(errs)))

all_errors = np.array(all_errors)
track_errors.sort(key=lambda x: -x[1])

print(f"\n=== 按track分析误差 ===")
print(f"有重投影误差的tracks: {len(track_errors)}")
print(f"track平均误差 mean: {np.mean([x[1] for x in track_errors]):.4f} px")
print(f"track平均误差 median: {np.median([x[1] for x in track_errors]):.4f} px")

# 误差分布
te_means = np.array([x[1] for x in track_errors])
print(f"track mean_err < 1px: {100*np.mean(te_means < 1):.1f}%")
print(f"track mean_err < 2px: {100*np.mean(te_means < 2):.1f}%")
print(f"track mean_err > 5px: {100*np.mean(te_means > 5):.1f}%")

print(f"\n最差20个tracks:")
for tid, mean_err, max_err, n_obs in track_errors[:20]:
    X = track_xyz[tid]
    print(f"  track {tid}: mean={mean_err:.3f} max={max_err:.3f} n_obs={n_obs} xyz=({X[0]:.4f},{X[1]:.4f},{X[2]:.4f})")

# ---- 检查：无畸变 vs 有畸变的差异 ----
print(f"\n=== 无畸变投影 vs 有畸变投影对比（前1000个有效track）===")
errors_nodist = []
errors_dist = []
count = 0
for tid in range(len(track_xyz)):
    if not xyz_nonzero[tid] or count >= 1000:
        continue
    X = track_xyz[tid]
    obs_start = int(track_obs_offset[tid])
    obs_end = int(track_obs_offset[tid + 1])
    for oi in range(obs_start, obs_end):
        img_idx = int(obs_image_index[oi])
        if img_idx not in pose_map:
            continue
        R, C = pose_map[img_idx]
        Xc = R @ (X - C)
        if Xc[2] <= 0:
            continue
        xn = Xc[0] / Xc[2]
        yn = Xc[1] / Xc[2]
        # 无畸变
        u_nd = fx*xn + cx
        v_nd = fy*yn + cy
        # 有畸变
        uv_d = project_with_distortion(X, R, C)
        if uv_d is None:
            continue
        err_nd = np.sqrt((u_nd - obs_u[oi])**2 + (v_nd - obs_v[oi])**2)
        err_d = np.sqrt((uv_d[0] - obs_u[oi])**2 + (uv_d[1] - obs_v[oi])**2)
        errors_nodist.append(err_nd)
        errors_dist.append(err_d)
    count += 1

errors_nodist = np.array(errors_nodist)
errors_dist = np.array(errors_dist)
print(f"无畸变: mean={errors_nodist.mean():.4f} median={np.median(errors_nodist):.4f}")
print(f"有畸变: mean={errors_dist.mean():.4f} median={np.median(errors_dist):.4f}")
print(f"畸变校正改善: {errors_nodist.mean() - errors_dist.mean():.4f} px")
