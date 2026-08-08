"""FlameTest 1.5x + 512 分辨率冒烟视觉验证
- 三相机黄色像素统计（帧数、面积、占比）
- camera_1 投影采样火焰区域颜色
- 抽帧保存 PNG
"""
import h5py
import numpy as np
import math
import os

H5 = "outputs/collect/2026.08.07/00.26.46_Level2_FlameTest/dataset/episode_0000.h5"
OUT = "outputs/verify_flame2"
os.makedirs(OUT, exist_ok=True)

# ---- 相机参数（与 yaml 一致）----
CAMS = {
    "camera_1": {"pos": [1.25, 0.4, 1.5], "quat": [0.5283, 0.32878, 0.41362, 0.66462], "focal_mm": 12.0},
    "camera_2": {"pos": [0.1, 0.0, 2.5], "quat": [0.70711, 0, 0, -0.70711], "focal_mm": 5.0},
    "camera_3": {"pos": None, "quat": None, "focal_mm": 5.0},  # pos 从 yaml 补
}

def quat_to_rotmat(qw, qx, qy, qz):
    """w,x,y,z -> 3x3 旋转矩阵 (列向量: R @ v_cam = v_world)"""
    n = math.sqrt(qw*qw + qx*qx + qy*qy + qz*qz)
    qw, qx, qy, qz = qw/n, qx/n, qy/n, qz/n
    return np.array([
        [1-2*(qy*qy+qz*qz), 2*(qx*qy-qz*qw), 2*(qx*qz+qy*qw)],
        [2*(qx*qy+qz*qw), 1-2*(qx*qx+qz*qz), 2*(qy*qz-qx*qw)],
        [2*(qx*qz-qy*qw), 2*(qy*qz+qx*qw), 1-2*(qx*qx+qy*qy)],
    ])

def world_to_pixel(p_w, cam):
    """世界坐标 -> 相机像素 (u,v)。相机 z 朝观察方向, 图像 u 右 v 下"""
    pos = np.array(cam["pos"])
    R = quat_to_rotmat(*cam["quat"])
    p_cam = R.T @ (np.array(p_w) - pos)
    focal_px = cam["focal_mm"] / 24.0 * 512
    if p_cam[2] >= 0:
        return None
    u = focal_px * p_cam[0] / (-p_cam[2]) + 256
    v = focal_px * p_cam[1] / (-p_cam[2]) + 256
    return (u, v)

def is_yellow(rgb):
    r, g, b = rgb[..., 0].astype(int), rgb[..., 1].astype(int), rgb[..., 2].astype(int)
    return (r > 200) & (g > 150) & (b < 150) & (r > b + 60) & (r > g)

f = h5py.File(H5, "r")
n_frames = f["camera_1_rgb"].shape[0]
print(f"frames: {n_frames}")

def get_frame(name, i):
    """取单帧并统一为 (H, W, 3)"""
    img = f[f"{name}_rgb"][i]
    if img.ndim == 3 and img.shape[0] == 3:
        img = np.transpose(img, (1, 2, 0))
    return img

# 火焰世界坐标（burner 在 (0.3,0.18,0.8)，火焰 local z 0.16-0.276 -> 世界 z 0.96-1.076）
FLAME_CENTER = np.array([0.3, 0.18, 1.02])
FLAME_TOP = np.array([0.3, 0.18, 1.076])

# camera_3 为机械臂手部相机（随动，无固定外参），保持 pos=None

for name, cam in CAMS.items():
    rgb = f[f"{name}_rgb"]
    total_yellow = 0
    yellow_frames = 0
    max_area = 0
    first_yellow = -1
    for i in range(n_frames):
        img = get_frame(name, i)
        mask = is_yellow(img)
        cnt = int(mask.sum())
        if cnt > 0:
            yellow_frames += 1
            if first_yellow < 0:
                first_yellow = i
            max_area = max(max_area, cnt)
            total_yellow += cnt
    ratio = yellow_frames / n_frames
    print(f"[{name}] yellow_frames={yellow_frames}/{n_frames} ({ratio:.0%}), "
          f"first_yellow={first_yellow}, max_area={max_area}px ({max_area/(512*512):.1%} of frame), "
          f"avg_yellow_per_frame={total_yellow/max(yellow_frames,1):.0f}px")

# camera_1 投影采样（取火焰一帧）
for name in ["camera_1", "camera_2"]:
    cam = CAMS[name]
    if cam["pos"] is None:
        continue
    img = get_frame(name, first_yellow) if first_yellow >= 0 else get_frame(name, n_frames//2)
    px_c = world_to_pixel(FLAME_CENTER, cam)
    px_t = world_to_pixel(FLAME_TOP, cam)
    print(f"[{name}] flame_center_proj={px_c}, flame_top_proj={px_t}")
    if px_c:
        u, v = int(px_c[0]), int(px_c[1])
        if 0 <= u < 512 and 0 <= v < 512:
            print(f"[{name}] center pixel rgb={img[v, u].tolist()}")
            # 5x5 邻域
            patch = img[max(0,v-2):v+3, max(0,u-2):u+3].reshape(-1, 3)
            print(f"[{name}] 5x5 patch mean={patch.mean(axis=0).astype(int).tolist()}, yellow_in_patch={int(is_yellow(patch.reshape(1,-1,3)).sum())}")

# 抽帧保存：燃烧中帧（黄色最多的一帧）+ 最后帧
for name in ["camera_1", "camera_2", "camera_3"]:
    rgb = f[f"{name}_rgb"]
    # 黄色最多的帧
    best_i, best_cnt = -1, 0
    for i in range(n_frames):
        img = get_frame(name, i)
        cnt = int(is_yellow(img).sum())
        if cnt > best_cnt:
            best_cnt, best_i = cnt, i
    for tag, i in [("burn", best_i), ("last", n_frames-1)]:
        if i < 0:
            continue
        img = get_frame(name, i)
        # 上采样 2x 便于查看（最近邻即可）
        big = np.kron(img, np.ones((2, 2, 1), dtype=np.uint8))
        import cv2
        cv2.imwrite(f"{OUT}/{name}_{tag}_f{i}.png", cv2.cvtColor(big, cv2.COLOR_RGB2BGR))
        print(f"saved {OUT}/{name}_{tag}_f{i}.png (yellow_px={best_cnt})")

print("DONE")
