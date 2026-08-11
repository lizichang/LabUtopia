"""诊断脚本：像素分析快照，判断酒精灯是否真的渲染出来了。

用户反馈"全部相机都看不到酒精灯"。文件层验证（Sdf.Layer）确认 /World/AlcoholLamp
存在于 v17.usd 且可见。本脚本不依赖文件层，直接看最终渲染像素：

  - camera_1：火焰特写，相机对准 (0.36,0.18,0.92)，灯在画幅正中央。
  - camera_2：俯视全局（x:-0.3~0.75, y:-0.22~0.28），灯 (0.36,0.18) 在覆盖内。

判读：
  - 若 camera_1 中心区域有非背景内容（亮焰/灯体/高亮）→ 灯渲染出来了。
  - 若整幅近黑或中心与四周同色 → 灯没渲染（或材质/光照太暗）。
  - flame_px：火焰色（r-b>60 的橙黄）像素数，>0 表示火焰/亮体在渲。

用法：conda activate labutopia; python scripts/diag_lamp_vis.py
"""
import os
import glob
import numpy as np
from PIL import Image

SNAP = "outputs/collect/2026.08.11/03.06.24_Level2_FlameTest/snapshot"


def stats(arr):
    return (f"mean={arr.mean():.1f} med={np.median(arr):.1f} "
            f"p99={np.percentile(arr, 99):.1f} max={arr.max():.0f}")


def flame_mask(img):
    r = img[..., 0].astype(int)
    g = img[..., 1].astype(int)
    b = img[..., 2].astype(int)
    return (r > 120) & (g > 50) & (r - b > 60)


def bright_mask(img, thr=150):
    return img.mean(axis=2) > thr


def main():
    frames = sorted(glob.glob(os.path.join(SNAP, "frame_*_camera_1.png")))
    if not frames:
        print(f"[diag] no snapshots found in {SNAP}", flush=True)
        return

    for cam in ["camera_1", "camera_2", "camera_3"]:
        print(f"\n=== {cam} ===", flush=True)
        for f in range(8):
            p = os.path.join(SNAP, f"frame_{f:04d}_{cam}.png")
            if not os.path.exists(p):
                continue
            img = np.asarray(Image.open(p).convert("RGB"))
            lum = img.mean(axis=2)
            fm = flame_mask(img)
            bm = bright_mask(img)
            h, w = lum.shape
            # 中心 40% 区域（相机1特写处火焰/灯体应在此）
            cy0, cy1 = int(h * 0.30), int(h * 0.70)
            cx0, cx1 = int(w * 0.30), int(w * 0.70)
            center = lum[cy0:cy1, cx0:cx1]
            print(f"  f{f}: lum={stats(lum)} flame_px={fm.sum()} bright_px={bm.sum()} "
                  f"center_mean={center.mean():.1f}", flush=True)
        # 各相机取一帧做中心/全局对比
        p = os.path.join(SNAP, "frame_0005_%s.png" % cam)
        if os.path.exists(p):
            img = np.asarray(Image.open(p).convert("RGB"))
            fm = flame_mask(img)
            if fm.sum() > 0:
                ys, xs = np.nonzero(fm)
                print(f"  [f5] flame centroid=({xs.mean():.0f},{ys.mean():.0f}) "
                      f"img=({img.shape[1]},{img.shape[0]})", flush=True)
    print("\n[diag] done.", flush=True)


if __name__ == "__main__":
    main()
