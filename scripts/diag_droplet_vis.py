"""诊断脚本：验证盐酸液滴在 camera_2 皿区特写下肉眼可辨。

用户反馈"机加盐酸的时候都看不到有东西滴加了上去"。v30 修复：
  - 液滴球半径 2.5mm -> 10mm（fix 脚本 enlarge_droplet）
  - flash 8 -> 22 帧 + 下落动画（task DROPLET_FLASH_FRAMES/FALL_DIST）
  - 液滴材质从蓝色改为淡水色（fix 脚本 tune_water_materials）

本脚本用决定性"做差法"：先渲染基线（液滴隐藏），再把液滴放到皿上方坠落点并
置可见，渲染第二帧。两帧 diff 区域 = 液滴确切渲染像素（bbox + 像素数 + 颜色）。
判据：bbox 宽高 >= 10px（10mm 球在 0.4m 深 ~17px）且 diff 色为淡水（b>=r）。

用法：conda activate labutopia; python scripts/diag_droplet_vis.py
"""
import os
import sys
import cv2
import numpy as np
from isaacsim import SimulationApp

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
os.chdir(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

simulation_config = {
    "headless": True,
    "extra_args": ["--/rtx/raytracing/fractionalCutoutOpacity=true"],
}
simulation_app = SimulationApp(simulation_config)

import omni
from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage
import omni.usd
from isaacsim.core.utils import extensions

extensions.enable_extension("omni.physx.bundle")
extensions.enable_extension("omni.usdphysics.ui")

from factories.robot_factory import create_robot
from utils.object_utils import ObjectUtils
from factories.task_factory import create_task

DROPLET = "/World/Droplet"
DROP_POS = np.array([0.20, 0.02, 0.83])  # 皿上方坠落点


def _grab(world, task):
    state = task.get_basic_state_info(object_path=task.wire_path)
    return np.asarray(
        state["camera_display"]["camera_2"].transpose(1, 2, 0),
        dtype=np.float32)


def _diff_bbox(a, b, thresh=25):
    d = np.abs(a - b).mean(axis=2)
    m = d > thresh
    if not m.any():
        return None, 0, None
    ys, xs = np.nonzero(m)
    bbox = (int(xs.min()), int(ys.min()), int(xs.max()), int(ys.max()))
    mean = tuple(int(v) for v in b[m][:].mean(axis=0))
    return bbox, int(m.sum()), mean


def main():
    from omegaconf import OmegaConf
    from datetime import datetime
    OmegaConf.register_new_resolver("now", lambda fmt: datetime.now().strftime(fmt))
    cfg = OmegaConf.load("config/level2_FlameTest.yaml")

    world = World(stage_units_in_meters=1.0, physics_prim_path="/physicsScene",
                  backend="numpy")
    robot = create_robot(cfg.robot.type, position=np.array(cfg.robot.position))
    stage = omni.usd.get_context().get_stage()
    add_reference_to_stage(usd_path=os.path.abspath(cfg.usd_path), prim_path="/World")
    ObjectUtils.get_instance(stage)

    task = create_task(cfg.task_type, cfg=cfg, world=world, stage=stage, robot=robot)
    task.reset()
    for _ in range(8):
        world.step(render=True)

    # 基线：液滴隐藏
    task._set_visibility(DROPLET, False)
    for _ in range(4):
        world.step(render=True)
    base = _grab(world, task)
    cv2.imwrite("/tmp/droplet_base.png",
                cv2.cvtColor(base.astype(np.uint8), cv2.COLOR_RGB2BGR))

    # 液滴可见：放到皿上方
    task._set_obj_world_plain(DROPLET, DROP_POS)
    task._set_visibility(DROPLET, True)
    for _ in range(4):
        world.step(render=True)
    with_drop = _grab(world, task)
    cv2.imwrite("/tmp/droplet_with.png",
                cv2.cvtColor(with_drop.astype(np.uint8), cv2.COLOR_RGB2BGR))

    bbox, cnt, mean = _diff_bbox(base, with_drop)
    h, w = with_drop.shape[:2]
    if bbox is None:
        print("[diag] NO DIFF — 液滴在 camera_2 里完全没渲染", flush=True)
    else:
        # 连通块分析：区分"集中的液滴斑" vs "散布的渲染噪声"
        d = np.abs(base - with_drop).mean(axis=2)
        m = (d > 25).astype(np.uint8)
        nlab, lab, stats, cents = cv2.connectedComponentsWithStats(m, 8)
        if nlab <= 1:
            print("[diag] diff px but no connected component", flush=True)
        else:
            biggest = max(range(1, nlab), key=lambda i: stats[i, cv2.CC_STAT_AREA])
            ax0, ay0, aw, ah, aa = stats[biggest]
            bx0, by0 = int(cents[biggest][0]), int(cents[biggest][1])
            blob_mask = lab == biggest
            rgb = with_drop[blob_mask].mean(axis=0)
            cx, cy = w / 2, h / 2
            offset = np.hypot(bx0 - cx, by0 - cy)
            darkish = rgb.mean() < 200  # 深玻璃 vs 白背景
            print(f"[diag] total diff px: {cnt}", flush=True)
            print(f"[diag] largest blob: area={aa}px bbox=({ax0},{ay0})-({ax0+aw},{ay0+ah}) "
                  f"size=({aw}x{ah}) centroid=({bx0:.0f},{by0:.0f})", flush=True)
            print(f"[diag] blob mean RGB=({rgb[0]:.0f},{rgb[1]:.0f},{rgb[2]:.0f}) "
                  f"center_offset={offset:.0f}px darkish={darkish}", flush=True)
            ok = (aw >= 10 and ah >= 10 and offset < 60 and darkish)
            print(f"[diag] {'PASS' if ok else 'FAIL'} "
                  f"(need >=10x10px blob within 60px of center, dark)", flush=True)
        print(f"[diag] full img mean RGB=({with_drop[...,0].mean():.0f},"
              f"{with_drop[...,1].mean():.0f},{with_drop[...,2].mean():.0f})", flush=True)
    print("[diag] done. images at /tmp/droplet_base.png /tmp/droplet_with.png", flush=True)
    simulation_app.close()


if __name__ == "__main__":
    main()
