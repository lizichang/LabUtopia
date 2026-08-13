# -*- coding: utf-8 -*-
"""独立渲染 lab_004.usd，多视角存 PNG，肉眼检查场景/器材形状。"""
from isaacsim import SimulationApp

simulation_app = SimulationApp({
    "headless": True,
    "extra_args": ["--/rtx/raytracing/fractionalCutoutOpacity=true"],
})

import os
import numpy as np
import cv2
from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.sensors.camera import Camera

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
USD = os.path.join(REPO, "assets", "chemistry_lab", "lab_004", "lab_004.usd")
OUT_DIR = "/tmp/lab004_render"
os.makedirs(OUT_DIR, exist_ok=True)


def mat2quat(M):
    """rotation matrix (columns=local axes in world) -> (w,x,y,z)."""
    tr = M[0, 0] + M[1, 1] + M[2, 2]
    if tr > 0:
        s = 0.5 / np.sqrt(tr + 1.0)
        w = 0.25 / s
        x = (M[2, 1] - M[1, 2]) * s
        y = (M[0, 2] - M[2, 0]) * s
        z = (M[1, 0] - M[0, 1]) * s
    else:
        i = int(np.argmax(np.diag(M)))
        j, k = (i + 1) % 3, (i + 2) % 3
        q = np.zeros(4)
        q[i] = np.sqrt(1 + M[i, i] - M[j, j] - M[k, k])
        q[j] = (M[j, i] + M[i, j]) / q[i]
        q[k] = (M[k, i] + M[i, k]) / q[i]
        q[3] = (M[k, j] - M[j, k]) / q[i]
        w, x, y, z = q[3], q[0], q[1], q[2]
    return np.array([w, x, y, z], dtype=float)


def look_at(eye, target):
    """eye->target 的朝向四元数（w,x,y,z）。相机朝 -Z。"""
    eye = np.array(eye, float)
    fwd = np.array(target, float) - eye
    fwd /= np.linalg.norm(fwd)
    z = -fwd  # local Z
    up = np.array([0.0, 0.0, 1.0])
    right = np.cross(up, z)
    if np.linalg.norm(right) < 1e-8:
        right = np.array([1.0, 0.0, 0.0])
    right /= np.linalg.norm(right)
    y = np.cross(z, right)
    M = np.array([right, y, z]).T  # columns = local axes in world
    return mat2quat(M)


VIEWS = [
    ("overview", (0.45, -0.50, 1.25), (0.28, 0.05, 0.85)),
    ("rack_tube", (0.32, -0.22, 1.10), (0.30, 0.08, 0.90)),
    ("wash_powder", (0.16, -0.25, 1.05), (0.24, 0.02, 0.88)),
    ("topdown", (0.28, 0.02, 1.35), (0.28, 0.02, 0.80)),
]

world = World()
add_reference_to_stage(usd_path=USD, prim_path="/World")
world.reset()

cams = {}
for idx, (name, eye, target) in enumerate(VIEWS):
    cam = Camera(
        prim_path=f"/World/DiagCam_{idx}",
        translation=np.array(eye),
        name=f"diag_{name}",
        frequency=20,
        resolution=(960, 720),
    )
    cam.set_world_pose(position=np.array(eye),
                       orientation=look_at(eye, target),
                       camera_axes="world")
    cam.initialize()
    cams[name] = cam

for _ in range(8):
    world.step(render=True)

for name, cam in cams.items():
    rgb = cam.get_rgb()  # H,W,3 BGR
    out = os.path.join(OUT_DIR, f"{name}.png")
    cv2.imwrite(out, rgb)
    print(f"saved {out} shape={rgb.shape}")

simulation_app.close()
print("DONE")
