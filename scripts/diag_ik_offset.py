"""诊断：IK 底座 / frame 一致性与"固定偏移"。

复刻 flametest_controller._init_collect_mode 的 IK 初始化：
  rp, rq = robot.get_world_pose(); solver.set_robot_base_pose(rp, rq)
然后对每个抓取目标解 IK（固定 home warm start），并打印：
  - robot.get_world_pose()（IK 底座假设）
  - 目标点 seg["pos"]
  - IK 解关节的 Lula FK（right_gripper frame）
  - 应用 IK 关节后实际 tool_center 世界位置（controller 到达判定用的 gripper_pos）
  - FK vs 目标、tool_center vs 目标 的偏差

若 tool_center - 目标 ≈ 常量（对所有目标相同）→ 固定偏移，底座或 frame 不一致。
若 FK - 目标 ≈ 0 但 tool_center - 目标 大 → IK frame 与 USD 夹爪几何不一致。

用法：conda activate labutopia; python scripts/diag_ik_offset.py
"""
import os
import sys
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
from isaacsim.core.utils.rotations import euler_angles_to_quat
from isaacsim.core.utils.types import ArticulationAction

extensions.enable_extension("omni.physx.bundle")
extensions.enable_extension("omni.usdphysics.ui")

from factories.robot_factory import create_robot
from utils.object_utils import ObjectUtils

# 目标点（与 controller / flametest_meta_actions/constants 抓取点一致）
TARGETS = {
    "STO_GRASP (HCl 瓶口)":  (0.1200, -0.2800, 0.8770),
    "STO_DESK (HCl 桌面)":   (0.1600, -0.2400, 0.8100),
    "DROP_GRASP (滴管)":     (0.5070, -0.0420, 0.9310),
    "SSTO_GRASP (样品瓶口)": (-0.05,   0.30,   0.8770),
    "SSTO_DESK (样品桌面)":  (-0.01,   0.26,   0.8100),
    "MATCH_GRASP":           (0.8868,  0.5939, 0.8150),
    "WIRE_GRASP":            (0.5456, -0.0417, 0.9770),
    "CAP_GRASP":             (0.6132,  0.5456, 0.8240),
}


def main():
    ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    os.chdir(ROOT)
    from omegaconf import OmegaConf
    from datetime import datetime
    OmegaConf.register_new_resolver("now", lambda fmt: datetime.now().strftime(fmt))
    cfg = OmegaConf.load(os.path.join(ROOT, "config", "level2_FlameTest.yaml"))

    world = World(stage_units_in_meters=1.0, physics_prim_path="/physicsScene",
                  backend="numpy")
    robot = create_robot(cfg.robot.type, position=np.array(cfg.robot.position))
    stage = omni.usd.get_context().get_stage()
    add_reference_to_stage(usd_path=os.path.abspath(cfg.usd_path), prim_path="/World")
    ObjectUtils.get_instance(stage)

    print(f"[diag] cfg.robot.position = {cfg.robot.position}", flush=True)
    world.reset()
    robot.initialize()
    rp, rq = robot.get_world_pose()
    print(f"[diag] robot.get_world_pose() = pos={np.round(rp, 4)} quat={np.round(rq, 4)}", flush=True)

    # 复刻 controller 的 IK 初始化
    from isaacsim.robot_motion.motion_generation import get_extension_path_from_name
    from isaacsim.robot_motion.motion_generation import LulaKinematicsSolver
    mg_path = get_extension_path_from_name("isaacsim.robot_motion.motion_generation")
    rmp_config_dir = os.path.join(mg_path, "motion_policy_configs")
    solver = LulaKinematicsSolver(
        robot_description_path=rmp_config_dir + "/franka/rmpflow/robot_descriptor.yaml",
        urdf_path=rmp_config_dir + "/franka/lula_franka_gen.urdf")
    solver.set_robot_base_pose(robot_position=rp, robot_orientation=rq)
    ik_home = np.array([0.012, -0.57, 0.0, -2.81, 0.0, 3.037, 0.741])
    orient = euler_angles_to_quat(np.array([0, np.pi, 0]))

    offsets = []
    for name, tgt in TARGETS.items():
        tgt = np.array(tgt, dtype=float)
        ik, ok = solver.compute_inverse_kinematics(
            frame_name="right_gripper",
            target_position=tgt,
            target_orientation=orient,
            warm_start=ik_home,
        )
        if not ok or ik is None:
            print(f"[diag] {name}: IK FAIL target={tgt}", flush=True)
            continue
        joints7 = np.asarray(ik, dtype=float)
        # Lula FK（right_gripper）
        fk_p, _ = solver.compute_forward_kinematics("right_gripper", joints7)
        fk_p = np.asarray(fk_p, dtype=float)
        # 应用关节，让物理算实际 tool_center
        tp = np.concatenate([joints7, [np.nan, np.nan]])
        robot.get_articulation_controller().apply_action(ArticulationAction(joint_positions=tp))
        for _ in range(25):
            world.step(render=True)
        gp = robot.get_gripper_position()

        err_fk = np.linalg.norm(fk_p - tgt)
        err_tc = np.linalg.norm(gp - tgt) if gp is not None else float("nan")
        delta = (gp - tgt) if gp is not None else np.full(3, np.nan)
        offsets.append(delta)
        print(f"[diag] {name}:")
        print(f"    target        = {np.round(tgt, 4)}")
        print(f"    LulaFK(r_grip)= {np.round(fk_p, 4)}  err={err_fk:.4f}")
        print(f"    tool_center   = {np.round(gp, 4)}  err={err_tc:.4f}")
        print(f"    tool_center - target = {np.round(delta, 4)}", flush=True)

    if len(offsets) >= 3:
        offs = np.array(offsets)
        print(f"\n[diag] 平均 tool_center-target 偏移 = {np.round(np.nanmean(offs, axis=0), 4)}")
        print(f"[diag] 各点偏移散度(std) = {np.round(np.nanstd(offs, axis=0), 4)}"
              f"  （std≈0 ⇒ 固定偏移）", flush=True)

    print("[diag] done.", flush=True)
    simulation_app.close()


if __name__ == "__main__":
    main()
