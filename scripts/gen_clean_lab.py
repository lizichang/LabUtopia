# -*- coding: utf-8 -*-
"""生成 lab_clean.usd —— 干净底场景（仅房间骨架+工作台，台面顶 0.80，烘平自包含）。

供新实验直接 Open/引用的起点：免除每次「从 lab_001 复制 + 删杂物 + 抬台面」的
重复样板（skill 坑 20 的固定首步集中到这里）。生成式脚本：lab_001 结构变更后
重跑本脚本即同步，绝不手改产物，避免底场景漂移。

基于 lab_001.usd 副本（Usd.Stage.Open 编辑 + stage.Export 烘平，不写回 lab_001）：
- 白名单删除 lab_001 自带器材/家具（含 Cabinet_01/02 离线 http payload，一并消除加载警告）
- 抬高工作台面 table+Cube，顶面 -> 0.80（d2s/flametest 约定，任务坐标常数按此调）

台面顶烤成 0.80：与仓库里已验证跑通的实验（d2s/flametest）坐标一致，新实验开箱对齐；
原 lab_001 原生是 0.78，历史实验靠各自脚本再抬，底场景把这一步固化。

用法：python scripts/gen_clean_lab.py   （运行环境：本地 conda env 有 pxr）
"""
import os
from pxr import Usd, UsdGeom, Gf

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(REPO, "assets", "scenes", "base", "lab_clean", "lab_clean.usd")
LAB001 = os.path.join(REPO, "assets", "scenes", "base", "lab_001", "lab_001.usd")

TABLE_TOP = 0.80
# lab_001 里保留的结构件/灯光/物理（房间骨架，其余全部真实删除）
KEEP = {"table", "Cube", "GroundPlane", "CylinderLight", "PhysicsScene", "Looks"}


def remove_lab001_equipment(stage):
    """真正删除 lab_001 自带器材/家具（白名单外全删，含 Cabinet 离线 payload）。"""
    world = stage.GetPrimAtPath("/World")
    removed = []
    for child in list(world.GetChildren()):
        name = child.GetName()
        if name in KEEP:
            continue
        stage.RemovePrim(child.GetPath())
        removed.append(name)
    print(f"[remove] deleted {len(removed)} lab_001 prims: {removed}")


def raise_worktop(stage, target_top=TABLE_TOP):
    """Cube/table 顶面抬到 target_top（编辑 lab_001 内存副本，Export 后不写回）。"""
    cube = stage.GetPrimAtPath("/World/Cube")
    if not cube.IsValid():
        print("[worktop] /World/Cube not found, skip")
        return
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"])
    top = bc.ComputeWorldBound(cube).ComputeAlignedRange().GetMax()[2]
    delta = target_top - top
    for path in ("/World/Cube", "/World/table"):
        p = stage.GetPrimAtPath(path)
        if not p.IsValid():
            continue
        ops = UsdGeom.Xformable(p).GetOrderedXformOps()
        tr = ops[0].Get()
        ops[0].Set(Gf.Vec3d(tr[0], tr[1], tr[2] + delta))
    print(f"[worktop] surface top {top:.4f} -> {target_top:.2f} (delta {delta:+.4f})")


def main():
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    stage = Usd.Stage.Open(LAB001)
    raise_worktop(stage)
    remove_lab001_equipment(stage)
    stage.Export(OUT)  # 烘平：单层自包含，带 lab_001 的 defaultPrim=/World
    print("SAVED", OUT)


if __name__ == "__main__":
    main()
