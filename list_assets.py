# list_assets.py - 在服务器 labutopia 环境下跑
from pxr import Usd, UsdGeom
import os

def list_prims(usd_path, depth=2):
    """列出 USD 文件中前几层的 prim"""
    stage = Usd.Stage.Open(usd_path)
    if not stage:
        print(f"  [无法打开] {usd_path}")
        return
    
    root = stage.GetPseudoRoot()
    count = 0
    def walk(prim, level):
        nonlocal count
        if level > depth:
            return
        for child in prim.GetChildren():
            kind = child.GetTypeName()
            if kind in ('Xform', 'Scope', 'Mesh'):
                print(f"  {'  '*level}{child.GetPath()} [{kind}]")
                count += 1
            walk(child, level+1)
    
    walk(root, 0)
    print(f"  --- 共 {count} 个 prim ---\n")

base = "/media/dky/Disk2TB/lizichang/LabUtopia/assets/chemistry_lab"
scenes = [
    f"{base}/lab_001/lab_001.usd",
    f"{base}/lab_003/lab_003.usd",
    f"{base}/lab_003/clock.usd",
    f"{base}/hard_task/Scene1_hard.usd",
    f"{base}/lab_003/SubUSDs/lab_015.usd",
]

for s in scenes:
    print(f"=== {os.path.basename(s)} ===")
    if os.path.exists(s):
        list_prims(s, depth=2)
    else:
        print(f"  文件不存在: {s}\n")