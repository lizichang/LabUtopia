# -*- coding: utf-8 -*-
"""USD 导出后处理（Blender 导出器丢 transmission 的补偿）。

用法: python post_fix_usd.py <in.usd> [--rules '{json}']
规则示例: '{"glass": {"transmission": 1.0}}'  -> 给名为 glass 的材质 Shader 加 inputs:transmission=1.0

附带功能: 删除残留的 /root/env_light（DomeLight）。
"""
import sys, json
from pxr import Usd, UsdShade, Sdf

usd_path = sys.argv[1]
rules = {}
for i, a in enumerate(sys.argv):
    if a == "--rules":
        rules = json.loads(sys.argv[i + 1])

stage = Usd.Stage.Open(usd_path)
assert stage, f"无法打开 {usd_path}"

# 1) 删除残留 env_light
for p in ("/root/env_light",):
    prim = stage.GetPrimAtPath(p)
    if prim and prim.IsValid():
        stage.RemovePrim(p)
        print("removed", p)

# 2) 按规则补 Shader 参数（PreviewSurface 的 transmission 等）
def find_shaders(prim, out):
    for child in prim.GetChildren():
        if child.GetTypeName() == "Material":
            for sub in child.GetChildren():
                if sub.GetTypeName() == "Shader":
                    out.append(sub)
        find_shaders(child, out)
    return out

shaders = find_shaders(stage.GetPseudoRoot(), [])
for sh in shaders:
    mat_name = sh.GetParent().GetName()
    if mat_name in rules:
        for attr_name, val in rules[mat_name].items():
            sh.CreateAttribute(
                f"inputs:{attr_name}", Sdf.ValueTypeNames.Float
            ).Set(val)
            print(f"set {sh.GetPath()} inputs:{attr_name} = {val}")

stage.Save()
print("SAVED", usd_path)
