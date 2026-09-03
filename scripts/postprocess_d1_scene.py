# -*- coding: utf-8 -*-
"""d1 场景后处理：塌壳 + 摆位副本提不透明度。

gen_d1 的 stage.Export 烘平引用时，会把 burette_acid.usd 内层 /burette_acid Scope
一起烘进来 → /World/BuretteAcid/burette_acid/{tube_body,ptfe_plug,...} 多一层空壳
（见记忆 usd-material-alpha-blender-visibility：资产 defaultPrim=/World 双嵌套）。

本脚本对指定 .usd：
1. 塌壳：把 /World/BuretteAcid 下"无 xformOps 且直接含 Mesh 子级"的空 Scope 的每个
   子 spec 用 Sdf.CopySpec 复制到 equip Xform 下同名子级，再删空 Scope。
   处理完 BuretteAcid 部件贴 Xform（/World/BuretteAcid/tube_body ...）。
2. --place：摆位副本专用。把 /World 下所有 opacity<0.9 的材质 surface shader 拉到
   0.95 + transmission/refractionStrength 清零 —— Blender 实体模式按 alpha 渲染，
   真玻璃 op0.15 会≈不可见；副本同几何同坐标纯渲染可见化。

用法:
  python postprocess_d1_scene.py assets/.../d1_acid_base_titration.usd            # 塌壳
  python postprocess_d1_scene.py assets/.../d1_place.usd --place                  # 塌壳+提不透明
"""
import os
import sys
from pxr import Usd, UsdGeom, UsdShade, Sdf


def flatten_geometry_scope(stage):
    """把 /World/BuretteAcid 下空 geometry Scope 的 mesh 子级上提到 Xform，删除空壳。"""
    n_moved = 0
    equip_name, scope_name = "BuretteAcid", "burette_acid"
    xform_path = f"/World/{equip_name}"
    scope_path = f"/World/{equip_name}/{scope_name}"
    xf = stage.GetPrimAtPath(xform_path)
    sc = stage.GetPrimAtPath(scope_path)
    if not (xf.IsValid() and sc.IsValid()):
        print(f"[flatten] {xform_path} 或其 {scope_name} 子级缺失, skip")
        return 0
    # 只塌空 Scope（无 xformOps）且直接含 mesh
    ops = UsdGeom.Xformable(sc).GetOrderedXformOps() if sc.IsA(UsdGeom.Xformable) else []
    if ops:
        print(f"[flatten] {scope_path} 带 xformOps {[o.GetOpName() for o in ops]}, skip")
        return 0
    layer = stage.GetRootLayer()
    xf_spec = layer.GetPrimAtPath(xform_path)
    sc_spec = layer.GetPrimAtPath(scope_path)
    child_names = list(sc_spec.nameChildren.keys())
    for cname in child_names:
        Sdf.CopySpec(layer, scope_path + "/" + cname, layer, xform_path + "/" + cname)
        n_moved += 1
    # 删空壳 scope 自身（其子 spec 已被复制走，可安全删）
    del xf_spec.nameChildren[scope_name]
    print(f"[flatten] {scope_path} -> {n_moved} 子级上提到 {xform_path}（空壳已删）")
    return n_moved


def make_place_visible(stage):
    """摆位副本：所有 opacity<0.9 材质 → 0.95，transmission/refractionStrength 清零。

    用直接枚举（Material 的 Shader 子级）而非表面输出连接解析——旧 pxr 的
    GetConnectedSource/GetConnectedSources 版本行为不一致（None/tuple/缺方法）。
    """
    n = 0
    for mat in Usd.PrimRange(stage.GetPseudoRoot()):
        if mat.GetTypeName() != "Material":
            continue
        for c in mat.GetChildren():
            if c.GetTypeName() != "Shader":
                continue
            sh = UsdShade.Shader(c)
            for inp_name in ("opacity", "transmission", "refractionStrength"):
                inp = sh.GetInput(inp_name)
                if not inp:
                    continue
                v = inp.Get()
                if inp_name == "opacity" and v is not None and v < 0.9:
                    inp.Set(0.95)
                    n += 1
                elif inp_name in ("transmission", "refractionStrength") and v:
                    inp.Set(0.0)
                    n += 1
    print(f"[place] {n} 个材质输入已调为摆位可见（op→0.95, transmission/refraction→0）")
    return n


def main():
    usd_path = sys.argv[1]
    is_place = "--place" in sys.argv[2:]
    st = Usd.Stage.Open(usd_path)
    flatten_geometry_scope(st)
    if is_place:
        make_place_visible(st)
    st.GetRootLayer().Save()
    print("SAVED", os.path.basename(usd_path), ("(摆位副本)" if is_place else "(塌壳)"))


if __name__ == "__main__":
    main()
