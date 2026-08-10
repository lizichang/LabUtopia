#!/usr/bin/env python3
"""修复 lab_flametest_v17.usd 的结构并补回缺失的效果 prim。

背景：
  v17 场景是从 Blender 导出的：所有器材包在 /root/World 下，材质在 /root/_materials，
  defaultPrim=/root。main.py 用 add_reference_to_stage(..., "/World") 按 defaultPrim
  引用整个 /root，导致运行期内容落在 /World/World/... 而不是 /World/...，
  FlameTestTask 里所有 /World/* 路径都找不到 → reset 时 TypeError 崩溃。

另外 v17 在重构为“引用类型”时丢失了火焰 / 滴液 / 水柱 / 皿内酸液等效果 prim
（旧场景 lab_flametest.usd 里有），焰色反应实验没有火焰视觉。

本脚本做两件事：
  1. 结构修复：把 /root/_materials、/root/env_light 移进 World，根 prim 改名为
     World，defaultPrim 设为 /World。这样按 defaultPrim 引用时内容直接落在 /World/...。
  2. 效果移植：从旧场景 lab_flametest.usd 把 flame_outer / flame_inner /
     flame_stain_* / Droplet / WaterJet / DishAcid（含材质）复制进 v17。

用法（仓库根目录执行）：
    python scripts/fix_flametest_v17.py

幂等：结构修复只做一次（检测到 /root 已不存在即跳过）；效果 prim 已存在则跳过。
"""
import os
import re
import sys

from pxr import Gf, Sdf, Usd, UsdShade


ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
V17 = os.path.join(ROOT, "assets", "chemistry_lab", "lab_flametest", "lab_flametest_v17.usd")
OLD = os.path.join(ROOT, "assets", "chemistry_lab", "lab_flametest", "lab_flametest.usd")


# 旧场景中需要移植的效果 prim：旧路径 -> 新场景路径
EFFECT_PRIMS = {
    "/World/BunsenBurner/flame_outer": "/World/BunsenBurner/flame_outer",
    "/World/BunsenBurner/flame_outer_mat": "/World/BunsenBurner/flame_outer_mat",
    "/World/BunsenBurner/flame_inner": "/World/BunsenBurner/flame_inner",
    "/World/BunsenBurner/flame_inner_mat": "/World/BunsenBurner/flame_inner_mat",
    "/World/BunsenBurner/flame_stain_yellow": "/World/BunsenBurner/flame_stain_yellow",
    "/World/BunsenBurner/flame_stain_yellow_mat": "/World/BunsenBurner/flame_stain_yellow_mat",
    "/World/BunsenBurner/flame_stain_purple": "/World/BunsenBurner/flame_stain_purple",
    "/World/BunsenBurner/flame_stain_purple_mat": "/World/BunsenBurner/flame_stain_purple_mat",
    "/World/BunsenBurner/flame_stain_green": "/World/BunsenBurner/flame_stain_green",
    "/World/BunsenBurner/flame_stain_green_mat": "/World/BunsenBurner/flame_stain_green_mat",
    "/World/BunsenBurner/flame_stain_red": "/World/BunsenBurner/flame_stain_red",
    "/World/BunsenBurner/flame_stain_red_mat": "/World/BunsenBurner/flame_stain_red_mat",
    "/World/BunsenBurner/flame_stain_orange": "/World/BunsenBurner/flame_stain_orange",
    "/World/BunsenBurner/flame_stain_orange_mat": "/World/BunsenBurner/flame_stain_orange_mat",
    "/World/BunsenBurner/flame_stain_blue": "/World/BunsenBurner/flame_stain_blue",
    "/World/BunsenBurner/flame_stain_blue_mat": "/World/BunsenBurner/flame_stain_blue_mat",
    "/World/Droplet": "/World/Droplet",
    "/World/Droplet_mat": "/World/Droplet_mat",
    "/World/WaterJet": "/World/WaterJet",
    "/World/WaterJet_mat": "/World/WaterJet_mat",
    "/World/DishAcid": "/World/DishAcid",
    "/World/DishAcid_mat": "/World/DishAcid_mat",
}


def _retarget_spec_paths(spec, replacements):
    """把 prim spec 子树里所有 relationship target / connection path 做前缀替换。"""
    def fix_path(path):
        s = str(path)
        for old, new in replacements:
            if s.startswith(old):
                return Sdf.Path(new + s[len(old):])
        return path

    for rel in spec.relationships:
        editor = rel.targetPathList
        items = list(editor.GetAddedOrExplicitItems())
        if not items:
            continue
        new_items = [fix_path(p) for p in items]
        if new_items != items:
            editor.ClearEdits()
            for p in new_items:
                editor.Add(p)
    for attr in spec.attributes:
        editor = attr.connectionPathList
        items = list(editor.GetAddedOrExplicitItems())
        if not items:
            continue
        new_items = [fix_path(p) for p in items]
        if new_items != items:
            editor.ClearEdits()
            for p in new_items:
                editor.Add(p)
    for child in spec.nameChildren:
        _retarget_spec_paths(child, replacements)


def _walk_prim_spec(spec, out):
    out.append(spec)
    for child in spec.nameChildren:
        _walk_prim_spec(child, out)


def fix_structure(v17_path):
    """把 Blender 导出的 /root/{World,_materials,env_light} 重排为单个 World 根 prim。"""
    layer = Sdf.Layer.FindOrOpen(v17_path)
    if layer is None:
        raise RuntimeError(f"cannot open layer: {v17_path}")
    if not layer.GetPrimAtPath("/root"):
        print("[fix] /root already gone, structure fix skipped")
        return

    # 1. 构造新 layer：根 prim World，复制 World 子树、材质、环境光
    new_layer = Sdf.Layer.CreateAnonymous("fix_flametest_v17")
    world_root = Sdf.CreatePrimInLayer(new_layer, "/World")
    world_root.typeName = "Xform"

    world = layer.GetPrimAtPath("/root/World")
    if world is None:
        raise RuntimeError("/root/World not found")
    for child in list(world.nameChildren):
        Sdf.CopySpec(layer, child.path, new_layer, Sdf.Path("/World/" + child.name))

    for src in ("/root/_materials", "/root/env_light"):
        spec = layer.GetPrimAtPath(src)
        if spec is not None:
            Sdf.CopySpec(layer, src, new_layer, Sdf.Path("/World/" + spec.name))

    # 2. 重定向所有内部路径：/root/_materials -> /World/_materials、/root/env_light -> /World/env_light、
    #    /root/World -> /World
    _retarget_spec_paths(
        new_layer.GetPrimAtPath("/World"),
        [("/root/_materials/", "/World/_materials/"),
         ("/root/env_light", "/World/env_light"),
         ("/root/World/", "/World/")],
    )

    # 用 token 形式（而非绝对路径 /World）：isaacsim kit 的 add_reference_to_stage
    # 会以 <defaultPrim> 解析，绝对路径形式在部分 kit/USD 版本里解析失败
    new_layer.defaultPrim = "World"
    _verify_layer(new_layer)
    # 返回新 layer；由调用方统一保存
    return new_layer


def _verify_layer(layer):
    stage = Usd.Stage.Open(layer)
    dp = stage.GetDefaultPrim()
    assert dp.IsValid() and dp.GetPath() == "/World", f"bad defaultPrim: {dp.GetPath()}"
    for name in ("SampleDish", "HClBottle", "PlatinumWire", "BunsenBurner", "_materials", "env_light"):
        assert stage.GetPrimAtPath(f"/World/{name}").IsValid(), f"missing /World/{name}"
    print("[fix] structure verify OK: root=/World, defaultPrim=/World")


def port_effects(target_layer, old_path):
    """从旧场景移植缺失的效果 prim（含材质）到 target_layer。"""
    old_layer = Sdf.Layer.FindOrOpen(old_path)
    if old_layer is None:
        raise RuntimeError(f"cannot open old scene: {old_path}")

    n = 0
    for old_p, new_p in EFFECT_PRIMS.items():
        if target_layer.GetPrimAtPath(new_p):
            continue
        old_spec = old_layer.GetPrimAtPath(old_p)
        if old_spec is None:
            print(f"[port] WARN source missing: {old_p}")
            continue
        Sdf.CopySpec(old_layer, old_p, target_layer, Sdf.Path(new_p))
        n += 1
    print(f"[port] copied {n} effect prims")


def bind_stain_materials(target_layer):
    """给 flame_stain_* 锥体绑定对应颜色材质（旧场景遗留：锥体无材质绑定，颜色显示不出来）。"""
    stage = Usd.Stage.Open(target_layer)
    colors = ("yellow", "purple", "green", "red", "orange", "blue")
    n = 0
    for color in colors:
        cone = stage.GetPrimAtPath(f"/World/BunsenBurner/flame_stain_{color}")
        mat = stage.GetPrimAtPath(f"/World/BunsenBurner/flame_stain_{color}_mat")
        if not cone.IsValid() or not mat.IsValid():
            continue
        if UsdShade.MaterialBindingAPI(cone).GetDirectBinding().GetMaterial():
            continue
        UsdShade.MaterialBindingAPI(cone).Bind(UsdShade.Material(mat))
        n += 1
    print(f"[port] bound {n} stain cone materials")


def reposition_dish(target_layer):
    """把表面皿从不可及位置 (0.6682,-0.22) 移到 (0.32,-0.22)。

    v22：v17 USD 里 SampleDish 在 x=0.6682，超出 Franka（base x=-0.3）桌面高度的
    实际工作半径（约 x≤0.5），RMP 卡在 x≈0.48 无法到位，P1 永远抓不到盘子。
    """
    spec = target_layer.GetPrimAtPath("/World/SampleDish")
    if spec is None:
        print("[dish] /World/SampleDish not found, skip")
        return
    tr = next((a for a in spec.attributes if a.name == "xformOp:translate"), None)
    cur = tr.default if tr is not None else None
    if cur is None:
        print("[dish] no translate op, skip")
        return
    cur_vec = Gf.Vec3d(*cur) if not isinstance(cur, Gf.Vec3d) else cur
    if abs(cur_vec[0] - 0.32) < 1e-4 and abs(cur_vec[1] + 0.22) < 1e-4:
        print("[dish] already at reachable position, skip")
        return
    tr.default = Gf.Vec3d(0.32, -0.22, 0.80)
    print(f"[dish] moved SampleDish {tuple(round(float(v), 4) for v in cur_vec)} -> (0.32, -0.22, 0.80)")


def fix_lighting(target_layer):
    """修复场景几乎全黑的问题。

    v17 的 env_light 是 Blender 导出的 DomeLight：贴图 color_0C0C0C.exr 几乎纯黑、
    intensity=1.0，等于没有环境光，桌面/器材/机械臂全部渲染成黑色（只有自发光的
    火焰可见）。保留贴图（RTX 的 DomeLight 需要贴图），把强度提到 4000，
    使有效光强 ≈ 4000×0.05 ≈ 200，得到均匀明亮的场景照明。
    """
    spec = target_layer.GetPrimAtPath("/World/env_light")
    if spec is None:
        print("[light] /World/env_light not found, skip")
        return
    tex = next((a for a in spec.attributes if a.name == "inputs:texture:file"), None)
    if tex is not None:
        cur = str(tex.default) if tex.default is not None else ""
        if "color_0C0C0C" not in cur:
            tex.default = Sdf.AssetPath("./textures/color_0C0C0C.exr")
            print(f"[light] restored env_light texture (was {cur!r})")
    intens = next((a for a in spec.attributes if a.name == "inputs:intensity"), None)
    if intens is None:
        # layer 里没有 intensity 属性（1.0 是 DomeLight schema 默认值），需要先创建
        intens = Sdf.AttributeSpec(spec, "inputs:intensity", Sdf.ValueTypeNames.Float)
    intens.default = 4000.0
    print("[light] env_light intensity -> 4000 (effective ~200)")


def main():
    if not os.path.exists(V17):
        print(f"ERROR: {V17} not found", file=sys.stderr)
        sys.exit(1)
    if not os.path.exists(OLD):
        print(f"ERROR: {OLD} not found", file=sys.stderr)
        sys.exit(1)

    layer = Sdf.Layer.FindOrOpen(V17)
    if layer is None:
        print(f"ERROR: cannot open {V17}", file=sys.stderr)
        sys.exit(1)

    if layer.GetPrimAtPath("/root"):
        layer = fix_structure(V17)
        assert layer is not None

    port_effects(layer, OLD)
    bind_stain_materials(layer)
    reposition_dish(layer)
    fix_lighting(layer)

    # 校验后写回原文件（先写临时文件再原子替换，避免半写状态）
    # 注意：defaultPrim 必须写成 token 形式（"World"）而非绝对路径（"/World"），
    # 否则 isaacsim kit 的 add_reference_to_stage 以 <defaultPrim> 引用时解析失败。
    tmp_usda = V17 + ".new.usda"
    tmp_crate = V17 + ".new.usd"
    for p in (tmp_usda, tmp_crate):
        if os.path.exists(p):
            os.remove(p)
    layer.Export(tmp_usda)
    with open(tmp_usda, "r", encoding="utf-8") as f:
        text = f.read()
    text = re.sub(r'defaultPrim = "/World"', 'defaultPrim = "World"', text, count=1)
    with open(tmp_usda, "w", encoding="utf-8") as f:
        f.write(text)
    token_layer = Sdf.Layer.FindOrOpen(tmp_usda)
    assert token_layer.defaultPrim == "World", f"defaultPrim not tokenized: {token_layer.defaultPrim!r}"
    token_layer.Export(tmp_crate)
    _verify_layer(token_layer)
    os.replace(tmp_crate, V17)
    if os.path.exists(tmp_usda):
        os.remove(tmp_usda)
    print(f"[fix] saved: {V17}")
    print("DONE")


if __name__ == "__main__":
    main()
