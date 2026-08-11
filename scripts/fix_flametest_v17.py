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

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade


ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
V17 = os.path.join(ROOT, "assets", "chemistry_lab", "lab_flametest", "lab_flametest_v17.usd")
OLD = os.path.join(ROOT, "assets", "chemistry_lab", "lab_flametest", "lab_flametest.usd")
LAMP = os.path.join(ROOT, "assets", "chemistry_lab", "alcohol_lamp.usd")

# v24：酒精灯替换本生灯后的世界位置（桌面 z=0.80）
LAMP_POS = (0.36, 0.18, 0.80)
# v24：表面皿固定位置（P3 滴液 / P5 蘸酸都在该处，机械臂不再搬动盘子）
DISH_FIXED_POS = (0.20, 0.02, 0.80)


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
    for name in ("SampleDish", "HClBottle", "PlatinumWire", "AlcoholLamp", "_materials", "env_light"):
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
        # 本生灯已删除（酒精灯替换），不再往其下移植火焰/染色锥
        if new_p.startswith("/World/BunsenBurner") and target_layer.GetPrimAtPath("/World/BunsenBurner") is None:
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
    """把表面皿固定到实验位 (0.20,0.02)，机械臂不再搬动它。

    v24：滴液（P3）和铂丝蘸酸（P5）都发生在表面皿上，因此盘子直接放在
    (0.20,0.02,0.80)，删除"取表面皿放中央"步骤（P1 / P13）。
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
    if (abs(cur_vec[0] - DISH_FIXED_POS[0]) < 1e-4
            and abs(cur_vec[1] - DISH_FIXED_POS[1]) < 1e-4
            and abs(cur_vec[2] - DISH_FIXED_POS[2]) < 1e-4):
        print("[dish] already at fixed position, skip")
    else:
        tr.default = Gf.Vec3d(*DISH_FIXED_POS)
        print(f"[dish] moved SampleDish {tuple(round(float(v), 4) for v in cur_vec)} -> {DISH_FIXED_POS}")
    # v25：表面皿放大 1.6x（6cm -> 9.6cm），用户反馈"玻璃皿看不清"，小皿在
    # 512px 快照里只有 ~25px。围绕局部原点缩放，中心仍在 (0.20,0.02)，不影响
    # 滴液/蘸酸等世界坐标操作。皿内实心圆盘随父级同步放大。
    scale = next((a for a in spec.attributes if a.name == "xformOp:scale"), None)
    if scale is not None:
        scale.default = Gf.Vec3f(1.6, 1.6, 1.6)
        print("[dish] SampleDish scaled 1.6x (6cm -> 9.6cm)")


def fix_lighting(target_layer):
    """修复场景几乎全黑的问题。

    v17 的 env_light 是 Blender 导出的 DomeLight：贴图 color_0C0C0C.exr 几乎纯黑、
    intensity=1.0，等于没有环境光，桌面/器材/机械臂全部渲染成黑色（只有自发光的
    火焰可见）。保留贴图（RTX 的 DomeLight 需要贴图），把强度提到 10000，
    使有效光强 ≈ 10000×0.05 ≈ 500，得到明亮均匀的场景照明。
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
    intens.default = 10000.0
    print("[light] env_light intensity -> 10000 (effective ~500)")


def fix_cylinder_light(target_layer):
    """效仿 lab_001：把 v17 里无效的 SphereLight（radius=0）原位改成 CylinderLight。

    v17 有个从 Blender 导出的光 prim /World/CylinderLight_010/CylinderLight_010，
    但被导出成 radius=0 的 SphereLight（体积为 0，不产生照明），而 env_light 的
    DomeLight 贴图又是纯黑（color_0C0C0C.exr 是 1×1 黑像素），所以场景全黑——
    这就是"调 env_light intensity 没用"的根本原因。

    lab_001（明亮参考）只用一个 CylinderLight（intensity=2000、length=100、
    radius=5、白光）就照亮整个场景，位置就在 (2.1, 1.057, 7)。这里照做，
    把已存在的 SphereLight 原位转成 CylinderLight（Xform 父节点已有同款位置），
    最小改动、幂等。
    """
    light_path = "/World/CylinderLight_010/CylinderLight_010"
    spec = target_layer.GetPrimAtPath(light_path)
    if spec is None:
        print("[light] /World/CylinderLight_010/CylinderLight_010 not found, skip")
        return
    if spec.typeName != "CylinderLight":
        spec.typeName = "CylinderLight"
        print("[light] SphereLight -> CylinderLight")

    def _set_attr(attr_name, value, vtype):
        attr = next((a for a in spec.attributes if a.name == attr_name), None)
        if attr is None:
            attr = Sdf.AttributeSpec(spec, attr_name, vtype)
        attr.default = value

    _set_attr("inputs:intensity", 1000.0, Sdf.ValueTypeNames.Float)
    _set_attr("inputs:length", 100.0, Sdf.ValueTypeNames.Float)
    _set_attr("inputs:radius", 5.0, Sdf.ValueTypeNames.Float)
    print("[light] CylinderLight (intensity 1000, length 100, radius 5, white)")


def swap_burner_to_lamp(target_layer, lamp_usd_path):
    """用酒精灯替换本生灯（v24）。

    - 本生灯的染色锥 flame_stain_*（含材质）迁到酒精灯下，scale 0.5 -> 1.5：
      酒精灯火焰半径 9mm，染色锥 12mm×1.5=18mm 能探出火焰，实现"只有铂丝
      周围火焰变黄"的局部焰色效果
    - alcohol_lamp.usd 的部件复制到 /World/AlcoholLamp（世界位置 LAMP_POS），
      内部材质绑定重定向到 /World/AlcoholLamp/_materials/
    - 删除 /World/BunsenBurner
    """
    if target_layer.GetPrimAtPath("/World/AlcoholLamp") is not None:
        print("[lamp] AlcoholLamp already present, skip")
        return
    lamp_layer = Sdf.Layer.FindOrOpen(lamp_usd_path)
    if lamp_layer is None:
        print("[lamp] WARN cannot open alcohol_lamp.usd, skip lamp swap")
        return

    # 0. 先建 AlcoholLamp 容器
    Sdf.CreatePrimInLayer(target_layer, "/World/AlcoholLamp")
    target_layer.GetPrimAtPath("/World/AlcoholLamp").typeName = "Xform"

    # 1. 迁移染色锥（本生灯还在）
    stain_colors = ("yellow", "purple", "green", "red", "orange", "blue")
    for color in stain_colors:
        src = f"/World/BunsenBurner/flame_stain_{color}"
        src_mat = f"/World/BunsenBurner/flame_stain_{color}_mat"
        if target_layer.GetPrimAtPath(src) is not None:
            Sdf.CopySpec(target_layer, src, target_layer,
                         Sdf.Path(f"/World/AlcoholLamp/flame_stain_{color}"))
        if target_layer.GetPrimAtPath(src_mat) is not None:
            Sdf.CopySpec(target_layer, src_mat, target_layer,
                         Sdf.Path(f"/World/AlcoholLamp/flame_stain_{color}_mat"))
    for color in stain_colors:
        cone = target_layer.GetPrimAtPath(f"/World/AlcoholLamp/flame_stain_{color}")
        if cone is None:
            continue
        _retarget_spec_paths(cone, [("/World/BunsenBurner/", "/World/AlcoholLamp/")])
        scale = next((a for a in cone.attributes if a.name == "xformOp:scale"), None)
        if scale is not None:
            scale.default = Gf.Vec3f(2.2, 2.2, 2.2)
    print("[lamp] stain cones migrated to AlcoholLamp (scale 2.2)")

    # 2. 复制酒精灯部件（/root 下所有子 prim）
    lamp_root = lamp_layer.GetPrimAtPath("/root")
    if lamp_root is not None:
        for child in list(lamp_root.nameChildren):
            Sdf.CopySpec(lamp_layer, child.path, target_layer,
                         Sdf.Path(f"/World/AlcoholLamp/{child.name}"))
    lamp_prim = target_layer.GetPrimAtPath("/World/AlcoholLamp")
    _retarget_spec_paths(
        lamp_prim,
        [("/root/_materials/", "/World/AlcoholLamp/_materials/"),
         ("/root/", "/World/AlcoholLamp/")],
    )
    tr = next((a for a in lamp_prim.attributes if a.name == "xformOp:translate"), None)
    if tr is None:
        tr = Sdf.AttributeSpec(lamp_prim, "xformOp:translate", Sdf.ValueTypeNames.Double3)
    tr.default = Gf.Vec3d(*LAMP_POS)
    order = next((a for a in lamp_prim.attributes if a.name == "xformOpOrder"), None)
    if order is None:
        order = Sdf.AttributeSpec(lamp_prim, "xformOpOrder", Sdf.ValueTypeNames.TokenArray)
        order.default = ["xformOp:translate"]
    print(f"[lamp] AlcoholLamp at {LAMP_POS}")

    # 3. 删除本生灯
    stage = Usd.Stage.Open(target_layer)
    if stage.GetPrimAtPath("/World/BunsenBurner").IsValid():
        stage.RemovePrim("/World/BunsenBurner")
        print("[lamp] removed /World/BunsenBurner")
    else:
        print("[lamp] BunsenBurner already removed")


def ensure_lamp_root_def(target_layer):
    """把 /World/AlcoholLamp 从 Over 转成 Def（幂等）。

    swap_burner_to_lamp 用 Sdf.CreatePrimInLayer 建灯容器，默认 specifier 是 Over。
    Over 根 prim 无底层 Def，在严格合成的 USD 查看器里可能不显示（用户打开文件
    "World 层级下没有酒精灯"），且部分工具里世界变换合成异常（usd-core 读灯子
    节点 world=[0,0,0]）。转成 Def 后任何工具都能正确显示与合成。
    """
    prim = target_layer.GetPrimAtPath("/World/AlcoholLamp")
    if prim is None:
        print("[lamp] AlcoholLamp missing, nothing to do")
        return
    if prim.specifier == Sdf.SpecifierDef:
        print("[lamp] AlcoholLamp already Def")
        return
    prim.specifier = Sdf.SpecifierDef
    print(f"[lamp] AlcoholLamp specifier Over -> Def (type={prim.typeName})")


# 每种焰色的 HDR 饱和色（与 tasks/flametest_task.py 的 FLAME_COLORS 一致）。
# v25：染色锥不再统一写成黄色——blue/green/red 等要显示各自特征色。
# v27（2026-08-11 diag_stain.py 决定性测试）：必须是"单通道主导"结构——
#   主导通道 ≈1.6，其余通道 ≤0.55。测试：yellow (1.6,0.55,0.15) 渲染出饱和黄
#   (227,216,109)；而 R 和 G 同时 >1 的旧值 (1.8,1.296,0.216) 被 CylinderLight
#   洗成奶油白 (243,237,162)。与皿内蓝盘 (0.15,0.55,1.6) 同原理：近黑 diffuse +
#   单通道高 emissive 才能出饱和色。值直接是最终 emissive，不再 ×1.8。
STAIN_COLORS = {
    "yellow": (1.60, 0.55, 0.15),   # 钠焰饱和黄（单通道 R 主导，已实测）
    "purple": (1.20, 0.35, 1.60),   # 钾焰紫（R+B 主导，G 压低）
    "green":  (0.35, 1.60, 0.45),   # 铜焰绿（G 主导）
    "red":    (1.60, 0.30, 0.20),   # 锶焰红（R 主导）
    "orange": (1.60, 0.45, 0.10),   # 钙焰橙（R 主导、G 次之）
    "blue":   (0.30, 0.55, 1.60),   # 铜蓝焰（B 主导，同皿内蓝盘良方）
}

# v26 染色锥烘焙几何尺寸（原 height/radius × scale 2.2）：让锥体不再依赖
# xformOp:scale（RTX 对 Cone prim 的 scale 在某些合成上下文下不渲染）。
# v30.2 缩小：用户反馈"火焰没那么大"——原 r=26mm h=66mm 的染色锥比真火焰还
# 大一圈，被当成"建模里的火焰太大"。
# v30.4 水滴形：用户反馈"染色锥形状不对，要像火焰那样上尖下圆，大小只比铂丝
# 头部的球（7mm）大一点点"。不再是直线锥 Cone，改用 _teardrop_flame_mesh 的
# 水滴 lathe 曲面（下圆上尖）。
# v30.5 放大 1.5 倍 + 定位铂丝尖端：用户反馈"有点小，放大 1.5 倍；初始定位不该
# 在酒精灯上，应在铂丝尖端"。尺寸 15mm 直径×22.5mm 高；translate 对齐铂丝头球
# 世界坐标 (0.5456,-0.0417,0.8101)。运行时浸入灯焰时 task 的 _position_stain_at_tip
# 会同步到实时尖端（保持 /World 独立 prim，不改父级）。
STAIN_GEOM = {
    "height": 0.0225,         # 水滴火焰高 ×1.5（22.5mm）
    "radius": 0.0075,         # 最宽处半径 ×1.5（直径 15mm）
}


def relocate_stain_cones(target_layer):
    """把染色锥/材质从 /World/AlcoholLamp/ 迁到顶层 /World/（v26 修复任务 #6）。

    诊断结论（diag_stain.py）：/World/AlcoholLamp 是引用型 over Xform，其下
    over 添加的子 prim（flame_stain_*）在 RTX usdrt population 里不渲染——
    无论位置/大小/材质都完全不可见；而顶层新增的 Cone 正常渲染。
    因此把染色锥放到 /World/flame_stain_{color}（顶层），材质放
    /World/flame_stain_{color}_mat，并把 scale 烘焙进 height/radius。
    幂等：顶层目标已存在则跳过复制，只清理旧位置。
    """
    colors = tuple(STAIN_COLORS)
    n = 0
    for color in colors:
        src = f"/World/AlcoholLamp/flame_stain_{color}"
        dst = f"/World/flame_stain_{color}"
        src_mat = f"/World/AlcoholLamp/flame_stain_{color}_mat"
        dst_mat = f"/World/flame_stain_{color}_mat"
        src_spec = target_layer.GetPrimAtPath(src)
        dst_spec = target_layer.GetPrimAtPath(dst)
        srcm_spec = target_layer.GetPrimAtPath(src_mat)
        if dst_spec is None and src_spec is not None:
            Sdf.CopySpec(target_layer, src_spec.path, target_layer, Sdf.Path(dst))
            dst_spec = target_layer.GetPrimAtPath(dst)
            _retarget_spec_paths(dst_spec, [("/World/AlcoholLamp/", "/World/")])
            # 烘焙 scale -> 几何尺寸，删除 scale op
            _bake_cone_geometry(dst_spec)
            n += 1
        if target_layer.GetPrimAtPath(dst_mat) is None and srcm_spec is not None:
            Sdf.CopySpec(target_layer, srcm_spec.path, target_layer, Sdf.Path(dst_mat))
            _retarget_spec_paths(target_layer.GetPrimAtPath(dst_mat),
                                 [("/World/AlcoholLamp/", "/World/")])
    # 清理灯下旧染色锥/材质（不渲染的残留）
    stage = Usd.Stage.Open(target_layer)
    removed = 0
    for color in colors:
        for p in (f"/World/AlcoholLamp/flame_stain_{color}",
                  f"/World/AlcoholLamp/flame_stain_{color}_mat"):
            if stage.GetPrimAtPath(p).IsValid():
                stage.RemovePrim(p)
                removed += 1
    print(f"[lamp] relocated {n} stain cones to /World top level (scale baked, "
          f"{removed} old prims removed)")


def _bake_cone_geometry(cone_spec):
    """把 Cone prim spec 的 xformOp:scale 烘焙成 height/radius，删掉 scale op。"""
    for attr in cone_spec.attributes:
        if attr.name == "height":
            attr.default = STAIN_GEOM["height"]
        elif attr.name == "radius":
            attr.default = STAIN_GEOM["radius"]
    scale = next((a for a in cone_spec.attributes if a.name == "xformOp:scale"), None)
    if scale is not None:
        cone_spec.RemoveProperty(scale)
    order = next((a for a in cone_spec.attributes if a.name == "xformOpOrder"), None)
    if order is not None:
        order.default = [tok for tok in order.default if tok != "xformOp:scale"]


def rebuild_stain_materials(target_layer):
    """用已知良方重建染色锥 + 材质（任务 #6，2026-08-11 诊断确认）。

    诊断（diag_stain.py）：relocate_stain_cones 用 Sdf.CopySpec 复制的染色锥
    /World/flame_stain_{color} 和材质，即使 Sdf dump 显示结构/值都正确，RTX 也
    不渲染——锥体渲染成与奶油台面同色(244,238,160)，完全看不见。对照：
      * 运行时新建的 Cone + dish_visible_disk_mat → 渲染出饱和蓝(135,212,242)
      * 皿内蓝盘（UsdGeom.Mesh.Define 新建 + 新建材质）→ 正常渲染
    结论：Sdf.CopySpec 复制来的 prim/material 带 usdrt population 不认的缺陷，
    必须删掉用 Usd API 重建（Cone.Define + Material.Define，近黑 diffuse +
    强 emissive）。幂等：每次运行都删旧重建并重新绑定。
    """
    stage = Usd.Stage.Open(target_layer)
    for color, base in STAIN_COLORS.items():
        cone_path = f"/World/flame_stain_{color}"
        mat_path = f"/World/flame_stain_{color}_mat"
        # 锥体：删掉旧 prim，重建为全新水滴 Mesh（v30.4：上尖下圆、只比铂丝头球
        # 7mm 大一点；几何烘焙，translate 对齐当前灯位）
        old_cone = stage.GetPrimAtPath(cone_path)
        if old_cone.IsValid():
            stage.RemovePrim(cone_path)
        pts, counts, idx, nrm = _teardrop_flame_mesh(
            STAIN_GEOM["height"], STAIN_GEOM["radius"])
        cone = UsdGeom.Mesh.Define(stage, cone_path)
        cone.GetPointsAttr().Set([Gf.Vec3f(*p) for p in pts])
        cone.GetFaceVertexCountsAttr().Set(counts)
        cone.GetFaceVertexIndicesAttr().Set(idx)
        cone.GetNormalsAttr().Set([Gf.Vec3f(*n) for n in nrm])
        cone.SetNormalsInterpolation("faceVarying")
        xs = [p[0] for p in pts]; ys = [p[1] for p in pts]; zs = [p[2] for p in pts]
        cone.GetExtentAttr().Set([(min(xs), min(ys), min(zs)),
                                  (max(xs), max(ys), max(zs))])
        cone.GetDoubleSidedAttr().Set(True)
        cone.GetSubdivisionSchemeAttr().Set("none")
        # v30.5：translate 对齐铂丝头球（用户反馈初始定位不该在酒精灯上，应在
        # 铂丝尖端）。球心世界坐标 (0.5456,-0.0417,0.8101)。运行时浸入灯焰时
        # task 的 _position_stain_at_tip 会同步到实时尖端，烘焙值只作静止位。
        UsdGeom.Xformable(cone).AddTranslateOp().Set(Gf.Vec3d(0.5456, -0.0417, 0.8101))
        # 材质：删旧重建（近黑 diffuse + 强 emissive）
        old_mat = stage.GetPrimAtPath(mat_path)
        if old_mat.IsValid():
            stage.RemovePrim(mat_path)
        mat = UsdShade.Material.Define(stage, mat_path)
        shader = UsdShade.Shader.Define(stage, mat_path + "/Shader")
        shader.CreateIdAttr("UsdPreviewSurface")
        emissive = tuple(base)  # v27: STAIN_COLORS 已是最终 HDR 值（单通道主导）
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
            Gf.Vec3f(0.01, 0.01, 0.01))
        shader.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(
            Gf.Vec3f(*emissive))
        shader.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(0.5)  # v30.5: 小火焰有点透明
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.3)  # 同皿内蓝盘/实测锥
        shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
        mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
        UsdShade.MaterialBindingAPI(cone).Bind(mat)
    print("[lamp] rebuilt stain flames fresh (v30.4 teardrop Mesh + dish recipe, "
          f"{STAIN_GEOM['height']*1000:.0f}mm tall x {STAIN_GEOM['radius']*2000:.0f}mm wide)")


def rebuild_flame_cones(target_layer):
    """把火焰锥从引用灯下迁到 /World 顶层并重建（v30.2 用户反馈"没看到火焰"）。

    根因：flame_outer/flame_inner 原本在 /World/AlcoholLamp 引用 prim 之下（over
    子 prim），RTX usdrt population 对这类子 prim 不渲染——与染色锥当初从
    /World/AlcoholLamp/flame_stain_* 迁到 /World 顶层才渲染完全同一现象（见
    relocate_stain_cones 注释）。此前"蓝焰可见"的验证其实是被皿内蓝圆盘
    （已删）的蓝像素污染的，真火焰从未渲染。

    修复：删旧火焰锥+材质，在 /World 顶层用 Cone.Define 重建——几何烘焙进
    height/radius、translate 用世界坐标（火焰底 0.903 在灯芯顶 0.9005 上方、
    顶端 ~0.933，落在任务 FLAME_Z(0.898,0.940) 检测带内）、近黑 diffuse + 强
    HDR 蓝 emissive（已验证良方）、默认 visible（不设 invisible——prim 在 population
    时 invisible 会阻止 RTX 材质初始化，之后翻 visible 仍渲染默认灰；熄灭由任务
    reset() 的 _set_flame_visible(False) 负责，点亮时再翻 visible）。

    v30.2 蓝焰材质修正（diag_flame9b/10/11 决定性测试）：原 emissive
    (0.50,0.90,1.80)/(0.90,1.30,2.20) 违反"单通道主导"配方（非主导通道 >0.55），
    RTX 渲染成默认灰 (177,186,184)，火焰完全不可见。把火焰锥绑定到已实测可渲染
    的黄色材质后同一根锥立即渲染出亮黄 (241,221,182)——证明几何/绑定/隐藏-显示
    流程全好，问题只在蓝色材质值。按配方提高主导 B、压低 R/G：outer
    (0.20,0.45,2.20)、inner (0.35,0.60,2.60) → 火焰核心渲染 (192,217,247)、
    B−R=55 的可见蓝焰（原值 B−R=35 偏白）。
    """
    stage = Usd.Stage.Open(target_layer)
    for path in ("/World/AlcoholLamp/flame_outer", "/World/AlcoholLamp/flame_inner",
                 "/World/AlcoholLamp/_materials/flame_outer_mat",
                 "/World/AlcoholLamp/_materials/flame_inner_mat"):
        if stage.GetPrimAtPath(path).IsValid():
            stage.RemovePrim(path)
    for name, h, r, zc, emissive in (
            ("flame_outer", 0.030, 0.007, 0.918, (0.20, 0.45, 2.20)),
            ("flame_inner", 0.018, 0.004, 0.9115, (0.35, 0.60, 2.60))):
        cone = UsdGeom.Cone.Define(stage, f"/World/{name}")
        cone.GetHeightAttr().Set(h)
        cone.GetRadiusAttr().Set(r)
        # 幂等：prim 若已存在（上一次运行建的），复用已有 translate op；缺失才新增。
        xform = UsdGeom.Xformable(cone)
        t_op = None
        for op in xform.GetOrderedXformOps():
            if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                t_op = op
                break
        if t_op is None:
            t_op = xform.AddTranslateOp()
        t_op.Set(Gf.Vec3d(0.36, 0.18, zc))
        # 不设默认 invisible：prim 若在 population 时 invisible，RTX 材质不初始化、
        # 之后翻 visible 仍渲染成默认灰（实测火焰蓝材质不显、红探针却显）。让任务
        # reset() 的 _set_flame_visible(False) 负责熄灭，点着时再翻 visible。
        UsdGeom.Imageable(cone).GetVisibilityAttr().Clear()  # 清旧 invisible，默认可见
        mat_path = f"/World/{name}_mat"
        mat = UsdShade.Material.Define(stage, mat_path)
        shader = UsdShade.Shader.Define(stage, mat_path + "/Shader")
        shader.CreateIdAttr("UsdPreviewSurface")
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
            Gf.Vec3f(0.01, 0.01, 0.01))
        shader.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(
            Gf.Vec3f(*emissive))
        shader.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(1.0)
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.3)
        shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
        mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
        UsdShade.MaterialBindingAPI(cone).Bind(mat)
    print("[lamp] flame cones relocated to /World top level + rebuilt (HDR blue)")


def tune_lamp_visuals(target_layer):
    """调优酒精灯火焰/染色锥渲染，让局部黄色与蓝色火焰对比明显。

    v25 修正（任务 #7 焰色正确性）：
    - 酒精灯火焰改为蓝色（酒精燃烧是蓝焰）：flame_outer 蓝、flame_inner 白蓝。
      之前被写成橙色/黄色，黄色染色锥在黄焰里几乎看不见（#6 可见性未确认的
      主因之一）。
    - 染色锥各用其特征色（HDR emissive > 1，探出蓝色火焰看得清），不再
      统一写成黄色。黄色锥保持饱和黄，与蓝焰形成对比。
    - 染色锥 scale 保持 2.2（r=26mm）探出火焰（r=9mm），实现"只有铂丝
      周围一圈变黄"的局部焰色。
    """
    def set_shader_colors(mat_path, diffuse=None, emissive=None, opacity=None):
        for ch_name in ("Shader", "原理化_BSDF", "Principled_BSDF"):
            shader = target_layer.GetPrimAtPath(f"{mat_path}/{ch_name}")
            if shader is None:
                continue
            for attr_name, val in (("inputs:diffuseColor", diffuse),
                                   ("inputs:emissiveColor", emissive),
                                   ("inputs:opacity", opacity)):
                if val is None:
                    continue
                attr = next((a for a in shader.attributes if a.name == attr_name), None)
                if attr is None:
                    attr = Sdf.AttributeSpec(shader, attr_name,
                                             Sdf.ValueTypeNames.Float3 if "Color" in attr_name else Sdf.ValueTypeNames.Float)
                attr.default = Gf.Vec3f(*val) if "Color" in attr_name else float(val)
            return

    # 蓝焰（酒精灯）
    set_shader_colors("/World/AlcoholLamp/_materials/flame_outer_mat",
                      diffuse=(0.25, 0.50, 0.85), emissive=(0.32, 0.62, 0.98), opacity=0.85)
    set_shader_colors("/World/AlcoholLamp/_materials/flame_inner_mat",
                      diffuse=(0.55, 0.80, 1.00), emissive=(0.55, 0.88, 1.20), opacity=0.90)
    # v30.2 火焰尺寸：用户反馈"火焰应该只有灯芯处有、是火焰形状、没那么大"。
    # 原 outer h=35mm r=9mm、inner h=22mm r=5mm 的实心锥是块大色块。缩成细长
    # 火舌——outer r=6mm h=28mm、inner r=3.5mm h=16mm，底仍在灯芯顶 0.9005
    # 之上、顶端 ~0.93，只有灯芯处有火。Cone prim 直接 Sdf 改 height/radius。
    stage = Usd.Stage.Open(target_layer)
    for name, h, r in (("flame_outer", 0.028, 0.006),
                       ("flame_inner", 0.016, 0.0035)):
        prim = stage.GetPrimAtPath(f"/World/AlcoholLamp/{name}")
        if prim.IsValid():
            UsdGeom.Cone(prim).GetHeightAttr().Set(h)
            UsdGeom.Cone(prim).GetRadiusAttr().Set(r)
    # 染色锥材质由 rebuild_stain_materials() 全新重建（已知良方：近黑 diffuse +
    # 强 emissive），这里不再重复处理。
    print("[lamp] visuals tuned (blue flame resized 35->28mm / stain rebuilt)")


def _set_shader_params(target_layer, mat_path, diffuse=None, emissive=None,
                       opacity=None, metallic=None, roughness=None, ior=None):
    """通用：设置材质 shader（任意子 prim 名）的常见参数。返回是否找到 shader。"""
    for ch_name in ("Shader", "原理化_BSDF", "Principled_BSDF"):
        shader = target_layer.GetPrimAtPath(f"{mat_path}/{ch_name}")
        if shader is None:
            continue
        for attr_name, val in (("inputs:diffuseColor", diffuse),
                               ("inputs:emissiveColor", emissive),
                               ("inputs:opacity", opacity),
                               ("inputs:metallic", metallic),
                               ("inputs:roughness", roughness),
                               ("inputs:ior", ior)):
            if val is None:
                continue
            attr = next((a for a in shader.attributes if a.name == attr_name), None)
            if attr is None:
                attr = Sdf.AttributeSpec(
                    shader, attr_name,
                    Sdf.ValueTypeNames.Float3 if "Color" in attr_name else Sdf.ValueTypeNames.Float)
            attr.default = (Gf.Vec3f(*val) if "Color" in attr_name else float(val))
        return True
    return False


def tune_lamp_materials(target_layer):
    """给酒精灯各部件上真实材质颜色（任务 #7 + 用户反馈"灯看不见"）。

    根因 A：alcohol_lamp.usd 导出时所有部件材质都是同一种灰 (0.8,0.8,0.8)，
    在修亮后的场景里与桌面同色、完全看不出灯。
    根因 B（本次新增）：玻璃灯身 body 是「中空薄壁玻璃壳」，RTX 无视
    diffuse/opacity/ior 一律按透射玻璃渲染成中性灰（cap/liquid/holder 是实体
    所以正常显色）。修复：给玻璃类材质加 emissiveColor 自发光——RTX 对
    emissive 的贡献是叠加的，不受透射影响（flame/stain 锥体就是用 emissive
    渲染正常的，已验证）。玻璃身/液面加浅蓝自发光 → 快照里能看出灯。

    - ceramic 陶瓷座：白瓷（奶油白）
    - glass 玻璃灯身：浅蓝玻璃 + 浅蓝自发光
    - liquid 酒精液：半透明淡黄 + 微自发光（透过玻璃身可见）
    - cotton 灯芯：棉白
    - char 烧焦芯端：深黑
    - capglass 灯帽：浅蓝玻璃 + 浅蓝自发光
    同时给玻璃身 mesh 开 doubleSided，防内壁全黑（同滴管问题）。
    """
    base = "/World/AlcoholLamp/_materials"
    _set_shader_params(target_layer, f"{base}/ceramic",
                       diffuse=(0.93, 0.91, 0.88), roughness=0.55, metallic=0.0)
    # 玻璃灯身：近无色玻璃（v30.2 用户反馈"酒精灯是黄色为啥不是玻璃"——之前
    # 用琥珀 diffuse+emissive 把整个灯身渲染成实心黄块，不像玻璃。玻璃身是
    # 中空薄壁玻璃壳，RTX 无视其 diffuse/opacity 只认 emissive，把 emissive 压到
    # 极弱（仅够描出壶形轮廓、不被当成透明空气），琥珀色由内部实心酒精液
    # （liquid，HDR 亮黄自发光）透过壶身透出 → "玻璃壶装酒精"的真实观感）。
    # v30.3 用户反馈"玻璃不够透明、酒精淡黄诡异"：玻璃 opacity 0.45->0.35 更透；
    # 同时酒精液 emissive 从 HDR 1.15 压到 ~0.28（不再整壶发亮黄）、opacity 1.0
    # ->0.7 半透明、diffuse 暖化，像"透明玻璃壶装淡琥珀酒精"而非发光黄块。
    _set_shader_params(target_layer, f"{base}/glass",
                       diffuse=(0.82, 0.85, 0.90), emissive=(0.05, 0.06, 0.07),
                       opacity=0.35, roughness=0.1)
    _set_shader_params(target_layer, f"{base}/capglass",
                       diffuse=(0.82, 0.85, 0.90), emissive=(0.05, 0.06, 0.07),
                       opacity=0.35, roughness=0.1)
    _set_shader_params(target_layer, f"{base}/cotton",
                       diffuse=(0.96, 0.95, 0.92), roughness=1.0, metallic=0.0)
    _set_shader_params(target_layer, f"{base}/char",
                       diffuse=(0.12, 0.12, 0.12), roughness=0.8, metallic=0.0)
    # 酒精液：实心圆柱。之前用 HDR 自发光(1.15,0.88,0.14)保持饱和亮黄，但那
    # 也是"整壶淡黄、不像玻璃装酒精"的元凶——HDR 强发光透过半透明玻璃壳把
    # 整个灯身照成黄色。v30.3：emissive 压到 ~0.28（非发光，仅微增亮），
    # diffuse 暖琥珀 + opacity 0.7 半透明 → 透明玻璃壶里能看到淡琥珀酒精液，
    # 不再整壶发黄。染色锥的 HDR 配方只留给染色锥/火焰用。
    _set_shader_params(target_layer, f"{base}/liquid",
                       diffuse=(0.32, 0.27, 0.16), emissive=(0.28, 0.24, 0.14),
                       opacity=0.7, roughness=0.3)

    # 玻璃身 / 灯帽 mesh 双面渲染（防内壁背向面被剔除全黑）
    stage = Usd.Stage.Open(target_layer)
    for mesh_path in ("/World/AlcoholLamp/body/body",
                      "/World/AlcoholLamp/cap/cap",
                      "/World/AlcoholLamp/liquid/liquid"):
        prim = stage.GetPrimAtPath(mesh_path)
        if prim.IsValid():
            mesh = UsdGeom.Mesh(prim)
            if not mesh.GetDoubleSidedAttr().Get():
                mesh.GetDoubleSidedAttr().Set(True)
    print("[lamp] materials tuned (white ceramic, amber glass, dark wick tip)")


def fix_inward_normals(target_layer):
    """修正 mesh 面法线方向（Blender 导出常见的内外翻转问题）。

    根因：alcohol_lamp.usd 里玻璃灯身 body mesh 93% 的面朝内侧（法线指向质心），
    且带 faceVarying 法线，RTX 用这些（错误的）法线做光照 → 灯身渲染全黑，
    无论材质 diffuse/opacity 怎么改都看不见（这正是用户反馈"酒精灯没显示出来"）。
    对比：cap / liquid / holder 法线全部朝外，正常渲染。

    修复：对每个朝内的面，反转顶点绕序并取反该面的角法线，使全部面朝外。
    """
    import numpy as np

    for path in ("/World/AlcoholLamp/body/body",
                 "/World/SampleDish/powder_002_002/powder_002_002_001"):
        prim = target_layer.GetPrimAtPath(path)
        if prim is None:
            print(f"[normals] {path}: not found, skip")
            continue
        # 直接基于 layer 读取几何（绕开 stage 遍历怪癖）
        def _attr(name):
            return next((a for a in prim.attributes if a.name == name), None)
        points = _attr("points").default if _attr("points") else None
        counts = _attr("faceVertexCounts").default if _attr("faceVertexCounts") else None
        indices = _attr("faceVertexIndices").default if _attr("faceVertexIndices") else None
        nrm_attr = _attr("normals")
        normal_vals = list(nrm_attr.default) if (nrm_attr and nrm_attr.default is not None) else None
        if points is None or counts is None or indices is None:
            print(f"[normals] {path}: missing geometry attrs, skip")
            continue
        pts = np.array(points, dtype=float)
        counts = list(counts)
        indices = list(indices)
        cen = pts.mean(axis=0)
        idx = 0
        n_idx = 0
        new_indices = indices[:]
        new_normals = list(normal_vals) if normal_vals else None
        changed = 0
        for nf in counts:
            face = indices[idx:idx + nf]
            a = pts[face[1]] - pts[face[0]]
            b = pts[face[2]] - pts[face[0]]
            n = np.cross(a, b)
            nn = np.linalg.norm(n)
            if nn > 1e-12:
                n = n / nn
                face_cen = pts[face].mean(axis=0)
                if np.dot(n, cen - face_cen) > 0:  # 指向质心 = 朝内
                    for k, old in enumerate(face[::-1]):
                        new_indices[idx + k] = old
                    if new_normals:
                        for k in range(nf):
                            new_normals[n_idx + k] = [-v for v in normal_vals[n_idx + k]]
                    changed += 1
            idx += nf
            n_idx += nf
        if changed:
            _attr("faceVertexIndices").default = new_indices
            if new_normals:
                nrm_attr.default = new_normals
            print(f"[normals] {path}: flipped {changed}/{len(counts)} inward faces")
        else:
            print(f"[normals] {path}: already outward ({len(counts)} faces)")


def fix_dish_material(target_layer):
    """让表面皿（玻璃皿）在亮场景里清晰可见（用户反馈"玻璃皿看不见"）。

    根因（2026-08-11 修正）：皿 mesh powder_002_002_001 有两个 GeomSubset——
        - dish_mat_002_002_001（玻璃壁）：metallic=0.85 的"金属玻璃"thin-shell
        - powder_mat_002_002_001（皿底粉末）：**白金属 metallic=0.85, roughness=0.9**，
          是真正的眩光源——把皿内蓝色圆盘反射漂白成近白，还让整皿看着像白金属碗。
    旧代码只改 dish_mat，漏了 powder_mat——所以快照里皿仍是白金属、蓝盘被漂白。

    修复：两者都改。
        - 玻璃壁：metallic=0、low roughness、无色透明玻璃（v30 用户反馈"皿上一层
          蓝"——之前是蓝玻璃+蓝 emissive。表面皿应为透明无色玻璃，改近无色玻璃）
        - 皿底粉末：metallic=0、roughness 中等、奶油白 diffuse → 亚光，不再眩光
    """
    glass_path = "/World/_materials/dish_mat_002_002_001"
    if not _set_shader_params(target_layer, glass_path,
                              diffuse=(0.84, 0.87, 0.91), emissive=(0.03, 0.04, 0.05),
                              opacity=0.85):
        print("[dish] dish glass material not found, skip")
        return
    print("[dish] dish glass -> clear glass (no blue)")
    # 皿底/粉末：改亚光浅灰，去掉金属眩光（否则反射漂白皿内圆盘）。v30.1 与
    # dish_visible_disk_mat 同改浅灰，给亮液滴留对比度（见 add_dish_visible_layer）
    powder_path = "/World/_materials/powder_mat_002_002_001"
    if not _set_shader_params(target_layer, powder_path,
                              diffuse=(0.50, 0.50, 0.52), emissive=(0.08, 0.08, 0.09),
                              opacity=1.0, metallic=0.0, roughness=0.5):
        print("[dish] powder material not found, skip")
        return
    print("[dish] dish powder -> matte light gray (no metallic glare)")


def _solid_cylinder_mesh(radius, height, segments=32):
    """生成实心圆柱 mesh 的 (points, faceVertexCounts, faceVertexIndices, normals)。

    轴为 Z、几何中心在原点（用 xform 平移摆位）。与酒精灯液面同款（Mesh 实心体，
    RTX 可靠渲染；之前用 UsdGeom.Cylinder prim 快照里渲染不出来）。

    normals 为 faceVarying 平直法线（每 face-vertex 一个），与 faceVertexIndices
    一一对应。必须显式带 normals + subdivisionScheme=none——本项目所有正常渲染的
    mesh 都带 normals，缺 normals 时 RTX 可能部分面渲染异常。
    """
    import numpy as np
    s = segments
    pts = [(0.0, 0.0, height / 2.0), (0.0, 0.0, -height / 2.0)]  # 顶心0 底心1
    for i in range(s):
        a = 2.0 * np.pi * i / s
        pts.append((radius * np.cos(a), radius * np.sin(a), height / 2.0))
    for i in range(s):
        a = 2.0 * np.pi * i / s
        pts.append((radius * np.cos(a), radius * np.sin(a), -height / 2.0))
    counts, idx, nrm = [], [], []
    for i in range(s):
        j = (i + 1) % s
        counts += [3, 3, 4]
        idx += [0, 2 + i, 2 + j]                      # 顶面
        nrm += [(0, 0, 1)] * 3
        idx += [1, 2 + s + j, 2 + s + i]              # 底面（反绕序，法线朝下）
        nrm += [(0, 0, -1)] * 3
        am = (2.0 * np.pi * i / s) + np.pi / s        # 侧面四边中点角度
        idx += [2 + i, 2 + j, 2 + s + j, 2 + s + i]   # 侧面
        nrm += [(np.cos(am), np.sin(am), 0.0)] * 4
    return pts, counts, idx, nrm


def _teardrop_flame_mesh(height, radius, segments=12):
    """生成水滴形火焰 mesh 的 (points, faceVertexCounts, faceVertexIndices, normals)。

    v30.4：染色锥从直线 Cone 改为"上尖下圆"的水滴曲面（用户："形状应该就像火焰
    那样子是一个水滴的形状上面尖下面圆"）。轴为 Z，底端在 z=0（火焰坐在灯芯/铂丝
    尖上），顶端尖收在 z=height。

    剖面轮廓（z 归一化, r 归一化）：
      (0.00,0.30) 圆底
      (0.28,1.00) 最宽处
      (1.00,0.00) 尖端
    lathe 曲面 + 底部封盘 + 顶部收尖。normals 为 faceVarying 平直法线，
    与 _solid_cylinder_mesh 同款（必须带 normals + subdivisionScheme=none
    才可靠渲染）。返回 (points, counts, idx, nrm)，由调用方写进 UsdGeom.Mesh。
    """
    import numpy as np
    # (z, r) 剖面——水滴：下圆上尖
    prof = [
        (0.00, 0.30),
        (0.10, 0.62),
        (0.20, 0.88),
        (0.28, 1.00),
        (0.38, 0.94),
        (0.50, 0.80),
        (0.62, 0.62),
        (0.74, 0.43),
        (0.86, 0.24),
        (0.94, 0.10),
        (1.00, 0.00),
    ]
    n = segments
    nr = len(prof) - 1            # 圆环行数（最后一行 r=0 是顶点）
    # 顶点：底心 0，环行 0..nr-1 各 n 个，顶点收尾
    pts = [(0.0, 0.0, 0.0)]       # 底心
    for (zs, rs) in prof[:nr]:
        z = zs * height
        r = rs * radius
        for i in range(n):
            a = 2.0 * np.pi * i / n
            pts.append((r * np.cos(a), r * np.sin(a), z))
    apex_idx = len(pts)
    pts.append((0.0, 0.0, height))  # 尖顶

    counts, idx, nrm = [], [], []

    def ring_off(rr):
        return 1 + rr * n

    # 底部封盘（法线朝下）
    for i in range(n):
        j = (i + 1) % n
        counts.append(3)
        idx += [0, 1 + i, 1 + j]
        nrm += [(0.0, 0.0, -1.0)] * 3
    # 每行剖面法线（(r,z) 平面内垂直于切线的平均方向）
    prof_n = []
    for i in range(nr):
        i0 = max(i - 1, 0)
        i1 = min(i + 1, nr)
        dr = (prof[i1][1] - prof[i0][1]) * radius
        dz = (prof[i1][0] - prof[i0][0]) * height
        t = np.hypot(dz, dr)
        prof_n.append((dz / t, -dr / t))
    # 侧面：行 i -> 行 i+1 的四边
    for i in range(nr - 1):
        for k in range(n):
            l = (k + 1) % n
            am = (2.0 * np.pi * k / n) + np.pi / n   # 四边中点角度
            nx, nz = prof_n[i]
            counts.append(4)
            idx += [ring_off(i) + k, ring_off(i) + l,
                    ring_off(i + 1) + l, ring_off(i + 1) + k]
            nrm += [(nx * np.cos(am), nx * np.sin(am), nz)] * 4
    # 顶部收尖（最后一行 -> 顶点）
    for k in range(n):
        l = (k + 1) % n
        am = (2.0 * np.pi * k / n) + np.pi / n
        nx, nz = prof_n[nr - 1]
        counts.append(3)
        idx += [ring_off(nr - 1) + k, ring_off(nr - 1) + l, apex_idx]
        nrm += [(nx * np.cos(am), nx * np.sin(am), nz)] * 3
    return pts, counts, idx, nrm


def remove_dish_visible_layer(target_layer):
    """移除皿内人工圆盘（v30.2 用户反馈"玻璃皿为什么看着有两个层次"）。

    之前 add_dish_visible_layer 为让皿在 RTX 下可见，在皿底上方加了一个实心
    浅灰圆盘 dish_visible_disk——它叠在真皿（powder_002_002_001，玻璃壁+
    粉末底双 GeomSubset）之上，俯视/侧视都像"皿里又有一层盘子"，正是用户
    看到的两层次。

    皿底粉末在 fix_dish_material 已改成浅灰亚光（0.50,0.50,0.52 渲染 ~185-214），
    足以给亮液滴（emissive 2.5 → ~235）当深色底衬（对比度 ~30-50），不再需要
    人工盘。因此直接删掉圆盘 prim 与它的材质，皿恢复单层。
    """
    disk_path = "/World/SampleDish/powder_002_002/dish_visible_disk"
    mat_path = "/World/_materials/dish_visible_disk_mat"
    stage = Usd.Stage.Open(target_layer)
    removed = 0
    for p in (disk_path, mat_path):
        if stage.GetPrimAtPath(p).IsValid():
            stage.RemovePrim(p)
            removed += 1
    print(f"[dish] removed artificial dish_visible_disk ({removed} prim(s) deleted); "
          f"single-layer dish = glass wall + gray powder (bright droplet contrast)")


def fix_dropper_glass(target_layer):
    """修复滴管玻璃管内壁全黑（任务 #5）。

    根因：玻璃材质 opacity=1.0 完全不透明，玻璃管（单面、中空）的内壁
    只有极弱间接光，从管口看进去一片黑。

    修复：材质改为半透明（opacity≈0.35），RTX 以透射/菲涅尔渲染，光线能
    透过外壁照到内壁；mesh 已 doubleSided=True（无需改）。保留 ior 1.45。
    """
    mat_path = "/World/_materials/glass_001_010/Principled_BSDF"
    shader = target_layer.GetPrimAtPath(mat_path)
    if shader is None:
        print("[dropper] glass material not found, skip")
        return
    for attr_name, val in (("inputs:opacity", 0.45),
                           ("inputs:roughness", 0.05)):
        attr = next((a for a in shader.attributes if a.name == attr_name), None)
        if attr is None:
            attr = Sdf.AttributeSpec(shader, attr_name,
                                     Sdf.ValueTypeNames.Float)
        attr.default = float(val)
    # 确认 mesh 双面渲染（防内壁被剔除）
    mesh_path = "/World/Dropper/glass_body_mesh_001_010/glass_body_mesh_001_010"
    stage = Usd.Stage.Open(target_layer)
    prim = stage.GetPrimAtPath(mesh_path)
    if prim.IsValid() and not UsdGeom.Mesh(prim).GetDoubleSidedAttr().Get():
        UsdGeom.Mesh(prim).GetDoubleSidedAttr().Set(True)
    print("[dropper] glass material -> translucent opacity 0.45 (inner wall visible)")


# v24 第 1 步（防穿模）：静态碰撞体。kinematic 被抓物体（瓶塞/灯帽/滴管/铂丝/
# 火柴）不加碰撞，避免 PhysX 对"每帧被 set_object_position 的碰撞体"产生怪行为。
STATIC_COLLIDERS = [
    "/World/table",
    "/World/TestTubeRack_001",
    "/World/HClBottle",
    "/World/SampleBottle",
    "/World/AlcoholLamp",
    "/World/SampleDish",
]
# 静态碰撞体子树里要跳过的 kinematic 部件
SKIP_KIN_CHILDREN = ("/cap", "/stopper_020", "/stopper_021")


def _collect_mesh_paths(spec, out, skip_prefixes):
    p = str(spec.path)
    if any(p.startswith(skip) for skip in skip_prefixes):
        return
    if spec.typeName == "Mesh":
        out.append(p)
    for child in spec.nameChildren:
        _collect_mesh_paths(child, out, skip_prefixes)


def add_static_collision(target_layer):
    """给桌面/试管架/瓶身/酒精灯身/表面皿加静态碰撞体 + 地面碰撞面。

    目的：机械臂（物理 articulation）不再穿过桌子和架子；kinematic 被抓物体
    （瓶塞/灯帽/滴管/铂丝/火柴）保持无碰撞，流程不受影响。
    """
    stage = Usd.Stage.Open(target_layer)
    n = 0
    for root_path in STATIC_COLLIDERS:
        spec = target_layer.GetPrimAtPath(root_path)
        if spec is None:
            print(f"[collision] {root_path} not found, skip")
            continue
        meshes = []
        _collect_mesh_paths(spec, meshes, [root_path + s for s in SKIP_KIN_CHILDREN])
        for path in meshes:
            prim = stage.GetPrimAtPath(path)
            if prim is None or not prim.IsValid():
                continue
            coll = UsdPhysics.CollisionAPI.Apply(prim)
            coll.GetCollisionEnabledAttr().Set(True)
            UsdPhysics.MeshCollisionAPI.Apply(prim)
            prim.GetAttribute("physics:approximation").Set("convexDecomposition")
            n += 1
    print(f"[collision] static colliders applied to {n} meshes")

    # 地面碰撞面（z=0，防机械臂摆到地面以下）
    ground = stage.GetPrimAtPath("/World/GroundPlane")
    if ground is None or not ground.IsValid():
        ground = stage.DefinePrim("/World/GroundPlane", "Plane")
        UsdGeom.Xformable(ground).AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.0))
    coll = UsdPhysics.CollisionAPI.Apply(ground)
    coll.GetCollisionEnabledAttr().Set(True)
    print("[collision] GroundPlane added at z=0")


# v24 第 2 步（防穿模）：瓶塞(25.2mm)×2 + 灯帽(37.2mm) 转真刚体。
# 照 assets/chemistry_lab/lab_003/clock.usd 里 beaker 的物理配方：
#   physics:rigidBodyEnabled=True, physics:collisionEnabled=True,
#   physics:approximation=convexDecomposition,
#   physxCollision:contactOffset=0.002, physxCollision:restOffset=-0.001
# 默认 physics:kinematicEnabled=True：流程期由任务每帧 set_object_position 驱动，
# 变换不被物理覆盖；任务在"落座"（瓶口/灯口）时临时关 kinematic 让 PhysX 物理套合，
# 读物理位姿后再锁回 kinematic（"盖到位后 kinematic 锁住"折中）。
# 小物体（火柴/滴管/铂丝）保持纯 kinematic，不在此列。
RIGID_KIN_PARENTS = [
    "/World/HClBottle/stopper_020",
    "/World/SampleBottle/stopper_021",
    "/World/AlcoholLamp/cap",
]


def make_stopper_cap_rigid(target_layer):
    """给两个瓶塞 + 灯帽的 mesh 加真刚体（kinematic 默认开）+ 凸包碰撞。

    RigidBodyAPI 放在每个父 prim 下的第一个 Mesh prim；其余 mesh 只加碰撞
    （作为同一物体的碰撞形状）。kinematicEnabled=True 让任务 teleport 生效。
    """
    stage = Usd.Stage.Open(target_layer)
    n = 0
    for root in RIGID_KIN_PARENTS:
        spec = target_layer.GetPrimAtPath(root)
        if spec is None:
            print(f"[rigid] {root} not found, skip")
            continue
        meshes = []
        _collect_mesh_paths(spec, meshes, [])
        if not meshes:
            print(f"[rigid] {root}: no mesh found, skip")
            continue
        body_prim = None
        for path in meshes:
            prim = stage.GetPrimAtPath(path)
            if prim is None or not prim.IsValid():
                continue
            coll = UsdPhysics.CollisionAPI.Apply(prim)
            coll.GetCollisionEnabledAttr().Set(True)
            UsdPhysics.MeshCollisionAPI.Apply(prim)
            prim.GetAttribute("physics:approximation").Set("convexDecomposition")
            prim.CreateAttribute(
                "physxCollision:contactOffset", Sdf.ValueTypeNames.Float
            ).Set(0.002)
            prim.CreateAttribute(
                "physxCollision:restOffset", Sdf.ValueTypeNames.Float
            ).Set(-0.001)
            if body_prim is None:
                body_prim = prim
        if body_prim is not None:
            rb = UsdPhysics.RigidBodyAPI.Apply(body_prim)
            rb.GetKinematicEnabledAttr().Set(True)
            n += 1
        print(f"[rigid] {root}: {len(meshes)} mesh(es), rigid body on {body_prim.GetPath()}")
    print(f"[rigid] rigid bodies applied to {n} objects")


def tune_water_materials(target_layer):
    """把液滴/皿内酸液/洗瓶水柱改成"高对比清液"材质（v30 用户反馈"看不到滴酸"）。

    根因（diag_droplet_red.py 决定性验证）：液滴 10mm 几何渲染正常（camera_2 中心
    30x29px），但 RTX 不渲染该球 diffuse（只认 emissive 自发光），低 emissive 渲染
    成透明、被亮背景透出吃掉（dark_glass opacity 1.0 也无效，delta≈0）。且奶油白
    皿 ~230 亮度使任何亮球对比度封顶 ~11（emissive 2.5 才 237）。修复（v30.1）：
    近黑 diffuse + 强中性 emissive 2.2 → 渲染 ~235 亮珠，配合皿改浅灰（~185），
    对比度 ~50 肉眼可辨。中性冷白读作"液滴"，非饱和蓝。"""
    # 液滴：下落中的酸滴——近黑 diffuse + 强 emissive（RTX 只认 emissive）
    _set_shader_params(target_layer, "/World/Droplet_mat",
                       diffuse=(0.01, 0.01, 0.01), emissive=(2.5, 2.5, 2.6),
                       opacity=0.95, roughness=0.3, metallic=0.0)
    # 皿内酸液：落在浅灰粉末上的液层——也需 emissive 才可见。v30.2 提到 2.0：
    # 浅灰皿底 ~185，emissive 1.4 只渲染 ~200 对比弱；2.0 → ~225 对比 ~40，
    # 滴酸堆积肉眼可辨（rebuild_dish_acid_pool 已把 Cylinder 换成实心 Mesh）。
    _set_shader_params(target_layer, "/World/DishAcid_mat",
                       diffuse=(0.01, 0.01, 0.01), emissive=(2.0, 2.0, 2.1),
                       opacity=0.90, roughness=0.3, metallic=0.0)
    # 洗瓶水柱：喷流——与液滴同族（近黑 diffuse + 强 emissive）
    _set_shader_params(target_layer, "/World/WaterJet_mat",
                       diffuse=(0.01, 0.01, 0.01), emissive=(1.8, 1.8, 1.9),
                       opacity=0.95, roughness=0.3, metallic=0.0)
    print("[water] droplet/dish-acid/water-jet -> bright neutral (no blue)")


def rebuild_dish_acid_pool(target_layer):
    """把皿内酸液 Cylinder 换成实心 Mesh 圆盘（v30.2 用户反馈"液体滴下没看到堆积"）。

    根因：/World/DishAcid 是 UsdGeom.Cylinder prim（r=7mm h=1.5mm）——RTX 对
    Cylinder prim 不渲染（与旧版 dish_visible_disk 用 Cylinder 渲染不出来同一坑，
    见旧 add_dish_visible_layer 注释），任务端 set_visible(True) 后仍无任何显示；
    且 r=7mm 太小，即使渲染也难辨。

    修复：删掉 Cylinder prim，用 _solid_cylinder_mesh 重建实心 Mesh 圆盘
    （r=20mm h=2mm 浅层酸液），放皿底粉末面上（z=0.807，皿粉末顶 ~0.806），
    绑定 /World/DishAcid_mat（tune_water_materials 已设近黑 diffuse + 强
    emissive，RTX 可靠显色）。实心 Mesh + subdivisionScheme=none + faceVarying
    normals 是本项目已验证良方，幂等（已存在则重设子分/绑定）。
    """
    acid_path = "/World/DishAcid"
    stage = Usd.Stage.Open(target_layer)
    old = stage.GetPrimAtPath(acid_path)
    if old.IsValid() and old.GetTypeName() != "Mesh":
        stage.RemovePrim(acid_path)
    prim = stage.GetPrimAtPath(acid_path)
    if not prim.IsValid():
        pts, counts, idx, nrm = _solid_cylinder_mesh(0.020, 0.002)
        mesh = UsdGeom.Mesh.Define(stage, acid_path)
        mesh.GetPointsAttr().Set([Gf.Vec3f(*p) for p in pts])
        mesh.GetFaceVertexCountsAttr().Set(counts)
        mesh.GetFaceVertexIndicesAttr().Set(idx)
        mesh.GetNormalsAttr().Set([Gf.Vec3f(*n) for n in nrm])
        mesh.SetNormalsInterpolation("faceVarying")
        xs = [p[0] for p in pts]; ys = [p[1] for p in pts]; zs = [p[2] for p in pts]
        mesh.GetExtentAttr().Set([(min(xs), min(ys), min(zs)), (max(xs), max(ys), max(zs))])
        mesh.GetDoubleSidedAttr().Set(True)
        UsdGeom.Xformable(mesh).AddTranslateOp().Set(Gf.Vec3d(0.2, 0.02, 0.807))
    prim = stage.GetPrimAtPath(acid_path)
    if prim.IsValid():
        UsdGeom.Mesh(prim).GetSubdivisionSchemeAttr().Set("none")
        UsdShade.MaterialBindingAPI(prim).Bind(
            UsdShade.Material(stage.GetPrimAtPath("/World/DishAcid_mat")))
    print("[water] DishAcid Cylinder -> solid mesh pool (r=20mm h=2mm @ z0.807)")


def enlarge_droplet(target_layer):
    """液滴球半径 2.5mm -> 10mm（v30 用户反馈"看不到滴加盐酸"）。原 Droplet
    是 r=2.5mm 的 Sphere，在相机里不可见；放大到 10mm 半径，配合任务端下落
    动画与更长闪现帧数，滴酸肉眼可辨。"""
    stage = Usd.Stage.Open(target_layer)
    droplet = stage.GetPrimAtPath("/World/Droplet")
    if droplet.IsValid():
        UsdGeom.Sphere(droplet).GetRadiusAttr().Set(0.010)
        print("[water] droplet radius 2.5mm -> 10mm")


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
    fix_cylinder_light(layer)
    swap_burner_to_lamp(layer, LAMP)
    ensure_lamp_root_def(layer)
    relocate_stain_cones(layer)
    rebuild_stain_materials(layer)
    tune_lamp_visuals(layer)
    rebuild_flame_cones(layer)
    tune_lamp_materials(layer)
    fix_inward_normals(layer)
    fix_dish_material(layer)
    remove_dish_visible_layer(layer)
    fix_dropper_glass(layer)
    tune_water_materials(layer)
    rebuild_dish_acid_pool(layer)
    enlarge_droplet(layer)
    add_static_collision(layer)
    make_stopper_cap_rigid(layer)

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
