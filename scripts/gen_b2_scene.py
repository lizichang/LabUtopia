# -*- coding: utf-8 -*-
"""生成 b2_alcohol_heat_liquid.usd —— B2 沸点测定（酒精灯加热试管液体）场景（烘平自包含）。

基于 lab_clean.usd（干净底场景：台面顶 z=0.80，defaultPrim=/World）：
- 引用 assets/equipment/ 真实器材（铁架台新钩版 / 酒精灯 / 石棉网 / 试管）
- 布局 = 用户 b2_tmp.usd 相对位置（2026-08-24），锚：铁架台铁柱在 (STAND_X, STAND_Y)，
  垂直堆叠中心线 x=铁架台x+0.100：
      酒精灯(桌面 z0.80) → 铁环(z0.910-0.918 托石棉网) → 石棉网(z0.919)
      → 试管(底 z0.9206 坐网上) → 钩(z1.216-1.227，挂温度计，未入场景)
- 去 4 件资产自带 env_light 残留（重复 DomeLight）；灯帽从灯顶挪到桌边(y-0.467)；
  火焰 flame_outer/flame_inner 初始隐藏（未点火，task 再动画）

用法：python scripts/gen_b2_scene.py   （运行环境：labutopia conda env 有 pxr）
"""
import os
import shutil
from pxr import Usd, UsdGeom, UsdShade, UsdLux, Sdf, Gf

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCENE_DIR = os.path.join(REPO, "assets", "scenes", "b_thermal", "b2_alcohol_heat_liquid")
OUT = os.path.join(SCENE_DIR, "b2_alcohol_heat_liquid.usd")
LAB_CLEAN = os.path.join(REPO, "assets", "scenes", "base", "lab_clean", "lab_clean.usd")
EQ = os.path.join(REPO, "assets", "equipment")
ENV_TEX_SRC = os.path.join(REPO, "assets", "scenes", "d_wetchem", "d2s_water_solubility",
                           "textures", "env_bright.png")

TABLE_TOP = 0.80
# 锚：铁架台铁柱世界位置。堆叠中心线 = 铁柱 + 0.100（b2_tmp：环心/试管/石棉网/酒精灯在 x=0.100）
STAND_X, STAND_Y = 0.20, 0.0
TUBE_X = STAND_X + 0.100
TUBE_Y = STAND_Y
GAUZE_Z = TABLE_TOP + 0.1194        # 石棉网中心（坐铁环上，环顶 0.918）
TUBE_BOTTOM_Z = TABLE_TOP + 0.1206  # 试管底（坐石棉网上，b2_tmp 试管 translate z=0.1206）

# (prim, asset_file, translate, scale)   tz=None → 动态贴台面（资产底座 min z -> 0.80）
EQUIP = [
    ("IronStand", "iron_stand.usd", (STAND_X, STAND_Y, None), None),
    ("AlcoholLamp", "alcohol_lamp.usd", (TUBE_X, TUBE_Y, None), None),
    ("AsbestosGauze", "asbestos_gauze.usd", (TUBE_X, TUBE_Y, GAUZE_Z), None),
    ("TestTube", "test_tube.usd", (TUBE_X, TUBE_Y, TUBE_BOTTOM_Z), None),
]


def asset_local_min_z(asset_file):
    """资产自身世界包围盒的 min z（判断底座相对原点的偏移）。"""
    st = Usd.Stage.Open(os.path.join(EQ, asset_file))
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"])
    r = bc.ComputeWorldBound(st.GetPseudoRoot()).ComputeAlignedRange()
    return r.GetMin()[2]


def add_equip(stage, name, asset, t, scale):
    prim = UsdGeom.Xform.Define(stage, f"/World/{name}")
    prim.GetPrim().GetReferences().AddReference(
        os.path.abspath(os.path.join(EQ, asset))
    )
    tx, ty, tz = t
    if tz is None:
        tz = TABLE_TOP - asset_local_min_z(asset)
        print(f"[equip] {name} base offset {asset_local_min_z(asset):+.4f} -> z {tz:.4f}")
    prim.AddTranslateOp().Set(Gf.Vec3d(tx, ty, tz))
    if scale is not None:
        prim.AddScaleOp().Set(Gf.Vec3f(scale, scale, scale))
    print(f"[equip] {name} <- {asset} at ({tx}, {ty}, {tz})" + (f" scale {scale}" if scale else ""))


def add_env_light(stage):
    """环境光（DomeLight + 亮环境贴图）：玻璃试管/酒精灯在无环境反射下照不亮。
    贴图路径先用相对 ./textures/，烘平后由 fix_env_light() 在场景层重新指向。"""
    light = UsdLux.DomeLight.Define(stage, "/World/env_light")
    light.GetIntensityAttr().Set(2000.0)
    light.GetColorAttr().Set(Gf.Vec3f(1, 1, 1))
    light.GetEnableColorTemperatureAttr().Set(False)
    light.GetTextureFileAttr().Set(Sdf.AssetPath("./textures/env_bright.png"))
    light.GetTextureFormatAttr().Set(UsdLux.Tokens.automatic)
    print("[env] DomeLight + env_bright.png (intensity 2000)")


def brighten_lights(st2):
    """主光太弱：lab_clean 的 CylinderLight 强度 2000 照不亮细玻璃件 → 12000。"""
    cyl = st2.GetPrimAtPath("/World/CylinderLight")
    if not cyl.IsValid():
        print("[light] /World/CylinderLight not found, skip")
        return
    UsdLux.CylinderLight(cyl).GetIntensityAttr().Set(12000.0)
    print("[light] CylinderLight intensity 2000 -> 12000")


def fix_env_light(st2):
    """修 env 贴图路径断链（Export 按 lab_clean 解析 ./textures/ → 失效），
    烘平后场景文件在 SCENE_DIR，相对 textures/ 能正确指向。"""
    env = st2.GetPrimAtPath("/World/env_light")
    if not env.IsValid():
        print("[env] /World/env_light not found, skip")
        return
    UsdLux.DomeLight(env).GetTextureFileAttr().Set(Sdf.AssetPath("textures/env_bright.png"))
    print("[env] texture -> textures/env_bright.png (scene-relative)")


def remove_asset_env_lights(st2):
    """去 4 件器材资产自带的 flametest 残留 DomeLight（/root/env_light），会与场景
    env_light 双灯、且近黑贴图压暗环境。遍历每个器材 prim 删 DomeLight / env_light。"""
    for name, *_ in EQUIP:
        p = st2.GetPrimAtPath(f"/World/{name}")
        if not p.IsValid():
            print(f"[clean] /World/{name} not found, skip")
            continue
        paths = [pp.GetPath() for pp in Usd.PrimRange(p)
                 if pp.GetTypeName() == "DomeLight" or "env_light" in pp.GetName()]
        for path in paths:
            st2.RemovePrim(path)
            print(f"[clean] removed {path}")
        if not paths:
            print(f"[clean] no DomeLight in {name}")


def move_lamp_cap(st2):
    """灯帽从灯顶挪到桌边（b2_tmp 用户布局：cap 放 y-0.467，闭口朝下贴台面）。
    资产 cap xform = [Translate(0,0,0) rotateX90 scale0.01]，mesh 在局部 z[0.076,0.107]，
    故 translate z=-0.076 让帽底(z=0.076)落回台面 0.80。"""
    cap = st2.GetPrimAtPath("/World/AlcoholLamp/cap")
    if not cap.IsValid():
        print("[clean] /World/AlcoholLamp/cap not found, skip")
        return
    xf = UsdGeom.Xformable(cap)
    tgt = Gf.Vec3d(0.0, -0.467, -0.076)
    ops = xf.GetOrderedXformOps()
    for op in ops:
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            op.Set(tgt)
            print(f"[clean] cap translate -> {tuple(tgt)}")
            return
    xf.AddTranslateOp().Set(tgt)
    print(f"[clean] cap (no translate op) add translate {tuple(tgt)}")


def hide_flames(st2):
    """火焰初始隐藏（实验未点火）：flame_outer/flame_inner 两个 Cone。"""
    for fl in ("flame_outer", "flame_inner"):
        p = st2.GetPrimAtPath(f"/World/AlcoholLamp/{fl}")
        if not p.IsValid():
            print(f"[clean] /World/AlcoholLamp/{fl} not found, skip")
            continue
        UsdGeom.Imageable(p).MakeInvisible()
        print(f"[clean] hidden {p.GetPath()}")


def verify(st2):
    """自检：打印各器材世界 bbox，断言垂直堆叠关系：
    铁架台底座贴台面、石棉网坐铁环上（0.4mm 间隙）、试管底坐石棉网上（≤1mm）、
    灯芯顶低于石棉网底（火焰区留白）、钩低于铁柱顶。"""
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"])
    names = ["IronStand", "AlcoholLamp", "AsbestosGauze", "TestTube"]
    boxes = {}
    for name in names:
        p = st2.GetPrimAtPath(f"/World/{name}")
        if not p.IsValid():
            print(f"[verify] /World/{name} MISSING")
            continue
        r = bc.ComputeWorldBound(p).ComputeAlignedRange()
        mn, mx = r.GetMin(), r.GetMax()
        boxes[name] = (mn, mx)
        print(f"[verify] {name:13s} min({mn[0]:+.4f},{mn[1]:+.4f},{mn[2]:+.4f}) "
              f"max({mx[0]:+.4f},{mx[1]:+.4f},{mx[2]:+.4f})")
    # 铁架台：底座贴台面，钩顶 < 铁柱顶
    smn, smx = boxes["IronStand"]
    assert abs(smn[2] - TABLE_TOP) < 0.002, f"IronStand base z {smn[2]} != table {TABLE_TOP}"
    # 石棉网坐铁环上：网底 > 环顶(0.918)，网底 − 环顶 ≤ 2mm
    gmn, gmx = boxes["AsbestosGauze"]
    ring_top = TABLE_TOP + 0.118
    assert gmn[2] >= ring_top - 0.002, f"gauze bottom {gmn[2]} below ring top {ring_top}"
    # 试管底坐石棉网上：管底 − 网顶 ≤ 1.5mm 且 ≥ 0
    tmn, tmx = boxes["TestTube"]
    assert 0 <= tmn[2] - gmx[2] <= 0.0015, f"tube bottom {tmn[2]} not on gauze top {gmx[2]}"
    # 酒精灯灯芯顶低于石棉网底（火焰区留白 ≥ 1.5cm）
    lmn, lmx = boxes["AlcoholLamp"]
    # 灯芯顶 = wick_tip 世界 z（资产内 0.1005 → 台面 0.80）
    wick_top = TABLE_TOP + 0.1005
    assert wick_top + 0.015 < gmn[2], f"flame gap {gmn[2] - wick_top:.3f} < 1.5cm"
    # 钩低于铁柱顶（铁柱顶 = 台面 0.80 + 0.46 = 1.26）
    assert smx[2] < TABLE_TOP + 0.461, f"IronStand top {smx[2]} exceeds pole top"
    # 灯帽在桌边（y 偏移 -0.467），帽底贴台面
    cap = st2.GetPrimAtPath("/World/AlcoholLamp/cap")
    r = bc.ComputeWorldBound(cap).ComputeAlignedRange()
    cmn, cmx = r.GetMin(), r.GetMax()
    print(f"[verify] cap     min({cmn[0]:+.4f},{cmn[1]:+.4f},{cmn[2]:+.4f}) "
          f"max({cmx[0]:+.4f},{cmx[1]:+.4f},{cmx[2]:+.4f})")
    assert abs(cmn[2] - TABLE_TOP) < 0.002, f"cap bottom {cmn[2]} not on table"
    assert cmx[1] < TUBE_Y, f"cap y {cmx[1]} not on -y side"
    print("[verify] OK: 台面贴底 / 网坐环上 / 管坐网上 / 灯在网下 / 钩在柱内")


def main():
    os.makedirs(os.path.join(SCENE_DIR, "textures"), exist_ok=True)
    if not os.path.exists(os.path.join(SCENE_DIR, "textures", "env_bright.png")):
        shutil.copy(ENV_TEX_SRC, os.path.join(SCENE_DIR, "textures", "env_bright.png"))
        print(f"[env] copied env_bright.png -> {os.path.join(SCENE_DIR, 'textures')}")

    stage = Usd.Stage.Open(LAB_CLEAN)
    for name, asset, t, scale in EQUIP:
        add_equip(stage, name, asset, t, scale)
    add_env_light(stage)
    stage.Export(OUT)  # 烘平：单层自包含，带 lab_clean 的 defaultPrim=/World

    st2 = Usd.Stage.Open(OUT)
    remove_asset_env_lights(st2)
    move_lamp_cap(st2)
    hide_flames(st2)
    brighten_lights(st2)
    fix_env_light(st2)
    verify(st2)
    st2.GetRootLayer().Save()
    print("SAVED", OUT)


if __name__ == "__main__":
    main()
