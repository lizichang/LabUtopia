# -*- coding: utf-8 -*-
"""重建 bar_magnet.usd —— 条形磁铁（教学用，红 N + 蓝 S 两半）。

inventory: 磁铁（条形）notes「长约10cm」。教学条形磁铁横截面常见 ~15×15mm。
几何：100(X)×15(Y)×15(Z)mm，沿 X 轴两半各 50mm（half_n 红=-X 端，half_s 蓝=+X 端），
底面 z=0（与 test_tube/sample_bottle 一致，add_equip 用 asset_local_min_z 自动贴台面）。

旧 bar_magnet.usd 三个 Mesh（half_n/half_s/letter_n）全部 0 顶点破损，只有材质无几何，
本脚本用 UsdGeom.Cube + scale/2（坑：Cube 默认 size=2，要目标尺寸须 /2）重建，幂等
（stage.CreateNew 全新文件 + Save）。

用法：python scripts/gen_bar_magnet.py
"""
import os
from pxr import Usd, UsdGeom, UsdShade, Sdf, Gf

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(REPO, "assets", "equipment", "bar_magnet.usd")

# 磁铁整体尺寸（米）
MAG_LEN, MAG_W, MAG_H = 0.100, 0.015, 0.015     # 100×15×15mm
HALF_LEN = MAG_LEN / 2.0

# 材质：教学磁铁两半喷涂（红 N / 蓝 S），非金属喷漆面，roughness ~0.35
PAINTS = {
    "red":  dict(diffuse=(0.72, 0.06, 0.06), roughness=0.35),
    "blue": dict(diffuse=(0.06, 0.10, 0.70), roughness=0.35),
}


def add_material(stage, prim, recipe):
    mat_path = str(prim.GetPath()) + "_mat"
    mat = UsdShade.Material.Define(stage, mat_path)
    sh = UsdShade.Shader.Define(stage, mat_path + "/Shader")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
        Gf.Vec3f(*recipe["diffuse"]))
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(recipe["roughness"])
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI(prim).Bind(mat)


def add_half(stage, path, half_index, paint):
    """half_index 0 → -X 端（x∈[-0.05,0]）；1 → +X 端（x∈[0,0.05]）。

    平移放父 Xform、缩放放子 Cube（同 E1 TestPaper 的 paper_xf/paper 拆分），
    避免同一 prim 上 scale+translate 的 xformOp 顺序歧义（translate 被 scale 误乘）。
    """
    xf = UsdGeom.Xform.Define(stage, path)
    cx = -HALF_LEN / 2 if half_index == 0 else +HALF_LEN / 2
    xf.AddTranslateOp().Set(Gf.Vec3d(cx, 0.0, MAG_H / 2))
    cube = UsdGeom.Cube.Define(stage, path + "/cube")
    cube.AddScaleOp().Set(Gf.Vec3f(HALF_LEN / 2, MAG_W / 2, MAG_H / 2))
    add_material(stage, cube.GetPrim(), PAINTS[paint])


def main():
    stage = Usd.Stage.CreateNew(OUT)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.Xform.Define(stage, "/World")
    magnet = UsdGeom.Xform.Define(stage, "/World/bar_magnet")
    add_half(stage, "/World/bar_magnet/half_n", 0, "red")
    add_half(stage, "/World/bar_magnet/half_s", 1, "blue")
    # 设 defaultPrim=/World/bar_magnet：场景 add_equip 引用时子 prim（half_n/half_s）
    # 直接组成 /World/BarMagnet 下，不嵌套一层 World（spatula.usd 同款做法）。
    stage.SetDefaultPrim(magnet.GetPrim())
    stage.GetRootLayer().Save()

    # 自检 bbox
    st2 = Usd.Stage.Open(OUT)
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"])
    r = bc.ComputeWorldBound(st2.GetPrimAtPath("/World/bar_magnet")).ComputeAlignedRange()
    mn, mx = r.GetMin(), r.GetMax()
    size = mx - mn
    print(f"[verify] bar_magnet min {tuple(round(v,4) for v in mn)} "
          f"max {tuple(round(v,4) for v in mx)} size {tuple(round(v,4) for v in size)}")
    assert abs(size[0] - MAG_LEN) < 1e-6, f"X={size[0]} != {MAG_LEN}"
    assert abs(size[1] - MAG_W) < 1e-6 and abs(size[2] - MAG_H) < 1e-6
    print("SAVED", OUT)


if __name__ == "__main__":
    main()
