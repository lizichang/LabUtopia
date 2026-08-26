# -*- coding: utf-8 -*-
"""生成 test_paper_holder.usd —— D6 试纸气体检测「试纸夹」（配重底座 + 立杆 + 顶部水平双片夹持）。

实物调研（2026-08-26，v12 目录 D6「试纸气体检测（通用）」用户重新设计）：
  原流程「从试纸盒抽取单条软性薄试纸再夹持」被用户否决（试纸太薄难夹，E1 已踩坑），
  改为专用试纸夹：试纸一开始已夹好，机械臂不再碰试纸，只需「润湿 → 移试管到试纸下
  → 观察变色 → 归位」。故本资产 = 金属夹持架（无试纸，试纸条由场景层预制 4 变体放在夹缝）。

结构规格表（配重底座 + 立杆 + 顶部水平双片夹持，把试纸条夹在 z≈0.99 悬挑伸出）：
  底座      圆柱 Ø60×10mm    金属（配重稳，深色）
  立杆      圆柱 Ø8×180mm    金属（银灰）
  夹持片    两片扁长方体 40×14×4mm，沿 +X 伸出，夹缝 2mm（试纸条 70×7×1mm 的后端滑入夹缝，
            前端悬挑，湿润端悬出）
  夹持片 z 中心 0.187(下)/0.193(上)，夹缝 0.189..0.191（试纸 1mm 厚放 0.19）

单位米，Z-up，原点=底座中心底面（贴台面 z=0.80）。analytic 圆柱/立方体（非 Mesh），
无 subdivisionScheme 问题。金属用 UsdPreviewSurface metallic。

用法：python scripts/gen_test_paper_holder.py   （运行环境：labutopia conda env 有 pxr）
"""
import os
from pxr import Usd, UsdGeom, UsdShade, Sdf, Gf

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
EQ = os.path.join(REPO, "assets", "equipment")
OUT = os.path.join(EQ, "test_paper_holder.usd")

# —— 材质配方 ——
METAL = dict(diffuseColor=(0.72, 0.74, 0.78), metallic=0.85, roughness=0.30)   # 银灰金属
DARK = dict(diffuseColor=(0.28, 0.30, 0.34), metallic=0.70, roughness=0.40)    # 深色配重底座


def add_material(stage, prim, recipe):
    mat_path = str(prim.GetPath()) + "_mat"
    mat = UsdShade.Material.Define(stage, mat_path)
    sh = UsdShade.Shader.Define(stage, mat_path + "/Shader")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
        Gf.Vec3f(*recipe["diffuseColor"]))
    sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(recipe["metallic"])
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(recipe["roughness"])
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI(prim).Bind(mat)


def add_cylinder(stage, path, radius, height, z_center, recipe):
    cyl = UsdGeom.Cylinder.Define(stage, path)
    cyl.CreateRadiusAttr(radius)
    cyl.CreateHeightAttr(height)
    cyl.CreateAxisAttr("Z")
    cyl.AddTranslateOp().Set(Gf.Vec3d(0, 0, z_center))
    add_material(stage, cyl.GetPrim(), recipe)
    return cyl


def add_box(stage, path, size, center, recipe):
    box = UsdGeom.Cube.Define(stage, path)
    # UsdGeom.Cube 默认 size=2（extent -1..+1），要目标尺寸须 scale=(W/2, L/2, T/2)。
    # xformOpOrder 必须 [translate, scale]（先 AddTranslateOp 再 AddScaleOp）——
    # 否则顺序 [scale, translate] 会把 translate 也乘上 scale，夹持片偏到近原点。
    box.AddTranslateOp().Set(Gf.Vec3d(center[0], center[1], center[2]))
    box.AddScaleOp().Set(Gf.Vec3f(size[0] / 2, size[1] / 2, size[2] / 2))
    add_material(stage, box.GetPrim(), recipe)
    return box


def main():
    stage = Usd.Stage.CreateNew(OUT)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    root = UsdGeom.Xform.Define(stage, "/TestPaperHolder")
    stage.SetDefaultPrim(root.GetPrim())

    # 底座 Ø60×10mm（深色配重）
    add_cylinder(stage, "/TestPaperHolder/Base", 0.03, 0.01, 0.005, DARK)
    # 立杆 Ø8×180mm（0.01 → 0.19，顶到 0.19）
    add_cylinder(stage, "/TestPaperHolder/Post", 0.004, 0.18, 0.10, METAL)
    # 夹持片（两片，沿 +X 伸出 40mm，夹缝 2mm；下片中心 z=0.187、上片 0.193）
    add_box(stage, "/TestPaperHolder/JawBottom", (0.04, 0.014, 0.004), (0.015, 0, 0.187), METAL)
    add_box(stage, "/TestPaperHolder/JawTop", (0.04, 0.014, 0.004), (0.015, 0, 0.193), METAL)

    stage.GetRootLayer().Save()
    print("SAVED", OUT)


if __name__ == "__main__":
    main()
