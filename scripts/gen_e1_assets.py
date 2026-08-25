# -*- coding: utf-8 -*-
"""生成 E1「pH 试纸检测」三样资产：pH试纸本 / 白瓷板 / 比色卡。单位：米。

按 labutopia-assets skill 实物调研通法，尺寸来源（2026-08-25 上网核实，非记忆）：
  pH试纸本 —— 广泛 pH 试纸本（屯團百貨「包装 45×69mm 一本 80 张」；普遍 45×70×6mm，
              单条 7×70mm，见 WHATMAN/Trusco 规格表）
  白瓷板   —— 白色陶瓷方板（白瓷反应板系列 85×54~115×90mm；E1 用平放试纸的方形白釉板，
              取 80×80×6mm 供 70mm 试纸条平放）
  比色卡   —— pH 1-14 通用指示剂比色卡（单行 14 色块，卡 90×40×1.2mm）

结构规格表（每部件：形状原语 / 关键尺寸 / 相对位置 / 材质）：
  ph_testpaper_book.usd
    /root/book   矩形纸盒(试纸叠) 45(X)×70(Y)×6(Z)mm 底 z=0 纸(0.95,0.93,0.82) rough0.9
    （抽出的试纸条已默认铺在白瓷板上，在 gen_e1_scene.py 里单独建，不在本资产内；
     资产只给静态的"一叠试纸"本体，避免与可抓取条重叠）
  white_porcelain_plate.usd
    /root/plate  白色釉面陶瓷方板 80(X)×80(Y)×6(Z)mm 底 z=0 白瓷(0.93,0.93,0.95) rough0.30
  ph_color_chart.usd
    /root/card   白色卡纸 90(X)×40(Y)×1.2(Z)mm 底 z=0 白(0.95,0.95,0.95) rough0.7
    /root/block_1..14  14 个色块 5(X)×28(Y)×0.5(Z)mm 单行沿 X（-32.5..+37.5mm）
               贴卡面上(z 1.2..1.7mm) pH1..14 通用指示剂色，matte 印刷墨

材质规范（两套管线统一口径）：纸 rough≈0.9；陶瓷 rough≈0.3（釉面微光泽）；卡纸 rough≈0.7；
色块 matte 印刷墨用纯 diffuse（比色卡应像印刷墨，非自发光）。若 headless RTX 下小色块
被洗白/不显色，再换「近黑 diffuse + 单通道主导 emissive」配方（flametest 坑 28）。

注意：色块先按纯 diffuse 建模（逻辑正确），渲染验证由用户做；不显色时再切 emissive 配方。
"""
import os
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))  # obj_gen.py / obj2usd.py
from obj_gen import MeshBuilder  # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT_USD = os.path.join(REPO, "assets", "equipment")
OUT_OBJ = os.path.join(tempfile.gettempdir(), "e1_assets_obj")


# ---------------------------------------------------------------- 网格 helper
def box(mb, cx, cy, cz, w, d, h, group):
    """轴对齐盒，中心 (cx,cy,cz)，尺寸 w(X) d(Y) h(Z)；每面独立 4 顶点 + 外指法线。"""
    x0, x1 = cx - w / 2, cx + w / 2
    y0, y1 = cy - d / 2, cy + d / 2
    z0, z1 = cz - h / 2, cz + h / 2

    def face(n, corners):
        idx = [mb._add_vert(p, n) for p in corners]
        mb._add_quad(idx[0], idx[1], idx[2], idx[3], group)

    face((0, 0, 1), [(x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)])   # 顶
    face((0, 0, -1), [(x0, y0, z0), (x0, y1, z0), (x1, y1, z0), (x1, y0, z0)])  # 底
    face((1, 0, 0), [(x1, y0, z0), (x1, y0, z1), (x1, y1, z1), (x1, y1, z0)])   # +X
    face((-1, 0, 0), [(x0, y0, z0), (x0, y1, z0), (x0, y1, z1), (x0, y0, z1)])  # -X
    face((0, 1, 0), [(x0, y1, z0), (x0, y1, z1), (x1, y1, z1), (x1, y1, z0)])   # +Y
    face((0, -1, 0), [(x0, y0, z0), (x1, y0, z0), (x1, y0, z1), (x0, y0, z1)])  # -Y


# ---------------------------------------------------------------- 部件
def build_book(mb):
    # 试纸叠本体：45×70×6，底 z=0（中心 z=0.003）
    box(mb, 0.0, 0.0, 0.003, 0.045, 0.070, 0.006, "book")


def build_plate(mb):
    # 白瓷方板：80×80×6，底 z=0
    box(mb, 0.0, 0.0, 0.003, 0.080, 0.080, 0.006, "plate")


PH_COLORS = [
    (1.00, 0.05, 0.05),   # pH 1  红
    (1.00, 0.25, 0.05),   # pH 2  红橙
    (1.00, 0.55, 0.05),   # pH 3  橙
    (1.00, 0.80, 0.05),   # pH 4  黄橙
    (0.95, 0.95, 0.10),   # pH 5  黄
    (0.65, 0.85, 0.10),   # pH 6  黄绿
    (0.10, 0.65, 0.15),   # pH 7  绿
    (0.10, 0.60, 0.50),   # pH 8  蓝绿
    (0.10, 0.45, 0.70),   # pH 9  蓝
    (0.10, 0.35, 0.65),   # pH 10 蓝
    (0.45, 0.25, 0.70),   # pH 11 紫
    (0.55, 0.20, 0.65),   # pH 12 紫
    (0.50, 0.10, 0.55),   # pH 13 深紫
    (0.40, 0.05, 0.45),   # pH 14 深紫
]


def build_chart(mb, groups):
    # 卡纸：90×40×1.2，底 z=0
    box(mb, 0.0, 0.0, 0.0006, 0.090, 0.040, 0.0012, "card")
    # 14 色块：5×28×0.5，单行沿 X（中心 -0.0325..+0.0375），贴卡面上（z 中心 0.00145）
    for i, c in enumerate(PH_COLORS):
        cx = -0.035 + (i + 0.5) * 0.005
        box(mb, cx, 0.0, 0.00145, 0.005, 0.028, 0.0005, f"block_{i + 1}")
        groups.append(f"block_{i + 1}")
    return groups


# ---------------------------------------------------------------- 材质
USD_MATS_BOOK = {
    "book": dict(diffuse=(0.95, 0.93, 0.82), roughness=0.90),
}

USD_MATS_PLATE = {
    "plate": dict(diffuse=(0.93, 0.93, 0.95), roughness=0.30),
}

USD_MATS_CHART = {"card": dict(diffuse=(0.95, 0.95, 0.95), roughness=0.70)}
for _i, _c in enumerate(PH_COLORS):
    USD_MATS_CHART[f"block_{_i + 1}"] = dict(diffuse=_c, roughness=0.40)


def make_usd(obj_path, out_usd, mat_specs):
    from pxr import Usd, UsdGeom, UsdShade, Sdf, Gf
    from obj2usd import parse_obj, write_mesh

    verts, vns, groups = parse_obj(obj_path)
    stage = Usd.Stage.CreateNew(out_usd)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, "Z")
    root = UsdGeom.Xform.Define(stage, "/root")
    stage.SetDefaultPrim(root.GetPrim())
    for g, faces in groups.items():
        mesh = write_mesh(stage, f"/root/{g}", verts, faces, vns)
        spec = mat_specs.get(g)
        if spec is None:
            continue
        mat_path = f"/root/{g}_mat"
        mat = UsdShade.Material.Define(stage, mat_path)
        sh = UsdShade.Shader.Define(stage, f"{mat_path}/Shader")
        sh.CreateIdAttr("UsdPreviewSurface")
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
            Gf.Vec3f(*spec["diffuse"]))
        if spec.get("opacity", 1.0) < 1.0:
            sh.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(spec["opacity"])
        if spec.get("emissive") is not None:
            sh.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(
                Gf.Vec3f(*spec["emissive"]))
        if spec.get("ior") is not None:
            sh.CreateInput("ior", Sdf.ValueTypeNames.Float).Set(spec["ior"])
        if spec.get("metallic", 0.0) > 0:
            sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(spec["metallic"])
        sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(spec["roughness"])
        mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
        UsdShade.MaterialBindingAPI(mesh).Bind(mat)
    stage.GetRootLayer().Save()
    print(f"{os.path.basename(out_usd)}: {len(groups)} prims OK")


def verify(out_usd):
    from pxr import Usd, UsdGeom
    stage = Usd.Stage.Open(out_usd)
    lo, hi, bad_n = None, None, 0
    for p in Usd.PrimRange(stage.GetPseudoRoot()):
        if not p.IsA(UsdGeom.Mesh):
            continue
        pts = p.GetAttribute("points").Get()
        nrm = p.GetAttribute("normals").Get()
        if nrm is not None:
            for n in nrm:
                ln = (n[0] ** 2 + n[1] ** 2 + n[2] ** 2) ** 0.5
                if ln > 1.0001 or ln < 0.9999:
                    bad_n += 1
        for v in pts:
            if lo is None:
                lo, hi = list(v), list(v)
            else:
                for k in range(3):
                    lo[k] = min(lo[k], v[k])
                    hi[k] = max(hi[k], v[k])
    sz = [(hi[k] - lo[k]) * 1000 for k in range(3)]
    print(f"verify: bbox=[{sz[0]:.0f},{sz[1]:.0f},{sz[2]:.0f}]mm z0={lo[2] * 1000:.1f}mm "
          f"up={UsdGeom.GetStageUpAxis(stage)} mpu={UsdGeom.GetStageMetersPerUnit(stage)} "
          f"bad_normals={bad_n}")


def _emit(name, groups, builder, mats):
    os.makedirs(OUT_OBJ, exist_ok=True)
    mb = MeshBuilder()
    builder(mb, groups)
    obj_path = os.path.join(OUT_OBJ, f"{name}.obj")
    with open(obj_path, "w", encoding="utf-8") as f:
        f.write(mb.to_obj(groups))
    out_usd = os.path.join(OUT_USD, f"{name}.usd")
    make_usd(obj_path, out_usd, mats)
    verify(out_usd)


def main():
    os.makedirs(OUT_USD, exist_ok=True)

    _emit("ph_testpaper_book", ["book"], lambda mb, g: build_book(mb), USD_MATS_BOOK)
    _emit("white_porcelain_plate", ["plate"], lambda mb, g: build_plate(mb), USD_MATS_PLATE)

    chart_groups = ["card"]
    _emit("ph_color_chart", chart_groups, build_chart, USD_MATS_CHART)

    print("DONE")


if __name__ == "__main__":
    main()
