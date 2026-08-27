# -*- coding: utf-8 -*-
"""生成旋光管（polarimeter tube / sample tube）资产，单位：米。

依据（2026-08-26 调研）：
  - lab_inventory.json：旋光管（1dm）＝长度 1dm（100mm）、旋光管（2dm）＝长 200mm，
    两端有玻璃盖帽（螺帽），用于旋光仪。
  - WXG-4 圆盘旋光仪配套样品管结构：玻璃圆管 + 一端圆球加液泡 + 两端螺旋螺帽密封
    + 玻璃盖片（护玻片）/橡皮圈；使用时加液泡一端朝上，气泡存入泡内不挡光路。
  - 外径无公开数据 → 按通用规格估：管身外径 Ø13（内径 Ø10、壁厚 1.5）、加液泡 Ø22、
    螺帽 Ø16×8mm；放旋光仪导轨（宽 4.1cm / 长 25cm）校验余量充足。

分组（group 名 = 未来 USD prim 路径，任务代码依赖）：
  tube_body / bulb / cap_1 / cap_2 / window_1 / window_2
  加液泡(bulb)泡顶开口＝加液口：横放泡朝上时从泡口加液，无需竖起来。

原点约定：管轴沿 Y（水平），中心在原点（y=0），加液泡在 +Y 端——与旋光仪
tube_rails 导轨沿 y、光源在 −y 一致；场景里放导轨时泡朝上（绕管轴转 90°）。

生成方式：MeshBuilder 沿 Z 轴 lathe 出管（含泡），再整体绕 X 轴 −90°（z→y）转成
水平，中心归零。1dm/2dm 两支仅有效光程 L 不同，共用 build_tube。
"""
import os
import sys
import tempfile
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))  # obj_gen.py / obj2usd.py
from obj_gen import MeshBuilder  # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT_USD = os.path.join(REPO, "assets", "equipment")
OUT_OBJ = os.path.join(tempfile.gettempdir(), "polarimeter_tube_obj")

S = 48  # 回转体段数

# 尺寸（米）
TUBE_R = 0.0065      # 管身外径 Ø13（壁厚 1.5 → 内径 Ø10）
BULB_R = 0.011       # 加液泡外径 Ø22
BULB_FROM_END = 0.030   # 泡起点离 +Y 管端距离
BULB_LEN = 0.025     # 泡轴向长度
FILL_R = 0.006       # 泡顶加液口外径 Ø12
FILL_H = 0.008       # 加液口短管高（朝上开口）
CAP_R = 0.008        # 螺帽外径 Ø16
CAP_LEN = 0.008      # 螺帽轴向长
WIN_R = 0.005        # 玻璃端窗 Ø10
WIN_THK = 0.0015     # 端窗厚

LENGTHS = {"1dm": 0.100, "2dm": 0.200}   # 有效光程（inventory：1dm=100mm / 2dm=200mm）


def build_tube(mb, L):
    """沿 Z 轴生成旋光管（管轴沿 z，之后统一转成沿 y）。

    结构（真实 WXG-4 样品管）：整长贯通直管（两端开口=通光路，螺帽+端窗密封两端）
    + 一端附近的加液泡鼓包（泡顶沿径向 = 垂直于管轴，加液口短管立其上，开口朝上，
    横放时泡朝上就能从泡口加液，无需竖起来）。"""
    # 直管身
    mb.lathe([(TUBE_R, 0.0), (TUBE_R, L)], S, "tube_body")

    # 加液泡鼓包：泡底/泡顶半径=TUBE_R 贴合管壁，中段鼓到 BULB_R（管壁加粗段）
    bulb_z0 = L - BULB_FROM_END
    bp = [(TUBE_R, 0.0),
          (0.0085, BULB_LEN * 0.15),
          (0.0100, BULB_LEN * 0.40),
          (BULB_R, BULB_LEN * 0.62),
          (0.0100, BULB_LEN * 0.85),
          (0.0085, BULB_LEN * 0.95),
          (TUBE_R, BULB_LEN)]
    mb.lathe([(r, z + bulb_z0) for r, z in bp], S, "bulb")

    # 两端螺帽（深灰金属，盖在管端外侧）
    mb.lathe([(CAP_R, -CAP_LEN), (CAP_R, 0.0)], S, "cap_1")
    mb.lathe([(CAP_R, L), (CAP_R, L + CAP_LEN)], S, "cap_2")

    # 玻璃端窗（薄实心圆盘，嵌螺帽内端面）
    mb.lathe([(WIN_R, 0.0), (WIN_R, WIN_THK)], S, "window_1",
             close_bottom=True, close_top=True)
    mb.lathe([(WIN_R, L), (WIN_R, L + WIN_THK)], S, "window_2",
             close_bottom=True, close_top=True)


def add_fill_port(mb, L):
    """加液口短管：绕 Z 轴（朝上）的开口短管，立在泡顶（径向最高点 z=BULB_R）。
    必须在 finalize（管轴转水平沿 y）之后调用——它不参与旋转，转后直接立在泡顶。"""
    bulb_cy = L / 2 - BULB_FROM_END + BULB_LEN / 2   # 泡中心在管轴 y 上的位置
    mb.lathe([(FILL_R, 0.0), (FILL_R, FILL_H)], S, "fill_port")
    n = len(mb.verts)
    for i in range(n - 2 * S, n):                    # fill_port 刚追加的 2 环顶点
        x, y, z = mb.verts[i]
        mb.verts[i] = (x, y + bulb_cy, z + BULB_R)   # 平移到泡顶 (0, bulb_cy, BULB_R)


def finalize(mb, L):
    """整体变换：中心归零（z 平移 −L/2）→ 绕 X 轴 −90°（z→y，管轴转水平沿 y）。
    加液泡随之到 +y 端、螺帽在 y=±(L/2) 外伸。顶点与法线一起转。"""
    for i in range(len(mb.verts)):
        x, y, z = mb.verts[i]
        mb.verts[i] = (x, y, z - L / 2)
    for i in range(len(mb.verts)):
        x, y, z = mb.verts[i]
        mb.verts[i] = (x, z, -y)
    for i in range(len(mb.norms)):
        nx, ny, nz = mb.norms[i]
        mb.norms[i] = (nx, nz, -ny)


USD_MATS = {
    "tube_body": dict(diffuse=(0.90, 0.95, 0.98), opacity=0.25, ior=1.5, roughness=0.05),
    "bulb":      dict(diffuse=(0.90, 0.95, 0.98), opacity=0.25, ior=1.5, roughness=0.05),
    "fill_port": dict(diffuse=(0.90, 0.95, 0.98), opacity=0.25, ior=1.5, roughness=0.05),
    "cap_1":     dict(diffuse=(0.55, 0.56, 0.58), metallic=0.8, roughness=0.30),
    "cap_2":     dict(diffuse=(0.55, 0.56, 0.58), metallic=0.8, roughness=0.30),
    "window_1":  dict(diffuse=(0.90, 0.95, 0.98), opacity=0.25, ior=1.5, roughness=0.05),
    "window_2":  dict(diffuse=(0.90, 0.95, 0.98), opacity=0.25, ior=1.5, roughness=0.05),
}

GROUPS = ["tube_body", "bulb", "fill_port", "cap_1", "cap_2", "window_1", "window_2"]


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
        if spec is not None:
            mat_path = f"/root/{g}_mat"
            mat = UsdShade.Material.Define(stage, mat_path)
            sh = UsdShade.Shader.Define(stage, f"{mat_path}/Shader")
            sh.CreateIdAttr("UsdPreviewSurface")
            sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
                Gf.Vec3f(*spec["diffuse"]))
            if spec.get("opacity", 1.0) < 1.0:
                sh.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(spec["opacity"])
            if spec.get("ior") is not None:
                sh.CreateInput("ior", Sdf.ValueTypeNames.Float).Set(spec["ior"])
            if spec.get("metallic", 0.0) > 0:
                sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(spec["metallic"])
            sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(spec["roughness"])
            mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
            UsdShade.MaterialBindingAPI(mesh).Bind(mat)
    stage.GetRootLayer().Save()
    print(f"{os.path.basename(out_usd)}: {len(groups)} prims OK")


def verify(out_usd, L, name):
    """验证：bbox、管轴沿 y（中心 y=0）、总长 = L+2×CAP_LEN、泡在 +y 端、法线单位向量。"""
    from pxr import Usd, UsdGeom
    stage = Usd.Stage.Open(out_usd)
    lo, hi, bad_n = None, None, 0
    for p in Usd.PrimRange(stage.GetPseudoRoot()):
        if not p.IsA(UsdGeom.Mesh):
            continue
        pts = p.GetAttribute("points").Get()
        nrm = p.GetAttribute("normals").Get()
        if nrm is not None:
            N = np.array([[n[0], n[1], n[2]] for n in nrm])
            lens = np.linalg.norm(N, axis=1)
            bad_n += int(np.sum((lens > 1.0001) | (lens < 0.9999)))
        P = np.array([[v[0], v[1], v[2]] for v in pts])
        mn, mx = P.min(0), P.max(0)
        lo = mn if lo is None else np.minimum(lo, mn)
        hi = mx if hi is None else np.maximum(hi, mx)
    sz = (hi - lo) * 1000
    exp_len = (L + 2 * CAP_LEN) * 1000
    # 泡底 z=−BULB_R、泡顶 z=+BULB_R，加液口从泡顶伸出到 +BULB_R+FILL_H（z 不再对称）；
    # x 对称 ±BULB_R（泡/管绕 y 回转）。
    ok = (abs((hi[1] + lo[1]) / 2) < 1e-4
          and abs(hi[1] - lo[1] - (L + 2 * CAP_LEN)) < 1e-3
          and abs(lo[2] + BULB_R) < 1e-3
          and abs(hi[2] - (BULB_R + FILL_H)) < 1e-3
          and abs(hi[0] + lo[0]) < 1e-4 and abs(hi[0] - lo[0] - BULB_R * 2) < 1e-3
          and bad_n == 0)
    print(f"[verify] {name}: bbox=[{sz[0]:.0f}x{sz[1]:.0f}x{sz[2]:.0f}]mm "
          f"y_len={hi[1]-lo[1]:.3f}m(exp {exp_len:.0f}mm) z[{lo[2]*1000:.0f},{hi[2]*1000:.0f}]"
          f"(泡Ø{BULB_R*2*1000:.0f}+加液口{FILL_H*1000:.0f}) "
          f"bad_normals={bad_n} -> {'OK' if ok else 'FAIL'}")
    assert ok, f"{name} verify FAIL"


def main():
    os.makedirs(OUT_USD, exist_ok=True)
    os.makedirs(OUT_OBJ, exist_ok=True)
    for key, L in LENGTHS.items():
        mb = MeshBuilder()
        build_tube(mb, L)
        finalize(mb, L)
        add_fill_port(mb, L)   # 加液口短管（不参与旋转，转后立在泡顶）
        obj_path = os.path.join(OUT_OBJ, f"polarimeter_tube_{key}.obj")
        with open(obj_path, "w", encoding="utf-8") as f:
            f.write(mb.to_obj(GROUPS))
        out_usd = os.path.join(OUT_USD, f"polarimeter_tube_{key}.usd")
        make_usd(obj_path, out_usd, USD_MATS)
        verify(out_usd, L, key)
    print("DONE")


if __name__ == "__main__":
    main()
