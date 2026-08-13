# -*- coding: utf-8 -*-
"""生成 D2 蒸馏水溶解性测试的 4 个资产 OBJ+USD（单位：米）。

资产清单：
  test_tube      试管 10mL      外径15mm 壁厚1.2mm 高120mm 圆底 玻璃
  spoon          药匙           手柄长6.5cm + 椭球勺头 不锈钢
  sample_bottle  待测样品瓶     棕色试剂瓶(无液面) 白瓶塞
  sample_powder  粉末堆(场景用) 灰白小丘 直径3cm

（test_tube_rack / wash_bottle 形状错误，已按用户要求从生成清单移除。）

设计要点：
- 药匙 asset 原点 = 手柄中点（夹爪抓取点），勺头中心在 +x 0.045 处
- 试管 asset 原点 = 管底（圆底最低点），管口朝 +z
"""
import os
import sys
import tempfile
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))  # obj_gen.py 与脚本同目录
from obj_gen import MeshBuilder  # noqa: E402

# 输出到 assets/equipment/（*_simple.usd，与 FBX 库版本 test_tube.usd 等区分；
# 供 gen_lab004_scene.py 引用、再生成 D2-S 场景）。
REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT_USD = os.path.join(REPO, "assets", "equipment")
# OBJ 是中间产物（make_usd 读它出 USD），写系统临时目录即可。
OUT_OBJ = os.path.join(tempfile.gettempdir(), "lab004_assets_obj")


# ---------------------------------------------------------------- helpers
def shift(mb, dx, dy, dz):
    """平移全部顶点。"""
    mb.verts = [(x + dx, y + dy, z + dz) for (x, y, z) in mb.verts]


def annulus(mb, cx, cy, z0, z1, r_in, r_out, seg, group):
    """圆环板（真孔）：顶面/底面/外壁/内壁，法线正确。"""
    v_top_o = []; v_top_i = []; v_bot_i = []; v_bot_o = []
    for j in range(seg):
        th = 2 * np.pi * j / seg
        c, s = np.cos(th), np.sin(th)
        v_top_o.append(mb._add_vert((cx + r_out * c, cy + r_out * s, z1), (0, 0, 1)))
        v_top_i.append(mb._add_vert((cx + r_in * c, cy + r_in * s, z1), (0, 0, 1)))
        v_bot_i.append(mb._add_vert((cx + r_in * c, cy + r_in * s, z0), (0, 0, -1)))
        v_bot_o.append(mb._add_vert((cx + r_out * c, cy + r_out * s, z0), (0, 0, -1)))
    for j in range(seg):
        j2 = (j + 1) % seg
        # 顶面（外环→内环，CCW 朝 +z）
        mb._add_quad(v_top_o[j], v_top_o[j2], v_top_i[j2], v_top_i[j], group)
        # 底面（内环→外环，朝 -z）
        mb._add_quad(v_bot_i[j], v_bot_i[j2], v_bot_o[j2], v_bot_o[j], group)
        # 外壁
        mb._add_quad(v_bot_o[j], v_bot_o[j2], v_top_o[j2], v_top_o[j], group)
        # 内壁
        mb._add_quad(v_bot_i[j2], v_bot_i[j], v_top_i[j], v_top_i[j2], group)


def ellipsoid_half(mb, cx, cy, cz, ax, ay, az, rings, seg, group):
    """实心半椭球（z 从 0 到 cz，底面平）：药匙勺头。"""
    v0 = len(mb.verts)
    # φ: 0(赤道) -> π/2(底)？需要 z 从 cz 到 0：用 φ=0..π/2, z=cz*cosφ
    # 顶点行: φ = 0(顶面边缘 z=cz) 到 π/2(底面 z=0)
    for i in range(rings + 1):
        phi = np.pi / 2 * i / rings
        for j in range(seg):
            th = 2 * np.pi * j / seg
            px = cx + ax * np.sin(phi) * np.cos(th)
            py = cy + ay * np.sin(phi) * np.sin(th)
            pz = cz * np.cos(phi)
            # 法线：椭球面法线 = 梯度方向
            n = np.array([np.cos(th) * np.sin(phi) / ax,
                          np.sin(th) * np.sin(phi) / ay,
                          np.cos(phi) / az])
            n /= np.linalg.norm(n)
            mb._add_vert((px, py, pz), tuple(n))
    # 侧面面片（顶行是退化点：φ=0 时 sin=0 -> 所有 j 是同一个点）
    for i in range(rings):
        for j in range(seg):
            j2 = (j + 1) % seg
            a = v0 + i * seg + j
            b = v0 + i * seg + j2
            c = v0 + (i + 1) * seg + j2
            d = v0 + (i + 1) * seg + j
            mb._add_quad(a, b, c, d, group)
    # 底面椭圆盘（z=0，法线 -z）
    ctr = mb._add_vert((cx, cy, 0), (0, 0, -1))
    ring = []
    for j in range(seg):
        th = 2 * np.pi * j / seg
        ring.append(mb._add_vert((cx + ax * np.cos(th), cy + ay * np.sin(th), 0), (0, 0, -1)))
    for j in range(seg):
        j2 = (j + 1) % seg
        mb._add_tri(ring[j2], ring[j], ctr, group)


# ---------------------------------------------------------------- 试管
def build_test_tube(mb):
    S = 40
    # 外壁：圆底(0->0.004 圆角) + 直管 0.004->0.120
    mb.lathe([(0.0000, 0.0000), (0.0045, 0.0008), (0.0065, 0.0022),
              (0.0075, 0.0040), (0.0075, 0.1200)], S, "tube")
    # 内壁（壁厚 1.2mm -> 内径 6.3mm），reverse 法线朝内
    mb.lathe([(0.0063, 0.0040), (0.0063, 0.1200)], S, "tube", reverse=True)
    # 底部内环带（连接外壁内底与内壁底，法线朝下）
    mb.lathe([(0.0075, 0.0040), (0.0063, 0.0040)], S, "tube")
    # 口部环带（连接内壁口与外部口，法线朝上）
    mb.lathe([(0.0063, 0.1200), (0.0075, 0.1200)], S, "tube")


# ---------------------------------------------------------------- 药匙
def build_spoon(mb):
    S = 24
    # handle 手柄：x 从 -0.035 到 +0.030，r=0.0025，中心高 z=0.0025
    mb.h_cylinder((-0.035, 0.0, 0.0025), (0.030, 0.0, 0.0025), 0.0025, S, "handle", cap=True)
    # spoon_head 勺头：半椭球，中心 (0.045, 0, 0.004)，半轴 (0.015, 0.006, 0.004)
    # y 宽 12mm < 试管内径 12.6mm，可伸入管口
    ellipsoid_half(mb, 0.045, 0.0, 0.004, 0.015, 0.006, 0.004, 8, S, "spoon_head")


# ---------------------------------------------------------------- 样品瓶（棕色，无液面）
def build_sample_bottle(mb):
    S = 40
    mb.lathe([(0.016, 0.000), (0.017, 0.006), (0.018, 0.018), (0.018, 0.035),
              (0.0175, 0.048), (0.015, 0.058), (0.0115, 0.064), (0.0098, 0.070)],
             S, "bottle", close_bottom=True)
    mb.lathe([(0.0103, 0.068), (0.0110, 0.073), (0.0126, 0.079)],
             S, "stopper", close_top=True)


# ---------------------------------------------------------------- 粉末堆
def build_sample_powder(mb):
    S = 32
    mb.lathe([(0.015, 0.000), (0.011, 0.0035), (0.006, 0.0055), (0.002, 0.0065)],
             S, "powder", close_bottom=True)


# ---------------------------------------------------------------- 材质
MTL = {
    "test_tube": """# test_tube.mtl
newmtl tube
Kd 0.80 0.88 0.95
Ks 0.75 0.82 0.90
Ns 200
d 0.35
""",
    "spoon": """# spoon.mtl
newmtl handle
Kd 0.70 0.71 0.74
Ks 0.85 0.86 0.90
Ns 180
d 1.0

newmtl spoon_head
Kd 0.72 0.73 0.76
Ks 0.85 0.86 0.90
Ns 180
d 1.0
""",
    "sample_bottle": """# sample_bottle.mtl
newmtl bottle
Kd 0.38 0.24 0.14
Ks 0.20 0.14 0.10
Ns 60
d 1.0

newmtl stopper
Kd 0.90 0.90 0.92
Ks 0.40 0.40 0.42
Ns 80
d 1.0
""",
    "sample_powder": """# sample_powder.mtl
newmtl powder
Kd 0.93 0.93 0.94
Ks 0.15 0.15 0.16
Ns 25
d 1.0
""",
}

USD_MATS = {
    "test_tube": {"tube": dict(diffuse=(0.80, 0.88, 0.95), opacity=0.35, roughness=0.05)},
    "spoon": {
        "handle": dict(diffuse=(0.70, 0.71, 0.74), metallic=0.85, roughness=0.35),
        "spoon_head": dict(diffuse=(0.72, 0.73, 0.76), metallic=0.85, roughness=0.35),
    },
    "sample_bottle": {
        "bottle": dict(diffuse=(0.38, 0.24, 0.14), roughness=0.3),
        "stopper": dict(diffuse=(0.90, 0.90, 0.92), roughness=0.4),
    },
    "sample_powder": {"powder": dict(diffuse=(0.93, 0.93, 0.94), roughness=0.85)},
}

GROUPS = {
    "test_tube": ["tube"],
    "spoon": ["handle", "spoon_head"],
    "sample_bottle": ["bottle", "stopper"],
    "sample_powder": ["powder"],
}

BUILDERS = {
    "test_tube": build_test_tube,
    "spoon": build_spoon,
    "sample_bottle": build_sample_bottle,
    "sample_powder": build_sample_powder,
}


def make_usd(obj_path, out_usd, mat_specs):
    """OBJ -> USD：分组转 prims + 材质绑定。"""
    from pxr import Usd, UsdGeom, UsdShade, Sdf, Gf
    from obj2usd import parse_obj, write_mesh

    verts, vns, groups = parse_obj(obj_path)
    stage = Usd.Stage.CreateNew(out_usd)
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
            if spec.get("emissive") is not None:
                sh.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(
                    Gf.Vec3f(*spec["emissive"]))
            if spec.get("metallic", 0.0) > 0:
                sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(spec["metallic"])
            sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(spec["roughness"])
            mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
            UsdShade.MaterialBindingAPI(mesh).Bind(mat)
    stage.GetRootLayer().Save()
    print(f"{out_usd}: {len(groups)} prims OK")


def main():
    os.makedirs(OUT_USD, exist_ok=True)
    os.makedirs(OUT_OBJ, exist_ok=True)
    for name in BUILDERS:
        mb = MeshBuilder()
        BUILDERS[name](mb)
        obj_path = os.path.join(OUT_OBJ, f"{name}.obj")
        obj = mb.to_obj(GROUPS[name])
        obj = obj.replace("# generated by obj_gen.py - units: meters",
                          "# generated by obj_gen.py - units: meters\n"
                          f"mtllib {name}.mtl", 1)
        with open(obj_path, "w", encoding="utf-8") as f:
            f.write(obj)
        with open(os.path.join(OUT_OBJ, f"{name}.mtl"), "w", encoding="utf-8") as f:
            f.write(MTL[name])
        make_usd(obj_path, os.path.join(OUT_USD, f"{name}_simple.usd"), USD_MATS[name])
    print("DONE")


if __name__ == "__main__":
    main()
