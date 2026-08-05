# -*- coding: utf-8 -*-
"""生成 D2 蒸馏水溶解性测试的 6 个资产 OBJ+USD（单位：米）。

资产清单：
  test_tube      试管 10mL      外径15mm 壁厚1.2mm 高120mm 圆底 玻璃
  test_tube_rack 试管架         底座15x6cm + 2立柱 + 圆孔板(孔r9mm) 木色
  spoon          药匙           手柄长6.5cm + 椭球勺头 不锈钢
  wash_bottle    蒸馏水洗瓶     鼓肚瓶 + 长细嘴 半透明白塑料
  sample_bottle  待测样品瓶     棕色试剂瓶(无液面) 白瓶塞
  sample_powder  粉末堆(场景用) 灰白小丘 直径3cm

设计要点：
- 药匙 asset 原点 = 手柄中点（夹爪抓取点），勺头中心在 +x 0.045 处
- 试管 asset 原点 = 管底（圆底最低点），管口朝 +z
- 试管架孔板 = 圆环板（真孔，夹爪可伸入），立柱用 lathe+shift 平移
"""
import os
import sys
import numpy as np

sys.path.insert(0, r"C:\Users\lenovo\.qoderworkcn\workspace\mrizywidwxqhzovl")
from obj_gen import MeshBuilder  # noqa: E402

OUT_USD = r"E:\浙江大学\星辰计划\LabVLA_第一期轮转\LabUtopia\assets\chemistry_lab"
OUT_OBJ = r"C:\Users\lenovo\.qoderworkcn\workspace\mrizywidwxqhzovl\outputs"


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


# ---------------------------------------------------------------- 试管架
def build_test_tube_rack(mb):
    S = 32
    # base 底座：15x6x0.8cm，z 0->0.008（手动内联 8 顶点 box）
    x0, y0, z0, x1, y1, z1 = -0.075, -0.030, 0.0, 0.075, 0.030, 0.008
    i = [mb._add_vert(p, n) for p, n in [
        ((x0, y0, z0), (0, 0, -1)), ((x1, y0, z0), (0, 0, -1)),
        ((x1, y1, z0), (0, 0, -1)), ((x0, y1, z0), (0, 0, -1)),
        ((x0, y0, z1), (0, 0, 1)), ((x1, y0, z1), (0, 0, 1)),
        ((x1, y1, z1), (0, 0, 1)), ((x0, y1, z1), (0, 0, 1)),
    ]]
    a, b, c, d, e, f, g, h = i
    mb._add_quad(a, b, c, d, "base")
    mb._add_quad(e, h, g, f, "base")
    mb._add_quad(a, e, f, b, "base")
    mb._add_quad(d, c, g, h, "base")
    mb._add_quad(a, d, h, e, "base")
    mb._add_quad(b, f, g, c, "base")
    # 立柱 x2：lathe 在原点生成后平移
    post_v0 = len(mb.verts)
    mb.lathe([(0.004, 0.008), (0.004, 0.050)], S, "post", close_bottom=True)
    post_verts = mb.verts[post_v0:]
    # 立柱1：( -0.055, 0 )
    dx, dy = -0.055, 0.0
    mb.verts[post_v0:] = [(x + dx, y + dy, z) for (x, y, z) in mb.verts[post_v0:]]
    # 立柱2：( +0.055, 0 )
    v0_2 = len(mb.verts)
    mb.lathe([(0.004, 0.008), (0.004, 0.050)], S, "post", close_bottom=True)
    dx, dy = 0.055, 0.0
    mb.verts[v0_2:] = [(x + dx, y + dy, z) for (x, y, z) in mb.verts[v0_2:]]
    # 孔板：圆环板 r_out=0.05 孔 r_in=0.0095，z 0.050->0.058，中心 (0,0)
    annulus(mb, 0.0, 0.0, 0.050, 0.058, 0.0095, 0.050, S, "plate")


# ---------------------------------------------------------------- 药匙
def build_spoon(mb):
    S = 24
    # handle 手柄：x 从 -0.035 到 +0.030，r=0.0025，中心高 z=0.0025
    mb.h_cylinder((-0.035, 0.0, 0.0025), (0.030, 0.0, 0.0025), 0.0025, S, "handle", cap=True)
    # spoon_head 勺头：半椭球，中心 (0.045, 0, 0.004)，半轴 (0.015, 0.006, 0.004)
    # y 宽 12mm < 试管内径 12.6mm，可伸入管口
    ellipsoid_half(mb, 0.045, 0.0, 0.004, 0.015, 0.006, 0.004, 8, S, "spoon_head")


# ---------------------------------------------------------------- 洗瓶
def build_wash_bottle(mb):
    S = 40
    # body+spout：瓶身鼓肚 0->0.082，肩部收窄，细长嘴 0.082->0.135
    mb.lathe([(0.028, 0.000), (0.031, 0.012), (0.033, 0.030), (0.031, 0.048),
              (0.025, 0.066), (0.016, 0.082), (0.010, 0.090),
              (0.006, 0.100), (0.005, 0.118), (0.005, 0.135)],
             S, "body", close_bottom=True)
    # cap 瓶盖：套在瓶颈（z 0.080 处），r 0.014-0.016
    mb.lathe([(0.014, 0.080), (0.016, 0.088), (0.016, 0.096), (0.014, 0.102)],
             S, "cap")


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
    "test_tube_rack": """# test_tube_rack.mtl
newmtl base
Kd 0.55 0.42 0.30
Ks 0.15 0.12 0.09
Ns 30
d 1.0

newmtl post
Kd 0.55 0.42 0.30
Ks 0.15 0.12 0.09
Ns 30
d 1.0

newmtl plate
Kd 0.58 0.45 0.32
Ks 0.15 0.12 0.09
Ns 30
d 1.0
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
    "wash_bottle": """# wash_bottle.mtl
newmtl body
Kd 0.90 0.92 0.95
Ks 0.55 0.58 0.62
Ns 120
d 0.70

newmtl cap
Kd 0.95 0.95 0.95
Ks 0.30 0.30 0.32
Ns 60
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
    "test_tube_rack": {
        "base": dict(diffuse=(0.55, 0.42, 0.30), roughness=0.6),
        "post": dict(diffuse=(0.55, 0.42, 0.30), roughness=0.6),
        "plate": dict(diffuse=(0.58, 0.45, 0.32), roughness=0.6),
    },
    "spoon": {
        "handle": dict(diffuse=(0.70, 0.71, 0.74), metallic=0.85, roughness=0.35),
        "spoon_head": dict(diffuse=(0.72, 0.73, 0.76), metallic=0.85, roughness=0.35),
    },
    "wash_bottle": {
        "body": dict(diffuse=(0.90, 0.92, 0.95), opacity=0.70, roughness=0.3),
        "cap": dict(diffuse=(0.95, 0.95, 0.95), roughness=0.4),
    },
    "sample_bottle": {
        "bottle": dict(diffuse=(0.38, 0.24, 0.14), roughness=0.3),
        "stopper": dict(diffuse=(0.90, 0.90, 0.92), roughness=0.4),
    },
    "sample_powder": {"powder": dict(diffuse=(0.93, 0.93, 0.94), roughness=0.85)},
}

GROUPS = {
    "test_tube": ["tube"],
    "test_tube_rack": ["base", "post", "plate"],
    "spoon": ["handle", "spoon_head"],
    "wash_bottle": ["body", "cap"],
    "sample_bottle": ["bottle", "stopper"],
    "sample_powder": ["powder"],
}

BUILDERS = {
    "test_tube": build_test_tube,
    "test_tube_rack": build_test_tube_rack,
    "spoon": build_spoon,
    "wash_bottle": build_wash_bottle,
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
        make_usd(obj_path, os.path.join(OUT_USD, f"{name}.usd"), USD_MATS[name])
    print("DONE")


if __name__ == "__main__":
    main()
