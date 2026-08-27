# -*- coding: utf-8 -*-
"""生成「带导气管的单孔橡皮塞」资产 rubber_stopper_delivery.usd（D7 气体鉴定专用）。

结构（defaultPrim=/root，米单位，Z-up）：
  /root/stopper  —— 引用 rubber_stopper_3.usd（橡皮塞体：Ø20.5mm 顶、锥形、高 24mm，
                    塞底 z=0、塞顶 z=0.024、塞中心轴 = 原点 Z 轴，材料黑橡胶自带）
  /root/tube     —— 玻璃导气管（MeshBuilder 扫掠空心弯管，∏ 形，透明玻璃）

导气管几何（橡皮塞局部系，塞底 z=0 / 塞顶 z=0.024）——「∏」形，末端悬于检验试管孔正前方：
  短竖直段 (0,0,0.024) → (0,0,0.055)            从塞顶向上 31mm 到桥面
  水平桥   (0,0,0.055) → (0.040,0.159,0.055)    斜向伸到检验试管上方（ΔX=40mm ΔY=159mm）
  长竖直段 (0.040,0.159,0.055) → (0.040,0.159,-0.020)  下探 75mm 到导气管末端
末端 (0.040,0.159,-0.020) 在塞底下方 20mm：2026-08-27 用户「上移距离太长超出导气管长度」→
加长下探段（50mm→75mm），使检验试管下浸后管口（末端 + 53mm 液上 + 15mm 浸深 = 末端 + 68mm）
仍低于导气管桥（末端 + 75mm），桥不落入管口。塞紧产气试管口（塞底 1.044）后末端世界
(0.44,0.079,1.024)。

玻璃管外径 Ø6（r_out 0.003）、内径 Ø4（r_in 0.002）、壁厚 1mm，两端开口（末端伸入液面，气体由此
逸出）；材质透明玻璃（opacity 0.30 / ior 1.5 / rough 0.05，同旋光管）。
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))  # obj_gen.py / obj2usd.py
from obj_gen import MeshBuilder  # noqa: E402
from obj2usd import write_mesh  # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
EQ = os.path.join(REPO, "assets", "equipment")
OUT_USD = os.path.join(EQ, "rubber_stopper_delivery.usd")
STOPPER_SRC = os.path.join(EQ, "rubber_stopper_3.usd")

S = 16                    # 扫掠圆周段数（细管够用）
R_OUT = 0.003             # 玻璃管外半径 Ø6mm
R_IN = 0.002              # 玻璃管内半径 Ø4mm

# —— 导气管路径（橡皮塞局部系，米）——
# 短竖直段 → 水平斜桥 → 长竖直段（端点即两处 90° 折角，细管折角在 Ø6mm 下肉眼难辨，不做倒圆）
PATH = [
    (0.000, 0.000, 0.024),   # P0 塞顶（短段起点，被橡皮塞遮住）
    (0.000, 0.000, 0.055),   # P1 桥面（折角 1）
    (0.040, 0.159, 0.055),   # P2 检验试管正上方（折角 2）
    (0.040, 0.159, -0.020),  # P3 导气管末端（伸入液面，下探 75mm）
]


def tube_sweep(mb, path, r_out, r_in, segments, group):
    """沿折线 path 扫掠空心圆管（外壁法线朝外、内壁法线朝内、两端开口）。

    path: [(x,y,z), ...] 中心线；法线系用固定 up=(0,0,1) 平行输运，折线在竖直平面内不扭曲。
    """
    pts = [np.asarray(p, dtype=float) for p in path]
    n = len(pts)
    tans = []
    for i in range(n):
        p0 = pts[max(i - 1, 0)]
        p1 = pts[min(i + 1, n - 1)]
        d = p1 - p0
        tans.append(d / np.linalg.norm(d))
    up = np.array([0.0, 0.0, 1.0])
    frames = []
    for i in range(n):
        t = tans[i]
        b = np.cross(t, up)
        nb = np.linalg.norm(b)
        b = b / nb if nb > 1e-6 else np.array([1.0, 0.0, 0.0])
        nrm = np.cross(b, t)
        nrm = nrm / np.linalg.norm(nrm)
        frames.append((t, nrm, b))

    ring_o = np.empty((n, segments), dtype=int)
    ring_i = np.empty((n, segments), dtype=int)
    for i in range(n):
        c = pts[i]
        _, nrm, b = frames[i]
        for j in range(segments):
            th = 2 * np.pi * j / segments
            d = nrm * np.cos(th) + b * np.sin(th)
            ring_o[i, j] = mb._add_vert(c + d * r_out, d)      # 外壁法线朝外
            ring_i[i, j] = mb._add_vert(c + d * r_in, -d)      # 内壁法线朝内

    for i in range(n - 1):
        for j in range(segments):
            j2 = (j + 1) % segments
            a, b_ = ring_o[i, j], ring_o[i, j2]
            c, d_ = ring_o[i + 1, j2], ring_o[i + 1, j]
            mb._add_quad(a, b_, c, d_, group)                  # 外壁
            a2, b2 = ring_i[i, j], ring_i[i, j2]
            c2, d2 = ring_i[i + 1, j2], ring_i[i + 1, j]
            mb._add_quad(b2, a2, d2, c2, group)                # 内壁（翻转绕向）


def build_glass(mb):
    tube_sweep(mb, PATH, R_OUT, R_IN, S, "tube")


GLASS = dict(diffuse=(0.90, 0.95, 0.98), opacity=0.30, ior=1.5, roughness=0.05)


def add_glass_material(stage, mesh):
    from pxr import UsdShade, Sdf, Gf
    mat_path = "/root/Looks/glass"
    mat = UsdShade.Material.Define(stage, mat_path)
    sh = UsdShade.Shader.Define(stage, mat_path + "/Shader")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*GLASS["diffuse"]))
    sh.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(GLASS["opacity"])
    sh.CreateInput("ior", Sdf.ValueTypeNames.Float).Set(GLASS["ior"])
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(GLASS["roughness"])
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI(mesh).Bind(mat)


def repair_time_samples(stage, mesh_path):
    """把引用自 rubber_stopper_3.usd 的网格时间采样值修复为 default 值。

    rubber_stopper_3.usd 由 OBJ 导入生成，points/normals/faceVertexCounts/faceVertexIndices
    都只写了时间采样 t=0.0 而无 default 值——BBoxCache(default)/GetPoints()(default) 读到空
    （与 b2 温度计 450 attrs 时间采样→default 修复同因：default 下只见无时间采样的部分）。
    逐属性读 t=0.0 采样值回写为 default（写入本层根 layer，覆盖引用层的弱采样），使塞体在
    default 时间也有完整几何与法线。
    """
    from pxr import Usd, UsdGeom
    m = UsdGeom.Mesh(stage.GetPrimAtPath(mesh_path))
    t0 = Usd.TimeCode(0.0)
    fixed = []
    for attr in m.GetPrim().GetAttributes():
        if attr.GetNumTimeSamples() > 0 and attr.Get() is None:
            v = attr.Get(t0)
            if v is not None:
                attr.Set(v)   # author default in root layer (override referenced time sample)
                fixed.append(attr.GetName())
    return fixed


def override_rubber_color(stage):
    """把引用自 rubber_stopper_3.usd 的塞体橡胶材质从近黑 (0.07,0.06,0.06) 覆盖为可见的红棕橡胶。

    2026-08-27 用户：'导气瓶没出现在桌面上，只有一个导管'——塞体几何/法线/材质绑定都正确（
    逐 prim 检查 bbox 0.570..0.590、336 点、法线全部朝外），根因是塞体橡胶近黑 diffuse 0.07，
    在 CylinderLight 12000 强光下仍几乎不可见，视觉上只看到透明玻璃导管。覆盖为红棕橡胶
    （实验室常见红橡胶塞）既真实又清晰可见。
    """
    from pxr import UsdShade, Sdf, Gf
    shader = UsdShade.Shader(stage.GetPrimAtPath("/root/stopper/Looks/rubber/shader"))
    if not shader.GetPrim().IsValid():
        print("[stopper] rubber shader not found, skip")
        return
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.45, 0.23, 0.18))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.55)
    print("[stopper] rubber override -> red-brown diffuse (0.45,0.23,0.18) roughness 0.55")


def main():
    from pxr import Usd, UsdGeom

    mb = MeshBuilder()
    build_glass(mb)

    stage = Usd.Stage.CreateNew(OUT_USD)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, "Z")
    root = UsdGeom.Xform.Define(stage, "/root")
    stage.SetDefaultPrim(root.GetPrim())

    # 橡皮塞体：引用 rubber_stopper_3.usd（defaultPrim=/World → 内容挂到 /root/stopper 下）
    stopper = UsdGeom.Xform.Define(stage, "/root/stopper")
    stopper.GetPrim().GetReferences().AddReference(os.path.relpath(STOPPER_SRC, EQ))
    print(f"[stopper] reference <- {os.path.basename(STOPPER_SRC)}")
    fixed = repair_time_samples(stage, "/root/stopper/rubber_stopper")
    print(f"[stopper] repaired time-sample attrs -> default: {fixed}")
    override_rubber_color(stage)

    # 玻璃导气管：把 MeshBuilder 网格直接写入 /root/tube
    UsdGeom.Xform.Define(stage, "/root/tube")
    verts = np.array(mb.verts, dtype=float)
    norms = np.array(mb.norms, dtype=float)
    faces = [list(f) for f in mb.faces]
    mesh = write_mesh(stage, "/root/tube/glass", verts, faces, norms)
    add_glass_material(stage, mesh)
    print(f"[tube] glass mesh {len(verts)} verts, {len(faces)} faces")

    stage.GetRootLayer().Save()

    # —— 验证 ——
    stage2 = Usd.Stage.Open(OUT_USD)
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"])
    lo, hi = None, None
    for p in Usd.PrimRange(stage2.GetPseudoRoot()):
        if p.IsA(UsdGeom.Mesh):
            r = bc.ComputeWorldBound(p).ComputeAlignedRange()
            mn, mx = r.GetMin(), r.GetMax()
            lo = mn if lo is None else np.minimum(lo, mn)
            hi = mx if hi is None else np.maximum(hi, mx)
    sz = hi - lo
    ok = (abs(lo[2] - PATH[-1][2]) < 1e-3             # 整体最低 = 末端开口环 z=-0.020（开口端，无 -R_OUT 帽）
          and abs(hi[2] - 0.055) < 1e-3 + R_OUT        # 桥面顶 z=0.055（+R_OUT 顶点）
          and hi[0] > 0.040 + R_OUT - 1e-3             # 末端 x 触及 0.040+R_OUT
          and hi[1] > 0.159 + R_OUT - 1e-3             # 末端 y 触及 0.159+R_OUT
          and lo[0] < -0.010)                          # 塞体 Ø20.5 → x 最低过 -0.010（tube 仅 -0.003）
    print(f"[verify] bbox min({lo[0]:+.4f},{lo[1]:+.4f},{lo[2]:+.4f}) "
          f"max({hi[0]:+.4f},{hi[1]:+.4f},{hi[2]:+.4f}) size({sz[0]*1000:.0f}x{sz[1]*1000:.0f}"
          f"x{sz[2]*1000:.0f})mm -> {'OK' if ok else 'FAIL'}")
    assert ok, "delivery stopper verify FAIL"
    print("SAVED", OUT_USD)


if __name__ == "__main__":
    main()
