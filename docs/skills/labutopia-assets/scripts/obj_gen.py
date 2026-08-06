# -*- coding: utf-8 -*-
"""生成精致的本生灯 + 铂丝 OBJ 资产（供 Blender 导入导出 USD）。

设计原则：
- 所有曲面均为参数化网格（lathe 回转体 / 圆环 / 球面 / 圆柱），带正确法线
- OBJ 用 o/g 分组：每个部件一个 group，名称 = 未来 USD prim 路径（任务代码依赖）
- 本生灯分组：base / tube / collar / side_tube / flame_outer / flame_inner
- 铂丝分组：handle / wire / loop
- 单位：米（Blender 导入时默认 1 unit = 1 m，与场景一致）
"""
import numpy as np
import os

OUT_DIR = r"C:\Users\lenovo\.qoderworkcn\workspace\mrizywidwxqhzovl\outputs"


# ---------------------------------------------------------------- 网格原语
class MeshBuilder:
    """累积顶点/法线/面，输出 OBJ 字符串。"""

    def __init__(self):
        self.verts = []   # list of (x, y, z)
        self.norms = []   # list of (nx, ny, nz)
        self.faces = []   # list of [(v0,v1,v2)] 或 [(v0,v1,v2,v3)]（0-based）
        self._group_faces = {}  # group -> face index range

    def _add_vert(self, p, n):
        idx = len(self.verts)
        self.verts.append(tuple(float(v) for v in p))
        self.norms.append(tuple(float(v) for v in n))
        return idx

    def _add_quad(self, a, b, c, d, group):
        self.faces.append((a, b, c, d))
        self._group_faces.setdefault(group, []).append(len(self.faces) - 1)

    def _add_tri(self, a, b, c, group):
        self.faces.append((a, b, c))
        self._group_faces.setdefault(group, []).append(len(self.faces) - 1)

    # -- lathe：绕 Z 轴回转 profile（r, z）点列；法线由 profile 切线决定
    def lathe(self, profile, segments, group, close_bottom=False, close_top=False, bottom_cap_r=None, top_cap_r=None, reverse=False):
        """profile: [(r, z), ...]。close_bottom/close_top: 端面圆盘。
        reverse=True 时法线翻转（内壁用）。"""
        n_rings = len(profile)
        ring_idx = np.empty((n_rings, segments), dtype=int)
        # 每个 profile 顶点的法线 = 相邻线段法线平均
        normals_rz = []
        for i in range(n_rings):
            r0, z0 = profile[max(i - 1, 0)]
            r1, z1 = profile[i]
            r2, z2 = profile[min(i + 1, n_rings - 1)]
            t1 = np.array([r1 - r0, z1 - z0])
            t2 = np.array([r2 - r1, z2 - z1])
            n = np.array([t1[1], -t1[0]]) + np.array([t2[1], -t2[0]])
            n /= np.linalg.norm(n)
            normals_rz.append(n)
        for i in range(n_rings):
            r, z = profile[i]
            nr, nz = normals_rz[i]
            for j in range(segments):
                th = 2 * np.pi * j / segments
                p = (r * np.cos(th), r * np.sin(th), z)
                n = (nr * np.cos(th), nr * np.sin(th), nz)
                if reverse:
                    n = (-n[0], -n[1], -n[2])
                ring_idx[i, j] = self._add_vert(p, n)
        for i in range(n_rings - 1):
            for j in range(segments):
                j2 = (j + 1) % segments
                a, b = ring_idx[i, j], ring_idx[i, j2]
                c, d = ring_idx[i + 1, j2], ring_idx[i + 1, j]
                if reverse:
                    self._add_quad(b, a, d, c, group)
                else:
                    self._add_quad(a, b, c, d, group)
        # 端面圆盘
        def _cap(z, r, up, ring_j):
            cap_center = self._add_vert((0, 0, z), (0, 0, 1.0 if up else -1.0))
            for j in range(segments):
                th = 2 * np.pi * j / segments
                p = (r * np.cos(th), r * np.sin(th), z)
                n = (0, 0, 1.0) if up else (0, 0, -1.0)
                ring_j[j] = self._add_vert(p, n)
            for j in range(segments):
                j2 = (j + 1) % segments
                if up:
                    self._add_tri(ring_j[j], ring_j[j2], cap_center, group)
                else:
                    self._add_tri(ring_j[j2], ring_j[j], cap_center, group)
        if close_bottom:
            ring_cap = np.empty(segments, dtype=int)
            _cap(profile[0][1], bottom_cap_r if bottom_cap_r is not None else profile[0][0], False, ring_cap)
        if close_top:
            ring_cap = np.empty(segments, dtype=int)
            _cap(profile[-1][1], top_cap_r if top_cap_r is not None else profile[-1][0], True, ring_cap)

    # -- 圆环面（torus）：中心 (cx,cy,cz)，主轴沿 Z，管径 r_tube，环径 R
    def torus(self, cx, cy, cz, R, r_tube, segments_u, segments_v, group):
        v0 = len(self.verts)
        for i in range(segments_u):
            u = 2 * np.pi * i / segments_u
            for j in range(segments_v):
                v = 2 * np.pi * j / segments_v
                px = cx + (R + r_tube * np.cos(v)) * np.cos(u)
                py = cy + (R + r_tube * np.cos(v)) * np.sin(u)
                pz = cz + r_tube * np.sin(v)
                nx = np.cos(v) * np.cos(u)
                ny = np.cos(v) * np.sin(u)
                nz = np.sin(v)
                self._add_vert((px, py, pz), (nx, ny, nz))
        for i in range(segments_u):
            for j in range(segments_v):
                j2 = (j + 1) % segments_v
                i2 = (i + 1) % segments_u
                a = v0 + i * segments_v + j
                b = v0 + i * segments_v + j2
                c = v0 + i2 * segments_v + j2
                d = v0 + i2 * segments_v + j
                self._add_quad(a, b, c, d, group)

    # -- 水平圆柱（侧管）：轴从 p0 到 p1（沿 X），半径 r
    def h_cylinder(self, p0, p1, r, segments, group, cap=False):
        """沿 X 方向的圆柱（侧管用）。法线朝外。"""
        L = np.linalg.norm(np.array(p1) - np.array(p0))
        dx = (np.array(p1) - np.array(p0)) / L
        v0 = len(self.verts)
        # 生成沿轴方向的圆环顶点
        for k in range(2):  # k=0 起点端, k=1 终点端
            base = np.array(p0) + dx * (0 if k == 0 else L)
            for j in range(segments):
                th = 2 * np.pi * j / segments
                # 局部 y-z 平面圆
                py = r * np.cos(th)
                pz = r * np.sin(th)
                p = base + np.array([0, py, pz])
                n = np.array([0, py, pz]) / r
                self._add_vert(p, n)
        for j in range(segments):
            j2 = (j + 1) % segments
            a = v0 + j          # 起点端环
            b = v0 + j2
            c = v0 + segments + j2  # 终点端环
            d = v0 + segments + j
            self._add_quad(a, b, c, d, group)
        if cap:
            # 终点端圆盘（封闭）
            ctr = self._add_vert(tuple(p1), tuple(dx))
            for j in range(segments):
                th = 2 * np.pi * j / segments
                p = p1 + np.array([0, r * np.cos(th), r * np.sin(th)])
                self._add_vert(tuple(p), tuple(dx))
            for j in range(segments):
                j2 = (j + 1) % segments
                self._add_tri(v0 + segments + j, v0 + segments + j2, ctr, group)

    # -- 球面
    def sphere(self, cx, cy, cz, r, rings, segments, group):
        v0 = len(self.verts)
        for i in range(rings + 1):
            phi = np.pi * i / rings
            for j in range(segments):
                th = 2 * np.pi * j / segments
                px = cx + r * np.sin(phi) * np.cos(th)
                py = cy + r * np.sin(phi) * np.sin(th)
                pz = cz + r * np.cos(phi)
                nx = np.sin(phi) * np.cos(th)
                ny = np.sin(phi) * np.sin(th)
                nz = np.cos(phi)
                self._add_vert((px, py, pz), (nx, ny, nz))
        for i in range(rings):
            for j in range(segments):
                j2 = (j + 1) % segments
                a = v0 + i * segments + j
                b = v0 + i * segments + j2
                c = v0 + (i + 1) * segments + j2
                d = v0 + (i + 1) * segments + j
                self._add_quad(a, b, c, d, group)

    # -- 输出 OBJ
    def to_obj(self, groups_order):
        lines = []
        lines.append("# generated by obj_gen.py - units: meters")
        for g in groups_order:
            lines.append(f"\ng {g}")
        for (x, y, z) in self.verts:
            lines.append(f"v {x:.6f} {y:.6f} {z:.6f}")
        for (nx, ny, nz) in self.norms:
            lines.append(f"vn {nx:.6f} {ny:.6f} {nz:.6f}")
        # 面：每个 group 输出自己的面
        for g in groups_order:
            lines.append(f"\ng {g}")
            lines.append(f"usemtl {g}")
            for f in self._group_faces.get(g, []):
                idxs = self.faces[f]
                lines.append("f " + " ".join(f"{i+1}//{i+1}" for i in idxs))
        return "\n".join(lines)


# ---------------------------------------------------------------- 本生灯
def build_bunsen(mb):
    S = 56          # 回转体段数（光滑度）
    S_TOR = 28      # 圆环段数

    # --- base 底座：锥台 + 底部圆盘 + 底边小倒角
    mb.lathe(
        [(0.048, 0.000), (0.043, 0.004), (0.040, 0.012), (0.031, 0.028), (0.030, 0.030)],
        S, "base", close_bottom=True,
    )
    # 底座顶面环带（灯管外壁相接处），避免破洞：直接由 lathe 顶边 + 灯管底边重合即可

    # --- tube 灯管：外壁(喇叭口) + 内壁 + 顶环带
    mb.lathe([(0.012, 0.030), (0.012, 0.148), (0.0140, 0.158)], S, "tube")          # 外壁 + 喇叭口
    mb.lathe([(0.0102, 0.156), (0.0095, 0.140)], S, "tube", reverse=True)           # 内壁（法线朝内）
    # 顶环带：连接外壁喇叭口上沿 (0.0140,0.158) 与内壁上沿 (0.0102,0.156)
    mb.lathe([(0.0140, 0.158), (0.0102, 0.156)], S, "tube")

    # --- collar 调节环：上环 + 凹槽 + 下环（套在灯管上）
    mb.lathe(
        [(0.0165, 0.034), (0.0165, 0.041), (0.0138, 0.046), (0.0165, 0.051), (0.0165, 0.058)],
        S, "collar",
    )
    mb.lathe([(0.0165, 0.034), (0.0165, 0.058)], S, "collar", reverse=True)          # 内壁（贴合灯管）

    # --- side_tube 进气管：水平圆柱 + 端部凸缘 + 端盖（从灯管表面伸出）
    mb.h_cylinder((0.012, 0.0, 0.045), (0.058, 0.0, 0.045), 0.0075, S, "side_tube")
    # 凸缘（略粗的短段）
    mb.h_cylinder((0.058, 0.0, 0.045), (0.063, 0.0, 0.045), 0.0100, S, "side_tube")
    mb.h_cylinder((0.058, 0.0, 0.045), (0.063, 0.0, 0.045), 0.0100, S, "side_tube", cap=True)
    # 端盖圆盘在 cap=True 时已生成（终点端）

    # --- flame_outer 外焰：弧形 profile 锥体，底部封闭
    mb.lathe(
        [(0.0200, 0.162), (0.0150, 0.176), (0.0090, 0.196), (0.0040, 0.238)],
        S, "flame_outer", close_bottom=True,
    )

    # --- flame_inner 内焰：细锥
    mb.lathe(
        [(0.0080, 0.160), (0.0045, 0.172), (0.0020, 0.190)],
        S, "flame_inner", close_bottom=True,
    )


# ---------------------------------------------------------------- 铂丝
def build_wire(mb):
    S = 32
    # --- handle 玻璃手柄：细管 + 顶部玻璃膨大球
    mb.lathe([(0.004, 0.000), (0.004, 0.112)], S, "handle", close_bottom=True)
    mb.sphere(0.0, 0.0, 0.114, 0.0055, 12, S, "handle")   # 顶部膨大（包裹铂丝根部）
    # --- wire 铂丝
    mb.lathe([(0.0012, 0.114), (0.0012, 0.162)], S, "wire")
    mb.lathe([(0.0012, 0.114), (0.0012, 0.162)], S, "wire", close_top=True)
    # --- loop 熔球
    mb.sphere(0.0, 0.0, 0.1655, 0.0035, 10, S, "loop")


# ---------------------------------------------------------------- 材质(mtl)
MTL_BUNSEN = """# bunsen_burner.mtl - 材质名 = 部件名（Blender 导入后自动关联）
newmtl base
Kd 0.55 0.56 0.60
Ks 0.60 0.60 0.62
Ns 120
d 1.0

newmtl tube
Kd 0.60 0.61 0.65
Ks 0.65 0.65 0.68
Ns 140
d 1.0

newmtl collar
Kd 0.78 0.79 0.82
Ks 0.85 0.85 0.88
Ns 160
d 1.0

newmtl side_tube
Kd 0.58 0.59 0.63
Ks 0.62 0.62 0.65
Ns 130
d 1.0

newmtl flame_outer
Kd 1.00 0.55 0.10
Ks 0.30 0.30 0.30
Ns 30
d 0.85

newmtl flame_inner
Kd 1.00 0.85 0.50
Ks 0.40 0.40 0.40
Ns 40
d 0.92
"""

MTL_WIRE = """# platinum_wire.mtl
newmtl handle
Kd 0.72 0.83 0.95
Ks 0.70 0.80 0.90
Ns 200
d 0.85

newmtl wire
Kd 0.88 0.88 0.92
Ks 0.95 0.95 1.00
Ns 220
d 1.0

newmtl loop
Kd 0.95 0.95 0.98
Ks 1.00 1.00 1.00
Ns 240
d 1.0
"""


# ---------------------------------------------------------------- 主流程
def main():
    os.makedirs(OUT_DIR, exist_ok=True)

    groups = ["base", "tube", "collar", "side_tube", "flame_outer", "flame_inner"]
    mb = MeshBuilder()
    build_bunsen(mb)
    obj = mb.to_obj(groups)
    with open(os.path.join(OUT_DIR, "bunsen_burner.obj"), "w", encoding="utf-8") as f:
        f.write(obj)
    with open(os.path.join(OUT_DIR, "bunsen_burner.mtl"), "w", encoding="utf-8") as f:
        f.write(MTL_BUNSEN)
    print(f"bunsen_burner: {len(mb.verts)} verts, {len(mb.faces)} faces")

    groups = ["handle", "wire", "loop"]
    mb = MeshBuilder()
    build_wire(mb)
    obj = mb.to_obj(groups)
    with open(os.path.join(OUT_DIR, "platinum_wire.obj"), "w", encoding="utf-8") as f:
        f.write(obj)
    with open(os.path.join(OUT_DIR, "platinum_wire.mtl"), "w", encoding="utf-8") as f:
        f.write(MTL_WIRE)
    print(f"platinum_wire: {len(mb.verts)} verts, {len(mb.faces)} faces")
    print("DONE")


if __name__ == "__main__":
    main()
