"""给 iron_stand.usd 铁柱加一个挂钩（钩架），照原铁环总成的样子（全圆柱结构）。

修正1：卡箍是 甜甜圈形（圆形活套），支臂是 横着的圆柱（Ø8）。
修正2（终稿）：钩子= 支臂末端竖直短棍 —— 用户定：就在支臂末端竖着装一根短棍。
       修正3：棍更短、位置更低、埋入支臂体无缝焊接（之前立在支臂顶弧面上、底部有缝隙）。

结构（/root/root/hook，与 /root/root/ring 平级）：
    卡箍: 甜甜圈 外Ø32 内Ø16 管Ø8 —— 套在 Ø12 铁柱上
    支臂: 圆柱 Ø8mm 沿 X —— 从铁柱(x=0.006)伸到 0.120（比原 0.100 长 2cm，比上版 0.130 减 1cm）
    钩:   竖直短棍 圆柱 Ø6 —— 轴(x=0.117, y=0)，z∈[0.420, 0.4273]（长 7.3mm = 原 22mm 的 1/3）。
         底部 z=0.420 埋入支臂心（支臂截面 Ø8 完全包住 Ø6 棍，无外露、无缝隙），
         前缘 x=0.120 与支臂端面齐平；露出支臂顶(z=0.424)之上约 3.3mm。

高度: 总成 z∈[0.416, 0.4273]（支臂心 0.420，竖棍顶 0.4273）—— 高于原铁环 z≈0.114。
     铁柱顶 0.460 留裕量（竖棍顶 0.4273 < 0.460）。

材质: 卡箍/支臂 深铁灰 (0.35,0.36,0.39)，钩 稍浅 (0.4,0.41,0.44) —— 与原 ring 分色一致。
碰撞: 钩 mesh 加 physics:collisionEnabled + convexDecomposition。
"""

import math

from pxr import Usd, UsdGeom, Sdf, Gf, UsdShade

PATH = "/media/dky/Disk2TB/lizichang/LabUtopia/assets/equipment/iron_stand.usd"
HOOK_PATH = "/root/root/hook"
MESH_PATH = "/root/root/hook/hook_002"
MAT_ROOT = "/root/_materials"

ZC = 0.420
Z_TOP = ZC + 0.004   # 0.424
Z_BOT = ZC - 0.004   # 0.416

CLAMP_COLOR = (0.35, 0.36, 0.39)
HOOK_COLOR = (0.40, 0.41, 0.44)

N_COLLET = 36
N_CYL = 20


def _newell(pts):
    nx = ny = nz = 0.0
    for i in range(len(pts)):
        p1, p2 = pts[i], pts[(i + 1) % len(pts)]
        nx += (p1[1] - p2[1]) * (p1[2] + p2[2])
        ny += (p1[2] - p2[2]) * (p1[0] + p2[0])
        nz += (p1[0] - p2[0]) * (p1[1] + p2[1])
    return nx, ny, nz


def build():
    all_pts, all_fvc, all_fvi, all_nrm = [], [], [], []
    subset_faces = {}

    def add_face(vlist, outward):
        """vlist: 3或4顶点 (x,y,z)。outward: 期望朝外的单位方向。自动翻转绕序。"""
        n = _newell(vlist)
        nl = math.sqrt(n[0] ** 2 + n[1] ** 2 + n[2] ** 2)
        if nl < 1e-9:
            n = outward
            nl = 1.0
        n = (n[0] / nl, n[1] / nl, n[2] / nl)
        if n[0] * outward[0] + n[1] * outward[1] + n[2] * outward[2] < 0:
            vlist = list(reversed(vlist))
            n = (-n[0], -n[1], -n[2])
        base = len(all_pts)
        for v in vlist:
            all_pts.append(v)
        k = len(vlist)
        all_fvc.append(k)
        all_fvi.extend([base + j for j in range(k)])
        all_nrm.extend([n] * k)
        return len(all_fvc) - 1

    def begin_part(name):
        subset_faces[name] = [len(all_fvc), 0]

    def end_part(name):
        subset_faces[name][1] = len(all_fvc) - subset_faces[name][0]

    def torus(R, r, cx, cy, cz, nmaj, nmin):
        """甜甜圈（圆环活套）：孔沿 Z（铁柱穿过孔）。R=主半径(XY平面)，r=管半径。"""
        def P(th_, ph_):
            ct, st = math.cos(th_), math.sin(th_)
            cp, sp = math.cos(ph_), math.sin(ph_)
            return (cx + (R + r * cp) * ct, cy + (R + r * cp) * st, cz + r * sp)

        for i in range(nmaj):
            th, th2 = 2 * math.pi * i / nmaj, 2 * math.pi * (i + 1) / nmaj
            for j in range(nmin):
                ph, ph2 = 2 * math.pi * j / nmin, 2 * math.pi * (j + 1) / nmin
                thm, phm = (th + th2) / 2, (ph + ph2) / 2
                ct, st = math.cos(thm), math.sin(thm)
                cp, sp = math.cos(phm), math.sin(phm)
                add_face([P(th, ph), P(th2, ph), P(th2, ph2), P(th, ph2)], (cp * ct, cp * st, sp))
        return len(all_fvc)

    def c_ring(R, r, cx, cy, cz, nmaj, nmin, gap_deg=60.0):
        """钥匙圈式 C 形开口圆环：主圆在 YZ 平面（孔朝 X），顶部开口 gap_deg 度。
        P(θ,φ) = (cx + r·cosφ, cy + R·cosθ − r·sinφ·sinθ, cz + R·sinθ + r·sinφ·cosθ)
        环体从 th_start(=π/2+gap/2) 绕到 th_end(=th_start+2π−gap)，两端封口朝开口侧。"""
        gap = math.radians(gap_deg)
        th_start = math.pi / 2.0 + gap / 2.0           # 开口东侧边界
        th_end = th_start + (2.0 * math.pi - gap)      # 开口西侧边界（绕一圈）
        n_span = max(8, int(round(nmaj * (2.0 * math.pi - gap) / (2.0 * math.pi))))

        def P(t_, p_):
            ctt, stt = math.cos(t_), math.sin(t_)
            cpp, spp = math.cos(p_), math.sin(p_)
            return (cx + r * cpp,
                    cy + R * ctt - r * spp * stt,
                    cz + R * stt + r * spp * ctt)

        for i in range(n_span):
            th = th_start + (th_end - th_start) * i / n_span
            th2 = th_start + (th_end - th_start) * (i + 1) / n_span
            thm = (th + th2) / 2
            ct, st = math.cos(thm), math.sin(thm)
            for j in range(nmin):
                ph, ph2 = 2 * math.pi * j / nmin, 2 * math.pi * (j + 1) / nmin
                phm = (ph + ph2) / 2
                cp, sp = math.cos(phm), math.sin(phm)
                outward = (cp, -sp * st, sp * ct)      # cosφ·X + sinφ·T(θm)
                add_face([P(th, ph), P(th2, ph), P(th2, ph2), P(th, ph2)], outward)
        # 开口两端封口（法向朝开口侧：th_start 端 −T，th_end 端 +T）
        for th_cap, sgn in ((th_start, -1.0), (th_end, 1.0)):
            ctt, stt = math.cos(th_cap), math.sin(th_cap)
            T = (0.0, -stt, ctt)
            c = (cx, cy + R * ctt, cz + R * stt)
            verts = []
            for j in range(nmin):
                ph = 2 * math.pi * j / nmin
                cp, sp = math.cos(ph), math.sin(ph)
                verts.append((c[0] + r * cp, c[1] + r * sp * T[1], c[2] + r * sp * T[2]))
            outward = (sgn * T[0], sgn * T[1], sgn * T[2])
            for j in range(nmin):
                j2 = (j + 1) % nmin
                add_face([c, verts[j], verts[j2]], outward)
        return len(all_fvc)

    # ---- 1. 卡箍：甜甜圈（圆环活套） R=0.012 r=0.004 -> 外Ø32 内Ø16 竖直Ø8 ----
    begin_part("hook_clamp_mat_002")
    torus(0.012, 0.004, 0.0, 0.0, ZC, N_COLLET, 16)
    end_part("hook_clamp_mat_002")

    # ---- 2. 支臂：圆柱 Ø8mm 沿 X, x∈[0.006,0.120]（比原 0.100 长 2cm、比上版 0.130 减 1cm）, 轴(y=0,z=0.420) ----
    begin_part("hook_arm_mat_002")
    arm = _cylinder_along_x(all_pts, all_fvc, all_fvi, all_nrm, add_face,
                            r=0.004, l0=0.006, l1=0.120, cy=0.0, cz=ZC, n=N_CYL)
    end_part("hook_arm_mat_002")

    # ---- 3. 钩：支臂末端竖直短棍（Ø6，轴 x=0.117，埋入支臂心 0.420 无缝焊接，顶 0.4273=1/3 原长） ----
    begin_part("hook_mat_002")
    _cylinder_along_z(all_pts, all_fvc, all_fvi, all_nrm, add_face,
                      r=0.003, l0=0.420, l1=0.4273, cx=0.117, cy=0.0, n=N_CYL)
    end_part("hook_mat_002")

    return all_pts, all_fvc, all_fvi, all_nrm, subset_faces


# 三个轴向的具体圆柱（直接写，避免占位错误）
def _cyl_base():
    def circle(r, n):
        return [(r * math.cos(2 * math.pi * i / n), r * math.sin(2 * math.pi * i / n)) for i in range(n)]
    return circle


def _cylinder_along_x(all_pts, all_fvc, all_fvi, all_nrm, add_face, r, l0, l1, cy, cz, n):
    cir = _cyl_base()(r, n)
    def P(t, i):
        u, v = cir[i]
        return (t, cy + u, cz + v)
    for i in range(n):
        j = (i + 1) % n
        mu = (cir[i][0] + cir[j][0]) / 2; mv = (cir[i][1] + cir[j][1]) / 2
        ml = math.hypot(mu, mv) or 1.0
        add_face([P(l0, i), P(l0, j), P(l1, j), P(l1, i)], (0, mu / ml, mv / ml))
    for i in range(n):
        j = (i + 1) % n
        add_face([P(l0, i), P(l0, j), (l0, cy, cz)], (-1, 0, 0))
        add_face([P(l1, i), P(l1, j), (l1, cy, cz)], (1, 0, 0))
    return len(all_fvc)


def _cylinder_along_y(all_pts, all_fvc, all_fvi, all_nrm, add_face, r, l0, l1, cx, cz, n):
    cir = _cyl_base()(r, n)
    def P(t, i):
        u, v = cir[i]
        return (cx + u, t, cz + v)
    for i in range(n):
        j = (i + 1) % n
        mu = (cir[i][0] + cir[j][0]) / 2; mv = (cir[i][1] + cir[j][1]) / 2
        ml = math.hypot(mu, mv) or 1.0
        add_face([P(l0, i), P(l0, j), P(l1, j), P(l1, i)], (mu / ml, 0, mv / ml))
    for i in range(n):
        j = (i + 1) % n
        add_face([P(l0, i), P(l0, j), (cx, l0, cz)], (0, -1, 0))
        add_face([P(l1, i), P(l1, j), (cx, l1, cz)], (0, 1, 0))
    return len(all_fvc)


def _cylinder_along_z(all_pts, all_fvc, all_fvi, all_nrm, add_face, r, l0, l1, cx, cy, n):
    cir = _cyl_base()(r, n)
    def P(t, i):
        u, v = cir[i]
        return (cx + u, cy + v, t)
    for i in range(n):
        j = (i + 1) % n
        mu = (cir[i][0] + cir[j][0]) / 2; mv = (cir[i][1] + cir[j][1]) / 2
        ml = math.hypot(mu, mv) or 1.0
        add_face([P(l0, i), P(l0, j), P(l1, j), P(l1, i)], (mu / ml, mv / ml, 0))
    for i in range(n):
        j = (i + 1) % n
        add_face([P(l0, i), P(l0, j), (cx, cy, l0)], (0, 0, -1))
        add_face([P(l1, i), P(l1, j), (cx, cy, l1)], (0, 0, 1))
    return len(all_fvc)


def make_material(stage, path, color):
    mat = UsdShade.Material.Define(stage, path)
    sh = UsdShade.Shader.Define(stage, path + "/Principled_BSDF")
    sh.CreateOutput("surface", Sdf.ValueTypeNames.Token)
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.85)
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.6)
    sh.CreateInput("ior", Sdf.ValueTypeNames.Float).Set(1.5)
    sh.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(1.0)
    sh.CreateInput("specular", Sdf.ValueTypeNames.Float).Set(0.5)
    sh.CreateInput("clearcoat", Sdf.ValueTypeNames.Float).Set(0.0)
    sh.CreateInput("clearcoatRoughness", Sdf.ValueTypeNames.Float).Set(0.03)
    mat.CreateSurfaceOutput().ConnectToSource(sh.GetOutput("surface"))
    return mat


def main():
    pts, fvc, fvi, nrm, subset_faces = build()

    stage = Usd.Stage.Open(PATH)
    tl = stage.GetRootLayer()

    if stage.GetPrimAtPath(HOOK_PATH).IsValid():
        stage.RemovePrim(HOOK_PATH)
        tl.Save()

    UsdGeom.Xform.Define(stage, HOOK_PATH)
    mesh = UsdGeom.Mesh.Define(stage, MESH_PATH)
    mesh.GetPointsAttr().Set([Gf.Vec3f(*p) for p in pts])
    mesh.GetFaceVertexCountsAttr().Set(fvc)
    mesh.GetFaceVertexIndicesAttr().Set(fvi)
    mesh.GetNormalsAttr().Set([Gf.Vec3f(*n) for n in nrm])
    mesh.SetNormalsInterpolation(UsdGeom.Tokens.faceVarying)
    mesh.GetSubdivisionSchemeAttr().Set("none")

    prim = mesh.GetPrim()
    prim.CreateAttribute("physics:collisionEnabled", Sdf.ValueTypeNames.Bool).Set(True)
    prim.CreateAttribute("physics:approximation", Sdf.ValueTypeNames.Token).Set("convexDecomposition")

    for sname, (f0, nf) in subset_faces.items():
        spath = f"{MESH_PATH}/{sname}"
        gs = UsdGeom.Subset.Define(stage, spath)
        gs.CreateFamilyNameAttr().Set("materialBind")
        gs.CreateElementTypeAttr().Set("face")
        gs.CreateIndicesAttr().Set(list(range(f0, f0 + nf)))

    mat_clamp = make_material(stage, f"{MAT_ROOT}/hook_clamp_mat_002", CLAMP_COLOR)
    mat_arm = make_material(stage, f"{MAT_ROOT}/hook_arm_mat_002", CLAMP_COLOR)
    mat_hook = make_material(stage, f"{MAT_ROOT}/hook_mat_002", HOOK_COLOR)

    UsdShade.MaterialBindingAPI(stage.GetPrimAtPath(f"{MESH_PATH}/hook_clamp_mat_002")).Bind(mat_clamp)
    UsdShade.MaterialBindingAPI(stage.GetPrimAtPath(f"{MESH_PATH}/hook_arm_mat_002")).Bind(mat_arm)
    UsdShade.MaterialBindingAPI(stage.GetPrimAtPath(f"{MESH_PATH}/hook_mat_002")).Bind(mat_hook)
    UsdShade.MaterialBindingAPI(prim).Bind(mat_clamp)

    tl.Save()

    # ---------- 自检 ----------
    print("== 自检 ==")
    st = Usd.Stage.Open(PATH)
    m = UsdGeom.Mesh(st.GetPrimAtPath(MESH_PATH))
    pts2 = [tuple(p) for p in m.GetPointsAttr().Get(0.0)]
    fvc2 = list(m.GetFaceVertexCountsAttr().Get(0.0))
    fvi2 = list(m.GetFaceVertexIndicesAttr().Get(0.0))
    nrm2 = list(m.GetNormalsAttr().Get(0.0))
    print(f"npoints={len(pts2)} nfaces={len(fvc2)} nfvi={len(fvi2)} nnormals={len(nrm2)}"
          f" (normals==sum(fvc)? {len(nrm2) == sum(fvc2)}) subdiv={m.GetSubdivisionSchemeAttr().Get()}")
    xs = [p[0] for p in pts2]; ys = [p[1] for p in pts2]; zs = [p[2] for p in pts2]
    print(f"bbox x[{min(xs):.4f},{max(xs):.4f}] y[{min(ys):.4f},{max(ys):.4f}] z[{min(zs):.4f},{max(zs):.4f}]")
    print(f"期望  x[-0.016,0.120] y[-0.016,0.016] z[0.416,0.4273]（竖棍顶 0.4273，前缘齐支臂端面 0.120）")

    # 顶点半径检查：卡箍应为空心圆（顶点到轴心的半径 ≥ 内孔 0.008-1e-4，≤ 外圆 0.016+1e-4）
    # 只检查 clamp 子集面
    def subset_vertex_radii(sname):
        gs = UsdGeom.Subset(st.GetPrimAtPath(f"{MESH_PATH}/{sname}"))
        idx = gs.GetIndicesAttr().Get()
        rs = []
        for fi in idx:
            base = sum(fvc2[:fi]); n = fvc2[fi]
            for k in range(n):
                v = pts2[fvi2[base + k]]
                rs.append(math.hypot(v[0], v[1]))
        return rs

    rs = subset_vertex_radii("hook_clamp_mat_002")
    print(f"卡箍半径 min={min(rs):.4f} max={max(rs):.4f}（期望 0.008~0.016，空心圆）")

    # 竖棍钩：顶点径向(距x=0.127轴)应∈[0, r=0.003]，z∈[0.420, 0.4273]
    gs_h = UsdGeom.Subset(st.GetPrimAtPath(f"{MESH_PATH}/hook_mat_002"))
    hv = []
    for fi in gs_h.GetIndicesAttr().Get():
        base = sum(fvc2[:fi]); n = fvc2[fi]
        for k in range(n):
            hv.append(pts2[fvi2[base + k]])
    hr = [math.hypot(p[0] - 0.117, p[1]) for p in hv]
    print(f"竖棍钩 径向 min={min(hr):.4f} max={max(hr):.4f}（期望 0~0.003，Ø6）")
    print(f"       z[{min(p[2] for p in hv):.4f},{max(p[2] for p in hv):.4f}]（期望 [0.420,0.4273]）")

    for sname in ["hook_clamp_mat_002", "hook_arm_mat_002", "hook_mat_002"]:
        gs = UsdGeom.Subset(st.GetPrimAtPath(f"{MESH_PATH}/{sname}"))
        idx = gs.GetIndicesAttr().Get()
        print(f"  {sname}: nfaces={len(idx)} family={gs.GetFamilyNameAttr().Get()} "
              f"elemType={gs.GetElementTypeAttr().Get()} range=[{min(idx)},{max(idx)}]")

    p2 = st.GetPrimAtPath(MESH_PATH)
    print(f"collisionEnabled={p2.GetAttribute('physics:collisionEnabled').Get()} "
          f"approx={p2.GetAttribute('physics:approximation').Get()}")


if __name__ == "__main__":
    main()
