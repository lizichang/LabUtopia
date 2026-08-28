# -*- coding: utf-8 -*-
"""A3 电导率仪动态线缆：task 层每帧重算 Catmull-Rom 过点样条，让电极线缆像软线一样跟随电极。

背景：conductivity_meter.usd 里 electrode 是独立 Xform（A3 机械臂夹电极帽 → 烧杯测量），
与机身之间无任何物理连线。方案2（2026-08-27 用户选定）= 一根多段圆柱软线：固定端锚在
机身后测量插座，移动端锚在电极帽顶，task 每帧按当前电极位置重算样条并写各段的
xformOp:transform —— 电极被夹走时线缆被拉长、弯曲、下垂，视觉完全像软线。

布线（纯 pxr 碰撞验证 2026-08-27 通过，静止态 + 移动态全程不穿机身）：
    帽顶 B → 下垂点 Q1=B+CONTROL_B（右外侧）→ 固定后右角 Q2=ANCHOR_C → 插座 ANCHOR_A
    固定端插进插座孔（插座圆柱区 = 允许区，不算穿模）
    用 Catmull-Rom 过点样条而非三次贝塞尔：贝塞尔控制点不在曲线上、中段易内切穿机身，
    过点样条真过每个路径点，布线可控。

坐标约定（全部为 资产局部 = /root 局部；CableRoot 自身无平移，其世界矩阵 = /root 世界）：
    ANCHOR_A   机身后测量插座中心   (-0.060, -0.1015, 0.088)   固定端（线插进插座孔）
    ANCHOR_C   固定后右角           (0.205, -0.108, 0.088)     机身右后外侧，防穿机身
    CONTROL_B  帽端下垂偏移         (0.020, -0.025, -0.070)    下垂点 Q1 = B + cb
    CAP_TOP    静止态电极帽顶       (0.195, 0.040, 0.165)      移动端初始值

矩阵约定（pxr 实测 2026-08-27 锁定）：Gf.Matrix4d 平移写**最后一行**（row-vector，
写 AddTransformOp 读回正确）；局部 +X/+Y 基 × 半径写第 1/2 行、局部 +Z 基 × 长度写第 3
行（圆柱 axis=Z）。世界→局部换算用 CableRoot 世界矩阵的 inverse.Transform(Vec3d)。

用法（A3 task 每帧）：
    cable = DynamicCable(stage, "/World/ConductivityMeter/CableRoot")  # 场景里资产根路径
    cable.update(anchor_b_world)   # anchor_b_world = 电极帽当前世界位置(Gf.Vec3d/Vec3f)
task 无需知道电缆局部空间：内部自动把世界点换算到 CableRoot 局部再算曲线。
"""
from pxr import Gf

# ---- 资产局部锚点常量（与 scripts/fix_conductivity_cable.py 一致，勿单边改） ----
ANCHOR_A = Gf.Vec3d(-0.060, -0.1015, 0.088)   # 机身后测量插座中心（固定端，线插进插座孔）
ANCHOR_C = Gf.Vec3d(0.205, -0.108, 0.088)     # 固定后右角（机身右后外侧）
CONTROL_B = Gf.Vec3d(0.020, -0.025, -0.070)   # 帽端下垂偏移 → Q1 = B + cb
CAP_TOP = Gf.Vec3d(0.195, 0.040, 0.165)       # 静止态电极帽顶（移动端初始值）
SEGMENTS = 120                                 # 段数（按三段弧长比例分配，帽端下垂段不少于 5 段）
                                               # 2026-08-27 用户嫌每段太长有瑕疵：30→120，段长 ~4mm
RADIUS = 0.0035                                # Ø7mm，与原 Blender 线缆 bevel 一致
SEG_OVERLAP = 1.08                             # 每段圆柱比弦长延长 8%：平端盖埋进邻段，接缝/棱消失


def _catmull(p0, p1, p2, p3, t):
    """均匀 Catmull-Rom：返回 p1→p2 段上参数 t 处点，切线由 p0/p3 决定。"""
    t2 = t * t
    t3 = t2 * t
    return 0.5 * ((2 * p1) + (-p0 + p2) * t + (2 * p0 - 5 * p1 + 4 * p2 - p3) * t2
                  + (-p0 + 3 * p1 - 3 * p2 + p3) * t3)


def _seg_matrix(a, b, radius, overlap=SEG_OVERLAP):
    """段圆柱矩阵：从 a 到 b（a,b 为曲线采样点）。平移在最后一行，+Z 对齐切线×长度，
    +X/+Y × radius。长度 = 弦长×overlap（8%）：平端盖伸出到邻段内部，接缝/棱被埋掉，
    整根线读起来是连续圆管。pxr 实测：写 AddTransformOp 后 bbox 中心恰为 (a+b)/2。"""
    d = b - a
    length = d.GetLength() * overlap
    z = d / length if length > 1e-9 else Gf.Vec3d(0.0, 0.0, 1.0)
    ref = Gf.Vec3d(0.0, 0.0, 1.0) if abs(z[2]) < 0.95 else Gf.Vec3d(1.0, 0.0, 0.0)
    y = Gf.Cross(z, ref).GetNormalized()
    x = Gf.Cross(y, z).GetNormalized()
    c = (a + b) * 0.5
    return Gf.Matrix4d(x[0] * radius, x[1] * radius, x[2] * radius, 0.0,
                       y[0] * radius, y[1] * radius, y[2] * radius, 0.0,
                       z[0] * length, z[1] * length, z[2] * length, 0.0,
                       c[0], c[1], c[2], 1.0)


class DynamicCable:
    """电导率仪软线：每帧按电极帽世界位置重算 Catmull-Rom 样条，写各段 transform。

    需要场景里已存在 {cable_root_path}/cable_seg_0..N-1 段圆柱（fix_conductivity_cable.py
    创建）。update() 接受**世界坐标**，内部用 CableRoot 世界矩阵逆变换到局部再算曲线，
    task 无需关心电缆局部空间。
    """

    def __init__(self, stage, cable_root_path):
        self._stage = stage
        self._root = cable_root_path
        self.segments = SEGMENTS
        self.radius = RADIUS
        self.anchor_a = ANCHOR_A
        self.anchor_c = ANCHOR_C
        self.control_b = CONTROL_B
        prim = stage.GetPrimAtPath(cable_root_path)
        if prim and prim.HasAttribute("cable:segments"):
            self.segments = prim.GetAttribute("cable:segments").Get()
            self.radius = prim.GetAttribute("cable:radius").Get()
            self.anchor_a = Gf.Vec3d(*(prim.GetAttribute("cable:anchor_a").Get()))
            self.anchor_c = Gf.Vec3d(*(prim.GetAttribute("cable:anchor_c").Get()))
            self.control_b = Gf.Vec3d(*(prim.GetAttribute("cable:control_b").Get()))
        # 段数按静止态三段弧长比例分配，__init__ 定死，避免每帧段边界跳变
        self._span_segs = self._allocate_spans(CAP_TOP)
        self._seg_paths = [f"{cable_root_path}/cable_seg_{i}" for i in range(self.segments)]
        self._world_inv = None

    # -- 每帧入口 ------------------------------------------------------------
    def update(self, anchor_b_world):
        """anchor_b_world：电极帽当前世界位置（Vec3d/Vec3f）。写各段 transform。"""
        loc = self.to_local(anchor_b_world)
        self.update_local(loc)

    def to_local(self, world_pt):
        """世界坐标 → CableRoot 局部坐标（缓存逆矩阵）。"""
        if self._world_inv is None:
            from pxr import UsdGeom, Usd
            xf = UsdGeom.Xformable(self._stage.GetPrimAtPath(self._root))
            world = xf.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            self._world_inv = world.GetInverse()
        return self._world_inv.Transform(Gf.Vec3d(world_pt))

    def _allocate_spans(self, b_local):
        """三段弧长按比例分配段数（最少 1，总 = self.segments）。"""
        q1 = b_local + self.control_b
        lens = [(q1 - b_local).GetLength(),
                (self.anchor_c - q1).GetLength(),
                (self.anchor_a - self.anchor_c).GetLength()]
        total = sum(lens)
        segs = [max(1, int(self.segments * l / total)) for l in lens]
        remain = self.segments - sum(segs)
        for i in sorted(range(3), key=lambda j: lens[j], reverse=True):
            if remain == 0:
                break
            segs[i] += 1
            remain -= 1
        return segs

    def sample_points(self, b_local, per_span=None):
        """返回 Catmull-Rom 过点样条的密集采样点（含首尾，共 sum(_span_segs)+1 个）。
        per_span 传入时每段固定采样 per_span 个（供碰撞验证用更密）。"""
        q1 = b_local + self.control_b
        spans = [(b_local, b_local, q1, self.anchor_c),
                 (b_local, q1, self.anchor_c, self.anchor_a),
                 (q1, self.anchor_c, self.anchor_a, self.anchor_a)]
        pts = [spans[0][0]]
        for (a, c1, c2, c3), n in zip(spans, self._span_segs):
            m = per_span if per_span else n
            for k in range(1, m + 1):
                pts.append(_catmull(a, c1, c2, c3, k / m))
        return pts

    def update_local(self, b_local):
        """b_local：电极帽在 CableRoot 局部空间的位置。"""
        pts = self.sample_points(b_local)
        for i in range(self.segments):
            self._write_seg(i, _seg_matrix(pts[i], pts[i + 1], self.radius))

    def _write_seg(self, i, m):
        from pxr import UsdGeom
        xf = UsdGeom.Xformable(self._stage.GetPrimAtPath(self._seg_paths[i]))
        op = None
        for o in xf.GetOrderedXformOps():
            if o.GetOpName() == "xformOp:transform":
                op = o
                break
        if op is None:
            op = xf.AddTransformOp()
        op.Set(m)

    # -- 供 fix/verify 用 ------------------------------------------------------
    def seg_bbox_center(self, i):
        from pxr import UsdGeom, Usd
        cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"])
        rng = cache.ComputeWorldBound(self._stage.GetPrimAtPath(self._seg_paths[i])).ComputeAlignedRange()
        lo, hi = rng.GetMin(), rng.GetMax()
        return (lo + hi) * 0.5
