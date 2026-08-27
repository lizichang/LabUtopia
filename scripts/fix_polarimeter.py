# -*- coding: utf-8 -*-
"""旋光仪资产后处理（三改）：槽更浅 + 槽后移 + 启动按钮大一倍。

polarimeter.usd 是 Blender 精模（本环境无法重跑 Blender），用 USD 后处理直接改
资产；源脚本 build_polarimeter_blender.py 参数已同步（保持一致）。

1. 槽变浅：tube_rails / windows / lamp_glow 整体 +z RAISE_Z=0.085m（窗口顶
   0.162+0.085=0.247 < 壳顶 0.250，几近贴顶；管中心世界 z 0.9225→0.9825→1.0075，
   机械臂放管几乎不用往下伸）。
2. 槽后移 BACK_Y=0.03m：外壳开口前缘 0.125→0.095、后缘 −0.125→−0.155（改 shell
   mesh 内缘顶点），chamber/hinge/lid 铰链与导轨/窗口/灯同步 −y 后移，前部顶板
   留出更多空间放按钮，布局更均衡。
3. 启动按钮大一倍：Ø32→Ø64（r 0.016→0.032），机顶前部 (0,0.18,0.253)，红色，
   同折光仪 /root/start_button（diffuse (0.85,0.12,0.1)、metallic 0、roughness 0.5）。

幂等：所有 translate 写绝对坐标；shell 顶点只把 y=±0.125 内缘移到 ±(0.125−BACK_Y)，
重跑时无 ±0.125 顶点即空操作。

用法：python scripts/fix_polarimeter.py
"""
import os
from pxr import Usd, UsdGeom, UsdShade, Sdf, Gf, Vt

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
USD = os.path.join(REPO, "assets", "equipment", "polarimeter.usd")

RAISE_Z = 0.085     # 槽变浅抬升量（米，从 Blender 原始几何起算）
BACK_Y = 0.03       # 槽后移量（米）：开口前缘 0.125→0.095、后缘 −0.125→−0.155
BUTTON = dict(x=0.0, y=0.18, z=0.253, r=0.032, h=0.006)  # 中心 z，顶 0.256 / 底 0.250 贴顶板；Ø64


def _translate_op(xf):
    for o in xf.GetOrderedXformOps():
        if o.GetOpName() == "xformOp:translate":
            return o
    return None


def _set_translate(xf, x, y, z):
    op = _translate_op(xf)
    if op is None:
        op = xf.AddTranslateOp()
    op.Set(Gf.Vec3d(x, y, z))


def raise_and_back(stage):
    """槽变浅（+z）+ 槽后移（−y）：导轨/窗口/灯 + 腔室内衬 + 铰链，全部绝对坐标。"""
    for name, z in (("tube_rails", RAISE_Z), ("windows", RAISE_Z),
                    ("lamp_glow", RAISE_Z), ("chamber", 0.0), ("hinge", 0.0)):
        p = stage.GetPrimAtPath(f"/root/{name}")
        if not p.IsValid():
            print(f"[raise] /root/{name} MISSING, skip")
            continue
        _set_translate(UsdGeom.Xformable(p), 0.0, -BACK_Y, z)
        print(f"[raise] /root/{name} translate (0, -{BACK_Y:.3f}, {z:.3f})")

    # lid 铰链后移（保留 rotateXYZ 120° 掀开态、scale）
    p = stage.GetPrimAtPath("/root/lid")
    if p.IsValid():
        _set_translate(UsdGeom.Xformable(p), 0.0, -0.133 - BACK_Y, 0.2505)
        print(f"[raise] /root/lid hinge y -0.133 -> {-0.133 - BACK_Y:.3f}")


def move_shell_opening(stage):
    """外壳开口后移：shell mesh 内缘顶点 y=±0.125 → ±(0.125−BACK_Y)。"""
    mesh = UsdGeom.Mesh(stage.GetPrimAtPath("/root/shell/shell"))
    pts = mesh.GetPointsAttr().Get()
    moved, newpts = 0, []
    for v in pts:
        y = v[1]
        if abs(y - 0.125) < 1e-4:
            y = 0.125 - BACK_Y
            moved += 1
        elif abs(y + 0.125) < 1e-4:
            y = -0.125 - BACK_Y
            moved += 1
        newpts.append(Gf.Vec3f(v[0], y, v[2]))
    mesh.GetPointsAttr().Set(Vt.Vec3fArray(newpts))
    print(f"[shell] opening back {BACK_Y:.3f}: moved {moved} inner-edge vertices")


def set_button(stage):
    """启动按钮大一倍 Ø64；删旧重建（幂等）。"""
    for path in ("/root/start_button", "/root/start_button_mat"):
        if stage.GetPrimAtPath(path).IsValid():
            stage.RemovePrim(path)
    btn = UsdGeom.Cylinder.Define(stage, "/root/start_button")
    btn.CreateRadiusAttr(BUTTON["r"])
    btn.CreateHeightAttr(BUTTON["h"])
    btn.CreateAxisAttr("Z")
    btn.AddTranslateOp().Set(Gf.Vec3d(BUTTON["x"], BUTTON["y"], BUTTON["z"]))

    mat = UsdShade.Material.Define(stage, "/root/start_button_mat")
    sh = UsdShade.Shader.Define(stage, "/root/start_button_mat/Shader")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.85, 0.12, 0.1))
    sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.5)
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI(btn).Bind(mat)
    print(f"[button] /root/start_button Ø{BUTTON['r'] * 2 * 1000:.0f} at "
          f"({BUTTON['x']},{BUTTON['y']},{BUTTON['z']})")


def verify(stage):
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"])

    def bb(path):
        r = bc.ComputeWorldBound(stage.GetPrimAtPath(path)).ComputeAlignedRange()
        return r.GetMin(), r.GetMax()

    lo, hi = bb("/root/tube_rails")
    assert abs(hi[2] - 0.201) < 0.001, f"rails top {hi[2]:.4f} != 0.201"
    assert abs((lo[1] + hi[1]) / 2 + 0.03) < 0.001, "rails center y != -0.03"
    lo, hi = bb("/root/windows")
    assert abs((lo[2] + hi[2]) / 2 - 0.205) < 0.001, "windows center z != 0.205"
    assert abs((lo[1] + hi[1]) / 2 + 0.03) < 0.001, "windows center y != -0.03"
    lo, hi = bb("/root/chamber")
    assert abs((lo[1] + hi[1]) / 2 + 0.03) < 0.001, "chamber center y != -0.03"
    lo, hi = bb("/root/hinge")
    assert abs((lo[1] + hi[1]) / 2 + 0.163) < 0.001, f"hinge center y != -0.163"
    # 外壳开口内缘已后移：无 y=±0.125 内缘顶点，有 y≈0.095 / y≈-0.155
    shell_ys = [v[1] for v in UsdGeom.Mesh(stage.GetPrimAtPath("/root/shell/shell")).GetPointsAttr().Get()]
    assert not any(abs(y - 0.125) < 1e-4 for y in shell_ys), "shell front inner edge still 0.125"
    assert not any(abs(y + 0.125) < 1e-4 for y in shell_ys), "shell rear inner edge still -0.125"
    assert any(abs(y - 0.095) < 1e-4 for y in shell_ys), "shell front inner edge not 0.095"
    assert any(abs(y + 0.155) < 1e-4 for y in shell_ys), "shell rear inner edge not -0.155"
    lo, hi = bb("/root/start_button")
    assert abs(hi[2] - 0.256) < 0.001, f"button top {hi[2]:.4f} != 0.256"
    assert abs(lo[2] - 0.250) < 0.001, f"button base {lo[2]:.4f} != 0.250"
    assert abs((hi[1] - lo[1]) - 0.064) < 0.001, f"button Ø {hi[1]-lo[1]:.4f} != 0.064"
    assert abs((lo[0] + hi[0]) / 2) < 0.001 and abs((lo[1] + hi[1]) / 2 - 0.18) < 0.001, "button xy off"
    print(f"[verify] rails top 0.201 back -0.03 / windows z 0.205 back -0.03 / "
          f"chamber back -0.03 / hinge -0.163 / shell opening 0.095..-0.155 / "
          f"button Ø64 top 0.256 — all OK")


def main():
    stage = Usd.Stage.Open(USD)
    raise_and_back(stage)
    move_shell_opening(stage)
    set_button(stage)
    stage.GetRootLayer().Save()

    stage2 = Usd.Stage.Open(USD)
    verify(stage2)
    print("SAVED", USD)


if __name__ == "__main__":
    main()
