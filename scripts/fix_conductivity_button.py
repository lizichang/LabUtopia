# -*- coding: utf-8 -*-
"""电导率仪资产后处理（机顶加红色「开始」键）。

conductivity_meter.usd 是 Blender 精模（本环境无法重跑 Blender），用 USD 后处理直接改
资产。2026-08-30 用户：front 面板「确认」键贴竖直面板、离机械臂太前+低 z，水平横向按
ORIENT_FWD（手指 +X）手腕近奇异（j5≈0）IK 卡住一分钟、手指朝下又按不到竖直面 → 把
开始键放机顶（deck 顶，垂直按下），折光仪 /root/start_button 同款。

按钮：红色矮圆柱 Ø32mm（r 0.016）高 6mm（h 0.006），axis Z，中心局部 (0.08,0.08,0.108)
→ 底 0.105 贴 deck 顶、顶 0.111。避开支架 stand（局部 x[-0.03,0.084] y[-0.10,-0.04]
z[0.105,0.232]，顶高 0.232）、电极 electrode（x[0.184,0.206]）、动态线缆 CableRoot
（y≤0.045）、前缘标签 label_model/label_brand（y≥0.104）——(0.08,0.08) 全清。

世界位（场景 rotZ90 + translate(0.4349,-0.213,0.80)，见 gen_a3_scene.py EQUIP Meter）：
  world x = 0.4349 - local_y = 0.3549
  world y = -0.213  + local_x = -0.133
  world z = 0.80 + local_z：中心 0.908 / 顶 0.911 / 底 0.905
距底座 (-0.10,-0.03) 水平 ~0.45m（比电极抓点 0.495m 更近），手指朝下可达。

材质同折光仪：UsdPreviewSurface diffuse(0.85,0.12,0.1)、metallic 0、roughness 0.5。
幂等：删旧 /root/start_button（+ 材质）重建。

用法：python scripts/fix_conductivity_button.py   （conda env labutopia 有 pxr）
"""
import os
from pxr import Usd, UsdGeom, UsdShade, Sdf, Gf

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
USD = os.path.join(REPO, "assets", "equipment", "conductivity_meter.usd")

BUTTON = dict(x=0.08, y=0.08, z=0.108, r=0.016, h=0.006)  # 中心局部；底 0.105 顶 0.111
# 场景里 Meter 的 transform（gen_a3_scene.py EQUIP 第 81 行：translate + rotZ90）
METER_T = (0.4349, -0.2130, 0.80)


def set_button(stage):
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


def remove_confirm_key(stage):
    """删掉 front 面板「确认」键 key_lbl_5（原来的开始按钮，贴竖直面板、水平横向按 IK 卡住）。

    2026-08-30 用户「把现在原来的开始按钮删掉，保留现在新建的红色」：key_lbl_5 是「确认」文字
    label（mesh，n=1276），叠在 keyboard 面板底部键位 (0.132,0.1024,0.022)。keyboard 是一整块
    mesh 面板，无法只删第 5 键的键帽 → 删掉「确认」文字 label 即不再有原来的开始按钮。
    """
    p = stage.GetPrimAtPath("/root/key_lbl_5")
    if p.IsValid():
        stage.RemovePrim("/root/key_lbl_5")
        print("[confirm] removed /root/key_lbl_5 (original 确认 start button)")
    else:
        print("[confirm] /root/key_lbl_5 not found, skip")


def verify(stage):
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"])
    r = bc.ComputeWorldBound(stage.GetPrimAtPath("/root/start_button")).ComputeAlignedRange()
    lo, hi = r.GetMin(), r.GetMax()
    # 局部 bbox：底 0.105（贴 deck 顶）、顶 0.111、Ø32mm、中心 (0.08,0.08)
    assert abs(lo[2] - 0.105) < 0.001, f"button base {lo[2]:.4f} != 0.105"
    assert abs(hi[2] - 0.111) < 0.001, f"button top {hi[2]:.4f} != 0.111"
    assert abs((hi[0] - lo[0]) - 0.032) < 0.001, f"button Ø {hi[0]-lo[0]:.4f} != 0.032"
    cx, cy = (lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2
    assert abs(cx - 0.08) < 0.001 and abs(cy - 0.08) < 0.001, f"button xy ({cx},{cy}) != (0.08,0.08)"
    # 世界位（rotZ90 + translate）：x=0.4349-y、y=-0.213+x、z=0.80+z
    wx = METER_T[0] - cy
    wy = METER_T[1] + cx
    wtop = METER_T[2] + hi[2]
    print(f"[verify] button local base 0.105 top 0.111 Ø32 center (0.08,0.08) — OK")
    print(f"[verify] world center ({wx:.4f},{wy:.4f},{METER_T[2] + BUTTON['z']:.4f}) "
          f"top z={wtop:.4f} base z={METER_T[2] + lo[2]:.4f}")


def main():
    stage = Usd.Stage.Open(USD)
    set_button(stage)
    remove_confirm_key(stage)
    stage.GetRootLayer().Save()

    stage2 = Usd.Stage.Open(USD)
    verify(stage2)
    print("SAVED", USD)


if __name__ == "__main__":
    main()
