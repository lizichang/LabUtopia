"""重烘焙 C1 焰色反应染色锥（水滴型火焰）为当前 STAIN_GEOM 尺寸。

用法：`python scripts/resize_c1_stain.py`（纯 pxr，不需要 Isaac Sim 运行环境）。

背景：染色锥几何烘焙在场景 crate（assets/scenes/c_flame/c1_flame_wire_solid/
c1_flame_wire_solid.usd）的 /World/flame_stain_{color} Mesh 里，由
scripts/fix_flametest_v17.py 的 rebuild_stain_materials() 生成（幂等：每次删旧重建）。

本脚本只复用该函数的 rebuild_stain_materials —— 按 STAIN_GEOM 减半后的新尺寸重建
6 个染色锥 + 材质，再以与 fix 脚本 main() 相同的「临时 usda → defaultPrim 转 token →
crate → 原子替换」方式写回场景。不动 fix 管线其它环节（灯光/火焰锥/滴管等维持现状）。

验证：改完后在 Isaac Sim 里跑
  python main.py --config-dir config --config-name level2_C1FlameWireSolid --snapshot 1 --headless
观察铂丝尖端的水滴型染色火焰是否为原来的一半。
"""

import importlib.util
import os
import re
import sys

# scripts/ 不是包，用 importlib 从文件路径加载 fix 脚本（复用 rebuild_stain_materials/STAIN_GEOM）
_HERE = os.path.dirname(os.path.abspath(__file__))
_FIX = os.path.join(_HERE, "fix_flametest_v17.py")
_spec = importlib.util.spec_from_file_location("fix_flametest_v17", _FIX)
fix = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(fix)

from pxr import Sdf  # noqa: E402

V17 = fix.V17


def main() -> None:
    if not os.path.exists(V17):
        print(f"ERROR: {V17} not found", file=sys.stderr)
        sys.exit(1)

    layer = Sdf.Layer.FindOrOpen(V17)
    if layer is None:
        print(f"ERROR: cannot open {V17}", file=sys.stderr)
        sys.exit(1)

    # 只重建染色锥（幂等），不动管线其它环节
    fix.rebuild_stain_materials(layer)

    # 写回：临时 usda → defaultPrim 转 token → crate → 原子替换（同 fix 脚本 main()）
    tmp_usda = V17 + ".new.usda"
    tmp_crate = V17 + ".new.usd"
    for p in (tmp_usda, tmp_crate):
        if os.path.exists(p):
            os.remove(p)
    layer.Export(tmp_usda)
    with open(tmp_usda, "r", encoding="utf-8") as f:
        text = f.read()
    text = re.sub(r'defaultPrim = "/World"', 'defaultPrim = "World"', text, count=1)
    with open(tmp_usda, "w", encoding="utf-8") as f:
        f.write(text)
    token_layer = Sdf.Layer.FindOrOpen(tmp_usda)
    assert token_layer.defaultPrim == "World", \
        f"defaultPrim not tokenized: {token_layer.defaultPrim!r}"
    token_layer.Export(tmp_crate)
    if os.path.exists(tmp_usda):
        os.remove(tmp_usda)
    os.replace(tmp_crate, V17)
    print(f"[resize_c1_stain] saved: {V17}")
    print(f"[resize_c1_stain] stain flame now "
          f"{fix.STAIN_GEOM['height']*1000:.2f}mm tall x "
          f"{fix.STAIN_GEOM['radius']*2000:.2f}mm wide (half of previous)")
    print("DONE")


if __name__ == "__main__":
    main()
