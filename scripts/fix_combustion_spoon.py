"""修复 combustion_spoon.usd 的两处 Blender 导入问题（一次性，幂等）。

背景（2026-09-01）：燃烧匙在 Isaac Sim 快照可见、在 Blender 导入后不可见，根因有二：
  1. 几何容器 /World/combustion_spoon 的 type 是空（''），Blender USD 导入器对
     无 type 的中间 prim 会跳过，其下的 bowl/rod/weld_nub 三个 mesh 全被丢弃。
     对照：药匙 mesh 直接在 Xform 下、试管架容器 type='Xform' 均正常。
  2. 材质仍是 metallic 0.9/0.85（Blender 视口无环境反射 → 金属反黑），与 gen 场景
     brighten_spoon 已改的 metallic 0 + 亮 diffuse 不一致。

修法（非重建模/重导出，只补元数据）：
  - /World/combustion_spoon 补 type='Xform'，/World/Looks 补 type='Scope'。
  - copper/chrome_rod 材质对齐 gen 配方：metallic 0 + 亮 diffuse + specular 0.5。

用法：python scripts/fix_combustion_spoon.py（labutopia conda env 有 pxr）。
"""
from pxr import Usd, UsdGeom, UsdShade, Sdf, Gf

ASSET = "/media/dky/Disk2TB/lizichang/LabUtopia/assets/equipment/combustion_spoon.usd"


def fix_types(stage):
    """给空 type 的容器 prim 补显式 type，Blender 导入器才认得。"""
    for path, typ in (("/World/combustion_spoon", "Xform"),
                      ("/World/Looks", "Scope")):
        p = stage.GetPrimAtPath(path)
        if not p.IsValid():
            print(f"[type] {path} INVALID, skip")
            continue
        old = p.GetTypeName()
        p.SetTypeName(typ)
        print(f"[type] {path}: '{old}' -> '{typ}'")


def fix_materials(stage):
    """对齐 gen brighten_spoon 配方：metallic 0 + 亮 diffuse。"""
    recipes = {
        "copper": ((0.82, 0.50, 0.26), 0.35),
        "chrome_rod": ((0.70, 0.75, 0.80), 0.25),
    }
    for mat_name, (diffuse, rough) in recipes.items():
        sh = stage.GetPrimAtPath(f"/World/Looks/{mat_name}/shader")
        if not sh.IsValid() or sh.GetTypeName() != "Shader":
            print(f"[mat] {mat_name} shader not found, skip")
            continue
        ush = UsdShade.Shader(sh)
        ush.GetInput("diffuseColor").Set(Gf.Vec3f(*diffuse))
        ush.GetInput("metallic").Set(0.0)
        ush.GetInput("roughness").Set(rough)
        ush.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(1.0)
        ush.CreateInput("specular", Sdf.ValueTypeNames.Float).Set(0.5)
        ush.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.0, 0.0, 0.0))
        print(f"[mat] {mat_name}: metallic 0, diffuse {diffuse}, rough {rough}, specular 0.5")


def main():
    stage = Usd.Stage.Open(ASSET)
    fix_types(stage)
    fix_materials(stage)
    stage.Save()
    print(f"SAVED {ASSET}")


if __name__ == "__main__":
    main()
