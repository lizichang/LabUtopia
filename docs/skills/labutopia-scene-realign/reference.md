# labutopia-scene-realign 参考

## 工具详解

### dump_scene_layout.py — 读任意场景的世界布局

```bash
python3 scripts/dump_scene_layout.py assets/scenes/b_thermal/b2_alcohol_heat_liquid/b2_tmp.usd /root/World
python3 scripts/dump_scene_layout.py <scene.usd> <prim1,prim2,...> [--depth N]   # N>1 递归
```

输出每行：`prim名  局部ops(T/R/S串)  世界bbox min/max`。bbox 用 `Gf.TimeCode()`（不是 Default()）——**若资产网格是时间采样在 0.0，Default() 只看到空几何**（坑 R3 的探测入口）。

用途：
- 解读用户 tmp：抄出每个器材的 translate/rot180/bbox → 填 layout.json
- 重生成后核对：dump 生成场景 vs tmp，逐项 bbox 对比

### validate_scene_layout.py — 验证变换模型（两种 op 顺序都试）

```bash
python3 scripts/validate_scene_layout.py /tmp/layout.json
```

layout.json 格式见脚本 docstring。每项同时试 A[R,T]（Rotate 先）与 B[T,R]（Translate 先），打印各自 world bbox 与 PASS/FAIL。**PASS 的那个顺序就是 gen 脚本该用的 op 序。**

## 变换模型推导

用户整组移动+旋转最常见 = **绕竖直轴转 180° + 平移**，模型 `new_pos = R(θ)·old_offset + new_anchor`：

- 对每个器材，旧世界坐标（gen 脚本原值）→ 减去旧锚得局部偏移 → 绕轴旋转 → 加新锚
- **验证不是用中间值，而是直接比较世界 bbox 与 tmp**（bbox 天然吸收旋转，最可靠）
- 旋转只影响 xy（绕 Z），z 不变；bbox 的 min/max 会被旋转交换（如 min.x 变 max.x）

**pxr XformOp 顺序（实证，坑 R1）：**
| ops 列表顺序 | 净效果 |
|---|---|
| `AddTranslateOp` 先、`AddRotateXYZOp` 后 | 先绕**局部原点**旋转，再平移（T 作用在旋转后的坐标系） |
| `AddRotateXYZOp` 先、`AddTranslateOp` 后 | 先平移，再绕**世界方向**旋转（得到错位置） |

B2 实证：`[T, R180]` 重现 tmp bbox；`[R180, T]` 得到明显错位置。**子件局部偏移也会被父旋转取反**（坑 R2）。

## 坑详解

- **R1 XformOp 顺序**：`[T,R]`=先绕局部原点旋转再平移（正确）；`[R,T]`=错位。有旋转先 validate 别猜。
- **R2 rot 取反子件局部偏移**：父绕 Z 转 180° 会把子件局部 y 偏移取反。B2 灯帽：局部 tgt −0.467 → +0.467，世界才仍落在灯 y−0.48 侧。改子件目标点前先想旋转方向。
- **R3 资产网格时间采样怪癖**：资产被某步写成 `points/fvc/fvi` 时间采样在 0.0、default 空 → `Usd.TimeCode.Default()`（=运行时/verify）只见空几何。症状：某资产 default bbox 明显小于应有（温度计只见挂环 z[1.07,1.08]，完整应到 z[0.806]）。修复：
  ```python
  val = attr.Get(Gf.TimeCode())   # 唯一采样点的值
  attr.Clear()                    # 清掉时间采样
  attr.Set(val)                   # 写回干净 default
  ```
  对每个 Mesh 每个带时间采样的属性做一遍；`len@default` 应恢复为完整点数。build 脚本若用 `.Set()` 写 default 则本身干净，不会复发。
- **R4 烘平 Export**：gen 脚本 `stage.Export()` 烘平自包含副本。**改资产文件后必须重跑 gen**，烘平副本才有修复/新几何；只改资产不重跑，场景还是旧的。
- **R5 效果 prim 必须相对基准**：气泡/蒸汽/液柱位置从试管中心（TUBE_X/TUBE_Y）派生，禁止把旧布局的绝对坐标硬编码进新代码——否则堆叠一挪效果就漂。
- **R6 verify 用 Default() 时间**：verify 的 bbox 断言在资产带时间采样时会误读（R3）。资产干净后 Default() 即完整。
- **R7 相机共享常量**：camera 位置/四元数是多个实验共用的，不随单个实验擅自改。堆叠移动后视线可能偏出加热区——**flag 给用户目检**，用户确认后再微调 config translation。
- **R8 task/controller 一般不用改**：task 按 prim 名/path 定位效果 prim，无布局坐标硬编码。**先 grep 确认**（`grep -n "0\.30\|TUBE\|坐标模式" task.py controller.py`），确认无坐标依赖才跳过。

## B2 实例示范（2026-08-25）

用户 tmp 整组绕 Z180°+平移。实测新世界 bbox（dump 工具读 tmp，逐项核对）：

| 器材 | translate | R180 | bbox min / max |
|---|---|---|---|
| IronStand | (0.6286,0.0029,0.80) | ✓ | (0.4746,-0.0621,0.80) / (0.7810,0.0679,1.26) |
| AlcoholLamp | (0.5286,0.0029,0.8002) | ✓ | (0.4850,-0.0407,0.80) / (0.5722,0.0465,0.9007) |
| AsbestosGauze | (0.5286,0.0029,0.9194) | ✓ | (0.4726,-0.0538,0.9184) / (0.5853,0.0589,0.9205) |
| TestTube | (0.5286,0.0029,0.9206) | ✓ | (0.5190,-0.0067,0.9206) / (0.5382,0.0125,1.0739) |
| TestTubeClamp | (0.5781,0.0238,1.0384) | ✓ | (0.5046,-0.0122,1.0240) / (0.6369,0.0393,1.0527) |
| TestTubeRack | (0.50,0.35,0.8965) | ✗ 未动 | (0.4573,0.2072,0.80) / (0.5427,0.4928,0.917) |
| Thermometer | (0.519,0.3496,0.808) | ✗ 未动 | (0.5140,0.3429,0.806) / (0.5240,0.3563,1.0842) |

关键改动落点：
- 常量 `STAND_X,STAND_Y = 0.6286,0.0029`；`TUBE_X = STAND_X − 0.100`；`CLAMP_T = (STAND_X−0.0505, STAND_Y+0.0209, TABLE_TOP+0.2384)`
- EQUIP 每项第 5 元素 `rot180`（堆叠 5 件 True，架/滴管/温度计 False）
- `add_equip`：`AddTranslateOp` 后 `if rot180: AddRotateXYZOp(0,0,180)`（坑 R1 的 B 顺序）
- 灯帽 tgt `(0.0,−0.467,−0.076)` → `(0.0,+0.467,−0.076)`（坑 R2）
- 气泡/蒸汽基础位从硬编码 x=0.30 改为相对 `(TUBE_X, TUBE_Y)`（坑 R5）
- 温度计资产 450 attrs 时间采样→default（坑 R3），改完重跑 gen（坑 R4）
- verify 试管夹断言从 `cmn[0] < STAND_X + 0.03` 改为 `cmx[0] >= STAND_X − 0.03` 和 `cmn[0] < TUBE_X + 0.03`（支臂伸柱 + 钳口抱管，随 R180 换向）
