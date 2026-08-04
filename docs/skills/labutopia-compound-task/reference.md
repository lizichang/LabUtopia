# LabUtopia 复合大动作制作参考（reference）

## DissolveTask 状态机模板（D2 已实现并验证）

```python
class DissolveTask(BaseTask):
    def __init__(self, cfg):
        # 读 cfg: paths(prim 路径) / offsets / thresholds
        # _read_translate 读取一次初始位置:
        self.spoon_orig_translate = self._read_translate(spoon_path)
        self.wash_orig_translate = self._read_translate(wash_path)

    def reset(self):
        # 参考点偏移链（全部从 prim 位置推导）:
        table_z = powder_pos[2]
        spoon_grasp_pos = spoon_pos + np.array([0.0, 0.0, 0.0025])      # 手柄中心
        powder_center = powder_pos + [0, 0, 0.0035]
        powder_scoop_pos = powder_center + scoop_insert_offset - spoon_head_offset
        tube_mouth_pos = tube_pos + [0, 0, 0.115]
        tube_transfer_pos = tube_mouth_pos - spoon_head_offset
        wash_grasp_pos = wash_orig + wash_grasp_offset
        wash_lift_pos = wash_grasp + [0, 0, 0.10]
        wash_pour_pos = tube_mouth + [0, 0, wash_pour_lift]
        shake_pos = tube_mouth + shake_offset

    def _update_spoon(self, ...):
        # rest -> (gripper 近 + joint7<0.025) attached
        #      -> [dwell 25f -> PowderOnSpoon reveal]
        #      -> [transfer dwell 25f -> TubeSample reveal, PowderOnSpoon hide]
        #      -> (joint7>0.03) released -> _return_to_origin
    def _update_wash(self, ...):
        # rest -> attached -> [gripper 到 pour 点停留 25f -> TubeWater reveal]
        #      -> released -> _return_to_origin
    def _update_dissolving(self, ...):
        # 关键门控: if not self.dissolved and self.water_added:
        #     gripper 距 shake_pos < 0.15 持续 60 帧 -> dissolved=True, TubeSample hide
        #     obs_done 30 帧
```

## Controller Phase 状态机模板

```python
class DissolveTaskController(BaseController):
    class Phase(Enum):
        SCOOP_POWDER / POUR_WATER / SHAKE / OBSERVE / FINISHED

    def _check_phase_success(self, state):
        # 相位推进条件（上一个动作的 object state 到位）:
        SCOOP_POWDER: spoon_state == 'released'
        POUR_WATER:   wash_state == 'released'
        SHAKE:        state['dissolved'] is True
        OBSERVE:      state['obs_done'] is True

    def _phase_action(self, phase, state, actions, ...):
        SCOOP_POWDER: scoop_controller.forward(
            grasp=state['spoon_grasp_position'],
            scoop=state['powder_scoop_position'],
            transfer=state['tube_transfer_position'],
            grasp_distance=0.006)
        POUR_WATER:   同一 scoop_controller（wash 模式），
            grasp=wash_grasp, scoop=wash_lift, transfer=wash_pour, grasp_distance=0.018
        SHAKE:        shake_controller.forward(current_joint_positions, orientation)
    def _advance_phase(self):
        if self._phase == SHAKE:
            self.shake_controller.set_initial_position(state['shake_position'])
            # 进入 SHAKE 前把摇动点喂给 shake 控制器（位置参数化）
```

## yaml 配置结构

```yaml
name: Level2_Dissolve
task_type: dissolve          # -> task_factory / controller_factory 的注册键
controller_type: dissolve
usd_path: assets/chemistry_lab/lab_004/lab_004.usd
prims:                        # /World/... 路径
  spoon: /World/Spoon
  powder: /World/SamplePowder
  ...
offsets: {...}               # 参考点偏移（可调参数）
thresholds: {...}            # joint7 吸附/释放、dwell 帧数
task:
  max_steps: 4000
cameras: [...]               # 复制现有 level2_*.yaml 的相机配置
robot: {...}                 # 复制现有配置
```

## 验证方法

1. **参考点 pxr 模拟**（本地无需 Isaac Sim）：
   - 加载场景 → 读 prim 世界坐标（`np.array(xf)[3,:3]`，行主序！）
   - 验证每个参考点：在工作区内、与物体几何对齐（勺头距粉末中心 2mm、距管口 0mm）
   - 验证勺头 y 范围完全落入管口 y 范围（无碰撞）
2. **静态检查**：py_compile、yaml 解析、factory 注册键匹配
3. **测试指令**：`python main.py --config-name levelX_<name>`，按检查表核对每个 phase 的日志输出

## 测试预期行为检查表（D2 示例）

- [ ] SCOOP_POWDER: 夹爪抓勺 → 舀取 → 粉末堆出现 → 转移至试管口 → 松开
- [ ] POUR_WATER: 夹爪抓洗瓶 → 提升 → 瓶口对试管口 → 试管中水柱出现 → 放回
- [ ] SHAKE: 夹爪到试管口附近摇动（y 方向摆动）→ 粉末消失（溶解）
- [ ] OBSERVE: 停留观察后任务结束
- [ ] 任何 phase 打印 "xxx failed!" → 把输出发回分析
