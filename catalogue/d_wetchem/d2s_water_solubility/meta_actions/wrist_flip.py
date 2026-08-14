"""WristFlipAction：直接转腕关节，把药匙由竖直放成水平（不动全臂 IK）。

用户方案（逐字）：「直接让爪子关节旋转90度由水平变成竖直不就可以了」。

背景：pick 旧方案⑥用 MoveAction 的 orient=ORIENT_FLAT 在**冻结 TCP** 下让 Lula
重解整个手臂 —— 全臂 IK 最难的退化情形，解不出 → 每帧 cmd=cur 保持不动 → 600 帧
force-done → 视频里看不到旋转（2026-08-14 实测根因）。本动作绕过 IK：只在腕关节簇
（panda_joint4/5/6，索引 3/4/5）上逐帧贪心推进，把药匙长轴（tool+X）由竖直转到
水平、同时弱偏好手指（tool+Z）朝上——纯关节空间、不依赖 IK 收敛，腕限位内必达。

约定自探测：Lula fk_pose 返回的旋转矩阵是行向量还是列向量约定，代码里无法静态
判定（rot_angle 用迹，对转置不变）。但翻转起点 = ORIENT_HORIZ（从 +X 侧接近），
手指 tool+Z 必朝 +X 世界——据此选「读出的 tool+Z 更贴近 (1,0,0)」的那种读法，
并在整个动作中保持一致。被选中的 tool+X 即药匙长轴真方向（无论 Lula 用哪种约定）。

漂移控制：目标里带 -DRIFT_W·|TCP 位移|，翻腕优先选「手部几乎不动」的关节方向，
防止手往下沉、勺尖碰架顶 0.917。⑤已先提到 H=1.15（勺尖 1.016）留裕量。
"""
import numpy as np
from isaacsim.core.utils.stage import get_stage_units
from isaacsim.core.utils.types import ArticulationAction

# 腕关节簇：panda_joint4(forearm roll)/5(wrist pitch)/6(wrist yaw)，索引 3/4/5。
# 不碰 panda_joint7(flange roll)——它绕 tool+Z 自转，不改变药匙朝向。
WRIST_JOINTS = (3, 4, 5)

PROBE = 0.05        # FK 探测微扰 (rad)
RATE = 0.015        # 每帧关节最大推进 (rad)，与 ik_engine.MAX_JOINT_DELTA 同级，动作从容可辨
DRIFT_W = 10.0      # 漂移惩罚权重（对齐增益 vs 手部位移的权衡）
DONE_ABSZ = 0.09    # 药匙长轴竖直分量 < 0.09（≈5°内水平）视为转到位
MAX_FRAMES = 300    # 兜底（合成验证 ~100 帧收敛）
UP = np.array([0.0, 0.0, 1.0])
FORWARD_X = np.array([1.0, 0.0, 0.0])   # 翻转起点手指朝 +X（从 +X 侧接近的已知事实）


class WristFlipAction:
    """原地把药匙放平：腕关节贪心翻转（不用全臂 IK）。

    forward(joints, gripper_pos, grip_target) -> ArticulationAction，接口与
    MoveAction/GripAction 一致，可直接排进 BaseMetaAction._build_actions()。
    臂通道 = 逐帧贪心命令，夹爪通道每帧发 grip_target（保持药匙夹住）。
    """

    def __init__(self, engine, dwell=15):
        self.engine = engine
        self.dwell = int(dwell)
        self.reset()

    def reset(self):
        self._frame = 0
        self._done = False
        self._row_conv = None    # 首次 forward 探测约定
        self._hold = 0

    # ------------------------------------------------------------------
    def _tool_axes(self, rot):
        """按探测到的约定读 tool+X（药匙长轴）/tool+Z（手指）世界方向。

        rot = Lula fk_pose 返回的 3x3。探测 = 选「tool+Z 更贴近 +X」的读法
        （翻转起点手指朝 +X）。row_conv=True → tool_i = rot[i,:]，否则 rot[:,i]。
        """
        if self._row_conv is None:
            # 两种读法下的 tool+Z 候选
            tz_row = rot[2, :]
            tz_col = rot[:, 2]
            self._row_conv = float(np.dot(tz_row, FORWARD_X)) >= float(np.dot(tz_col, FORWARD_X))
        if self._row_conv:
            return rot[0, :], rot[2, :]
        return rot[:, 0], rot[:, 2]

    # ------------------------------------------------------------------
    def forward(self, joints, gripper_pos, grip_target):
        cur = np.asarray(joints[:7], dtype=float)
        cur_pos, cur_rot = self.engine.fk_pose(cur)
        cur_pos = np.asarray(cur_pos, dtype=float)

        toolx, toolz = self._tool_axes(cur_rot)
        absz = float(abs(toolx[2]))          # 药匙长轴竖直分量（0 = 水平）
        upz = float(np.dot(toolz, UP))       # 手指朝上程度

        if absz < DONE_ABSZ:
            # 已放平：保持冻结，dwell 帧后完成
            self._hold += 1
            cmd = cur
        else:
            # 贪心：在腕关节簇里找「药匙更水平 + 手指更朝上 + 手几乎不动」的关节+方向
            best_j, best_dir, best_score = -1, 1.0, -1e9
            for j in WRIST_JOINTS:
                for s in (1.0, -1.0):
                    jp = cur.copy()
                    jp[j] += s * PROBE
                    pp, rr = self.engine.fk_pose(jp)
                    pp = np.asarray(pp, dtype=float)
                    jx, jz = self._tool_axes(rr)
                    gain = (absz - abs(jx[2])) / PROBE          # 药匙转水平
                    gain += 0.5 * (float(np.dot(jz, UP)) - upz) / PROBE   # 弱偏手指朝上
                    drift = float(np.linalg.norm(pp - cur_pos))
                    score = gain - DRIFT_W * drift
                    if score > best_score:
                        best_j, best_dir, best_score = j, s, score

            if best_j < 0 or best_score <= 0:
                # 腕簇内找不到有利推进（可能腕关节到限位）：放宽漂移权再试一轮
                best_j, best_dir, best_score = -1, 1.0, -1e9
                for j in WRIST_JOINTS:
                    for s in (1.0, -1.0):
                        jp = cur.copy()
                        jp[j] += s * PROBE
                        pp, rr = self.engine.fk_pose(jp)
                        pp = np.asarray(pp, dtype=float)
                        jx, jz = self._tool_axes(rr)
                        gain = (absz - abs(jx[2])) / PROBE
                        gain += 0.5 * (float(np.dot(jz, UP)) - upz) / PROBE
                        drift = float(np.linalg.norm(pp - cur_pos))
                        score = gain - DRIFT_W_LOCAL * drift
                        if score > best_score:
                            best_j, best_dir, best_score = j, s, score

            cmd = cur.copy()
            if best_j >= 0 and best_score > 0:
                cmd[best_j] = cur[best_j] + float(np.clip(best_dir * RATE, -0.015, 0.015))

        target = np.full(joints.shape[0], np.nan)
        target[:7] = cmd
        target[7] = grip_target / get_stage_units()
        target[8] = grip_target / get_stage_units()

        self._frame += 1
        if self._hold >= self.dwell or self._frame >= MAX_FRAMES:
            self._done = True
        return ArticulationAction(joint_positions=target)

    def is_done(self):
        return self._done
