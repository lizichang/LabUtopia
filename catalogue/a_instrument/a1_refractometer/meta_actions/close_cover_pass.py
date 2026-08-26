"""元动作 ③：合盖（关圆盖）——从盖子 +y 侧往 -y 推，盖自动合平盖住棱镜井。

合盖方法（用户 2026-08-25）：爪子先待在盖子侧面，再沿 -y 方向移动把盖子一推，
盖子自动合平。场景里盖子 = /World/Refractometer/Cover（绕 well 后沿 X 轴铰链，掀开态
rotateX=-55 立起、前缘抬到 z≈0.9505）。盖子无碰撞（gen 只建 Xform+Cube），故"推"是
爪子轨迹越过前缘 → task._CoverLifecycle 检测爪子 y<0.115 触发 rotateX 平滑转 0（自动
合平），非爪子物理推动。

轨迹（TCP 世界坐标，手指朝下、爪开 GRIP_OPEN）：
  ① 高位接近盖子后方   mv(COVER_APPROACH)      # +y 铰链侧上方 (0.30,0.16,H)
  ② 下探到板面中上部   mv(COVER_PUSH_START)    # z 0.94，板面 +y 侧
  ③ 往 -y 推越过前缘   mv(COVER_PUSH_END, 20)  # y 0.16→0.10，task 触发合盖动画
  ④ 回退高位           mv(COVER_APPROACH)
"""
from ._base import BaseMetaAction, mv
from .constants import COVER_APPROACH, COVER_PUSH_START, COVER_PUSH_END


class CloseCoverPass(BaseMetaAction):
    """合盖：从 +y 侧 -y 推盖子前缘，触发自动合平。"""

    def _build_actions(self):
        e = self.engine
        return [
            mv(e, COVER_APPROACH),       # ① 高位接近盖子后方（铰链 +y 侧）
            mv(e, COVER_PUSH_START),     # ② 下探到板面中上部（z 0.94）
            mv(e, COVER_PUSH_END, 20),   # ③ 往 -y 推，越过前缘 → task 合盖动画
            mv(e, COVER_APPROACH),       # ④ 回退高位
        ]
