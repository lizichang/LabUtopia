import os
import argparse
import subprocess
import sys
from isaacsim import SimulationApp

# ---- 实验结果输入接口（运行前，通用）----
# 每个实验在 config.yaml 的 `experiment_result` 块声明可观察结果字段：
#   <字段名>:
#     label:   中文显示名（交互询问时用）
#     type:    bool（有无，如是否沉淀） | enum（有限选项，如液体/火焰颜色） | number（测量值）
#     options: [...]   # enum 用
#     default: ...     # 未输入时用
# 运行前可经 CLI `--result <字段>=<值>`（可多个）或交互询问输入；值按 schema
# 校验后写回 cfg.<字段> 顶层，task 侧直接读 cfg.<字段>。焰色的 `flame_color`
# 是第一个试点（--flame-color 为其快捷别名）。
def _validate_result_field(spec, field, value):
    """按 schema 校验并规范化单个结果字段值；非法抛 ValueError。"""
    kind = spec.get("type", "enum")
    if kind == "bool":
        v = str(value).strip().lower()
        if v in ("yes", "y", "true", "1", "有", "是"):
            return True
        if v in ("no", "n", "false", "0", "无", "否"):
            return False
        raise ValueError(f"[result] {field} 需为 bool（yes/no）")
    if kind == "enum":
        v = str(value).strip()
        for opt in spec.options:
            if str(opt).lower() == v.lower():
                return opt
        raise ValueError(f"[result] {field} 需为 {'/'.join(str(o) for o in spec.options)}")
    if kind == "number":
        try:
            return float(value)
        except (TypeError, ValueError):
            raise ValueError(f"[result] {field} 需为数字")
    return str(value)


def _prompt_result_field(spec, field):
    """按 schema 交互询问一个结果字段；空回车 = 用 default。"""
    kind = spec.get("type", "enum")
    label = spec.get("label", field)
    default = spec.get("default")
    hint = "yes/no" if kind == "bool" else (
        "/".join(str(o) for o in spec.options) if kind == "enum" else "数字")
    while True:
        ans = input(f"{label}? [{hint}]（回车=默认 {default}）: ").strip()
        if not ans:
            return default
        try:
            return _validate_result_field(spec, field, ans)
        except ValueError as e:
            print(str(e))


def resolve_experiment_results(cfg, cli_results):
    """按 cfg.experiment_result schema 确定各结果字段值（CLI > 交互 > default）。

    把结果写回 cfg.<字段> 顶层（task 直接读，如 cfg.flame_color）；
    返回 {字段: 值}。cfg 无 experiment_result 时返回 {}（非结果实验不生效）。
    """
    schema = cfg.get("experiment_result")
    if schema is None:
        return {}
    interactive = sys.stdin.isatty()
    resolved = {}
    for field, spec in schema.items():
        if field in cli_results:
            try:
                resolved[field] = _validate_result_field(spec, field, cli_results[field])
            except ValueError as e:
                if interactive:
                    resolved[field] = _prompt_result_field(spec, field)
                else:
                    raise SystemExit(str(e))
        elif interactive:
            resolved[field] = _prompt_result_field(spec, field)
        else:
            resolved[field] = spec.get("default")
        cfg[field] = resolved[field]
    return resolved


# Parse command line arguments
def parse_args():
    parser = argparse.ArgumentParser(description='LabSim Simulation Environment')
    parser.add_argument('--backend', type=str, default='numpy',
                       choices=['numpy', 'gpu'],
                       help='Backend choice: numpy (CPU) or gpu')
    parser.add_argument('--headless', action='store_true',
                       help='Run in headless mode (default is with GUI)')
    parser.add_argument('--no-video', action='store_true',
                       help='Disable video display and saving')
    parser.add_argument('--config-name', type=str, default='level3_HeatLiquid',
                       help='Configuration file name (without .yaml extension)')
    parser.add_argument('--config-dir', type=str, default='config',
                       help='Configuration directory path (default: config)')
    parser.add_argument('--result', action='append', default=None, metavar='FIELD=VALUE',
                       help='实验结果字段=值（可多次指定）；字段由 config 的 experiment_result 声明')
    parser.add_argument('--flame-color', type=str, default=None,
                       help='焰色反应现象火焰颜色（= --result flame_color=<值> 的快捷写法）')
    parser.add_argument('--snapshot', type=int, default=0,
                       help='Save N frames as PNG images and exit (quick visual check, no video)')
    parser.add_argument('--snapshot-warmup', type=int, default=12,
                       help='Snapshot 模式抓帧前预热的 render step 数：静态相机（非臂相机）在 '
                            'task.reset() 末尾才 initialize，首个 render step 时 RTX 传感器尚未产出 '
                            '首帧 → get_rgb() 全零 → 空帧（曾导致 camera_1/2 快照时好时坏全黑）。')
    return parser.parse_args()

# Get command line arguments
args = parse_args()

# Set up simulation app based on arguments
simulation_config = {
    "headless": args.headless,
    "extra_args": ["--/rtx/raytracing/fractionalCutoutOpacity=true"],
}

simulation_app = SimulationApp(simulation_config)

import hydra
from omegaconf import OmegaConf
import cv2
import numpy as np

import omni
from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage
import omni.usd
from isaacsim.core.utils import extensions

extensions.enable_extension("omni.physx.bundle")
extensions.enable_extension("omni.usdphysics.ui")

from factories.robot_factory import create_robot
from utils.object_utils import ObjectUtils
from factories.task_factory import create_task
from factories.controller_factory import create_controller
from catalogue.factory import register_catalogue_actions

def _stack_camera_images(images):
    """混合分辨率相机图横排拼接：np.hstack 要求高度一致，先统一到最小高（宽等比缩放）。

    2026-08-27 加：camera_4 特写 1920×1920、其余 512×512，直接 hstack 会崩；
    INTER_AREA 下采样保留清晰度，各相机独立快照 PNG 仍是原始分辨率。
    """
    if not images:
        return None
    h = min(img.shape[0] for img in images)
    resized = []
    for img in images:
        if img.shape[0] != h:
            w = max(1, int(round(img.shape[1] * h / img.shape[0])))
            img = cv2.resize(img, (w, h), interpolation=cv2.INTER_AREA)
        resized.append(img)
    return np.hstack(resized)


class FFmpegVideoWriter:
    """ffmpeg 子进程视频写入器（fragmented MP4 模式）。

    使用 -movflags +frag_keyframe+empty_moov 生成分片 MP4：
    - 每个 keyframe 处开始一个新 fragment，各 fragment 独立可播放
    - 即使主进程被 OOM SIGKILL，已写入的 fragment 仍然有效
    - 视频文件可播放到最后一个完整 fragment（最多丢失 1 秒）

    每帧 write 后 flush，确保数据及时到达 ffmpeg 子进程，
    避免主进程被杀时 Python 缓冲区中的帧丢失。
    """

    def __init__(self, path, width, height, fps=60.0):
        self.path = path
        cmd = ["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo",
               "-pix_fmt", "bgr24", "-s", f"{width}x{height}", "-r", str(fps),
               "-i", "-", "-an", "-c:v", "libx264", "-preset", "ultrafast",
               "-threads", "0", "-crf", "23", "-pix_fmt", "yuv420p",
               "-g", "60",
               "-movflags", "+frag_keyframe+empty_moov",
               path]
        self._proc = subprocess.Popen(cmd, stdin=subprocess.PIPE)

    def write(self, frame):
        self._proc.stdin.write(frame.tobytes())
        self._proc.stdin.flush()

    def release(self):
        try:
            self._proc.stdin.close()
            self._proc.wait(timeout=600)
        except Exception as e:
            print(f"[video] ffmpeg finalize warn: {e}")
            self._proc.kill()


def main():
    hydra.initialize(config_path=args.config_dir, job_name=args.config_name)
    cfg = hydra.compose(config_name=args.config_name)
    # 实验结果输入接口（运行前，通用）：CLI `--result 字段=值` / `--flame-color`
    # （焰色快捷）> 交互询问 > schema default。交互只在 stdin 为终端时触发，
    # 自动化/测试子进程（stdin 非 TTY）自动跳过；值按 schema 校验；结果写回
    # cfg.<字段> 顶层并随 config.yaml 落盘，便于复现。
    cli_results = {}
    if args.flame_color is not None:
        cli_results["flame_color"] = args.flame_color
    for kv in (args.result or []):
        if "=" not in kv:
            raise SystemExit(f"[result] --result 需为 字段=值 格式，收到 {kv!r}")
        k, _, v = kv.partition("=")
        cli_results[k.strip()] = v.strip()
    if "experiment_result" in cfg:
        resolved = resolve_experiment_results(cfg, cli_results)
        if resolved:
            print("[result] 实验结果：", ", ".join(f"{k}={v}" for k, v in resolved.items()))
    os.makedirs(cfg.multi_run.run_dir, exist_ok=True)
    OmegaConf.save(cfg, cfg.multi_run.run_dir + "/config.yaml")

    # Set backend based on command line arguments
    if args.backend == 'gpu':
        world = World(stage_units_in_meters=1, device="cpu")
        physx_interface = omni.physx.get_physx_interface()
        physx_interface.overwrite_gpu_setting(1)
    else:
        world = World(stage_units_in_meters=1.0, physics_prim_path="/physicsScene", backend="numpy")
    
    # Override configuration based on command line arguments
    if args.no_video or args.snapshot > 0:
        save_video = False
        show_video = False
    else:
        save_video = True
        # cv2.imshow 走 OpenCV 的 Qt 后端：无 DISPLAY（headless 服务器/容器）时
        # 连不上 xcb，QApplicationPrivate::init 直接 qFatal abort（核心已转储）。
        # 只在有图形会话时显示；save_video（mp4 录制）与显示无关，保持开启。
        show_video = (not args.headless) and bool(os.environ.get("DISPLAY"))

    robot = create_robot(
        cfg.robot.type,
        position=np.array(cfg.robot.position)
    )
    
    stage = omni.usd.get_context().get_stage()
    add_reference_to_stage(usd_path=os.path.abspath(cfg.usd_path), prim_path="/World")
    
    ObjectUtils.get_instance(stage)

    # catalogue 动作库注册（每个动作一个独立 snake_case 键，类复用现有实现）
    register_catalogue_actions()

    task = create_task(
        cfg.task_type,
        cfg=cfg,
        world=world,
        stage=stage,
        robot=robot,
    )
    
    task_controller = create_controller(
        cfg.controller_type,
        cfg=cfg,
        robot=robot,
    )
    
    video_writer = None
    task.reset()

    # --snapshot 模式：静态相机（camera_1/2）在 task.reset() 末尾才 initialize，首个
    # world.step(render=True) 时 RTX 传感器尚未渲染出帧 → get_rgb() 返回全零 → 空帧 PNG
    # （B4 快照 camera_1/2 偶发全黑根因；臂相机 camera_3 在机器人设置期已渲染多帧所以总正常）。
    # 预热 N 个 render step 让所有相机传感器都产出至少一帧后再抓帧。视频模式不受影响。
    if args.snapshot > 0:
        for _ in range(args.snapshot_warmup):
            world.step(render=True)

    # --snapshot 模式：只抓 N 帧 PNG 就退出，不生成视频
    snapshot_count = 0
    snapshot_dir = None
    if args.snapshot > 0:
        snapshot_dir = os.path.join(cfg.multi_run.run_dir, "snapshot")
        os.makedirs(snapshot_dir, exist_ok=True)
        print(f"[snapshot] Will save {args.snapshot} frames to {snapshot_dir}")
    
    while simulation_app.is_running():
        world.step(render=True)
        
        if world.is_stopped():
            task_controller.reset_needed = True
            
        if world.is_playing():
            if task_controller.need_reset() or task.need_reset():
                if task_controller.episode_num() >= cfg.max_episodes:
                    # 先 close：等待 h5 异步写入完成并 shutdown 进程池，释放 worker 继承的
                    # ffmpeg 管道写端 fd（否则 ffmpeg 收不到 EOF，finalize 时无限等待）
                    task_controller.close()
                    if video_writer is not None:
                        print("[video] releasing writer")
                        video_writer.release()
                        video_writer = None
                        print("[video] released OK")
                    simulation_app.close()
                    cv2.destroyAllWindows()
                    break
                if video_writer is not None:
                    print("[video] releasing writer")
                    video_writer.release()
                    video_writer = None
                    print("[video] released OK")
                task_controller.reset()
                task.reset()

                continue
                
            state = task.step()
            if state is None:
                continue
            
            # --snapshot 模式：存 PNG 并在 N 帧后退出
            if args.snapshot > 0 and snapshot_dir is not None:
                camera_images = []
                for cam_name, image_data in state['camera_display'].items():
                    display_img = cv2.cvtColor(image_data.transpose(1, 2, 0), cv2.COLOR_RGB2BGR)
                    camera_images.append((cam_name, display_img))
                
                if camera_images:
                    # 存每个相机单独的图
                    for cam_name, img in camera_images:
                        safe_name = cam_name.replace('/', '_')
                        fname = f"frame_{snapshot_count:04d}_{safe_name}.png"
                        cv2.imwrite(os.path.join(snapshot_dir, fname), img)
                    
                    # 也存一个拼接的全景图
                    combined = _stack_camera_images([img for _, img in camera_images])
                    cv2.imwrite(os.path.join(snapshot_dir, f"frame_{snapshot_count:04d}_combined.png"), combined)
                    print(f"[snapshot] Saved frame {snapshot_count + 1}/{args.snapshot}")
                
                snapshot_count += 1
                if snapshot_count >= args.snapshot:
                    print(f"[snapshot] Done! {snapshot_count} frames saved to {snapshot_dir}")
                    simulation_app.close()
                    break
            
            action, done, is_success = task_controller.step(state)
            if action is not None:
                robot.get_articulation_controller().apply_action(action)
            if done:
                task_controller.print_failure_reason()
                task.on_task_complete(is_success)
                continue
            
            if save_video or show_video:
                camera_images = []
                for _, image_data in state['camera_display'].items():
                    display_img = cv2.cvtColor(image_data.transpose(1, 2, 0), cv2.COLOR_RGB2BGR)
                    camera_images.append(display_img)
                
                if camera_images:
                    combined_img = _stack_camera_images(camera_images)
                    total_width = 0
                    for idx, img in enumerate(camera_images):
                        label = f"Camera {idx+1} ({cfg.cameras[idx].image_type})"
                        cv2.putText(combined_img, label, (total_width + 2, 20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.25, (255, 255, 255), 1)
                        total_width += img.shape[1]
                    if show_video:
                        cv2.imshow('Camera Views', combined_img)
                        cv2.waitKey(1)
                    if save_video:
                        output_dir = os.path.join(cfg.multi_run.run_dir, "video")
                        os.makedirs(output_dir, exist_ok=True)
                        output_path = os.path.join(output_dir, f"episode_{task_controller._episode_num}.mp4")
                        if video_writer is None:
                            height, width = combined_img.shape[:2]
                            video_writer = FFmpegVideoWriter(output_path, width, height)
                        video_writer.write(combined_img)


if __name__ == "__main__":
    main()
