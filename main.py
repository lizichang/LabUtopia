import os
import argparse
import subprocess
from isaacsim import SimulationApp

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
    parser.add_argument('--config-name', type=str, default='level3_Heat_Liquid',
                       help='Configuration file name (without .yaml extension)')
    parser.add_argument('--config-dir', type=str, default='config',
                       help='Configuration directory path (default: config)')
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
    if args.no_video:
        save_video = False
        show_video = False
    else:
        save_video = True
        show_video = not args.headless

    robot = create_robot(
        cfg.robot.type,
        position=np.array(cfg.robot.position)
    )
    
    stage = omni.usd.get_context().get_stage()
    add_reference_to_stage(usd_path=os.path.abspath(cfg.usd_path), prim_path="/World")
    
    ObjectUtils.get_instance(stage)
    
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
                    combined_img = np.hstack(camera_images)
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
