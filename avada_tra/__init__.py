from mjlab.tasks.registry import register_mjlab_task
from src.tasks.tracking.rl import MotionTrackingOnPolicyRunner

from .env_cfgs import avada_flat_tracking_env_cfg
from .rl_cfg import avada_tracking_ppo_runner_cfg

register_mjlab_task(
    task_id="Avada-Tracking",
    env_cfg=avada_flat_tracking_env_cfg(),
    play_env_cfg=avada_flat_tracking_env_cfg(play=True),
    rl_cfg=avada_tracking_ppo_runner_cfg(),
    runner_cls=MotionTrackingOnPolicyRunner,
)

register_mjlab_task(
    task_id="Avada-Tracking-No-State-Estimation",
    env_cfg=avada_flat_tracking_env_cfg(has_state_estimation=False),
    play_env_cfg=avada_flat_tracking_env_cfg(has_state_estimation=False, play=True),
    rl_cfg=avada_tracking_ppo_runner_cfg(),
    runner_cls=MotionTrackingOnPolicyRunner,
)
