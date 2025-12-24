from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping, Tuple

from .backends.dryrun import DryRunBackend
from .backends.mujoco_sim import MujocoSimBackend
from .errors import SkillPlanError
from .normalize import normalize_skillplan


@dataclass
class ExecutorOptions:
    backend: str = "dryrun"
    max_actions: int = 0

    # MuJoCo options
    model_path: str = "robot_digital_twin/gazebo/urdf/dual_gp4.urdf.xacro"
    sim_dt: float = 0.01
    realtime: bool = False
    viewer: bool = False

    # ROS service backend options
    ros_services: Tuple[str, ...] = ()
    ros_wait_for_execution: bool = True
    ros_wait_for_service: float = 10.0

    verbose: bool = True


class SkillPlanExecutor:
    def __init__(self, options: ExecutorOptions):
        self.options = options

    def run(self, plan: Mapping[str, Any]) -> None:
        plan, normalize_warnings = normalize_skillplan(plan)
        if normalize_warnings and self.options.verbose:
            for w in normalize_warnings:
                print(f"[warn] {w}")

        backend = self.options.backend.lower().strip()
        if backend == "dryrun":
            DryRunBackend(verbose=self.options.verbose).run(plan, max_actions=self.options.max_actions)
            return
        if backend in ("mujoco", "mj", "mujoco_sim"):
            MujocoSimBackend(
                model_path=self.options.model_path,
                sim_dt=self.options.sim_dt,
                realtime=self.options.realtime,
                viewer=self.options.viewer,
                verbose=self.options.verbose,
            ).run(plan, max_actions=self.options.max_actions)
            return
        if backend in ("ros", "rosservice", "ros_service"):
            from .backends.ros_service import RosServiceBackend

            RosServiceBackend(
                services=self.options.ros_services,
                wait_for_execution=self.options.ros_wait_for_execution,
                wait_for_service=self.options.ros_wait_for_service,
                verbose=self.options.verbose,
            ).run(plan, max_actions=self.options.max_actions)
            return
        raise SkillPlanError(f"Unknown backend: {self.options.backend!r}")
