from __future__ import annotations

import argparse
from pathlib import Path
import sys
from typing import List, Optional

from .executor import ExecutorOptions, SkillPlanExecutor
from .errors import SkillPlanError
from .io import load_json


def _default_model_path() -> str:
    here = Path(__file__).resolve()
    workspace_root = here.parents[2]  # .../catkin_ws/src
    candidates = [
        workspace_root / "robot_digital_twin/gazebo/urdf/dual_gp4.urdf.xacro",
        workspace_root / "robot_digital_twin/gazebo/urdf/dual_gp4.urdf",
        Path("robot_digital_twin/gazebo/urdf/dual_gp4.urdf.xacro"),
        Path("robot_digital_twin/gazebo/urdf/dual_gp4.urdf"),
    ]
    for c in candidates:
        if c.exists():
            return str(c)
    return "robot_digital_twin/gazebo/urdf/dual_gp4.urdf.xacro"


def _build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Execute/validate AIDF skillplan.json")
    p.add_argument("--plan", required=True, help="Path to skillplan.json")
    p.add_argument("--backend", default="dryrun", choices=["dryrun", "mujoco", "ros"], help="Execution backend")
    p.add_argument("--max-actions", type=int, default=0, help="Limit number of actions (0 = all)")

    p.add_argument(
        "--model",
        default=_default_model_path(),
        help="MuJoCo model path (.urdf or .xacro). Used for --backend mujoco.",
    )
    p.add_argument("--dt", type=float, default=0.01, help="Playback timestep for interpolation (seconds)")
    p.add_argument("--realtime", action="store_true", help="Sleep to approximate realtime playback")
    p.add_argument("--viewer", action="store_true", help="Open MuJoCo viewer (requires GUI)")

    p.add_argument(
        "--ros-service",
        action="append",
        default=[],
        help=(
            "Per-robot ExecuteKnownTrajectory service mapping: <robot_id|robot_name>=<service|namespace>. "
            "Example: left_arm=/yk_destroyer/yk_execute_trajectory or 0=/yk_destroyer"
        ),
    )
    p.add_argument(
        "--ros-wait-for-service",
        type=float,
        default=10.0,
        help="Seconds to wait for each ROS service to be available (backend=ros)",
    )
    p.add_argument("--ros-no-wait", action="store_true", help="Call ROS service with wait_for_execution=false (backend=ros)")
    p.add_argument("--quiet", action="store_true", help="Less output")
    return p


def main(argv: Optional[List[str]] = None) -> int:
    args = _build_parser().parse_args(argv)

    plan = load_json(args.plan)
    opts = ExecutorOptions(
        backend=args.backend,
        max_actions=args.max_actions,
        model_path=args.model,
        sim_dt=args.dt,
        realtime=args.realtime,
        viewer=args.viewer,
        ros_services=tuple(args.ros_service or []),
        ros_wait_for_service=float(args.ros_wait_for_service),
        ros_wait_for_execution=not bool(args.ros_no_wait),
        verbose=not args.quiet,
    )
    try:
        SkillPlanExecutor(opts).run(plan)
        return 0
    except SkillPlanError as exc:
        print(f"[error] {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
