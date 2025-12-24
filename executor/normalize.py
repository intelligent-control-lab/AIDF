from __future__ import annotations

from typing import Any, Dict, List, Mapping, Tuple

from .errors import ValidationError


def normalize_skillplan(plan: Mapping[str, Any]) -> Tuple[Mapping[str, Any], Tuple[str, ...]]:
    """
    Best-effort normalization for common planner/exporter differences.

    This intentionally mutates the provided plan when it is a dict-like object.
    """
    if not isinstance(plan, dict):
        return plan, ()

    warnings: List[str] = []
    counts: Dict[str, int] = {}
    missing_traj_robot_id: Dict[int, int] = {}

    robots = plan.get("robots")
    if not isinstance(robots, list):
        return plan, ()

    robot_by_id: Dict[int, Dict[str, Any]] = {}
    for r in robots:
        if not isinstance(r, dict):
            continue
        rid = r.get("id")
        dof = r.get("dof")
        if isinstance(rid, int) and isinstance(dof, int) and dof > 0:
            robot_by_id[rid] = r

    def _note(msg: str) -> None:
        counts[msg] = counts.get(msg, 0) + 1

    def _trim_joint_list(joints: Any, dof: int, ctx: str) -> None:
        if not isinstance(joints, list):
            return
        n = len(joints)
        if n == dof:
            return
        if n > dof:
            del joints[dof:]
            _note(f"{ctx}: trimmed joint_positions from {n} to {dof}")
            return
        raise ValidationError(f"{ctx}: joint_positions length mismatch: expected {dof}, got {n}")

    initial_scene = plan.get("initial_scene")
    if isinstance(initial_scene, dict):
        init_robots = initial_scene.get("robots")
        if isinstance(init_robots, list):
            for rs in init_robots:
                if not isinstance(rs, dict):
                    continue
                rid = rs.get("robot_id")
                if not isinstance(rid, int) or rid not in robot_by_id:
                    continue
                dof = int(robot_by_id[rid].get("dof"))
                _trim_joint_list(rs.get("joint_positions"), dof, "initial_scene.robots[*].joint_positions")

    actions = plan.get("actions")
    if isinstance(actions, list):
        for aidx, action in enumerate(actions):
            if not isinstance(action, dict):
                continue

            robot = action.get("robot")
            rid = robot.get("id") if isinstance(robot, dict) else None
            if not isinstance(rid, int) or rid not in robot_by_id:
                continue
            dof = int(robot_by_id[rid].get("dof"))

            goal = action.get("goal")
            if isinstance(goal, dict):
                _trim_joint_list(goal.get("joint_positions"), dof, "actions[*].goal.joint_positions")

            traj = action.get("trajectory")
            if isinstance(traj, dict):
                tr_rid = traj.get("robot_id")
                if tr_rid is None or tr_rid == -1:
                    traj["robot_id"] = rid
                    missing_traj_robot_id[rid] = missing_traj_robot_id.get(rid, 0) + 1
                elif isinstance(tr_rid, int) and tr_rid != rid:
                    raise ValidationError(
                        f"actions[{aidx}].trajectory.robot_id mismatch: action robot.id={rid}, traj.robot_id={tr_rid}"
                    )

                points = traj.get("points")
                if isinstance(points, list):
                    for p in points:
                        if not isinstance(p, dict):
                            continue
                        _trim_joint_list(
                            p.get("joint_positions"), dof, "actions[*].trajectory.points[*].joint_positions"
                        )

    if missing_traj_robot_id:
        total = sum(missing_traj_robot_id.values())
        per_robot = ", ".join(f"robot {rid}: {cnt}" for rid, cnt in sorted(missing_traj_robot_id.items()))
        warnings.append(f"actions[*].trajectory.robot_id: filled missing robot_id for {total} actions ({per_robot})")

    for msg, c in sorted(counts.items()):
        warnings.append(msg if c == 1 else f"{msg} (x{c})")
    return plan, tuple(warnings)
