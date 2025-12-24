from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Set, Tuple

from .errors import ValidationError


@dataclass(frozen=True)
class ValidationReport:
    warnings: Tuple[str, ...] = ()


def _require(cond: bool, msg: str) -> None:
    if not cond:
        raise ValidationError(msg)


def _is_number(x: Any) -> bool:
    return isinstance(x, (int, float)) and not (isinstance(x, float) and (math.isnan(x) or math.isinf(x)))


def _require_number(x: Any, ctx: str) -> None:
    _require(_is_number(x), f"{ctx} must be a finite number, got {type(x).__name__}: {x!r}")


def _require_str(x: Any, ctx: str) -> None:
    _require(isinstance(x, str) and x != "", f"{ctx} must be a non-empty string")


def _require_int(x: Any, ctx: str) -> None:
    _require(isinstance(x, int), f"{ctx} must be an int, got {type(x).__name__}")


def _require_list(x: Any, ctx: str) -> None:
    _require(isinstance(x, list), f"{ctx} must be a list, got {type(x).__name__}")


def validate_skillplan(plan: Mapping[str, Any]) -> ValidationReport:
    warnings: List[str] = []

    _require_str(plan.get("schema"), "schema")
    _require(plan.get("schema") == "aidf.skillplan", f"Unsupported schema: {plan.get('schema')!r}")
    _require_int(plan.get("schema_version"), "schema_version")
    _require(plan.get("schema_version") >= 1, "schema_version must be >= 1")

    robots = plan.get("robots")
    _require_list(robots, "robots")
    robot_by_id: Dict[int, Dict[str, Any]] = {}
    for idx, robot in enumerate(robots):
        _require(isinstance(robot, dict), f"robots[{idx}] must be an object")
        rid = robot.get("id")
        _require_int(rid, f"robots[{idx}].id")
        _require(rid not in robot_by_id, f"Duplicate robot id: {rid}")
        _require_str(robot.get("name"), f"robots[{idx}].name")
        _require_int(robot.get("dof"), f"robots[{idx}].dof")
        _require(robot.get("dof") > 0, f"robots[{idx}].dof must be > 0")
        robot_by_id[rid] = robot

    initial_scene = plan.get("initial_scene")
    _require(isinstance(initial_scene, dict), "initial_scene must be an object")
    init_robots = initial_scene.get("robots")
    _require_list(init_robots, "initial_scene.robots")
    init_by_id: Dict[int, Dict[str, Any]] = {}
    for idx, rs in enumerate(init_robots):
        _require(isinstance(rs, dict), f"initial_scene.robots[{idx}] must be an object")
        rid = rs.get("robot_id")
        _require_int(rid, f"initial_scene.robots[{idx}].robot_id")
        _require(rid in robot_by_id, f"initial_scene.robots[{idx}].robot_id={rid} not in robots[]")
        _require(rid not in init_by_id, f"Duplicate initial robot state for robot_id={rid}")
        joints = rs.get("joint_positions")
        _require_list(joints, f"initial_scene.robots[{idx}].joint_positions")
        _require(
            len(joints) == int(robot_by_id[rid]["dof"]),
            f"initial_scene.robots[{idx}].joint_positions length mismatch: expected {robot_by_id[rid]['dof']}, got {len(joints)}",
        )
        for j, v in enumerate(joints):
            _require_number(v, f"initial_scene.robots[{idx}].joint_positions[{j}]")
        init_by_id[rid] = rs
    if len(init_by_id) != len(robot_by_id):
        missing = sorted(set(robot_by_id.keys()) - set(init_by_id.keys()))
        warnings.append(f"initial_scene missing robot states for robot_ids: {missing}")

    actions = plan.get("actions")
    _require_list(actions, "actions")
    action_by_id: Dict[str, Dict[str, Any]] = {}
    for idx, action in enumerate(actions):
        _require(isinstance(action, dict), f"actions[{idx}] must be an object")
        aid = action.get("id")
        _require_str(aid, f"actions[{idx}].id")
        _require(aid not in action_by_id, f"Duplicate action id: {aid}")
        action_by_id[aid] = action

    for idx, action in enumerate(actions):
        aid = action["id"]

        deps = action.get("depends_on", [])
        _require_list(deps, f"actions[{idx}].depends_on")
        for dep in deps:
            _require_str(dep, f"actions[{idx}].depends_on[]")
            _require(dep in action_by_id, f"actions[{idx}] depends_on unknown action id: {dep}")
            _require(dep != aid, f"actions[{idx}] cannot depend on itself: {aid}")

        robot = action.get("robot")
        _require(isinstance(robot, dict), f"actions[{idx}].robot must be an object")
        rid = robot.get("id")
        _require_int(rid, f"actions[{idx}].robot.id")
        _require(rid in robot_by_id, f"actions[{idx}].robot.id={rid} not in robots[]")
        _require_str(robot.get("name"), f"actions[{idx}].robot.name")

        goal = action.get("goal")
        traj = action.get("trajectory")

        has_goal = isinstance(goal, dict) and isinstance(goal.get("joint_positions"), list) and len(goal.get("joint_positions")) > 0
        has_traj = isinstance(traj, dict) and isinstance(traj.get("points"), list) and len(traj.get("points")) > 0
        _require(has_goal or has_traj, f"actions[{idx}] must have at least one of goal.joint_positions or trajectory.points")

        if has_goal:
            joints = goal.get("joint_positions", [])
            _require_list(joints, f"actions[{idx}].goal.joint_positions")
            _require(
                len(joints) == int(robot_by_id[rid]["dof"]),
                f"actions[{idx}].goal.joint_positions length mismatch: expected {robot_by_id[rid]['dof']}, got {len(joints)}",
            )
            for j, v in enumerate(joints):
                _require_number(v, f"actions[{idx}].goal.joint_positions[{j}]")

        if has_traj:
            _require_int(traj.get("robot_id"), f"actions[{idx}].trajectory.robot_id")
            _require(
                traj.get("robot_id") == rid,
                f"actions[{idx}].trajectory.robot_id mismatch: action robot.id={rid}, traj.robot_id={traj.get('robot_id')}",
            )
            points = traj.get("points", [])
            _require_list(points, f"actions[{idx}].trajectory.points")
            last_t: Optional[float] = None
            for pidx, p in enumerate(points):
                _require(isinstance(p, dict), f"actions[{idx}].trajectory.points[{pidx}] must be an object")
                if "t" in p:
                    _require_number(p["t"], f"actions[{idx}].trajectory.points[{pidx}].t")
                    t = float(p["t"])
                    if last_t is not None and t < last_t - 1e-12:
                        raise ValidationError(
                            f"actions[{idx}].trajectory.points times must be non-decreasing: t[{pidx-1}]={last_t}, t[{pidx}]={t}"
                        )
                    last_t = t
                joints = p.get("joint_positions")
                _require_list(joints, f"actions[{idx}].trajectory.points[{pidx}].joint_positions")
                _require(
                    len(joints) == int(robot_by_id[rid]["dof"]),
                    f"actions[{idx}].trajectory.points[{pidx}].joint_positions length mismatch: expected {robot_by_id[rid]['dof']}, got {len(joints)}",
                )
                for j, v in enumerate(joints):
                    _require_number(v, f"actions[{idx}].trajectory.points[{pidx}].joint_positions[{j}]")

        scene_updates = action.get("scene_updates", {})
        if not isinstance(scene_updates, dict):
            warnings.append(f"actions[{idx}].scene_updates is not an object (ignored)")
            continue

        attachments = scene_updates.get("attachments", [])
        if isinstance(attachments, list):
            for eidx, evt in enumerate(attachments):
                if not isinstance(evt, dict):
                    warnings.append(f"actions[{idx}].scene_updates.attachments[{eidx}] not an object (ignored)")
                    continue
                if "action" in evt:
                    if evt["action"] not in ("attach", "detach"):
                        warnings.append(
                            f"actions[{idx}].scene_updates.attachments[{eidx}].action unknown: {evt['action']!r}"
                        )
        else:
            warnings.append(f"actions[{idx}].scene_updates.attachments is not a list (ignored)")

    # Dependency DAG check (cycle detection via Kahn)
    _ = topo_sort_actions(actions)

    return ValidationReport(warnings=tuple(warnings))


def topo_sort_actions(actions: Sequence[Mapping[str, Any]]) -> List[Mapping[str, Any]]:
    action_by_id: Dict[str, Mapping[str, Any]] = {}
    incoming: Dict[str, Set[str]] = {}
    outgoing: Dict[str, Set[str]] = {}

    for action in actions:
        aid = action.get("id")
        if isinstance(aid, str):
            action_by_id[aid] = action

    for aid in action_by_id:
        incoming[aid] = set()
        outgoing[aid] = set()

    for action in actions:
        aid = action.get("id")
        if not isinstance(aid, str) or aid not in action_by_id:
            continue
        deps = action.get("depends_on", [])
        if not isinstance(deps, list):
            continue
        for dep in deps:
            if not isinstance(dep, str) or dep not in action_by_id:
                continue
            incoming[aid].add(dep)
            outgoing[dep].add(aid)

    ready: List[str] = [aid for aid, deps in incoming.items() if not deps]
    ready.sort()
    ordered: List[Mapping[str, Any]] = []

    while ready:
        aid = ready.pop(0)
        ordered.append(action_by_id[aid])
        for succ in sorted(outgoing[aid]):
            incoming[succ].discard(aid)
            if not incoming[succ]:
                ready.append(succ)
        ready.sort()

    if len(ordered) != len(action_by_id):
        remaining = sorted(aid for aid, deps in incoming.items() if deps)
        raise ValidationError(f"Dependency graph has a cycle or missing nodes; remaining actions: {remaining}")

    # Preserve original action objects ordering beyond id sort? Return in topological order.
    return ordered

