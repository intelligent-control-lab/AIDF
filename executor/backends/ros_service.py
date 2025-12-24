from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple

from ..errors import SkillPlanError, ValidationError
from ..validate import topo_sort_actions, validate_skillplan


def _import_ros():
    try:
        import rospy  # type: ignore
        from moveit_msgs.msg import MoveItErrorCodes, RobotTrajectory  # type: ignore
        from moveit_msgs.srv import ExecuteKnownTrajectory, ExecuteKnownTrajectoryRequest  # type: ignore
        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint  # type: ignore
    except Exception as exc:
        raise SkillPlanError(
            "ROS Python modules not available. Source your ROS env (e.g., `source /opt/ros/noetic/setup.bash` and your workspace) "
            "and ensure `moveit_msgs` is installed."
        ) from exc

    return (
        rospy,
        MoveItErrorCodes,
        RobotTrajectory,
        ExecuteKnownTrajectory,
        ExecuteKnownTrajectoryRequest,
        JointTrajectory,
        JointTrajectoryPoint,
    )


def _norm_service_name(s: str) -> str:
    v = (s or "").strip()
    if not v:
        raise ValidationError("Empty ROS service name/namespace")
    if not v.startswith("/"):
        v = "/" + v
    v = v.rstrip("/")
    if v.endswith("yk_execute_trajectory"):
        return v
    return v + "/yk_execute_trajectory"


def _parse_kv_pairs(pairs: Sequence[str]) -> Tuple[Dict[int, str], Dict[str, str]]:
    by_id: Dict[int, str] = {}
    by_name: Dict[str, str] = {}
    for raw in pairs:
        if "=" not in raw:
            raise ValidationError(f"--ros-service must be in the form <robot>=<service>, got: {raw!r}")
        k, v = raw.split("=", 1)
        key = k.strip()
        svc = _norm_service_name(v)
        if key.isdigit():
            by_id[int(key)] = svc
        else:
            by_name[key] = svc
    return by_id, by_name


def _robot_joint_names(robot: Mapping[str, Any]) -> List[str]:
    name = robot.get("name")
    dof = int(robot.get("dof", 0))
    if not isinstance(name, str) or not name:
        raise ValidationError("robot.name missing")
    name = name.lstrip("/")
    if dof <= 0:
        raise ValidationError(f"robot[{name}].dof must be > 0")

    jn = robot.get("joint_names")
    if isinstance(jn, list) and all(isinstance(x, str) and x for x in jn):
        if len(jn) != dof:
            raise ValidationError(f"robot[{name}].joint_names length mismatch: expected {dof}, got {len(jn)}")
        return list(jn)

    return [f"{name}_joint_{i}" for i in range(1, dof + 1)]


@dataclass
class RosServiceBackend:
    services: Sequence[str] = ()
    wait_for_execution: bool = True
    wait_for_service: float = 10.0
    verbose: bool = True

    def run(self, plan: Mapping[str, Any], *, max_actions: int = 0) -> None:
        report = validate_skillplan(plan)
        if report.warnings and self.verbose:
            for w in report.warnings:
                print(f"[warn] {w}")

        rospy, MoveItErrorCodes, RobotTrajectory, ExecuteKnownTrajectory, ExecuteKnownTrajectoryRequest, JointTrajectory, JointTrajectoryPoint = _import_ros()

        robots = plan.get("robots", [])
        if not isinstance(robots, list) or not robots:
            raise ValidationError("No robots found in plan")
        robots_by_id: Dict[int, Dict[str, Any]] = {}
        robots_by_name: Dict[str, Dict[str, Any]] = {}
        for r in robots:
            if not isinstance(r, dict):
                continue
            rid = r.get("id")
            name = r.get("name")
            if isinstance(rid, int):
                robots_by_id[rid] = r
            if isinstance(name, str) and name:
                robots_by_name[name] = r

        svc_by_id_override, svc_by_name_override = _parse_kv_pairs(self.services)

        svc_by_id: Dict[int, str] = {}
        joint_names_by_id: Dict[int, List[str]] = {}
        for rid, robot in sorted(robots_by_id.items()):
            name = str(robot.get("name", rid)).lstrip("/")
            default_service = _norm_service_name(f"/{name}/yk_execute_trajectory")
            svc = svc_by_id_override.get(rid) or svc_by_name_override.get(name) or default_service
            svc_by_id[rid] = svc
            joint_names_by_id[rid] = _robot_joint_names(robot)

        rospy.init_node("skillplan_executor", anonymous=True, disable_signals=True)

        clients: Dict[int, Any] = {}
        for rid, svc in sorted(svc_by_id.items()):
            if self.verbose:
                print(f"[ros] robot[{rid}] service={svc}")
            try:
                rospy.wait_for_service(svc, timeout=float(self.wait_for_service))
            except Exception as exc:
                raise SkillPlanError(f"Timed out waiting for service {svc!r}. Is the robot driver up?") from exc
            clients[rid] = rospy.ServiceProxy(svc, ExecuteKnownTrajectory, persistent=True)

        actions = topo_sort_actions(plan.get("actions", []))
        if max_actions > 0:
            actions = actions[:max_actions]

        for idx, action in enumerate(actions):
            aid = action.get("id", f"#{idx}")
            robot = action.get("robot") or {}
            rid = robot.get("id")
            if not isinstance(rid, int) or rid not in clients:
                raise ValidationError(f"actions[{idx}].robot.id missing/unknown: {rid!r}")

            traj = action.get("trajectory") if isinstance(action.get("trajectory"), dict) else None
            points = traj.get("points") if traj and isinstance(traj.get("points"), list) else None

            jt = JointTrajectory()
            jt.joint_names = list(joint_names_by_id[rid])

            if points:
                last_t: Optional[float] = None
                for pidx, p in enumerate(points):
                    if not isinstance(p, dict):
                        continue
                    q = p.get("joint_positions", [])
                    if not isinstance(q, list):
                        raise ValidationError(f"actions[{idx}].trajectory.points[{pidx}].joint_positions missing")
                    t = p.get("t")
                    if t is None:
                        t = 0.0 if last_t is None else float(last_t) + 0.01
                    t = float(t)
                    if last_t is not None and t < last_t - 1e-12:
                        raise ValidationError(
                            f"actions[{idx}].trajectory.points times must be non-decreasing: t[{pidx-1}]={last_t}, t[{pidx}]={t}"
                        )
                    last_t = t
                    pt = JointTrajectoryPoint()
                    pt.positions = [float(v) for v in q]
                    pt.time_from_start = rospy.Duration.from_sec(max(0.0, t))
                    jt.points.append(pt)
            else:
                goal = action.get("goal") if isinstance(action.get("goal"), dict) else None
                q = goal.get("joint_positions") if goal else None
                if not isinstance(q, list) or not q:
                    raise ValidationError(f"actions[{idx}] has no trajectory.points and no goal.joint_positions")
                pt = JointTrajectoryPoint()
                pt.positions = [float(v) for v in q]
                pt.time_from_start = rospy.Duration.from_sec(0.0)
                jt.points.append(pt)

            robot_traj = RobotTrajectory()
            robot_traj.joint_trajectory = jt

            req = ExecuteKnownTrajectoryRequest()
            req.trajectory = robot_traj
            req.wait_for_execution = bool(self.wait_for_execution)

            if self.verbose:
                rname = robot.get("name", rid)
                sname = (action.get("skill") or {}).get("name")
                print(f"[ros] action {idx+1}/{len(actions)} {aid} skill={sname} robot={rname} points={len(jt.points)}")

            try:
                res = clients[rid](req)
            except Exception as exc:
                raise SkillPlanError(f"ROS service call failed for robot[{rid}] {svc_by_id[rid]!r} (action {aid})") from exc
            code = getattr(res, "error_code", None)
            val = getattr(code, "val", None)
            if val != int(MoveItErrorCodes.SUCCESS):
                raise SkillPlanError(f"Execution failed for action {aid}: MoveItErrorCodes={val}")
