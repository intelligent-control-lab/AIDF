from __future__ import annotations

import re
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple

import mujoco
import numpy as np

from ..errors import SkillPlanError, ValidationError
from ..validate import topo_sort_actions, validate_skillplan


def _load_model_xml(path: str) -> Tuple[str, str]:
    p = Path(path)
    if not p.exists():
        raise SkillPlanError(f"Model file not found: {path}")

    if p.suffix == ".xacro":
        # Best-effort: call xacro to expand to URDF.
        # Requires a ROS environment; if unavailable, ask user to pass a .urdf file instead.
        xacro_exe = "xacro"
        cmd: List[str]
        if Path("/usr/bin/rosrun").exists():
            cmd = ["rosrun", "xacro", "xacro", str(p)]
        else:
            cmd = [xacro_exe, str(p)]
        try:
            res = subprocess.run(cmd, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            return res.stdout, "urdf"
        except Exception as exc:
            # Common case: xacro not available (unsourced ROS). If a sibling URDF exists, fall back.
            if p.name.endswith(".urdf.xacro"):
                urdf_fallback = p.with_suffix("")  # strip ".xacro" -> ".urdf"
                if urdf_fallback.exists():
                    return str(urdf_fallback), "path"
            raise SkillPlanError(
                f"Failed to expand xacro via {' '.join(cmd)}; run from a sourced ROS env or pass a .urdf file. Error: {exc}"
            ) from exc

    # MuJoCo 3.x can load URDF directly via from_xml_path when the root tag is <robot>.
    return str(p), "path"


def _list_joint_names(model: mujoco.MjModel) -> List[str]:
    out: List[str] = []
    for j in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j)
        if name:
            out.append(name)
    return out


def _robot_joint_names(model: mujoco.MjModel, robot_name: str, dof: int) -> List[str]:
    all_joints = _list_joint_names(model)
    prefix = f"{robot_name}_joint_"
    candidates = [j for j in all_joints if j.startswith(prefix)]
    if not candidates:
        raise ValidationError(f"No MuJoCo joints found with prefix {prefix!r}; available joints: {all_joints}")

    def joint_index(n: str) -> int:
        m = re.match(re.escape(prefix) + r"(\\d+)$", n)
        return int(m.group(1)) if m else 999999

    candidates.sort(key=joint_index)
    if len(candidates) != dof:
        raise ValidationError(f"Expected {dof} joints for robot {robot_name!r}, found {len(candidates)}: {candidates}")
    return candidates


def _try_frame_viewer_camera(viewer: Any, model: mujoco.MjModel) -> None:
    # For URDF-imported models, MuJoCo's default camera looks at (0,0,0). Our dual-arm model
    # is centered around z≈1.2, so the initial view can appear completely black until the user
    # hits "Align"/"Reset". Frame the model automatically when possible.
    try:
        lookat = getattr(viewer, "cam").lookat
        lookat[:] = model.stat.center
        viewer.cam.distance = float(max(0.1, 2.5 * model.stat.extent))
        viewer.cam.azimuth = 90.0
        viewer.cam.elevation = -45.0
    except Exception:
        # Best-effort only; viewer API can vary across MuJoCo versions.
        return


@dataclass
class MujocoSimBackend:
    model_path: str
    sim_dt: float = 0.01
    realtime: bool = False
    viewer: bool = False
    verbose: bool = True

    def run(self, plan: Mapping[str, Any], *, max_actions: int = 0) -> None:
        report = validate_skillplan(plan)
        if report.warnings and self.verbose:
            for w in report.warnings:
                print(f"[warn] {w}")

        model_xml, mode = _load_model_xml(self.model_path)
        if mode == "path":
            model = mujoco.MjModel.from_xml_path(model_xml)
        else:
            model = mujoco.MjModel.from_xml_string(model_xml)

        data = mujoco.MjData(model)

        robots = {r["id"]: r for r in plan.get("robots", []) if isinstance(r, dict) and "id" in r}
        if not robots:
            raise ValidationError("No robots found in plan")

        # Build joint mapping for each robot.
        robot_qpos_adrs: Dict[int, List[int]] = {}
        for rid, robot in sorted(robots.items()):
            name = robot.get("name")
            dof = int(robot.get("dof", 0))
            if not isinstance(name, str) or not name:
                raise ValidationError(f"robot[{rid}].name missing")
            joint_names = _robot_joint_names(model, name, dof)
            qpos_adrs: List[int] = []
            for jn in joint_names:
                jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jn)
                if jid < 0:
                    raise ValidationError(f"Joint {jn!r} not found in MuJoCo model")
                qpos_adrs.append(int(model.jnt_qposadr[jid]))
            robot_qpos_adrs[rid] = qpos_adrs
            if self.verbose:
                print(f"[mujoco] robot[{rid}] {name}: joints={joint_names}")

        # Set initial joint positions.
        init_scene = plan.get("initial_scene", {})
        init_robots = init_scene.get("robots", [])
        for rs in init_robots:
            if not isinstance(rs, dict):
                continue
            rid = rs.get("robot_id")
            if not isinstance(rid, int) or rid not in robot_qpos_adrs:
                continue
            q = rs.get("joint_positions", [])
            if not isinstance(q, list):
                continue
            qpos = np.array([float(v) for v in q], dtype=float)
            for adr, val in zip(robot_qpos_adrs[rid], qpos):
                data.qpos[adr] = val
        data.time = 0.0
        mujoco.mj_forward(model, data)

        actions = topo_sort_actions(plan.get("actions", []))
        if max_actions > 0:
            actions = actions[:max_actions]

        if self.viewer:
            self._run_with_viewer(model, data, actions, robot_qpos_adrs)
        else:
            self._run_headless(model, data, actions, robot_qpos_adrs)

    def _run_headless(
        self, model: mujoco.MjModel, data: mujoco.MjData, actions: Sequence[Mapping[str, Any]], robot_qpos_adrs: Mapping[int, Sequence[int]]
    ) -> None:
        global_time = float(data.time)
        for idx, action in enumerate(actions):
            aid = action.get("id", f"#{idx}")
            rid = int(action.get("robot", {}).get("id"))
            traj = action.get("trajectory") or {}
            points = traj.get("points") if isinstance(traj, dict) else None
            if not isinstance(points, list) or not points:
                continue
            if self.verbose:
                sname = (action.get("skill") or {}).get("name")
                rname = (action.get("robot") or {}).get("name")
                print(f"[mujoco] action {idx+1}/{len(actions)} {aid} skill={sname} robot={rname} points={len(points)}")

            qpos_adrs = robot_qpos_adrs[rid]
            prev_q: Optional[np.ndarray] = None
            prev_t: Optional[float] = None
            for pidx, p in enumerate(points):
                q = np.array([float(v) for v in p.get("joint_positions", [])], dtype=float)
                t = float(p.get("t", prev_t + self.sim_dt if prev_t is not None else 0.0))
                if prev_q is None:
                    for adr, val in zip(qpos_adrs, q):
                        data.qpos[adr] = val
                    data.time = global_time
                    mujoco.mj_forward(model, data)
                    prev_q = q
                    prev_t = t
                    continue

                dt = max(0.0, t - float(prev_t))
                steps = max(1, int(round(dt / self.sim_dt))) if self.sim_dt > 0 else 1
                for s in range(1, steps + 1):
                    alpha = float(s) / float(steps)
                    qi = (1.0 - alpha) * prev_q + alpha * q
                    for adr, val in zip(qpos_adrs, qi):
                        data.qpos[adr] = float(val)
                    global_time += dt / steps if steps > 0 else 0.0
                    data.time = global_time
                    mujoco.mj_forward(model, data)
                    if self.realtime and dt > 0.0:
                        time.sleep(dt / steps)

                prev_q = q
                prev_t = t

    def _run_with_viewer(
        self, model: mujoco.MjModel, data: mujoco.MjData, actions: Sequence[Mapping[str, Any]], robot_qpos_adrs: Mapping[int, Sequence[int]]
    ) -> None:
        import mujoco.viewer

        global_time = float(data.time)
        with mujoco.viewer.launch_passive(model, data) as viewer:
            _try_frame_viewer_camera(viewer, model)
            viewer.sync()
            for idx, action in enumerate(actions):
                aid = action.get("id", f"#{idx}")
                rid = int(action.get("robot", {}).get("id"))
                traj = action.get("trajectory") or {}
                points = traj.get("points") if isinstance(traj, dict) else None
                if not isinstance(points, list) or not points:
                    continue
                if self.verbose:
                    sname = (action.get("skill") or {}).get("name")
                    rname = (action.get("robot") or {}).get("name")
                    print(f"[mujoco] action {idx+1}/{len(actions)} {aid} skill={sname} robot={rname} points={len(points)}")

                qpos_adrs = robot_qpos_adrs[rid]
                prev_q: Optional[np.ndarray] = None
                prev_t: Optional[float] = None
                for pidx, p in enumerate(points):
                    if not viewer.is_running():
                        return
                    q = np.array([float(v) for v in p.get("joint_positions", [])], dtype=float)
                    t = float(p.get("t", prev_t + self.sim_dt if prev_t is not None else 0.0))
                    if prev_q is None:
                        for adr, val in zip(qpos_adrs, q):
                            data.qpos[adr] = float(val)
                        data.time = global_time
                        mujoco.mj_forward(model, data)
                        viewer.sync()
                        prev_q = q
                        prev_t = t
                        continue

                    dt = max(0.0, t - float(prev_t))
                    steps = max(1, int(round(dt / self.sim_dt))) if self.sim_dt > 0 else 1
                    for s in range(1, steps + 1):
                        if not viewer.is_running():
                            return
                        alpha = float(s) / float(steps)
                        qi = (1.0 - alpha) * prev_q + alpha * q
                        for adr, val in zip(qpos_adrs, qi):
                            data.qpos[adr] = float(val)
                        global_time += dt / steps if steps > 0 else 0.0
                        data.time = global_time
                        mujoco.mj_forward(model, data)
                        viewer.sync()
                        if self.realtime and dt > 0.0:
                            time.sleep(dt / steps)

                    prev_q = q
                    prev_t = t
