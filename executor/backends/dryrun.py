from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, Iterable, List, Mapping, Sequence

from ..validate import topo_sort_actions, validate_skillplan


@dataclass
class DryRunBackend:
    verbose: bool = True

    def run(self, plan: Mapping[str, Any], *, max_actions: int = 0) -> None:
        report = validate_skillplan(plan)
        if report.warnings and self.verbose:
            for w in report.warnings:
                print(f"[warn] {w}")

        actions = topo_sort_actions(plan.get("actions", []))
        if max_actions > 0:
            actions = actions[:max_actions]

        robots = {r["id"]: r for r in plan.get("robots", []) if isinstance(r, dict) and "id" in r}
        print(f"schema={plan.get('schema')} version={plan.get('schema_version')} robots={len(robots)} actions={len(actions)}")

        per_robot = {rid: 0 for rid in robots.keys()}
        total_points = 0
        for a in actions:
            rid = a.get("robot", {}).get("id")
            if rid in per_robot:
                per_robot[rid] += 1
            traj = a.get("trajectory") or {}
            pts = traj.get("points") if isinstance(traj, dict) else None
            if isinstance(pts, list):
                total_points += len(pts)

        for rid, count in sorted(per_robot.items()):
            name = robots[rid].get("name", str(rid))
            print(f"robot[{rid}]={name}: actions={count}")
        print(f"total trajectory points={total_points}")

