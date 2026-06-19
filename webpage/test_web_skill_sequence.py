#!/usr/bin/env python3
"""Send an ordered MetaSkill sequence through the AIDF webpage API.

Run this while the Docker web services are up. Use --start-simulator if the
webplan_lego process has not already been started from the page.
"""

from __future__ import annotations

import argparse
import json
import re
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SKILLPLAN = REPO_ROOT / "config" / "lego_tasks" / "skillplan.json"
LEGO_TASKS_DIR = REPO_ROOT / "config" / "lego_tasks"
ASSEMBLY_TASKS_DIR = LEGO_TASKS_DIR / "assembly_tasks"
ENV_SETUP_DIR = LEGO_TASKS_DIR / "env_setup"


def load_json(path: Path) -> Any:
    with path.open("r", encoding="utf-8") as file:
        return json.load(file)


def numeric_items(data: dict[str, Any]) -> list[tuple[str, Any]]:
    return sorted(data.items(), key=lambda item: int(item[0]))


def available_task_names() -> list[str]:
    assembly_names = {path.stem for path in ASSEMBLY_TASKS_DIR.glob("*.json")}
    env_names = {
        path.name[len("env_setup_"):-len(".json")]
        for path in ENV_SETUP_DIR.glob("env_setup_*.json")
    }
    return sorted(assembly_names & env_names)


def post_json(base_url: str, endpoint: str, payload: dict[str, Any], timeout: float) -> dict[str, Any]:
    body = json.dumps(payload).encode("utf-8")
    request = urllib.request.Request(
        base_url.rstrip("/") + endpoint,
        data=body,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    try:
        with urllib.request.urlopen(request, timeout=timeout) as response:
            text = response.read().decode("utf-8")
            return json.loads(text) if text else {}
    except urllib.error.HTTPError as exc:
        text = exc.read().decode("utf-8", errors="replace")
        try:
            return json.loads(text)
        except json.JSONDecodeError:
            return {"status": -1, "output": text or str(exc)}


def normalize_sequence_entry(entry: dict[str, Any], command_id: int) -> dict[str, Any]:
    target = entry.get("target") or entry.get("target_location") or {
        "x": entry.get("target-x", entry.get("x", 0)),
        "y": entry.get("target-y", entry.get("y", 0)),
        "z": entry.get("target-z", entry.get("z", 0)),
        "ori": entry.get("ori", 0),
    }
    return {
        "command_id": command_id,
        "skill": entry["skill"],
        "object": entry.get("object", "none"),
        "robot_id": int(entry.get("robot-id", entry.get("robot", entry.get("robot_id", 0)))),
        "target": {
            "x": int(target.get("x", 0)),
            "y": int(target.get("y", 0)),
            "z": int(target.get("z", 0)),
            "ori": int(target.get("ori", 0)),
        },
        "skill_parameters": entry.get("skill_parameters", {}),
    }


def load_sequence_file(path: Path) -> list[dict[str, Any]]:
    data = load_json(path)
    if isinstance(data, list):
        entries = data
    elif isinstance(data, dict):
        entries = [entry for _, entry in numeric_items(data)]
    else:
        raise ValueError(f"Unsupported sequence JSON shape in {path}")

    commands = []
    for idx, entry in enumerate(entries, start=1):
        commands.append(normalize_sequence_entry(entry, int(time.time() * 1000) + idx))
    return commands


def load_skillplan_file(path: Path) -> tuple[list[dict[str, Any]], str]:
    data = load_json(path)
    actions = data.get("actions", [])
    if not actions:
        raise ValueError(f"No actions found in skillplan {path}")

    first_action_by_meta: dict[int, dict[str, Any]] = {}
    for action in actions:
        meta = action.get("meta_skill", {})
        if "index" not in meta:
            continue
        index = int(meta["index"])
        first_action_by_meta.setdefault(index, action)

    commands = []
    for idx, action in sorted(first_action_by_meta.items()):
        constraints = action.get("constraints", {})
        command_id = int(time.time() * 1000) + idx + 1
        commands.append({
            "command_id": command_id,
            "skill": action["meta_skill"]["name"],
            "object": action.get("object", {}).get("name", "none"),
            "robot_id": int(action.get("robot", {}).get("id", 0)),
            "target": {
                "x": int(constraints.get("x", 0)),
                "y": int(constraints.get("y", 0)),
                "z": int(constraints.get("z", 0)),
                "ori": int(constraints.get("ori", 0)),
            },
            "skill_parameters": {},
        })

    task_name = data.get("tasks", {}).get("name", path.stem)
    return commands, task_name


def brick_id_from_object_name(name: str) -> int | None:
    match = re.match(r"^b(\d+)_", name)
    return int(match.group(1)) if match else None


def object_instance_sort_key(name: str) -> tuple[int, int | str]:
    match = re.search(r"(\d+)$", name)
    if match:
        return (0, int(match.group(1)))
    return (1, name)


def command_skill_for_task_step(step: dict[str, Any]) -> str:
    if int(step.get("manipulate_type", 0)) == 1:
        return "PickHandoverAndPlace"
    if int(step.get("support_x", -1)) != -1:
        return "PickAndPlaceWithSupport"
    return "PickAndPlace"


def derive_task_sequence(task: str) -> list[dict[str, Any]]:
    task_path = ASSEMBLY_TASKS_DIR / f"{task}.json"
    env_path = ENV_SETUP_DIR / f"env_setup_{task}.json"
    if not task_path.exists() or not env_path.exists():
        supported = ", ".join(available_task_names())
        raise ValueError(
            f"Task '{task}' requires both {task_path} and {env_path}. "
            f"Supported derived tasks: {supported}"
        )

    task_data = load_json(task_path)
    env_data = load_json(env_path)

    objects_by_brick: dict[int, list[str]] = {}
    for object_name in env_data:
        brick_id = brick_id_from_object_name(object_name)
        if brick_id is not None:
            objects_by_brick.setdefault(brick_id, []).append(object_name)
    for names in objects_by_brick.values():
        names.sort(key=object_instance_sort_key)

    used_by_brick: dict[int, int] = {}
    commands = []
    for idx, (_, step) in enumerate(numeric_items(task_data), start=1):
        brick_id = int(step["brick_id"])
        available = objects_by_brick.get(brick_id, [])
        if not available:
            raise ValueError(f"No object in {env_path} matches brick_id={brick_id}")

        brick_seq = step.get("brick_seq")
        if brick_seq is not None and int(brick_seq) > 0:
            object_name = available[(int(brick_seq) - 1) % len(available)]
        else:
            used_idx = used_by_brick.get(brick_id, 0)
            object_name = available[used_idx % len(available)]
            used_by_brick[brick_id] = used_idx + 1

        commands.append({
            "command_id": int(time.time() * 1000) + idx,
            "skill": command_skill_for_task_step(step),
            "object": object_name,
            "robot_id": (idx - 1) % 2,
            "target": {
                "x": int(step["x"]),
                "y": int(step["y"]),
                "z": int(step["z"]),
                "ori": int(step.get("ori", 0)),
            },
            "skill_parameters": {},
        })
    return commands


def append_place_with_support_probe(commands: list[dict[str, Any]]) -> None:
    if any(command["skill"] == "PlaceWithSupport" for command in commands):
        return
    if not commands:
        raise ValueError("Cannot append PlaceWithSupport probe to an empty sequence")

    base = next((cmd for cmd in reversed(commands) if cmd["skill"] != "TranslateWithRotation"), commands[-1])
    commands.append({
        "command_id": int(time.time() * 1000) + len(commands) + 1,
        "skill": "PlaceWithSupport",
        "object": base["object"],
        "robot_id": base["robot_id"],
        "target": dict(base["target"]),
        "skill_parameters": {},
    })


def append_translate_with_rotation_probe(commands: list[dict[str, Any]]) -> None:
    if any(command["skill"] == "TranslateWithRotation" for command in commands):
        return
    commands.append({
        "command_id": int(time.time() * 1000) + len(commands) + 1,
        "skill": "TranslateWithRotation",
        "object": "none",
        "robot_id": 0,
        "target": {"x": 0, "y": 0, "z": 0, "ori": 0},
        "skill_parameters": {
            "Translate": {"speed": 1.0, "offset": [0.0, 0.0, 0.02]},
            "Rotate": {"speed": 1.0, "angle": 0.0},
        },
    })


def infer_simulator_task(sequence: Path) -> str:
    name = sequence.stem
    if "cliff" in name:
        return "cliff"
    return name


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group()
    source.add_argument("--skillplan", type=Path,
                        help=f"Skillplan JSON to replay. Default: {DEFAULT_SKILLPLAN}")
    source.add_argument("--sequence", type=Path,
                        help="Ordered skill sequence JSON.")
    source.add_argument("--task", help="Task name to derive and test, e.g. --task faucet.")
    parser.add_argument("--list-tasks", action="store_true",
                        help="List task names that can be used with --task, then exit.")
    parser.add_argument("--base-url", default="http://127.0.0.1:5000")
    parser.add_argument("--start-simulator", action="store_true",
                        help="POST /start_simulator before sending the sequence.")
    parser.add_argument("--sim-task",
                        help="Task name to pass to /start_simulator. Defaults to --task, or inferred from --sequence.")
    parser.add_argument("--simulator", default="Moveit")
    parser.add_argument("--robot", default="gp4")
    parser.add_argument("--include-place-with-support", action="store_true", default=False,
                        help="Append a PlaceWithSupport probe if the selected sequence lacks one.")
    parser.add_argument("--no-place-with-support", action="store_false",
                        dest="include_place_with_support")
    parser.add_argument("--include-translate", action="store_true",
                        help="Append TranslateWithRotation. Requires the YK services when built with HAVE_YK_TASKS.")
    parser.add_argument("--delay", type=float, default=0.5, help="Seconds to wait between commands.")
    parser.add_argument("--timeout", type=float, default=30.0, help="HTTP timeout per request.")
    parser.add_argument("--dry-run", action="store_true", help="Print commands without sending them.")
    parser.add_argument("--fail-fast", action="store_true", help="Stop on the first failed command.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    try:
        if args.list_tasks:
            print("\n".join(available_task_names()))
            return 0

        if args.task:
            commands = derive_task_sequence(args.task)
            sim_task = args.sim_task or args.task
        elif args.sequence:
            commands = load_sequence_file(args.sequence)
            sim_task = args.sim_task or infer_simulator_task(args.sequence)
        else:
            commands, skillplan_task = load_skillplan_file(args.skillplan or DEFAULT_SKILLPLAN)
            sim_task = args.sim_task or skillplan_task
    except ValueError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2

    if args.include_place_with_support:
        append_place_with_support_probe(commands)
    if args.include_translate:
        append_translate_with_rotation_probe(commands)

    print(f"Prepared {len(commands)} command(s):")
    for idx, command in enumerate(commands, start=1):
        target = command["target"]
        print(
            f"{idx:02d}. {command['skill']} object={command['object']} "
            f"robot={command['robot_id']} target=({target['x']}, {target['y']}, {target['z']}, {target['ori']})"
        )
        print()

    if args.dry_run:
        print(json.dumps(commands, indent=2))
        return 0

    if args.start_simulator:
        response = post_json(args.base_url, "/start_simulator", {
            "simulator": args.simulator,
            "robot": args.robot,
            "task": sim_task,
        }, args.timeout)
        print("start_simulator:", json.dumps(response, indent=2))
        if response.get("status") == -1:
            return 1
        time.sleep(2.0)

    failures = 0
    for idx, command in enumerate(commands, start=1):
        response = post_json(args.base_url, "/run_simulation", command, args.timeout)
        ok = response.get("status") == 0
        print(f"{idx:02d}. {'PASS' if ok else 'FAIL'} {command['skill']}: {response.get('output', response)}")
        print()
        if not ok:
            failures += 1
            if args.fail_fast:
                break
        time.sleep(args.delay)

    if failures:
        print(f"{failures} command(s) failed", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
