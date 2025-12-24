from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict

from .errors import SkillPlanError


def load_json(path: str) -> Dict[str, Any]:
    p = Path(path)
    if not p.is_file():
        raise SkillPlanError(f"SkillPlan not found: {path}")
    try:
        return json.loads(p.read_text())
    except Exception as exc:
        raise SkillPlanError(f"Failed to parse JSON: {path}: {exc}") from exc

