"""Resolve ``{{ref.path}}`` template strings in an LLM plan against tool-call results.

Channel-3 tool calling already computes exact geometry (goals, offsets,
namespaces) via deterministic tools — ``find_group_placement_in_room``,
``find_free_group_goals_in_room``, ``get_positions``, etc. Until now the model
still had to *retype* those numbers into the final plan JSON, and that retyping
step is where a 14B local model drops a decimal, copies the wrong robot's
goal, or falls back to a hallucinated coordinate when it loses track of which
tool result belonged to which group. ``occupancy_rewrite.py`` exists as a
server-side safety net for exactly this failure mode.

This module lets the model cite a tool result by reference instead of
retyping it. Every referenceable tool result is tagged with a ``_ref`` id by
``ToolExecutor`` (see ``tool_executor.py``); the model may then write, in any
plan field that would otherwise hold a literal value:

    "{{t3.placements[0].mapf_goals}}"
    "{{t3.placements[0].follower_ns}}"
    "{{t2.mapf_leaf.goals}}"

instead of retyping the numbers/strings verbatim. ``resolve_plan_templates``
walks the parsed plan and replaces every such string with the value it points
to. This is pure and side-effect free: given the same plan + registry it
always resolves the same way, and it is intentionally strict — an unresolved
or malformed reference is always an error (never a silent no-op), because
a template with no matching tool call is always a model mistake.

Partial substitution inside a larger string (e.g. ``"prefix {{t1.x}} suffix"``)
is NOT supported — the field must be the template string exactly. That keeps
resolution unambiguous and avoids guessing a target type for interpolation.
"""

from __future__ import annotations

import re
from typing import Any

__all__ = [
    "PlanTemplateError",
    "is_template",
    "resolve_template",
    "resolve_plan_templates",
    "find_used_refs",
]


class PlanTemplateError(ValueError):
    """A ``{{ref.path}}`` template referenced an unknown ref id or path."""


# Whole-string match only: {{ref.path.to[0].value}}
_TEMPLATE_RE = re.compile(
    r"^\{\{\s*([A-Za-z_]\w*)((?:\.[A-Za-z_]\w*|\.\d+|\[\d+\])*)\s*\}\}$"
)
# Splits ".field", ".5" and "[5]" path segments out of the captured path tail.
_PATH_TOKEN_RE = re.compile(r"\.([A-Za-z_]\w*|\d+)|\[(\d+)\]")


def is_template(value: Any) -> bool:
    """True if ``value`` is a string that is *exactly* a ``{{ref.path}}`` reference."""
    return isinstance(value, str) and _TEMPLATE_RE.match(value) is not None


def _describe(ref: str, path: str) -> str:
    return f"{{{{{ref}{path}}}}}"


def _resolve_path(root: Any, path: str, *, ref: str) -> Any:
    current = root
    for match in _PATH_TOKEN_RE.finditer(path):
        token = match.group(1) if match.group(1) is not None else match.group(2)
        if isinstance(current, list):
            try:
                idx = int(token)
            except ValueError as exc:
                raise PlanTemplateError(
                    f"{_describe(ref, path)}: expected a list index, got {token!r}"
                ) from exc
            if not (0 <= idx < len(current)):
                raise PlanTemplateError(
                    f"{_describe(ref, path)}: index {idx} out of range "
                    f"(list has {len(current)} item(s))"
                )
            current = current[idx]
        elif isinstance(current, dict):
            if token in current:
                current = current[token]
            else:
                known = ", ".join(sorted(str(k) for k in current.keys())) or "(empty)"
                raise PlanTemplateError(
                    f"{_describe(ref, path)}: key {token!r} not found. "
                    f"Available keys here: {known}"
                )
        else:
            raise PlanTemplateError(
                f"{_describe(ref, path)}: cannot look up {token!r} inside "
                f"a {type(current).__name__} value"
            )
    return current


def resolve_template(value: str, registry: dict[str, dict]) -> Any:
    """Resolve a single ``{{ref.path}}`` string against the tool-call registry.

    Raises ``PlanTemplateError`` with an explanation good enough to feed
    straight back to the model as a repair prompt.
    """
    match = _TEMPLATE_RE.match(value)
    if not match:
        raise PlanTemplateError(f"not a valid {{ref.path}} template: {value!r}")
    ref, path = match.group(1), match.group(2)
    if ref not in registry:
        available = ", ".join(sorted(registry.keys())) or "(no tool calls made this turn)"
        raise PlanTemplateError(
            f"{value}: unknown tool-call reference {ref!r}. "
            f"Available references this turn: {available}"
        )
    return _resolve_path(registry[ref], path, ref=ref)


def resolve_plan_templates(node: Any, registry: dict[str, dict] | None) -> Any:
    """Recursively replace every ``{{ref.path}}`` string in ``node``.

    Returns a new structure; ``node`` is not mutated. Safe to call with an
    empty registry — any template then simply fails to resolve (raises),
    which is correct: a template with no available tool calls is always a
    model mistake, not a silent no-op.
    """
    registry = registry or {}
    if isinstance(node, str):
        return resolve_template(node, registry) if is_template(node) else node
    if isinstance(node, list):
        return [resolve_plan_templates(item, registry) for item in node]
    if isinstance(node, dict):
        return {key: resolve_plan_templates(val, registry) for key, val in node.items()}
    return node


def find_used_refs(node: Any) -> list[str]:
    """Scan ``node`` for ``{{ref.path}}`` strings and return the referenced ids.

    Read-only, no resolution/validation — purely for logging/diagnostics, so
    a caller can report "plan cited t1, t3 out of 3 tool results available"
    without needing a registry at all. Order of first appearance, deduped.
    """
    seen: list[str] = []

    def _walk(value: Any) -> None:
        if isinstance(value, str):
            match = _TEMPLATE_RE.match(value)
            if match and match.group(1) not in seen:
                seen.append(match.group(1))
        elif isinstance(value, list):
            for item in value:
                _walk(item)
        elif isinstance(value, dict):
            for val in value.values():
                _walk(val)

    _walk(node)
    return seen
