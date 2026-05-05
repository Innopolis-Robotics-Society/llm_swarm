"""Парсинг per-robot цветов из Stage .world файла.

Stage задаёт цвета строками вида:
    warehouse_robot ( name "robot_0" pose [...] color "cyan" )

Этот модуль читает их и возвращает {robot_name: rgba_string}, где
rgba_string — формат, принимаемый URDF-материалом ("R G B A").
"""

from __future__ import annotations

import logging
import re

logger = logging.getLogger(__name__)

# Имена цветов Stage (X11) → RGBA-строка для URDF.
COLOR_MAP: dict[str, str] = {
    "cyan":    "0 1 1 1",
    "magenta": "1 0 1 1",
    "green":   "0 0.8 0 1",
    "orange":  "1 0.5 0 1",
    "yellow":  "1 1 0 1",
    "blue":    "0 0 1 1",
    "red":     "1 0 0 1",
}

_FALLBACK_RGBA = "0.5 0.5 0.5 1"

_ROBOT_RE = re.compile(
    r'warehouse_robot\s*\(\s*name\s+"([^"]+)".*?color\s+"([^"]+)"',
    re.DOTALL,
)


def color_to_rgba(name: str) -> str:
    rgba = COLOR_MAP.get(name.lower())
    if rgba is None:
        logger.warning("Unknown Stage color '%s', falling back to grey", name)
        return _FALLBACK_RGBA
    return rgba


def parse_robot_colors(world_path: str) -> dict[str, str]:
    """Возвращает {robot_name: rgba_string} из .world файла."""
    with open(world_path, "r") as f:
        text = f.read()
    return {
        name: color_to_rgba(color)
        for name, color in _ROBOT_RE.findall(text)
    }
