"""Build V5 MoveBlend programs containing BlendL segments only."""
from __future__ import annotations

from collections.abc import Iterable, Sequence


def blend_l(target: Sequence[float], speed: str,
            zone: tuple[float, float] = (0.0, 0.0)) -> str:
    values = tuple(float(value) for value in target)
    if len(values) != 7:
        raise ValueError(f"BlendL needs a 7-value robottarget, got {len(values)}")
    if not speed or any(ch.isspace() for ch in speed):
        raise ValueError(f"invalid BlendL speed: {speed!r}")
    pose = ",".join(f"{value:.9f}" for value in values)
    return (
        "{MoveBlend --type=insert_line --robottarget_value={"
        + pose
        + f"}} --zone={{{float(zone[0]):.1f},{float(zone[1]):.1f}}} --speed={speed}}}"
    )


def move_blend(targets: Iterable[Sequence[float]], speed: str,
               zone: tuple[float, float] = (0.0, 0.0)) -> list[str]:
    points = list(targets)
    if not points:
        raise ValueError("MoveBlend needs at least one BlendL target")
    return [
        "{MoveBlend --type=first_insert}",
        *(blend_l(target, speed, zone) for target in points),
        "{MoveBlend --type=start}",
    ]

