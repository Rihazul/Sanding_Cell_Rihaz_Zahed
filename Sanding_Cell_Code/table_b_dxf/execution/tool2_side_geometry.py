from __future__ import annotations

import math
from typing import Sequence


TOOL2_APPROACH_OUTWARD_MM = 15.0
TOOL2_LIFT_Z_MM = -55.0
TOOL2_CONTACT_Z_MM = 0.0

# Bottom side: with one J7 station, Tool 2 can safely sand 700 mm in local X.
TOOL2_BOTTOM_SAFE_LOCAL_X = (0.0, 700.0)

# Left side: operator/robot-side calibration values provided for side-face entry.
TOOL2_LEFT_PREFORCE_LOCAL_X_MM = 155.0
TOOL2_LEFT_AXIS7_OFFSET_FROM_TOTAL_X_MM = -130.0

# Top side reach envelope. These are local robot-base X ranges at the given UCS Y.
# Below 340 mm, Tool 2 behaves like a rectangle x=[0,625]. At/above 340 mm,
# interpolate between the measured rows.
TOOL2_TOP_RECT_BELOW_Y_MM = 340.0
TOOL2_TOP_RECT_LOCAL_X = (0.0, 625.0)
TOOL2_TOP_REACH_ROWS: tuple[tuple[float, float, float], ...] = (
    (340.0, -1150.0, 625.0),
    (515.0, -1070.0, 585.0),
    (750.0, -895.0, 415.0),
    (935.0, -650.0, 195.0),
)


def tool2_top_local_x_span_at_y(y_mm: float) -> tuple[float, float] | None:
    """Return Tool 2 top-side local X reach at UCS Y.

    This is only for Tool 2 top-side planning. It must not be mixed with the
    Tool 1/3/4 top-surface reach envelope.
    """
    y = float(y_mm)
    if y < TOOL2_TOP_RECT_BELOW_Y_MM:
        return TOOL2_TOP_RECT_LOCAL_X

    rows = TOOL2_TOP_REACH_ROWS
    if y <= rows[0][0]:
        return (rows[0][1], rows[0][2])
    if y >= rows[-1][0]:
        if math.isclose(y, rows[-1][0], abs_tol=1e-6):
            return (rows[-1][1], rows[-1][2])
        return None

    for (y0, x0_min, x0_max), (y1, x1_min, x1_max) in zip(rows, rows[1:]):
        if y0 <= y <= y1:
            span = y1 - y0
            t = 0.0 if abs(span) < 1e-9 else (y - y0) / span
            return (
                x0_min + (x1_min - x0_min) * t,
                x0_max + (x1_max - x0_max) * t,
            )
    return None


def tool2_left_axis7_position(x_total_mm: float) -> float:
    """J7 station for the left side, based on the provided x_total - 130 rule."""
    return float(x_total_mm) + TOOL2_LEFT_AXIS7_OFFSET_FROM_TOTAL_X_MM


def tool2_outward_approach_point(
    side: str,
    contact_point: Sequence[float],
    outward_mm: float = TOOL2_APPROACH_OUTWARD_MM,
) -> list[float]:
    """Return the pre-force XY point 15 mm outward from the side contact point.

    Coordinates are still viewer/UCS XY. The robot-base conversion happens later
    using axis7_position_mm.
    """
    if len(contact_point) < 2:
        raise ValueError(f"Invalid Tool 2 contact point: {contact_point!r}")
    x = float(contact_point[0])
    y = float(contact_point[1])
    if side == "right":
        return [x - outward_mm, y]
    if side == "bottom":
        return [x, y - outward_mm]
    if side == "left":
        return [x + outward_mm, y]
    if side == "top":
        return [x, y + outward_mm]
    raise ValueError(f"Unknown Tool 2 side: {side!r}")


def split_tool2_bottom_segments(x_total_mm: float) -> list[tuple[float, float]]:
    """Return bottom-side segments as (axis7_position_mm, local_x_end)."""
    x_total = max(0.0, float(x_total_mm))
    max_span = TOOL2_BOTTOM_SAFE_LOCAL_X[1] - TOOL2_BOTTOM_SAFE_LOCAL_X[0]
    if x_total <= 0.0:
        return []

    segments: list[tuple[float, float]] = []
    start = 0.0
    while start < x_total - 1e-6:
        end = min(x_total, start + max_span)
        segments.append((start, end - start))
        start = end
    return segments


def split_tool2_top_segments(x_total_mm: float, y_total_mm: float) -> list[tuple[float, float, float]]:
    """Return top-side right-to-left segments as (axis7_position_mm, local_start_x, local_end_x).

    The generated local segment is reachable at the provided J7 station and moves
    right-to-left along the top side. It uses the measured Tool 2 top reach span at
    the door height.
    """
    x_total = max(0.0, float(x_total_mm))
    span = tool2_top_local_x_span_at_y(float(y_total_mm))
    if x_total <= 0.0 or span is None:
        return []

    local_min, local_max = span
    max_width = max(1.0, local_max - local_min)
    segments: list[tuple[float, float, float]] = []
    right = x_total
    while right > 1e-6:
        left = max(0.0, right - max_width)
        axis7 = left - local_min
        local_start = right - axis7
        local_end = left - axis7
        segments.append((axis7, local_start, local_end))
        right = left
    return segments