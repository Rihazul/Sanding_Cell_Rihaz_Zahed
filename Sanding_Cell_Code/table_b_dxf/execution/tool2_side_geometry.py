from __future__ import annotations

import math
from typing import NamedTuple, Sequence


class Tool2HorizontalSegment(NamedTuple):
    """One bottom/top horizontal side segment reachable at a single 7th-axis station."""
    side: str
    axis7: float
    local_start_x: float
    local_end_x: float
    y: float


TOOL2_APPROACH_OUTWARD_MM = 15.0
TOOL2_LIFT_Z_MM = -68.0
# Bottom uses the same safe height as the other sides; it is special only in
# staying 5 mm off Y=0 at RY=-22 rather than backing off 15 mm at RY=0.
TOOL2_BOTTOM_LIFT_Z_MM = -55.0
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


def tool2_lift_z_for_side(side: str) -> float:
    if side == "bottom":
        return TOOL2_BOTTOM_LIFT_Z_MM
    return TOOL2_LIFT_Z_MM

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


def _clamp(value: float, low: float, high: float) -> float:
    return min(max(float(value), float(low)), float(high))


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


def plan_tool2_side_toolpaths(
    x_lo: float, y_lo: float, x_hi: float, y_hi: float,
) -> list[dict[str, object]]:
    """Reach-split contact toolpaths for the four Tool 2 sides, tagged with their J7 station.

    Returns the SAME reach split and 7th-axis positions the runner uses, as viewer-space
    contact polylines so the 2D viewer can colour each piece by station like tools 1/3/4.
    Each entry is {path_id, side_label, points:[[x,y],...], axis7_position_mm}. Pure/robot-free.
    """
    x_lo = float(x_lo); y_lo = float(y_lo); x_hi = float(x_hi); y_hi = float(y_hi)
    x_total = max(0.0, x_hi - x_lo)
    y_total = max(0.0, y_hi - y_lo)

    # Build the SAME station model the runner traverses: bottom + top(increasing) segments,
    # grouped into 7th-axis stations. This is the single source of truth — the runner uses the
    # same functions — so the preview shows exactly what the robot does at each stop (e.g. a
    # top piece AND the right side AND a bottom piece all at J7=0 => 3 passes at that stop).
    bottom_segments = build_tool2_bottom_segments(x_total)
    top_segments = build_tool2_top_segments_increasing(x_total, y_total)
    stations = group_tool2_segments_by_station([*bottom_segments, *top_segments])
    left_axis7 = tool2_left_axis7_position(x_total)

    out: list[dict[str, object]] = []

    def _emit(side: str, axis7: float, points: list[list[float]]) -> None:
        out.append({"side_label": side, "axis7_position_mm": float(axis7), "points": points})

    # Walk stations in ascending 7th-axis order (the runner's order). RIGHT runs at the J7≈0
    # station; LEFT runs at the final station — exactly where _run_tool2_station_traversal
    # interleaves them.
    for station_index, (axis7, segs) in enumerate(stations):
        is_j7_zero = abs(axis7) <= 1.0
        is_final = station_index == len(stations) - 1
        # Order within the station: top pieces first, then the right side (only at J7=0),
        # then bottom pieces — mirroring the traversal at J7=0; other stations top then bottom.
        for seg in segs:
            if seg.side != "top":
                continue
            sx = x_lo + seg.axis7 + seg.local_start_x
            ex = x_lo + seg.axis7 + seg.local_end_x
            _emit("top", seg.axis7, [[sx, y_hi], [ex, y_hi]])
        if is_j7_zero:
            _emit("right", 0.0, [[x_lo, y_hi], [x_lo, y_lo]])
        for seg in segs:
            if seg.side != "bottom":
                continue
            sx = x_lo + seg.axis7 + seg.local_start_x
            ex = x_lo + seg.axis7 + seg.local_end_x
            _emit("bottom", seg.axis7, [[sx, y_lo], [ex, y_lo]])
        if is_final:
            _emit("left", left_axis7, [[x_hi, y_lo], [x_hi, y_hi]])

    # If the left side's own 7th-axis station sits beyond the last bottom/top station, it is a
    # distinct stop — emit it after the loop instead so it is not merged into the final one.
    # (Handled above by attaching to the final station; keep run order by axis then insertion.)
    out.sort(key=lambda tp: float(tp["axis7_position_mm"]))
    for run_index, tp in enumerate(out):
        tp["run_index"] = run_index
        tp["path_id"] = f"tool2_{run_index:02d}_{tp['side_label']}"
    return out


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


# --- Station model shared by the runner AND the preview -----------------------------------
# These build the exact bottom/top station layout the robot traverses, so the 2D preview and
# the executor agree on which passes run at each 7th-axis stop. Moved here (robot-free) from
# tool2_side_runner so both can import them without pulling in the robot Server module.

def build_tool2_bottom_segments(x_total_mm: float) -> list[Tool2HorizontalSegment]:
    return [
        Tool2HorizontalSegment("bottom", axis7, 0.0, local_x_end, -15.0)
        for axis7, local_x_end in split_tool2_bottom_segments(x_total_mm)
    ]


def build_tool2_top_segments_increasing(x_total_mm: float, y_total_mm: float) -> list[Tool2HorizontalSegment]:
    """Top segments in INCREASING 7th-axis order — the ones the runner actually executes.

    Critically this can place a top segment at axis7=0 (when the reach span covers local X=0),
    which is why the J7=0 station runs a top piece in addition to the right side and a bottom
    piece. The preview must use THIS, not the decreasing split_tool2_top_segments, to match.
    """
    x_total = max(0.0, float(x_total_mm))
    y = float(y_total_mm) + 15.0
    span = tool2_top_local_x_span_at_y(float(y_total_mm))
    if x_total <= 0.0 or span is None:
        return []

    local_min, local_max = span
    if local_max <= local_min:
        return []

    # If the full top edge fits at one J7 station, keep it as one station. The
    # previous greedy splitter started at J7=0 and could leave a tiny remaining
    # top segment at a far rail position, which made small doors jump to an
    # unnecessary station after the bottom pass.
    full_fit_axis_min = max(0.0, x_total - local_max)
    full_fit_axis_max = max(0.0, -local_min)
    if full_fit_axis_min <= full_fit_axis_max + 1e-6:
        preferred_axis = tool2_left_axis7_position(x_total)
        axis7 = _clamp(preferred_axis, full_fit_axis_min, full_fit_axis_max)
        return [
            Tool2HorizontalSegment(
                "top",
                axis7,
                0.0 - axis7,
                x_total - axis7,
                y,
            )
        ]

    segments: list[Tool2HorizontalSegment] = []
    start = 0.0
    first_axis7 = 0.0
    if local_min <= 0.0 <= local_max:
        first_end = min(x_total, first_axis7 + local_max)
        if first_end > start + 1e-6:
            segments.append(Tool2HorizontalSegment("top", first_axis7, start - first_axis7, first_end - first_axis7, y))
            start = first_end

    while start < x_total - 1e-6:
        axis7 = start - local_min
        end = min(x_total, axis7 + local_max)
        if end <= start + 1e-6:
            break
        segments.append(Tool2HorizontalSegment("top", axis7, start - axis7, end - axis7, y))
        start = end
    return segments


def group_tool2_segments_by_station(
    segments: list[Tool2HorizontalSegment], tolerance_mm: float = 1.0,
) -> list[tuple[float, list[Tool2HorizontalSegment]]]:
    ordered = sorted(segments, key=lambda segment: (segment.axis7, 0 if segment.side == "bottom" else 1))
    stations: list[tuple[float, list[Tool2HorizontalSegment]]] = []
    for segment in ordered:
        if stations and abs(stations[-1][0] - segment.axis7) <= tolerance_mm:
            stations[-1][1].append(segment)
        else:
            stations.append((segment.axis7, [segment]))
    return stations

