from __future__ import annotations

"""DIAGNOSTIC ONLY — safe to delete.

Writes table_b_dxf/debug/origin_diagnostic_<job_id>.txt every time a DXF is parsed,
so the origin/overhang bug can be inspected on the robot computer where the real
files are uploaded. It reads the DXF and the parser's own helpers but changes no
parse logic and never raises into the parse path.
"""

import logging
import math
from datetime import datetime
from pathlib import Path
from typing import Any

logger = logging.getLogger(__name__)

_DEBUG_DIR = Path(__file__).resolve().parent / "debug"
# Raw LINE endpoints are dumped when within this distance (drawing units) of the
# computed top-right corner, to expose stray fragments in coordinate form.
_NEAR_CORNER_UNITS = 20.0


def _line_endpoints(doc: Any) -> list[dict[str, Any]]:
    """Every LINE's raw endpoints (drawing units), with layer, straight from the DXF."""
    lines: list[dict[str, Any]] = []
    try:
        for entity in doc.modelspace():
            if entity.dxftype() != "LINE":
                continue
            start = entity.dxf.start
            end = entity.dxf.end
            lines.append({
                "layer": str(entity.dxf.get("layer", "0")),
                "x0": float(start.x), "y0": float(start.y),
                "x1": float(end.x), "y1": float(end.y),
            })
    except Exception as error:  # noqa: BLE001
        logger.warning("origin_diagnostic: could not read LINE endpoints: %s", error)
    return lines


def _layer_stats_from_items(items: list[dict[str, Any]]) -> dict[str, dict[str, Any]]:
    """Per-layer entity count + bbox, computed from the parser's loop/open-path items.

    `items` are the parser's loops + open_paths. Their points may be raw (pre-
    normalization) or transformed depending on when this runs, so the caller states
    which in the report header.
    """
    stats: dict[str, dict[str, Any]] = {}
    for item in items:
        layer = str(item.get("layer", "0"))
        pts = item.get("points") or []
        rec = stats.setdefault(layer, {"count": 0, "min_x": math.inf, "min_y": math.inf,
                                       "max_x": -math.inf, "max_y": -math.inf})
        rec["count"] += 1
        for p in pts:
            rec["min_x"] = min(rec["min_x"], p[0])
            rec["min_y"] = min(rec["min_y"], p[1])
            rec["max_x"] = max(rec["max_x"], p[0])
            rec["max_y"] = max(rec["max_y"], p[1])
    return stats


def write_origin_diagnostic(
    job_id: str,
    doc: Any,
    raw_layer_items: list[dict[str, Any]],
    normalization: dict[str, Any],
    contour_layer_names: set[str],
) -> None:
    """Write the origin diagnostic report. Never raises into the parse path.

    Parameters
    ----------
    raw_layer_items : loops + open_paths captured BEFORE normalization (raw drawing
        units), so per-layer bboxes reflect the untouched DXF.
    normalization : the dict returned by _normalize_geometry (already computed).
    contour_layer_names : the parser's current _CONTOUR_LAYER_NAMES set.
    """
    try:
        _DEBUG_DIR.mkdir(parents=True, exist_ok=True)
        out_path = _DEBUG_DIR / f"origin_diagnostic_{job_id}.txt"

        lines: list[str] = []
        w = lines.append

        w("=" * 78)
        w(f"Table B DXF origin diagnostic")
        w(f"job_id     : {job_id}")
        w(f"written_at : {datetime.now().isoformat(timespec='seconds')}")
        try:
            insunits = doc.header.get("$INSUNITS", "MISSING")
        except Exception:  # noqa: BLE001
            insunits = "unreadable"
        w(f"$INSUNITS  : {insunits}  (1=inches, 4=mm)")
        w("NOTE: coordinates below are RAW DRAWING UNITS unless marked otherwise.")
        w("=" * 78)

        # 1. Every layer: entity count + bbox (raw).
        w("")
        w("[1] LAYERS (raw drawing units) — count + bounding box per layer")
        stats = _layer_stats_from_items(raw_layer_items)
        for layer in sorted(stats):
            s = stats[layer]
            w(f"    {layer:16s} n={s['count']:<4d} "
              f"x[{s['min_x']:.4f}, {s['max_x']:.4f}] y[{s['min_y']:.4f}, {s['max_y']:.4f}]")

        # 2. Which layers define the origin bbox — allowlist matches, exclusion
        #    matches, and the final selection, computed with the parser's own rule.
        w("")
        w("[2] _origin_reference_points layer decision")
        try:
            from .parser import (
                _NON_OUTLINE_LAYER_NAMES as exclusion_names,
                _select_origin_layers,
            )
        except Exception:  # noqa: BLE001
            exclusion_names = set()
            _select_origin_layers = None

        w(f"    allowlist (kept if matched)   : {sorted(contour_layer_names)}")
        w(f"    exclusion (never outline)     : {sorted(exclusion_names)}")

        allow_hits = [ly for ly in sorted(stats) if ly.strip().lower() in contour_layer_names]
        excl_hits = [ly for ly in sorted(stats) if ly.strip().lower() in exclusion_names]
        w(f"    layers matching allowlist     : {allow_hits if allow_hits else '(none)'}")
        w(f"    layers matching exclusion     : {excl_hits if excl_hits else '(none)'}")

        origin_source = normalization.get("origin_source", "unknown")
        w(f"    origin_source reported by parser : {origin_source}")

        if _select_origin_layers is not None:
            selected, sel_source = _select_origin_layers(set(stats.keys()))
            rejected = [ly for ly in sorted(stats) if ly not in selected]
            w(f"    selection rule outcome        : {sel_source}")
            w(f"    FINAL layers used for origin bbox : {sorted(selected)}")
            w(f"    layers REJECTED from origin bbox  : {rejected if rejected else '(none)'}")
        else:
            w("    (could not import parser selection rule to report final layers)")

        # 3 + 4. Final origin and bbox (normalized frame, mm).
        w("")
        w("[3][4] Computed origin + bbox (normalized machine frame, mm)")
        part_bbox = normalization.get("part_bbox") or {}
        size = normalization.get("size") or [None, None]
        w(f"    part_bbox min_x = {part_bbox.get('min_x')}")
        w(f"    part_bbox min_y = {part_bbox.get('min_y')}")
        w(f"    part_bbox max_x = {part_bbox.get('max_x')}")
        w(f"    part_bbox max_y = {part_bbox.get('max_y')}")
        w(f"    size (X,Y)      = {size}")
        w(f"    rotated         = {normalization.get('rotated')}")
        w("    NOTE: the machine origin is (0,0) at the part's bottom-right corner by")
        w("    construction (normalized_x = max_x - raw_x, normalized_y = raw_y - min_y).")

        # 5. Top-right corner the code treats as the true corner.
        # In the normalized mm frame that is (max_x, max_y) of the part bbox; in RAW
        # drawing units it is the (raw_max_x, raw_max_y) of the outline points, which
        # is what the stray-fragment dump below is measured against.
        raw_corner = _raw_top_right_from_outline(raw_layer_items, contour_layer_names)
        w("")
        w("[5] Top-right corner")
        if part_bbox:
            w(f"    normalized mm frame : (max_x={part_bbox.get('max_x')}, max_y={part_bbox.get('max_y')})")
        if raw_corner is not None:
            w(f"    RAW drawing units   : (x={raw_corner[0]:.4f}, y={raw_corner[1]:.4f})  "
              f"[source: {raw_corner[2]}]")
        else:
            w("    RAW drawing units   : (could not determine)")

        # 6. Raw LINE endpoints within _NEAR_CORNER_UNITS of that raw top-right corner.
        w("")
        w(f"[6] Raw LINE endpoints within {_NEAR_CORNER_UNITS} drawing units of the raw top-right corner")
        if raw_corner is None:
            w("    (skipped — no raw corner)")
        else:
            cx, cy, _ = raw_corner
            hits = 0
            for ln in _line_endpoints(doc):
                d0 = math.hypot(ln["x0"] - cx, ln["y0"] - cy)
                d1 = math.hypot(ln["x1"] - cx, ln["y1"] - cy)
                if min(d0, d1) <= _NEAR_CORNER_UNITS:
                    hits += 1
                    w(f"    layer={ln['layer']:14s} "
                      f"({ln['x0']:.4f},{ln['y0']:.4f}) -> ({ln['x1']:.4f},{ln['y1']:.4f})  "
                      f"| dist_to_corner: start={d0:.4f} end={d1:.4f}")
            if hits == 0:
                w("    (no LINE endpoints within range)")

        w("")
        w("=" * 78)
        w("END — diagnostic only; safe to delete this file and origin_diagnostic.py.")

        out_path.write_text("\n".join(lines), encoding="utf-8")
        logger.info("origin_diagnostic written: %s", out_path)
    except Exception as error:  # noqa: BLE001 — diagnostics must never break a parse
        logger.warning("origin_diagnostic failed for job %s: %s", job_id, error)


def _raw_top_right_from_outline(
    raw_layer_items: list[dict[str, Any]],
    contour_layer_names: set[str],
) -> tuple[float, float, str] | None:
    """Raw (drawing-unit) top-right corner used as the reference for the fragment dump.

    Uses the contour-layer points when present (matching _origin_reference_points),
    else all points. Returns (x, y, source) or None.
    """
    # Use the parser's own layer-selection rule so the reported corner matches what
    # the origin bbox is actually derived from.
    try:
        from .parser import _select_origin_layers
        layers = {str(item.get("layer", "")) for item in raw_layer_items}
        selected, source = _select_origin_layers(layers)
    except Exception:  # noqa: BLE001
        selected, source = None, "all_entities"

    all_pts: list[list[float]] = []
    sel_pts: list[list[float]] = []
    for item in raw_layer_items:
        pts = item.get("points") or []
        all_pts.extend(pts)
        if selected is not None and str(item.get("layer", "")) in selected:
            sel_pts.extend(pts)
    pts = sel_pts if sel_pts else all_pts
    if not pts:
        return None
    max_x = max(p[0] for p in pts)
    max_y = max(p[1] for p in pts)
    return max_x, max_y, source if sel_pts else "all_entities"
