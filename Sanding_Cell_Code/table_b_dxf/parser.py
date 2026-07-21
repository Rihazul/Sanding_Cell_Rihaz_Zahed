from __future__ import annotations

import json
import logging
import math
from pathlib import Path
from typing import Any

import ezdxf

from .jobs import get_table_b_dxf_job_paths

logger = logging.getLogger(__name__)

_CIRCLE_SEGMENTS = 72
_MAX_INSERT_DEPTH = 6
# Curve flattening tolerance expressed in MILLIMETERS. ezdxf's flattening() works in
# drawing units, so this is converted per-file via the drawing's units. Keeping it in
# mm means an inch drawing and a mm drawing are approximated to the same real-world
# precision; a fixed unit value would flatten an inch drawing 25.4x coarser (0.5 units
# = 12.7 mm), skewing the bbox the machine origin is derived from.
_FLATTEN_DISTANCE_MM = 0.5

# Machine-frame normalization. The sanding cell expects the part in a fixed frame:
# origin at the bottom-right of the model bbox, +X pointing left, +Y pointing up.
# The long dimension must run along X: if the uploaded part is taller than wide
# (Y > X), rotate it 90 degrees first. The Y (short) dimension of the table maxes
# out at ~37 inches (918 mm) — parts beyond that are flagged.
_MAX_Y_MM = 918.0

# $INSUNITS code -> inches per drawing unit (used only to decide the 36" rotation).
_UNITS_TO_INCHES = {
    1: 1.0,        # inches
    2: 12.0,       # feet
    4: 1.0 / 25.4,  # millimeters
    5: 1.0 / 2.54,  # centimeters
    6: 1000.0 / 25.4,  # meters
}


def _inches_per_unit(doc: Any, all_points: list[list[float]] | None = None) -> float:
    """Inches per drawing unit.

    Uses $INSUNITS when it is set to a real unit. When it is unitless/unknown — or
    tagged inches/feet but the part is implausibly large for that (a sanity check) —
    the units are inferred from the model size: manufacturing frames are hundreds
    to thousands of units in millimeters but only tens in inches. Getting this right
    matters because the fixed-mm toolpath offsets/steps are scaled by it.
    """
    try:
        code = int(doc.header.get("$INSUNITS", 0))
    except Exception:  # noqa: BLE001
        code = 0
    explicit = _UNITS_TO_INCHES.get(code)

    max_dim = 0.0
    if all_points:
        xs = [p[0] for p in all_points]
        ys = [p[1] for p in all_points]
        max_dim = max(max(xs) - min(xs), max(ys) - min(ys))

    mm_per_unit = 1.0 / 25.4  # inches-per-unit value for a millimeter drawing

    if explicit is None:
        # Unitless: >100 units is far too big for an inch-scale part → millimeters.
        return mm_per_unit if max_dim > 100 else 1.0
    if explicit >= 1.0 and max_dim > 200:
        # Tagged inches/feet but >200 units (>5 m) — mislabeled; treat as millimeters.
        return mm_per_unit
    return explicit


def _flatten_distance_units(doc: Any) -> float:
    """Curve flattening tolerance in DRAWING UNITS for this document.

    Derived from $INSUNITS only — the size-based unit fallback needs points, which
    do not exist until entities are flattened. An unset/unknown $INSUNITS keeps the
    historical 0.5-unit behaviour, which is already correct for mm drawings.
    """
    try:
        code = int(doc.header.get("$INSUNITS", 0))
    except Exception:  # noqa: BLE001
        code = 0
    inches_per_unit = _UNITS_TO_INCHES.get(code)
    if not inches_per_unit:
        return _FLATTEN_DISTANCE_MM
    mm_per_unit = inches_per_unit * 25.4
    if mm_per_unit <= 0:
        return _FLATTEN_DISTANCE_MM
    return _FLATTEN_DISTANCE_MM / mm_per_unit


def _all_points(loops: list[dict[str, Any]], open_paths: list[dict[str, Any]]) -> list[list[float]]:
    pts: list[list[float]] = []
    for loop in loops:
        pts.extend(loop["points"])
    for path in open_paths:
        pts.extend(path["points"])
    return pts


# The machine origin is the part's bottom-right corner, so the origin bbox must come
# from the door outline alone. Other layers can overhang it (e.g. grooves drawn past
# the edge) and would drag the origin off the part by the overhang distance.
#
# Layer selection for the origin bbox (all matches case-insensitive):
#   - _CONTOUR_LAYER_NAMES: layers that ARE the outline. "0" is AutoCAD's default
#     layer, which many exports draw the outline on.
#   - _NON_OUTLINE_LAYER_NAMES: layers that are never the outline (grooves, notes,
#     dimensions, construction/center/hidden lines, text/annotation).
# Rule: if any allowlisted layer is present, use only those. Otherwise use every
# layer EXCEPT the excluded ones (so a file with just "0" + "Groove" keeps "0",
# drops "Groove"). Only as a last resort (everything excluded) fall back to all.
_CONTOUR_LAYER_NAMES = {"contour", "outer", "outline", "0"}
_NON_OUTLINE_LAYER_NAMES = {
    "groove", "notes", "dimensions", "dim", "center", "centerline",
    "construction", "hidden", "text", "annotation",
}


def _select_origin_layers(layers: set[str]) -> tuple[set[str], str]:
    """Decide which layer names define the origin bbox, plus the source label.

    `layers` are the raw layer names present in the drawing.
    """
    allow = {ly for ly in layers if ly.strip().lower() in _CONTOUR_LAYER_NAMES}
    if allow:
        return allow, "contour_layer"
    non_excluded = {ly for ly in layers if ly.strip().lower() not in _NON_OUTLINE_LAYER_NAMES}
    if non_excluded:
        return non_excluded, "non_excluded_layers"
    return set(layers), "all_entities"


def _origin_reference_points(
    loops: list[dict[str, Any]],
    open_paths: list[dict[str, Any]],
) -> tuple[list[list[float]], str]:
    """Points that define the machine-origin bbox, plus the source used."""
    items = (*loops, *open_paths)
    layers = {str(item.get("layer", "")) for item in items}
    selected, source = _select_origin_layers(layers)
    pts: list[list[float]] = []
    for item in items:
        if str(item.get("layer", "")) in selected:
            pts.extend(item["points"])
    if pts:
        return pts, source
    return _all_points(loops, open_paths), "all_entities"


def _detect_outline_bbox(
    loops: list[dict[str, Any]],
    open_paths: list[dict[str, Any]],
) -> dict[str, float] | None:
    """Bbox of the door outline, stitched from line segments (largest closed polygon).

    Runs on the transformed (mm, machine-frame) geometry. Feeds every loop/open-path
    polyline into shapely polygonize with a small self-snap to weld hand-drawn gaps —
    the same technique the operator's line-to-surface detection uses. The largest
    resulting polygon is the outer door boundary; dangling fragments (e.g. prongs on
    the outline's layer) never close into a polygon, so they are excluded here where
    a layer or bbox filter cannot separate them.

    Returns the outline bbox (mm) or None when nothing closes.
    """
    try:
        from shapely.geometry import LineString
        from shapely.ops import polygonize, snap, unary_union
    except Exception as error:  # noqa: BLE001 — shapely should be installed; never fail the parse
        logger.warning("Table B DXF outline detection: shapely unavailable: %s", error)
        return None

    segments = []
    for item in (*loops, *open_paths):
        pts = item.get("points") or []
        if len(pts) >= 2:
            segments.append([(float(p[0]), float(p[1])) for p in pts])
    if not segments:
        return None

    # Snap tolerance scaled to the model size so it welds gaps without merging real
    # features. Model spans hundreds of mm; ~0.5mm closes hand-drawn gaps.
    all_pts = [p for seg in segments for p in seg]
    span = max(
        max(p[0] for p in all_pts) - min(p[0] for p in all_pts),
        max(p[1] for p in all_pts) - min(p[1] for p in all_pts),
        1.0,
    )
    snap_tol = max(0.5, span * 1e-3)

    try:
        lines = [LineString(seg) for seg in segments]
        merged = unary_union(lines)
        merged = snap(merged, merged, snap_tol)
        polygons = list(polygonize(unary_union(merged)))
    except Exception as error:  # noqa: BLE001 — a geometry hiccup must never fail the parse
        logger.warning("Table B DXF outline detection: polygonize failed: %s", error)
        return None

    if not polygons:
        return None

    # Use the UNION of all closed faces, not the single largest. Stray lines (e.g. a
    # prong extending past the door) slice the outline so polygonize returns several
    # partial faces — pockets plus the frame pieces around them — and the largest one
    # alone is just a pocket. Those faces TILE the real door, so their union recovers
    # the true outline. Dangling open lines never bound a face, so the prong is
    # naturally excluded from the union.
    try:
        outline = unary_union(polygons)
        min_x, min_y, max_x, max_y = outline.bounds
    except Exception as error:  # noqa: BLE001
        logger.warning("Table B DXF outline detection: union of faces failed: %s", error)
        return None

    # Safety guard: the union must span most of the geometry in at least one axis. If
    # the faces cover only a small corner (the outline never really closed), fall back
    # to part_bbox rather than shrink the door to a fragment.
    geo_min_x = min(p[0] for p in all_pts)
    geo_max_x = max(p[0] for p in all_pts)
    geo_min_y = min(p[1] for p in all_pts)
    geo_max_y = max(p[1] for p in all_pts)
    geo_w = max(geo_max_x - geo_min_x, 1e-9)
    geo_h = max(geo_max_y - geo_min_y, 1e-9)
    covers_w = (max_x - min_x) >= 0.9 * geo_w
    covers_h = (max_y - min_y) >= 0.9 * geo_h
    if not covers_w and not covers_h:
        logger.warning(
            "Table B DXF outline detection: union of faces (%.1f x %.1f) far smaller "
            "than geometry span (%.1f x %.1f); treating as no clean outline.",
            max_x - min_x, max_y - min_y, geo_w, geo_h,
        )
        return None

    # Exterior ring of the union polygon, so consumers can follow the ACTUAL door
    # boundary (curved edges included) rather than just its bounding rectangle. For a
    # MultiPolygon (disjoint faces that didn't merge) take the largest part's exterior.
    polygon_pts: list[list[float]] | None = None
    try:
        geom = outline
        if geom.geom_type == "MultiPolygon":
            geom = max(geom.geoms, key=lambda g: g.area)
        if geom.geom_type == "Polygon":
            polygon_pts = [[float(x), float(y)] for x, y in geom.exterior.coords]
    except Exception as error:  # noqa: BLE001 — polygon extraction is best-effort
        logger.warning("Table B DXF outline detection: exterior extraction failed: %s", error)

    return {
        "min_x": float(min_x),
        "min_y": float(min_y),
        "max_x": float(max_x),
        "max_y": float(max_y),
        "polygon": polygon_pts,
    }


def _normalize_geometry(
    loops: list[dict[str, Any]],
    open_paths: list[dict[str, Any]],
    inches_per_unit: float,
) -> dict[str, Any]:
    """Transform parsed geometry into the machine frame, in place.

    Steps (exactly as specified for Table B):
      1. If the uploaded part is taller than wide (Y size > X size), rotate every
         point 90 degrees first so the long side runs along X, then recompute the
         bounding box. A part already wider than tall is kept as-is.
      2. Re-origin to the bottom-right of the bbox with axes flipped:
             normalized_x = max_x - raw_x   (origin on the right, +X points left)
             normalized_y = raw_y - min_y   (+Y points up)
      3. Scale everything to MILLIMETERS (inch drawings are multiplied by 25.4), so
         the stored geometry — and the viewer's coordinate readout — is always mm.

    Rigid rotation + reflection preserve area and length, but per-item bbox /
    width / height / aspect are recomputed from the transformed points.
    """
    # The origin/bbox comes from the part outline when the drawing names one, so
    # geometry that overhangs the part (grooves drawn past the edge) cannot shift
    # the origin off the part's corner. All geometry is still transformed below.
    pts, origin_source = _origin_reference_points(loops, open_paths)
    if not pts:
        return {"applied": False, "rotated": False}

    raw_min_x = min(p[0] for p in pts)
    raw_max_x = max(p[0] for p in pts)
    raw_min_y = min(p[1] for p in pts)
    raw_max_y = max(p[1] for p in pts)
    # Rotate only when the part is portrait (taller than wide) so the long
    # dimension ends up along X.
    rotated = (raw_max_y - raw_min_y) > (raw_max_x - raw_min_x)

    def rotate(p: list[float]) -> list[float]:
        # 90 degrees CCW: (x, y) -> (-y, x). Turns a tall part into a wide one.
        return [-p[1], p[0]] if rotated else [p[0], p[1]]

    rotated_pts = [rotate(p) for p in pts]
    min_x = min(p[0] for p in rotated_pts)
    max_x = max(p[0] for p in rotated_pts)
    min_y = min(p[1] for p in rotated_pts)
    max_y = max(p[1] for p in rotated_pts)

    # Millimeters per drawing unit — inch drawings scale by 25.4, mm drawings by 1.
    mm_per_unit = inches_per_unit * 25.4

    def transform(p: list[float]) -> list[float]:
        rx, ry = rotate(p)
        return [(max_x - rx) * mm_per_unit, (ry - min_y) * mm_per_unit]

    for loop in loops:
        loop["points"] = [transform(p) for p in loop["points"]]
        bbox = _bbox(loop["points"])
        loop["bbox"] = bbox
        loop["area"] = _polygon_area(loop["points"])
        width = bbox["max_x"] - bbox["min_x"]
        height = bbox["max_y"] - bbox["min_y"]
        shorter = max(min(width, height), 1e-9)
        loop["width"] = width
        loop["height"] = height
        loop["aspect_ratio"] = max(width, height) / shorter

    for path in open_paths:
        path["points"] = [transform(p) for p in path["points"]]
        path["bbox"] = _bbox(path["points"])
        path["length"] = _polyline_length(path["points"])

    # Stored geometry is now in millimeters, so sizes are mm directly.
    final_x = (max_x - min_x) * mm_per_unit
    final_y = (max_y - min_y) * mm_per_unit
    y_size_mm = final_y

    # Stitch the transformed line geometry into closed polygons; the largest is the
    # door outline. Dangling fragments (e.g. prongs on the outline's layer) never
    # close into a polygon, so this excludes them where a layer/bbox filter cannot.
    outline_detected = _detect_outline_bbox(loops, open_paths)
    # Split the detection into a pure bbox (existing consumers) and the exterior
    # polygon (curved-outline consumers, e.g. curved frame sections).
    outline_polygon: list[list[float]] | None = None
    outline_bbox: dict[str, float] | None = None
    if outline_detected is not None:
        outline_polygon = outline_detected.get("polygon")
        outline_bbox = {
            "min_x": outline_detected["min_x"],
            "min_y": outline_detected["min_y"],
            "max_x": outline_detected["max_x"],
            "max_y": outline_detected["max_y"],
        }
    return {
        "applied": True,
        "rotated": rotated,
        # Stored geometry is millimeters (inch drawings were converted).
        "units": "mm",
        "mm_per_unit": 1.0,
        "source_inches_per_unit": inches_per_unit,
        # Which geometry defined the origin bbox: the outline layer, or everything.
        "origin_source": origin_source,
        # The part's extent in the machine frame (mm). This is the authoritative
        # frame/outer boundary: it comes from the outline alone, so consumers must use
        # it rather than re-deriving a bbox from all geometry — layers such as grooves
        # can overhang the part and would inflate that bbox.
        "part_bbox": {"min_x": 0.0, "min_y": 0.0, "max_x": final_x, "max_y": final_y},
        # The door outline stitched from line segments (largest closed polygon), in
        # the machine frame (mm). Excludes dangling fragments that a layer/bbox filter
        # cannot (e.g. prongs sharing the outline's layer). null when nothing closed —
        # consumers then fall back to part_bbox.
        "outline_bbox": outline_bbox,
        # The door outline as an ordered exterior ring (mm, machine frame), with curved
        # edges kept as flattened line segments. null when no clean outline closed.
        # Consumers use this to follow the ACTUAL boundary (e.g. clip frame sections to
        # a curved door) instead of the bounding rectangle.
        "outline_polygon": outline_polygon,
        "size": [final_x, final_y],
        "y_size_mm": y_size_mm,
        "max_y_mm": _MAX_Y_MM,
        # The Y (short) side must fit the table; flag parts that exceed the limit.
        "y_within_limit": y_size_mm < _MAX_Y_MM,
    }

# Keep every real closed loop, including thin strips. Only genuinely degenerate
# (near-zero-area / collinear) loops are dropped. Configurable if needed.
MIN_LOOP_AREA = 1e-6
# A loop counts as "narrow" (thin strip) when its long side is this many times
# its short side — used for diagnostics/logging only, never to filter.
NARROW_ASPECT_RATIO = 3.0


def _polygon_area(points: list[list[float]]) -> float:
    """Absolute polygon area via the shoelace formula."""
    n = len(points)
    if n < 3:
        return 0.0
    total = 0.0
    for i in range(n):
        x1, y1 = points[i]
        x2, y2 = points[(i + 1) % n]
        total += (x1 * y2) - (x2 * y1)
    return abs(total) / 2.0


def _polyline_length(points: list[list[float]]) -> float:
    """Total length along an (open) polyline."""
    total = 0.0
    for i in range(len(points) - 1):
        (x0, y0), (x1, y1) = points[i], points[i + 1]
        total += math.hypot(x1 - x0, y1 - y0)
    return total


def _bbox(points: list[list[float]]) -> dict[str, float] | None:
    if not points:
        return None
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    return {"min_x": min(xs), "min_y": min(ys), "max_x": max(xs), "max_y": max(ys)}


def _size(points: list[list[float]]) -> float:
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    return max(max(xs) - min(xs), max(ys) - min(ys), 1.0)


def _looks_closed(points: list[list[float]]) -> bool:
    """True if the first and last points coincide within a size-relative tolerance."""
    if len(points) < 3:
        return False
    (x0, y0), (x1, y1) = points[0], points[-1]
    return math.hypot(x1 - x0, y1 - y0) <= _size(points) * 1e-4


def _dedupe_closing_point(points: list[list[float]]) -> list[list[float]]:
    """Drop a duplicated final point so a closed ring is not doubled."""
    if (
        len(points) >= 2
        and math.isclose(points[0][0], points[-1][0], abs_tol=1e-9)
        and math.isclose(points[0][1], points[-1][1], abs_tol=1e-9)
    ):
        return points[:-1]
    return points


def _iter_entities(container: Any, depth: int = 0):
    """Yield entities, exploding INSERT block references into their contents."""
    for entity in container:
        if entity.dxftype() == "INSERT" and depth < _MAX_INSERT_DEPTH:
            try:
                yield from _iter_entities(entity.virtual_entities(), depth + 1)
                continue
            except Exception as error:  # noqa: BLE001 - a bad block must not fail the whole parse
                logger.warning("Table B DXF Assisted could not explode INSERT: %s", error)
                continue
        yield entity


def _flatten(entity: Any, distance: float) -> list[list[float]] | None:
    """Sample a curved/spline entity into [x, y] points (ezdxf flattening).

    `distance` is the sagitta tolerance in drawing units for this document.
    """
    try:
        return [[float(p.x), float(p.y)] for p in entity.flattening(distance)]
    except Exception:  # noqa: BLE001
        return None


def _entity_polyline(entity: Any, flatten_distance: float) -> tuple[list[list[float]] | None, bool]:
    """Return (points, is_closed) for a supported entity, else (None, False).

    Closure is detected from the entity's closed flag OR coincident endpoints,
    which covers DXFs that close a shape by repeating the first vertex.

    `flatten_distance` is the curve tolerance in drawing units for this document.
    """
    dxftype = entity.dxftype()

    if dxftype == "LWPOLYLINE":
        pts = [[float(x), float(y)] for x, y in entity.get_points("xy")]
        closed = bool(entity.closed) or _looks_closed(pts)
        return _dedupe_closing_point(pts), closed

    if dxftype == "POLYLINE":
        pts = [[float(v.dxf.location.x), float(v.dxf.location.y)] for v in entity.vertices]
        closed = bool(getattr(entity, "is_closed", False)) or _looks_closed(pts)
        return _dedupe_closing_point(pts), closed

    if dxftype == "CIRCLE":
        center = entity.dxf.center
        radius = float(entity.dxf.radius)
        pts = [
            [
                float(center.x + radius * math.cos(2 * math.pi * i / _CIRCLE_SEGMENTS)),
                float(center.y + radius * math.sin(2 * math.pi * i / _CIRCLE_SEGMENTS)),
            ]
            for i in range(_CIRCLE_SEGMENTS)
        ]
        return pts, True

    if dxftype == "ELLIPSE":
        pts = _flatten(entity, flatten_distance)
        if not pts:
            return None, False
        span = abs(
            float(getattr(entity.dxf, "end_param", 0.0)) - float(getattr(entity.dxf, "start_param", 0.0))
        )
        closed = abs(span - 2 * math.pi) < 1e-3 or _looks_closed(pts)
        return _dedupe_closing_point(pts), closed

    if dxftype == "SPLINE":
        pts = _flatten(entity, flatten_distance)
        if not pts:
            return None, False
        closed = bool(getattr(entity, "closed", False)) or _looks_closed(pts)
        return _dedupe_closing_point(pts), closed

    if dxftype == "ARC":
        pts = _flatten(entity, flatten_distance)
        if not pts:
            return None, False
        return pts, False

    if dxftype == "LINE":
        start = entity.dxf.start
        end = entity.dxf.end
        return [[float(start.x), float(start.y)], [float(end.x), float(end.y)]], False

    return None, False


def parse_dxf_loops(job_id: str) -> dict[str, Any]:
    """Parse a job's uploaded DXF into selectable closed loops + open display paths.

    Closed loops (LWPOLYLINE/POLYLINE closed by flag or coincident endpoints,
    plus CIRCLE / ELLIPSE / closed SPLINE) are selectable regions. Open geometry
    (lines, arcs, open polylines) is returned separately so the whole 2D drawing
    can be shown for context. Blocks (INSERT) are exploded. Persists the result
    to parsed/{job_id}/parsed_loops.json.
    """
    paths = get_table_b_dxf_job_paths(job_id)
    source_path = Path(paths["source_dxf"])
    if not source_path.exists():
        raise FileNotFoundError(f"Uploaded DXF not found for job {job_id}: {source_path}")

    try:
        doc = ezdxf.readfile(str(source_path))
    except ezdxf.DXFStructureError as error:
        raise ValueError(f"Invalid or corrupt DXF file: {error}") from error

    logger.info("Table B DXF Assisted DXF loaded: job_id=%s file=%s", job_id, source_path)

    msp = doc.modelspace()

    # Curve tolerance in this drawing's units, so inch and mm files are flattened to
    # the same real-world precision. Curves feed the bbox the machine origin comes
    # from, so a units-blind tolerance shifts the origin on inch drawings.
    flatten_distance = _flatten_distance_units(doc)
    logger.info(
        "Table B DXF Assisted flatten tolerance: job_id=%s distance_units=%.6f (%.2f mm target)",
        job_id, flatten_distance, _FLATTEN_DISTANCE_MM,
    )

    loops: list[dict[str, Any]] = []
    open_paths: list[dict[str, Any]] = []
    total_scanned = 0
    unsupported_ignored = 0
    degenerate_filtered = 0
    entity_types: set[str] = set()

    for index, entity in enumerate(_iter_entities(msp)):
        total_scanned += 1
        dxftype = entity.dxftype()
        entity_types.add(dxftype)

        try:
            points, closed = _entity_polyline(entity, flatten_distance)
        except Exception as error:  # noqa: BLE001 - never let one bad entity fail the parse
            logger.warning("Table B DXF Assisted skipped unreadable %s: %s", dxftype, error)
            unsupported_ignored += 1
            continue

        if not points:
            unsupported_ignored += 1
            continue

        handle = entity.dxf.get("handle", None)
        entity_id = str(handle) if handle else f"{dxftype}-{index}"
        layer = str(entity.dxf.get("layer", "0"))

        if closed and len(points) >= 3:
            area = _polygon_area(points)
            if area <= MIN_LOOP_AREA:
                # Degenerate / near-zero-area (collinear) — not a real region.
                degenerate_filtered += 1
                continue
            bbox = _bbox(points)
            width = bbox["max_x"] - bbox["min_x"]
            height = bbox["max_y"] - bbox["min_y"]
            shorter = max(min(width, height), 1e-9)
            longer = max(width, height)
            loops.append({
                "loop_id": entity_id,
                "entity_id": entity_id,
                "type": dxftype,
                "layer": layer,
                "points": points,
                "closed": True,
                "bbox": bbox,
                "area": area,
                "width": width,
                "height": height,
                "aspect_ratio": longer / shorter,
            })
        elif len(points) >= 2:
            # Open selectable guide geometry (lines / arcs / open polylines).
            open_paths.append({
                "entity_id": entity_id,
                "type": "line_entity",
                "dxf_type": dxftype,
                "layer": layer,
                "points": points,
                "bbox": _bbox(points),
                "length": _polyline_length(points),
                "closed": False,
            })
        else:
            unsupported_ignored += 1

    # DIAGNOSTIC ONLY (safe to remove): snapshot raw layer + points BEFORE
    # normalization mutates them in place, so the diagnostic can report the untouched
    # drawing-unit geometry.
    _diag_raw_items = [
        {"layer": item.get("layer", "0"), "points": [list(p) for p in item.get("points", [])]}
        for item in (*loops, *open_paths)
    ]

    # Transform everything into the machine frame (origin bottom-right, +X left,
    # +Y up, with a 90-degree rotation when the part is taller than wide). Units are
    # resolved from $INSUNITS with a size-based fallback so mm/inch parts scale right.
    inches_per_unit = _inches_per_unit(doc, _all_points(loops, open_paths))
    normalization = _normalize_geometry(loops, open_paths, inches_per_unit)

    # DIAGNOSTIC ONLY (safe to remove): write the origin/overhang report to
    # table_b_dxf/debug/origin_diagnostic_<job_id>.txt. Never affects the parse.
    try:
        from .origin_diagnostic import write_origin_diagnostic

        write_origin_diagnostic(
            job_id=job_id,
            doc=doc,
            raw_layer_items=_diag_raw_items,
            normalization=normalization,
            contour_layer_names=_CONTOUR_LAYER_NAMES,
        )
    except Exception as _diag_error:  # noqa: BLE001 — diagnostics must never break a parse
        logger.warning("origin_diagnostic hook failed for job %s: %s", job_id, _diag_error)

    narrow_loops = sum(1 for loop in loops if loop["aspect_ratio"] >= NARROW_ASPECT_RATIO)

    result = {
        "job_id": job_id,
        "status": "parsed",
        "loops": loops,
        "open_paths": open_paths,
        "normalization": normalization,
        "summary": {
            "total_entities_scanned": total_scanned,
            "closed_loops_found": len(loops),
            "closed_loops_count": len(loops),
            "narrow_loops_found": narrow_loops,
            "open_paths_found": len(open_paths),
            "open_line_entities_count": len(open_paths),
            "loops_filtered_out": degenerate_filtered,
            "unsupported_entities_count": unsupported_ignored,
            "open_entities_ignored": len(open_paths) + unsupported_ignored,
            "entity_types_found": sorted(entity_types),
        },
    }

    parsed_path = Path(paths["parsed_loops"])
    parsed_path.parent.mkdir(parents=True, exist_ok=True)
    parsed_path.write_text(json.dumps(result, indent=2), encoding="utf-8")

    logger.info(
        "Table B DXF Assisted parse complete: job_id=%s scanned=%s total_loops=%s narrow_loops=%s loops_filtered_out=%s open_paths=%s unsupported=%s normalized=%s rotated=%s saved=%s",
        job_id,
        total_scanned,
        len(loops),
        narrow_loops,
        degenerate_filtered,
        len(open_paths),
        unsupported_ignored,
        normalization.get("applied"),
        normalization.get("rotated"),
        parsed_path,
    )

    return result
