from __future__ import annotations

"""Clip computed frame sections to the door outline (shapely).

The frontend builds coarse rectangular frame sections over the door's bounding box.
On a curved door, sections that straddle the curved edge must be trimmed to the real
boundary so their sanding passes follow the curve (as continuous point-to-point moves)
instead of overrunning it. Doing the intersection here with shapely is exact and
handles concave outlines and holes, which a pure-JS clip does not.
"""

import logging
from typing import Any

from .frame_classification import (
    _coarsen_sorted,
    _corner_vertices,
    _ring_has_curve,
    _ring_has_non_axis_edge,
    _section_effective_thickness,
)
from .frame_geometry import _as_polygons, _polygon, _rings_of, compute_frame_area
from .frame_section_builder import (
    _merge_blocks_into_curved_regions,
    _outer_non_axis_boundary_regions,
    _polygon_from_section,
    _section_from_polygon,
    _split_band_into_strips,
    _split_corner_band_into_legs,
    curved_pocket_bands,
    diagonal_zone_box,
    split_diagonal_zone_into_arms,
    _split_curved_band_by_geometry,
    _split_sections_by_reach,
    build_structural_curved_frame_sections,
)
from .frame_toolpaths import (
    _clip_line_segment_to_polygon,
    _curved_band_following_centerline,
    _generate_section_toolpaths,
    _oriented_toolpath,
    _split_curve_at_apex,
)

logger = logging.getLogger(__name__)


def compute_frame_zigzag_fill(
    outer_polygon: list[list[float]] | None,
    pocket_polygons: list[list[list[float]]] | None,
    surface3d_polygons: list[list[list[float]]] | None,
    pass_width_mm: float = 75.0,
    overlap_mm: float = 0.0,
) -> dict[str, Any]:
    """Zigzag (serpentine) fill of the frame surface, clipped to the real polygon.

    For "Frame Level on the whole door": fill frame_area = outer − pockets − 3D with
    vertical passes stepping in X at (pass_width − overlap), each pass CLIPPED to the
    frame polygon so it stops at the door's curved edge instead of overrunning the
    bounding rectangle. Passes alternate direction (serpentine), matching the frontend
    zigzag pattern but curve-aware.

    Returns:
      { ok, rings, toolpaths: [{points, tool, operation_type, direction}], frame_area }
    where each toolpath is one continuous vertical pass (as flattened points).
    """
    try:
        from shapely.geometry import LineString
        from shapely.ops import unary_union
    except Exception as error:  # noqa: BLE001
        logger.warning("Table B DXF frame zigzag: shapely unavailable: %s", error)
        return {"ok": False, "reason": "shapely_unavailable", "rings": [], "toolpaths": []}

    outer = _polygon(outer_polygon)
    if outer is None:
        return {"ok": False, "reason": "no_outer_polygon", "rings": [], "toolpaths": []}

    obstacles = []
    for group in (pocket_polygons or [], surface3d_polygons or []):
        for pts in group:
            p = _polygon(pts)
            if p is not None:
                obstacles.append(p)
    frame_geom = outer if not obstacles else outer.difference(unary_union(obstacles))
    if frame_geom.is_empty or frame_geom.area <= 1e-6:
        return {"ok": True, "rings": [], "toolpaths": [], "frame_area": 0.0}

    minx, miny, maxx, maxy = [float(v) for v in frame_geom.bounds]
    step = max(float(pass_width_mm) - max(0.0, float(overlap_mm)), 1.0)

    toolpaths: list[dict[str, Any]] = []
    # In the normalized frame the machine origin (0,0) is the bottom-right corner, which
    # is min-x / min-y. So the fill must START AT THE ORIGIN CORNER (minx, miny): step X
    # from minx up to maxx (physical right -> left) and run each pass BOTTOM -> TOP first
    # (y0 -> y1), then alternate (serpentine). The first toolpath point is the origin
    # corner; the last pass lands on the far (leftmost) side.
    x = minx
    toggle = 0
    seq = 0
    # A tiny inset keeps a vertical scanline off the exact left/right edge so the very
    # first/last pass still intersects the polygon interior.
    while x <= maxx + 1e-6:
        scan = LineString([(x, miny - 1.0), (x, maxy + 1.0)])
        clipped = scan.intersection(frame_geom)
        # The scanline may cross the frame in several disjoint spans (e.g. between two
        # pockets); emit each span as its own pass so no move crosses a hole.
        spans = []
        if clipped.geom_type == "LineString" and not clipped.is_empty:
            spans = [clipped]
        elif clipped.geom_type == "MultiLineString":
            spans = [g for g in clipped.geoms if g.length > 1e-6]
        for span in spans:
            ys = [c[1] for c in span.coords]
            y0, y1 = min(ys), max(ys)
            # toggle 0 => bottom->top, toggle 1 => top->bottom (serpentine)
            pass_pts = [[x, y0], [x, y1]] if not toggle else [[x, y1], [x, y0]]
            toolpaths.append({
                "path_id": f"frame_zigzag_{seq:03d}",
                "points": pass_pts,
                "tool": "tool_4_frame",
                "operation_type": "frame_zigzag_pass",
                "direction": "Y",
            })
            seq += 1
        x += step
        toggle = 1 - toggle

    rings = _rings_of(frame_geom)
    return {
        "ok": True,
        "rings": rings,
        "toolpaths": toolpaths,
        "frame_area": sum(r["area"] for r in rings),
        "pass_count": len(toolpaths),
    }


def compute_frame_sections_and_toolpaths(
    outer_polygon: list[list[float]] | None,
    pocket_polygons: list[list[list[float]]] | None,
    surface3d_polygons: list[list[list[float]]] | None,
    pass_width_mm: float = 75.0,
    offset_mm: float = 50.0,
    overlap_mm: float = 0.0,
    reach_x_mm: float = 515.0,
    reach_y_mm: float = 750.0,
) -> dict[str, Any]:
    """Compute frame sections and clipped straight/zigzag toolpaths from frame_area.

    This keeps the previous frontend constraints, but the geometry source is now the
    true frame surface: outer door minus pockets and 3D contour regions.
    """
    try:
        from shapely.geometry import LineString, box
        from shapely.ops import unary_union
    except Exception as error:  # noqa: BLE001
        logger.warning("Table B DXF frame toolpath: shapely unavailable: %s", error)
        area = compute_frame_area(outer_polygon, pocket_polygons, surface3d_polygons)
        return {"ok": False, **area, "sections": [], "chunks": [], "toolpaths": [], "reason": "shapely_unavailable"}

    outer = _polygon(outer_polygon)
    if outer is None:
        return {"ok": False, "reason": "no_outer_polygon", "rings": [], "sections": [], "chunks": [], "toolpaths": []}

    obstacles = []
    obstacle_points: list[list[float]] = []
    obstacle_bounds: list[tuple[float, float, float, float]] = []
    for group in (pocket_polygons or [], surface3d_polygons or []):
        for pts in group:
            p = _polygon(pts)
            if p is not None:
                obstacles.append(p)
                obstacle_bounds.append(tuple(float(v) for v in p.bounds))
                # Only REAL corners feed the grid lines. A smooth (arc-flattened) edge
                # contributes no grid line — otherwise its dozens of near-collinear
                # vertices carve the curved frame into stacked micro-cells. Sharp corners
                # (turn > ~18°) are kept so rectangular/triangular pockets still grid.
                obstacle_points.extend(_corner_vertices(pts, angle_deg=18.0))

    frame_geom = outer
    if obstacles:
        frame_geom = outer.difference(unary_union(obstacles))
    if frame_geom.is_empty or frame_geom.area <= 1e-6:
        return {"ok": True, "rings": [], "sections": [], "chunks": [], "toolpaths": [], "frame_area": 0.0, "obstacle_count": len(obstacles)}

    structural_curved_sections = build_structural_curved_frame_sections(
        frame_geom,
        outer,
        obstacles,
        float(pass_width_mm),
    )

    # CLAIM ORDER MATTERS:
    # 1) Curved pocket bands are exact, corner-anchored Case C sections. Claim them first
    #    from the full frame so later grid/edge stages cannot consume or reshape them.
    # 2) Diagonal Case B zones are computed from the remaining frame.
    # 3) Rectangular Case A grid sections are computed last from what remains.
    curve_bands = curved_pocket_bands(obstacles, outer, frame_geom, float(pass_width_mm))
    curve_union = unary_union(curve_bands) if curve_bands else None
    remaining_frame = frame_geom
    if curve_union is not None:
        remaining_frame = frame_geom.difference(curve_union)

    grid_source = remaining_frame if not remaining_frame.is_empty and remaining_frame.area > 1e-6 else None

    outer_edge_regions = _outer_non_axis_boundary_regions(
        outer_polygon,
        grid_source,
        pass_width_mm=pass_width_mm,
        offset_mm=offset_mm,
    ) if grid_source is not None else []

    diag_zone = diagonal_zone_box(obstacles, grid_source) if grid_source is not None else None
    diag_frame = None
    grid_frame = grid_source
    if diag_zone is not None and grid_source is not None:
        inside = grid_source.intersection(diag_zone)
        outside = grid_source.difference(diag_zone)
        if not inside.is_empty and inside.area > 1e-6:
            diag_frame = inside
            grid_frame = outside if not outside.is_empty and outside.area > 1e-6 else None

    block_geoms: list[Any] = []
    partial_geoms: list[Any] = []
    min_area = 1e-5

    if grid_frame is not None and not grid_frame.is_empty and grid_frame.area > min_area:
        minx, miny, maxx, maxy = [float(v) for v in grid_frame.bounds]
        # Grid lines come only from obstacle CORNERS (arc points were already filtered out
        # in _corner_vertices). A light coarsening still collapses corners that fall within
        # a few mm of each other so we never make sub-tool slivers.
        grid_merge = 8.0
        xs = _coarsen_sorted([
            minx,
            maxx,
            *[float(p[0]) for p in obstacle_points if minx - 1e-6 <= float(p[0]) <= maxx + 1e-6],
            *[v for bounds in obstacle_bounds for v in (bounds[0], bounds[2]) if minx - 1e-6 <= v <= maxx + 1e-6],
        ], grid_merge)
        ys = _coarsen_sorted([
            miny,
            maxy,
            *[float(p[1]) for p in obstacle_points if miny - 1e-6 <= float(p[1]) <= maxy + 1e-6],
            *[v for bounds in obstacle_bounds for v in (bounds[1], bounds[3]) if miny - 1e-6 <= v <= maxy + 1e-6],
        ], grid_merge)

        if len(xs) >= 2 and len(ys) >= 2:
            full: list[list[bool]] = [[False for _ in range(len(xs) - 1)] for _ in range(len(ys) - 1)]
            used: list[list[bool]] = [[False for _ in range(len(xs) - 1)] for _ in range(len(ys) - 1)]

            for j in range(len(ys) - 1):
                for i in range(len(xs) - 1):
                    cell = box(xs[i], ys[j], xs[i + 1], ys[j + 1])
                    if cell.area <= min_area:
                        continue
                    inter = cell.intersection(grid_frame)
                    if inter.is_empty or inter.area <= min_area:
                        continue
                    if abs(inter.area - cell.area) / max(cell.area, min_area) <= 1e-4:
                        full[j][i] = True
                    else:
                        partial_geoms.extend(_as_polygons(inter))

            # Merge FULL cells into maximal rectangular blocks. These are the genuinely
            # rectangular frame chunks the grid is good at.
            for j in range(len(ys) - 1):
                for i in range(len(xs) - 1):
                    if not full[j][i] or used[j][i]:
                        continue
                    i2 = i
                    while i2 + 1 < len(xs) - 1 and full[j][i2 + 1] and not used[j][i2 + 1]:
                        i2 += 1
                    j2 = j
                    can_extend = True
                    while can_extend and j2 + 1 < len(ys) - 1:
                        for k in range(i, i2 + 1):
                            if not full[j2 + 1][k] or used[j2 + 1][k]:
                                can_extend = False
                                break
                        if can_extend:
                            j2 += 1
                    for jj in range(j, j2 + 1):
                        for ii in range(i, i2 + 1):
                            used[jj][ii] = True
                    block_geoms.append(box(xs[i], ys[j], xs[i2 + 1], ys[j2 + 1]))

    sections: list[dict[str, Any]] = []
    counter = 0

    merged_partials = _as_polygons(unary_union(partial_geoms)) if partial_geoms else []

    # Case C can also appear as a curved end inside an otherwise rectangular frame area.
    # Protect those curved partials before the generic rectangle/diagonal merge; otherwise
    # they get absorbed into grid blocks and are later displayed as mixed rectangle passes.
    protected_curve_polys: list[Any] = []
    straight_partials: list[Any] = []
    for partial in merged_partials:
        base = partial
        exterior = [[float(x), float(y)] for x, y in base.exterior.coords[:-1]] if getattr(base, "geom_type", None) == "Polygon" else []
        if _ring_has_curve(exterior):
            protected_curve_polys.append(base)
        else:
            straight_partials.append(base)

    protected_curve_union = unary_union(protected_curve_polys) if protected_curve_polys else None
    if protected_curve_union is not None:
        # Keep rectangular blocks from overlapping the curved-frame sections.
        trimmed_blocks: list[Any] = []
        for block in block_geoms:
            for piece in _as_polygons(block.difference(protected_curve_union)):
                if not piece.is_empty and piece.area > 1e-3:
                    trimmed_blocks.append(piece)
        block_geoms = trimmed_blocks

    merged_partials, block_geoms = _merge_blocks_into_curved_regions(straight_partials, block_geoms, pass_width_mm)

    edge_polys = _as_polygons(unary_union(outer_edge_regions)) if outer_edge_regions else []
    if protected_curve_polys:
        protected_union = unary_union(protected_curve_polys)
        trimmed_edges: list[Any] = []
        for edge in edge_polys:
            for piece in _as_polygons(edge.difference(protected_union)):
                if not piece.is_empty and piece.area > 1e-3:
                    trimmed_edges.append(piece)
        edge_polys = trimmed_edges

    other_polys: list[Any] = []
    for region in merged_partials:
        for band in _split_curved_band_by_geometry(region, pass_width_mm, reach_x_mm, reach_y_mm):
            exterior = [[float(x), float(y)] for x, y in band.exterior.coords[:-1]] if getattr(band, "geom_type", None) == "Polygon" else []
            if _ring_has_curve(exterior):
                protected_curve_polys.append(band)
            else:
                other_polys.extend(_split_band_into_strips(band))
    other_polys.extend(block_geoms)

    # Case B diagonal sections are already isolated from curved claims, so their arms can
    # be split cleanly without stealing the Case C curved-band area.
    if diag_frame is not None:
        for arm in split_diagonal_zone_into_arms(diag_frame, diag_zone, float(pass_width_mm)):
            other_polys.extend(_split_band_into_strips(arm))

    strip_polys: list[Any] = list(edge_polys)
    if edge_polys:
        edge_union = unary_union(edge_polys)
        for poly in other_polys:
            remainder = poly.difference(edge_union)
            for piece in _as_polygons(remainder):
                if not piece.is_empty and piece.area > 1e-3:
                    strip_polys.append(piece)
    else:
        strip_polys.extend(other_polys)

    # Split only the unprotected remainder into corner legs. The claimed curved pocket
    # bands are appended after this so they remain whole curved sections.
    leg_polys: list[Any] = []
    for poly in strip_polys:
        leg_polys.extend(_split_corner_band_into_legs(poly))
    strip_polys = leg_polys

    # Protected curved sections are appended after generic splitting so they keep their
    # true curved section shape. This covers both corner-anchored pocket bands and curved
    # ends discovered from the computed frame itself.
    for band in [*protected_curve_polys, *(curve_bands or [])]:
        if not band.is_empty and band.area > 1e-3:
            strip_polys.append(band)

    for poly in sorted(strip_polys, key=lambda p: p.area, reverse=True):
        section = _section_from_polygon(poly, counter + 1, True)
        if section:
            counter += 1
            sections.append(section)

    if structural_curved_sections:
        # Curved-pocket models use operator-intended structural sections instead of the
        # grid fragments. Rectangular and triangular models keep the existing fallback.
        sections = structural_curved_sections

    # Route each section by thickness:
    #   • THIN band (≤ ~1 tool width): one continuous centerline over the FULL section,
    #     then split that LINE by reach → clean straight arm passes (not stubby chunks).
    #   • WIDE section (> ~1 tool width): reach-split the polygon and serpentine-fill each
    #     chunk (the big edge rectangles).
    import math

    band_pass_width = max(float(pass_width_mm), 1e-6)
    band_offset = max(float(offset_mm), 0.0)
    band_step = max(band_pass_width - min(max(float(overlap_mm), 0.0), 100.0), 10.0)
    band_half = band_pass_width / 2.0

    thin_sections: list[dict[str, Any]] = []
    wide_sections: list[dict[str, Any]] = []
    for section in sections:
        poly = _polygon_from_section(section)
        if poly is None:
            continue
        mode = section.get("toolpath_mode")
        is_curve_section = bool(section.get("curved"))
        is_single_pass = mode == "centerline" or is_curve_section or _section_effective_thickness(poly) <= band_pass_width * 1.1
        (wide_sections if mode == "zigzag_fill" else thin_sections if is_single_pass else wide_sections).append(section)

    toolpaths: list[dict[str, Any]] = []
    seq = 0
    # Thin bands → ONE continuous straight centerline per band. A straight sanding stroke
    # is a single linear move; we do NOT chop it at reach-cell boundaries (that produced
    # two collinear passes meeting mid-arm). The robot repositions along the same line as
    # needed — the goal is fewer, whole sections, not more fragments.
    for section in thin_sections:
        poly = _polygon_from_section(section)
        if poly is None or poly.is_empty or poly.geom_type != "Polygon":
            continue
        pending_strokes: list[list[list[float]]] = []
        if section.get("section_strategy") == "between_pockets_vertical":
            from shapely.geometry import LineString

            minx, miny, maxx, maxy = [float(v) for v in poly.bounds]
            cx = (minx + maxx) / 2.0
            reach = (maxy - miny) + band_pass_width + 10.0
            centerline = _clip_line_segment_to_polygon(
                LineString([(cx, miny - reach), (cx, maxy + reach)]),
                poly,
            ) or []
            direction = "Y"
            strategy = "structural_vertical_centerline"
        elif section.get("section_strategy") == "bottom_curved_band":
            # The lower band follows the door's arc. Sanding it wants a few LONG STRAIGHT
            # strokes, not one wobbly polyline: split the band's centerline at the arc apex
            # so each half becomes a single angled stroke, joined by a short connector at
            # the peak. A fixed-y chord is not usable here — it drifts out of the band near
            # the ends and can land inside a pocket.
            centerline = _curved_band_following_centerline(
                poly, band_pass_width, band_offset,
            )
            extra_strokes = _split_curve_at_apex(centerline, band_half)
            direction = "X"
            strategy = "structural_lower_band_centerline"
            if extra_strokes:
                centerline = extra_strokes[0]
                pending_strokes = extra_strokes[1:]
        else:
            centerline, direction, strategy = _oriented_toolpath(
                poly,
                band_pass_width,
                band_offset,
                band_step,
                band_half,
                force_band=bool(section.get("curved") or section.get("toolpath_mode") == "centerline"),
            )
        if len(centerline) < 2:
            continue
        seg_len = sum(math.dist(centerline[i], centerline[i + 1]) for i in range(len(centerline) - 1))
        if seg_len < band_half:  # drop a degenerate sliver
            continue
        for stroke in [centerline, *pending_strokes]:
            if len(stroke) < 2:
                continue
            seq += 1
            toolpaths.append({
                "path_id": f"frame_path_{seq:03d}",
                "source_section_id": section.get("section_id"),
                "region_type": "computed_frame",
                "operation_type": "frame_section_pass",
                "points": stroke,
                "direction": direction,
                "path_strategy": strategy,
                "start_point": stroke[0],
                "end_point": stroke[-1],
            })

    # Wide sections → reach-split + serpentine fill per chunk.
    wide_chunks = _split_sections_by_reach(wide_sections, reach_x_mm, reach_y_mm)
    for tp in _generate_section_toolpaths(wide_chunks, pass_width_mm, offset_mm, overlap_mm):
        seq += 1
        tp["path_id"] = f"frame_path_{seq:03d}"
        toolpaths.append(tp)

    # Preview chunks should match what the operator sees as sections. Wide rectangles
    # still need reach-split verification; thin/curved bands are single-pass sections,
    # so returning them unsplit avoids fake rectangular boxes over curved frame areas.
    chunks = _split_sections_by_reach(wide_sections, reach_x_mm, reach_y_mm)
    chunk_counter = len(chunks)
    for section in thin_sections:
        chunk_counter += 1
        chunk = dict(section)
        chunk["chunk_id"] = f"frame_chunk_{chunk_counter:03d}"
        chunk["parent_section_id"] = section.get("section_id")
        chunk["requires_axis_position"] = False
        chunks.append(chunk)

    rings = _rings_of(frame_geom)

    # Report frame area that ended up with NO pass. Silently dropping a section is the
    # dangerous failure mode: the operator sees a preview that looks plausible while part
    # of the frame is never sanded. It happens on malformed input (e.g. a 3D contour drawn
    # a few mm from its pocket, which shatters the frame into sub-tool fragments), so
    # surface it rather than hide it.
    covered_ids = {tp.get("source_section_id") for tp in toolpaths}
    uncovered = [s for s in sections if s.get("section_id") not in covered_ids]
    uncovered_area = float(sum(float(s.get("area") or 0.0) for s in uncovered))
    if uncovered:
        logger.warning(
            "Table B DXF frame: %d of %d sections produced NO toolpath (%.0f mm2 of frame "
            "left uncovered). Check for obstacles drawn very close together or an offset "
            "larger than the section.",
            len(uncovered), len(sections), uncovered_area,
        )

    return {
        "ok": True,
        "rings": rings,
        "sections": sections,
        "chunks": chunks,
        "toolpaths": toolpaths,
        "outer_area": float(outer.area),
        "frame_area": sum(r["area"] for r in rings),
        "obstacle_count": len(obstacles),
        "uncovered_section_ids": [s.get("section_id") for s in uncovered],
        "uncovered_area": uncovered_area,
    }


