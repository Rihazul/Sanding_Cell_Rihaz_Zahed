import React from 'react';

// Lightweight 2D DXF viewer. Renders parsed closed loops as SVG polygons with
// zoom (wheel), pan (drag), fit-to-screen, hover highlight and click select /
// deselect. Also overlays the computed frame (transparent green + light
// rectangle blocks) and generated centerline toolpaths (dashed red with
// direction arrows and start/end markers). Inline styles only — the project
// ships a frozen Tailwind snapshot that does not compile new utility classes.

// Region categories operators can assign, with their legend colors.
export const DXF_REGION_TYPES = ['outer', 'pocket', 'surface3d', 'frame'];
export const DXF_REGION_META = {
  outer: { label: 'Outer Boundary', color: '#3b82f6' }, // frame blue — outer leftover is frame too
  pocket: { label: 'Pocket', color: '#f59e0b' }, // yellow/orange fill
  surface3d: { label: '3D Contour', color: '#9333ea' }, // purple fill/ring
  frame: { label: 'Frame Level', color: '#3b82f6' }, // blue transparent fill
};

const UNASSIGNED_FILL = '#e2e8f0';
const UNASSIGNED_STROKE = '#334155';
const HOVER_STROKE = '#0284c7';
const SELECTED_STROKE = '#0891b2';
// Bright, high-contrast outline used when a region is highlighted from the
// assignment list (row hover), so the operator can tell which row is which.
const HIGHLIGHT_STROKE = '#f43f5e';
const HIGHLIGHT_STROKE_WIDTH = 5;

// Consistent base stroke width for all DXF geometry so nothing looks faint.
const BASE_STROKE_WIDTH = 1.6;
// Open 3D contour guide lines: solid and clearly visible (never dashed/dotted —
// DXF linetype styling is intentionally ignored in the operator viewer).
const OPEN_PATH_STROKE = '#475569';
const OPEN_PATH_WIDTH = 1.6;
const LINE_HOVER_STROKE = '#0284c7';
const LINE_SELECTED_STROKE = '#db2777';

// Confirmed manual-surface fill colors.
const OPERATION_FILL = {
  pocket_floor: '#f59e0b',
  surface_3d_area: '#9333ea',
  frame_level: '#3b82f6',
  outer_boundary: '#3b82f6', // frame blue — outer leftover is frame too
};

// Lower number = higher priority (subtracts from everything below it). The outer
// boundary is the lowest priority frame region: it fills last, so every assigned
// pocket / 3D / frame region is cut out of it, leaving only the leftover frame.
const REGION_PRIORITY = {
  pocket: 1,
  surface3d: 2,
  frame: 3,
  outer: 4,
};

const OPERATION_PRIORITY = {
  pocket_floor: 1,
  surface_3d_area: 2,
  frame_level: 3,
  outer_boundary: 4,
};

function regionPriority(type) {
  return REGION_PRIORITY[type] ?? 99;
}

function operationPriority(operation) {
  return OPERATION_PRIORITY[operation] ?? 99;
}

const FRAME_FILL = 'rgba(59, 130, 246, 0.18)'; // transparent blue
const FRAME_STROKE = '#3b82f6';
const RECT_FILL = 'rgba(59, 130, 246, 0.10)'; // light transparent blocks
const RECT_STROKE = 'rgba(59, 130, 246, 0.55)';
const TOOLPATH_COLOR = '#ef4444'; // red
const TOOLPATH_SELECTED_COLOR = '#b91c1c';
const POCKET_TOOLPATH_COLOR = '#2563eb'; // blue — Tool 3 rectangular contour
const POCKET_ZIGZAG_COLOR = '#16a34a'; // green — Tool 4 zigzag fill
const CONTOUR_TOOLPATH_COLOR = '#9333ea'; // purple — 3D-contour ring toolpath
const START_MARKER_FILL = '#ffffff';
const START_MARKER_STROKE = '#16a34a';

// One colour per 7th-axis station. Passes sharing a colour are executed from the SAME
// station, so the operator can see at a glance what runs together and how many times the
// axis has to reposition. Frame sections that combine into one reachable run therefore
// appear as a single colour rather than one colour per section.
const STATION_COLORS = [
  '#dc2626', // red
  '#2563eb', // blue
  '#16a34a', // green
  '#d97706', // amber
  '#9333ea', // purple
  '#0891b2', // cyan
  '#db2777', // pink
  '#65a30d', // lime
];
const STATION_UNREACHABLE_COLOR = '#94a3b8'; // grey - no station can reach this pass

const stationAxisBucket = (axisPositionMm) => {
  const axis = Number(axisPositionMm);
  return Number.isFinite(axis) ? Math.round(axis / 10) * 10 : null;
};

export const stationColor = (stationIndex, axisPositionMm = null) => {
  const axisBucket = stationAxisBucket(axisPositionMm);
  if (axisBucket !== null) {
    return STATION_COLORS[Math.abs(Math.round(axisBucket / 10)) % STATION_COLORS.length];
  }
  return stationIndex === null || stationIndex === undefined
    ? STATION_UNREACHABLE_COLOR
    : STATION_COLORS[stationIndex % STATION_COLORS.length];
};

const stationLegendKey = (seg) => {
  const axisBucket = stationAxisBucket(seg.axis7_position_mm);
  if (axisBucket !== null) return `x:${axisBucket}`;
  return seg.station_index === null || seg.station_index === undefined ? null : `i:${seg.station_index}`;
};

// Which tool each pass group belongs to, so the viewer can filter to one tool at a time.
// Tool 4 owns the frame passes AND the pocket zigzag (both are the wide raster tool).
const TOOL_LABELS = {
  tool_3: 'Tool 3 · Pocket edge',
  tool_4: 'Tool 4 · Frame & zigzag',
  tool_1: 'Tool 1 · 3D contour',
};
const TOOL_FILTER_ORDER = ['tool_3', 'tool_4', 'tool_1'];

const controlButtonStyle = {
  cursor: 'pointer',
  padding: '4px 10px',
  fontSize: '12px',
  fontWeight: 600,
  color: '#0f172a',
  background: '#ffffff',
  border: '1px solid #cbd5e1',
  borderRadius: '8px',
};

function computeFitBBox(loops, openPaths, framePolygons) {
  // Frame the meaningful part: prefer closed loops (the actual regions), then
  // open geometry, then the frame. This keeps stray drafting geometry (title
  // blocks, dimension lines, far-off points) from shrinking the part.
  const sources =
    loops.length > 0
      ? loops.map((loop) => loop.points)
      : openPaths.length > 0
      ? openPaths.map((path) => path.points)
      : framePolygons.map((poly) => poly.exterior || []);

  let minX = Infinity;
  let minY = Infinity;
  let maxX = -Infinity;
  let maxY = -Infinity;
  for (const points of sources) {
    for (const point of points) {
      const x = point[0];
      const y = point[1];
      if (x < minX) minX = x;
      if (y < minY) minY = y;
      if (x > maxX) maxX = x;
      if (y > maxY) maxY = y;
    }
  }
  if (!Number.isFinite(minX)) return null;
  return { minX, minY, maxX, maxY };
}

function ringPathD(points) {
  return points.map((point, index) => `${index ? 'L' : 'M'} ${point[0]} ${point[1]}`).join(' ') + ' Z';
}

// Signature of a ring by bbox + area, independent of vertex order/rotation, so
// the same region from two sources maps to one key.
function holeSignature(points) {
  let minX = Infinity;
  let minY = Infinity;
  let maxX = -Infinity;
  let maxY = -Infinity;
  let twiceArea = 0;
  const n = points.length;
  for (let i = 0; i < n; i++) {
    const x = points[i][0];
    const y = points[i][1];
    if (x < minX) minX = x;
    if (y < minY) minY = y;
    if (x > maxX) maxX = x;
    if (y > maxY) maxY = y;
    const [x2, y2] = points[(i + 1) % n];
    twiceArea += x * y2 - x2 * y;
  }
  const area = Math.abs(twiceArea) / 2;
  return `${Math.round(minX)},${Math.round(minY)},${Math.round(maxX)},${Math.round(maxY)},${Math.round(area)}`;
}

// Drop duplicate holes (same region from multiple sources). Subtracting a region
// twice with fill-rule evenodd cancels out and re-fills it — this prevents that.
function dedupeHoles(holes) {
  const seen = new Set();
  const out = [];
  for (const hole of holes) {
    if (!hole || hole.length < 3) continue;
    const key = holeSignature(hole);
    if (seen.has(key)) continue;
    seen.add(key);
    out.push(hole);
  }
  return out;
}

// Keep only the OUTERMOST footprint of each nested group of holes. A hole sitting
// inside another hole would, under fill-rule evenodd, re-fill the inner area — so
// when a region (e.g. the outer boundary) is subtracted from, only each region's
// outer edge should be cut, never its own inner loops.
function outermostHoles(holes) {
  return holes.filter(
    (hole, i) => !holes.some((other, j) => j !== i && other.length >= 3 && polygonNested(hole, other)),
  );
}

// Standard hole cleanup for any evenodd fill: remove exact duplicates, then drop
// any hole nested inside another so nothing re-fills.
function cleanHoles(holes) {
  return outermostHoles(dedupeHoles(holes));
}

// How close (in pixels) the cursor must be to a loop's outline to count as
// "pointing at that line" — makes thin nested contour rings easy to hover/click.
const HIT_TOLERANCE_PX = 10;

function pointInPolygon(x, y, points) {
  let inside = false;
  for (let i = 0, j = points.length - 1; i < points.length; j = i++) {
    const xi = points[i][0];
    const yi = points[i][1];
    const xj = points[j][0];
    const yj = points[j][1];
    const intersect = yi > y !== yj > y && x < ((xj - xi) * (y - yi)) / (yj - yi + 1e-12) + xi;
    if (intersect) inside = !inside;
  }
  return inside;
}

function pointOnPolygonBoundary(x, y, points) {
  const tolerance = 1e-4;
  for (let i = 0, j = points.length - 1; i < points.length; j = i++) {
    if (distToSegment(x, y, points[j][0], points[j][1], points[i][0], points[i][1]) <= tolerance) {
      return true;
    }
  }
  return false;
}

function polygonNested(inner, outer) {
  return (
    inner &&
    inner.length > 0 &&
    inner.every((point) => pointInPolygon(point[0], point[1], outer) || pointOnPolygonBoundary(point[0], point[1], outer))
  );
}


function distToSegment(px, py, ax, ay, bx, by) {
  const dx = bx - ax;
  const dy = by - ay;
  const lenSq = dx * dx + dy * dy;
  let t = lenSq > 0 ? ((px - ax) * dx + (py - ay) * dy) / lenSq : 0;
  t = Math.max(0, Math.min(1, t));
  return Math.hypot(px - (ax + t * dx), py - (ay + t * dy));
}

function distToPolygon(x, y, points) {
  let min = Infinity;
  for (let i = 0, j = points.length - 1; i < points.length; j = i++) {
    const d = distToSegment(x, y, points[j][0], points[j][1], points[i][0], points[i][1]);
    if (d < min) min = d;
  }
  return min;
}

// Distance to an OPEN polyline (no wrap-around from last vertex to first).
function distToPolyline(x, y, points) {
  let min = Infinity;
  for (let i = 0; i < points.length - 1; i++) {
    const d = distToSegment(x, y, points[i][0], points[i][1], points[i + 1][0], points[i + 1][1]);
    if (d < min) min = d;
  }
  return min;
}

// Nearest open guide line within tolerance (for "build surface" line selection).
function lineAtPoint(openPaths, worldX, worldY, toleranceWorld) {
  let nearest = null;
  let nearestDist = Infinity;
  for (const path of openPaths) {
    if (!path.points || path.points.length < 2) continue;
    const d = distToPolyline(worldX, worldY, path.points);
    if (d < nearestDist) {
      nearestDist = d;
      nearest = path;
    }
  }
  return nearest && nearestDist <= toleranceWorld ? nearest : null;
}

// The loop under the cursor, resolved for thin nested contour rings:
//   1) if the cursor is near any ring's OUTLINE (within tolerance), pick the
//      loop with the NEAREST edge — you're pointing at that line, regardless of
//      how thin the band is. This is what makes every ring hoverable;
//   2) otherwise (in open space far from edges) pick the SMALLEST loop that
//      CONTAINS the point — e.g. clicking the middle of a large pocket floor.
function loopAtPoint(loops, worldX, worldY, toleranceWorld) {
  let nearest = null;
  let nearestDist = Infinity;
  for (const loop of loops) {
    const d = distToPolygon(worldX, worldY, loop.points);
    if (d < nearestDist || (d === nearestDist && nearest && loop.area < nearest.area)) {
      nearestDist = d;
      nearest = loop;
    }
  }
  if (nearest && nearestDist <= toleranceWorld) return nearest;

  let containing = null;
  for (const loop of loops) {
    if (pointInPolygon(worldX, worldY, loop.points)) {
      if (!containing || loop.area < containing.area) containing = loop;
    }
  }
  return containing || null;
}

/**
 * @param {{
 *   loops?: any[],
 *   openPaths?: any[],
 *   selectedIds?: string[],
 *   highlightId?: string | null,
 *   assignments?: Record<string, string>,
 *   onToggleSelect?: (loop: any) => void,
 *   selectionMode?: 'loop' | 'line' | 'ring',
 *   selectedLineIds?: string[],
 *   onToggleLine?: (line: any) => void,
 *   surfacePreview?: { outer: number[][], holes?: number[][][] } | null,
 *   ringPreview?: { outer: number[][], hole: number[][] } | null,
 *   manualSurfaces?: any[],
 *   framePolygons?: any[],
 *   frameAreaPolygons?: any[],
 *   frameRectangles?: any[],
 *   frameToolpaths?: any[],
 *   pocketToolpaths?: any[],
 *   pocketZigzag?: any[],
 *   contourToolpaths?: any[],
 *   frameZigzag?: any[],
 *   frameSections?: any[],
 *   frameChunks?: any[],
 *   frameSectionPaths?: any[],
 *   showFrame?: boolean,
 *   showToolpaths?: boolean,
 *   selectedToolpathId?: string | null,
 *   selectedFrameSectionId?: string | null,
 *   selectedFramePathId?: string | null,
 *   onSelectToolpath?: (rectId: string) => void,
 * }} props
 */
export default function Dxf2DViewer({
  loops = [],
  openPaths = [],
  selectedIds = [],
  highlightId = null,
  assignments = {},
  onToggleSelect,
  selectionMode = 'loop',
  selectedLineIds = [],
  onToggleLine,
  surfacePreview = null,
  ringPreview = null,
  manualSurfaces = [],
  framePolygons = [],
  frameAreaPolygons = [],
  frameRectangles = [],
  frameToolpaths = [],
  pocketToolpaths = [],
  pocketZigzag = [],
  contourToolpaths = [],
  frameZigzag = [],
  frameSections = [],
  frameChunks = [],
  frameSectionPaths = [],
  showFrame = true,
  showToolpaths = true,
  selectedToolpathId = null,
  selectedFrameSectionId = null,
  selectedFramePathId = null,
  onSelectToolpath,
}) {
  const containerRef = React.useRef(null);
  const dragRef = React.useRef(null);
  const lastFitSigRef = React.useRef(null);

  const [viewport, setViewport] = React.useState({ w: 800, h: 480 });
  // World -> screen: screenX = tx - s*worldX ; screenY = ty - s*worldY.
  // X is flipped so the machine-frame geometry (origin at right, +X pointing
  // left) renders un-mirrored: the origin sits at the model's right edge and the
  // +X axis points left, matching the drawing the operator expects.
  const [transform, setTransform] = React.useState({ s: 1, tx: 400, ty: 240 });
  const [hoveredId, setHoveredId] = React.useState(null);
  const [hoveredLineId, setHoveredLineId] = React.useState(null);
  const [isPanning, setIsPanning] = React.useState(false);
  // Which tool's passes to show. Colour = 7th-axis station, so showing one tool at a time
  // keeps station colours from colliding across tools and lets the legend group by stop.
  const [toolFilter, setToolFilter] = React.useState('all'); // 'all' | 'tool_3' | 'tool_4' | 'tool_1'
  // The station legend is a floating panel the operator can collapse (so it stops covering
  // the model) and drag out of the way.
  const [legendCollapsed, setLegendCollapsed] = React.useState(false);
  const [legendPos, setLegendPos] = React.useState({ x: 8, y: 8 });
  const legendDragRef = React.useRef(null);
  // World coordinate under the cursor (for the live readout). null when off-canvas.
  const [cursorWorld, setCursorWorld] = React.useState(null);

  const toScreen = React.useCallback(
    (x, y) => [transform.tx - transform.s * x, transform.ty - transform.s * y],
    [transform],
  );

  React.useEffect(() => {
    const el = containerRef.current;
    if (!el) return undefined;
    const update = () => setViewport({ w: el.clientWidth || 800, h: el.clientHeight || 480 });
    update();
    const observer = new ResizeObserver(update);
    observer.observe(el);
    return () => observer.disconnect();
  }, []);

  const fitToScreen = React.useCallback(() => {
    // Read the container's live size so fit is correct even if the viewport
    // state hasn't caught up to a layout/resize yet.
    const el = containerRef.current;
    const w = (el && el.clientWidth) || viewport.w;
    const h = (el && el.clientHeight) || viewport.h;
    const bbox = computeFitBBox(loops, openPaths, framePolygons);
    if (!bbox) {
      setTransform({ s: 1, tx: w / 2, ty: h / 2 });
      return;
    }
    const bw = Math.max(bbox.maxX - bbox.minX, 1e-6);
    const bh = Math.max(bbox.maxY - bbox.minY, 1e-6);
    const s = 0.9 * Math.min(w / bw, h / bh);
    const cx = (bbox.minX + bbox.maxX) / 2;
    const cy = (bbox.minY + bbox.maxY) / 2;
    // screenX = tx - s*cx must equal w/2  ->  tx = w/2 + s*cx (X flipped).
    setTransform({ s, tx: w / 2 + s * cx, ty: h / 2 + s * cy });
  }, [loops, openPaths, framePolygons, viewport]);

  // Auto-fit whenever a new drawing (loops or open paths) or a resize arrives.
  React.useEffect(() => {
    if (!loops.length && !openPaths.length) return undefined;
    const signature =
      `${loops.map((loop) => loop.entity_id).join('|')}` +
      `#${openPaths.map((path) => path.entity_id).join('|')}@${viewport.w}x${viewport.h}`;
    if (lastFitSigRef.current !== signature) {
      lastFitSigRef.current = signature;
      // Defer one frame so the container has its final laid-out size.
      const id = requestAnimationFrame(() => fitToScreen());
      return () => cancelAnimationFrame(id);
    }
    return undefined;
  }, [loops, openPaths, viewport, fitToScreen]);

  // Native non-passive wheel listener so preventDefault works for zoom.
  React.useEffect(() => {
    const el = containerRef.current;
    if (!el) return undefined;
    const onWheel = (event) => {
      event.preventDefault();
      const rect = el.getBoundingClientRect();
      const mx = event.clientX - rect.left;
      const my = event.clientY - rect.top;
      setTransform((t) => {
        const factor = event.deltaY < 0 ? 1.1 : 1 / 1.1;
        const ns = Math.max(1e-4, Math.min(1e6, t.s * factor));
        const worldX = (t.tx - mx) / t.s;
        const worldY = (t.ty - my) / t.s;
        return { s: ns, tx: mx + ns * worldX, ty: my + ns * worldY };
      });
    };
    el.addEventListener('wheel', onWheel, { passive: false });
    return () => el.removeEventListener('wheel', onWheel);
  }, []);

  const zoomBy = (factor) => {
    setTransform((t) => {
      const ns = Math.max(1e-4, Math.min(1e6, t.s * factor));
      const cx = viewport.w / 2;
      const cy = viewport.h / 2;
      const worldX = (t.tx - cx) / t.s;
      const worldY = (t.ty - cy) / t.s;
      return { s: ns, tx: cx + ns * worldX, ty: cy + ns * worldY };
    });
  };

  // Drag the legend panel by its title bar. Positions are clamped to the viewer so it can
  // never be dragged fully off-screen.
  const onLegendDragStart = (event) => {
    event.stopPropagation();
    legendDragRef.current = { startX: event.clientX, startY: event.clientY, origX: legendPos.x, origY: legendPos.y };
    const move = (e) => {
      if (!legendDragRef.current) return;
      const dx = e.clientX - legendDragRef.current.startX;
      const dy = e.clientY - legendDragRef.current.startY;
      const nx = Math.max(0, Math.min(viewport.w - 60, legendDragRef.current.origX + dx));
      const ny = Math.max(0, Math.min(viewport.h - 24, legendDragRef.current.origY + dy));
      setLegendPos({ x: nx, y: ny });
    };
    const up = () => {
      legendDragRef.current = null;
      window.removeEventListener('mousemove', move);
      window.removeEventListener('mouseup', up);
    };
    window.addEventListener('mousemove', move);
    window.addEventListener('mouseup', up);
  };

  const toWorld = (clientX, clientY) => {
    const rect = containerRef.current ? containerRef.current.getBoundingClientRect() : { left: 0, top: 0 };
    const sx = clientX - rect.left;
    const sy = clientY - rect.top;
    // Inverse of screenX = tx - s*worldX (X flipped) and screenY = ty - s*worldY.
    return [(transform.tx - sx) / transform.s, (transform.ty - sy) / transform.s];
  };

  const loopUnderCursor = (clientX, clientY) => {
    const [wx, wy] = toWorld(clientX, clientY);
    return loopAtPoint(loops, wx, wy, HIT_TOLERANCE_PX / transform.s);
  };

  const lineUnderCursor = (clientX, clientY) => {
    const [wx, wy] = toWorld(clientX, clientY);
    return lineAtPoint(openPaths, wx, wy, HIT_TOLERANCE_PX / transform.s);
  };

  const startPan = (event) => {
    dragRef.current = {
      startX: event.clientX,
      startY: event.clientY,
      origTx: transform.tx,
      origTy: transform.ty,
      moved: false,
    };
    setIsPanning(true);
  };

  const movePan = (event) => {
    // Live cursor coordinate for the readout (updated even while panning).
    setCursorWorld(toWorld(event.clientX, event.clientY));
    const drag = dragRef.current;
    if (drag) {
      const dx = event.clientX - drag.startX;
      const dy = event.clientY - drag.startY;
      if (!drag.moved && Math.hypot(dx, dy) > 4) drag.moved = true;
      if (drag.moved) {
        setTransform((t) => ({ ...t, tx: drag.origTx + dx, ty: drag.origTy + dy }));
        return;
      }
    }
    // Not dragging: update hover. In "line" mode we hover open guide lines and
    // ignore loops; in "loop" mode we hover closed loops.
    if (selectionMode === 'line') {
      const line = lineUnderCursor(event.clientX, event.clientY);
      const next = line ? line.entity_id : null;
      setHoveredLineId((prev) => (prev === next ? prev : next));
      setHoveredId((prev) => (prev === null ? prev : null));
    } else {
      const loop = loopUnderCursor(event.clientX, event.clientY);
      const next = loop ? loop.entity_id : null;
      setHoveredId((prev) => (prev === next ? prev : next));
      setHoveredLineId((prev) => (prev === null ? prev : null));
    }
  };

  const endPan = (event) => {
    const drag = dragRef.current;
    dragRef.current = null;
    setIsPanning(false);
    if (!(drag && !drag.moved && event && typeof event.clientX === 'number')) return;

    // A click (negligible drag). In "line" mode select the nearest guide line
    // (prioritized over closed loops); in "loop" mode select the loop.
    if (selectionMode === 'line') {
      const line = lineUnderCursor(event.clientX, event.clientY);
      console.log('[DXF Viewer] line click hit-test', {
        picked_line: line ? line.entity_id : null,
        dxf_type: line ? line.dxf_type : null,
      });
      if (line) onToggleLine?.(line);
      return;
    }

    const [wx, wy] = toWorld(event.clientX, event.clientY);
    const containing = loops.filter((loop) => pointInPolygon(wx, wy, loop.points));
    const loop = loopAtPoint(loops, wx, wy, HIT_TOLERANCE_PX / transform.s);
    console.log('[DXF Viewer] click hit-test', {
      loops_under_cursor: containing.length,
      picked: loop ? loop.loop_id || loop.entity_id : null,
    });
    if (loop) onToggleSelect?.(loop);
  };

  const handleMouseLeave = () => {
    dragRef.current = null;
    setIsPanning(false);
    setHoveredId(null);
    setHoveredLineId(null);
    setCursorWorld(null);
  };

  const hoveredLoop = loops.find((loop) => loop.entity_id === hoveredId) || null;
  const hoveredLine = openPaths.find((path) => path.entity_id === hoveredLineId) || null;
  // Draw largest loops first (bottom) and smallest last (on top) so thin strips
  // aren't hidden behind the outer contour; keep the selected loop on top so its
  // outline is always visible.
  const renderLoops = [...loops].sort((a, b) => {
    const aSel = selectedIds.includes(a.entity_id) ? 1 : 0;
    const bSel = selectedIds.includes(b.entity_id) ? 1 : 0;
    if (aSel !== bSel) return aSel - bSel;
    return b.area - a.area;
  });

  const higherPriorityHoles = (outer, targetPriority, sourceSurfaceId = null) => {
    if (targetPriority >= 99 || !outer || outer.length < 3) return [];
    const loopHoles = loops
      .filter((loop) => regionPriority(assignments[loop.entity_id]) < targetPriority && polygonNested(loop.points, outer))
      .map((loop) => loop.points);
    const surfaceHoles = manualSurfaces
      .filter(
        (surface) =>
          surface.id !== sourceSurfaceId &&
          operationPriority(surface.assigned_operation) < targetPriority &&
          polygonNested(surface.outer_points || surface.points || [], outer),
      )
      // Cut the surface's FULL outer footprint only — never its inner holes. A
      // hole nested inside the already-cut footprint would re-fill under evenodd,
      // bleeding the lower-priority fill back into an assigned region.
      .map((surface) => surface.outer_points || surface.points || []);
    return [...loopHoles, ...surfaceHoles];
  };
  const renderOpenPath = (path) => {
    const isLineSelected = selectedLineIds.includes(path.entity_id);
    const isLineHovered = hoveredLineId === path.entity_id;
    const isHighlighted = highlightId != null && path.entity_id === highlightId;
    const stroke = isHighlighted
      ? HIGHLIGHT_STROKE
      : isLineSelected ? LINE_SELECTED_STROKE : isLineHovered ? LINE_HOVER_STROKE : OPEN_PATH_STROKE;
    const strokeWidth = isHighlighted ? HIGHLIGHT_STROKE_WIDTH : isLineSelected ? 4 : isLineHovered ? 3.2 : OPEN_PATH_WIDTH;
    return (
      <polyline
        key={`open-${path.entity_id}`}
        points={path.points.map((point) => `${point[0]},${point[1]}`).join(' ')}
        fill="none"
        stroke={stroke}
        strokeWidth={strokeWidth}
        strokeLinejoin="round"
        strokeLinecap="round"
        vectorEffect="non-scaling-stroke"
        pointerEvents="none"
      />
    );
  };

  // A pass group is drawn when toolpaths are on AND the tool filter admits its tool.
  const toolVisible = (tool) => showToolpaths && (toolFilter === 'all' || toolFilter === tool);

  const isToolpathStartSegment = (seg) => /_0$/.test(String(seg.id || '')) || seg.seq === 0;
  // Pocket toolpath overlays (Tool 3 rectangular contour): directional line + a
  // mid-segment arrow showing travel direction, constant pixel size like above.
  const pocketToolpathOverlays = toolVisible('tool_3')
    ? pocketToolpaths.map((seg) => {
        const [x0, y0] = toScreen(seg.start[0], seg.start[1]);
        const [x1, y1] = toScreen(seg.end[0], seg.end[1]);
        const dx = x1 - x0;
        const dy = y1 - y0;
        const len = Math.hypot(dx, dy) || 1;
        const ux = dx / len;
        const uy = dy / len;
        const arrowSize = 11;
        // Arrowhead placed at the segment midpoint, pointing along travel.
        const mx = (x0 + x1) / 2;
        const my = (y0 + y1) / 2;
        const tipX = mx + arrowSize * 0.5 * ux;
        const tipY = my + arrowSize * 0.5 * uy;
        const baseX = tipX - arrowSize * ux;
        const baseY = tipY - arrowSize * uy;
        const px = -uy;
        const py = ux;
        const halfW = arrowSize * 0.5;
        const arrow = `${tipX},${tipY} ${baseX + px * halfW},${baseY + py * halfW} ${baseX - px * halfW},${baseY - py * halfW}`;
        return { seg, x0, y0, x1, y1, arrow, isStart: isToolpathStartSegment(seg) };
      })
    : [];

  // 3D-contour ring toolpath overlays (purple rectangular contour).
  const contourToolpathOverlays = toolVisible('tool_1')
    ? contourToolpaths.map((seg) => {
        const [x0, y0] = toScreen(seg.start[0], seg.start[1]);
        const [x1, y1] = toScreen(seg.end[0], seg.end[1]);
        const dx = x1 - x0;
        const dy = y1 - y0;
        const len = Math.hypot(dx, dy) || 1;
        const ux = dx / len;
        const uy = dy / len;
        const arrowSize = 11;
        const mx = (x0 + x1) / 2;
        const my = (y0 + y1) / 2;
        const tipX = mx + arrowSize * 0.5 * ux;
        const tipY = my + arrowSize * 0.5 * uy;
        const baseX = tipX - arrowSize * ux;
        const baseY = tipY - arrowSize * uy;
        const px = -uy;
        const py = ux;
        const halfW = arrowSize * 0.5;
        const arrow = `${tipX},${tipY} ${baseX + px * halfW},${baseY + py * halfW} ${baseX - px * halfW},${baseY - py * halfW}`;
        return { seg, x0, y0, x1, y1, arrow, isStart: isToolpathStartSegment(seg) };
      })
    : [];

  // Tool 4 zigzag fill overlays: thin green passes with a mid-segment travel arrow;
  // start marker at the first point (bottom-right).
  const pocketZigzagOverlays = toolVisible('tool_4')
    ? pocketZigzag.map((seg) => {
        const [x0, y0] = toScreen(seg.start[0], seg.start[1]);
        const [x1, y1] = toScreen(seg.end[0], seg.end[1]);
        const dx = x1 - x0;
        const dy = y1 - y0;
        const len = Math.hypot(dx, dy) || 1;
        const ux = dx / len;
        const uy = dy / len;
        const arrowSize = 9;
        const mx = (x0 + x1) / 2;
        const my = (y0 + y1) / 2;
        const tipX = mx + arrowSize * 0.5 * ux;
        const tipY = my + arrowSize * 0.5 * uy;
        const baseX = tipX - arrowSize * ux;
        const baseY = tipY - arrowSize * uy;
        const px = -uy;
        const py = ux;
        const halfW = arrowSize * 0.5;
        const arrow = `${tipX},${tipY} ${baseX + px * halfW},${baseY + py * halfW} ${baseX - px * halfW},${baseY - py * halfW}`;
        return { seg, x0, y0, x1, y1, arrow, isStart: isToolpathStartSegment(seg) };
      })
    : [];

  // Frame Tool 4 zigzag overlays: red passes (no offset, from the origin) with a
  // mid-segment travel arrow and a start marker at the origin.
  const frameZigzagOverlays = toolVisible('tool_4')
    ? frameZigzag.map((seg) => {
        const [x0, y0] = toScreen(seg.start[0], seg.start[1]);
        const [x1, y1] = toScreen(seg.end[0], seg.end[1]);
        const dx = x1 - x0;
        const dy = y1 - y0;
        const len = Math.hypot(dx, dy) || 1;
        const ux = dx / len;
        const uy = dy / len;
        const arrowSize = 9;
        const mx = (x0 + x1) / 2;
        const my = (y0 + y1) / 2;
        const tipX = mx + arrowSize * 0.5 * ux;
        const tipY = my + arrowSize * 0.5 * uy;
        const baseX = tipX - arrowSize * ux;
        const baseY = tipY - arrowSize * uy;
        const px = -uy;
        const py = ux;
        const halfW = arrowSize * 0.5;
        const arrow = `${tipX},${tipY} ${baseX + px * halfW},${baseY + py * halfW} ${baseX - px * halfW},${baseY - py * halfW}`;
        return { seg, x0, y0, x1, y1, arrow, isStart: isToolpathStartSegment(seg) };
      })
    : [];

  // Frame-section preview paths: red passes per chunk (single centerline or a
  // radius-based zigzag), rendered as connected segments with travel arrows and a
  // start marker at the first point of each path.
  const frameSectionPathOverlays = toolVisible('tool_4')
    ? frameSectionPaths.flatMap((path) => {
        const pts = path.points || [];
        const segs = [];
        for (let i = 0; i < pts.length - 1; i++) {
          const [x0, y0] = toScreen(pts[i][0], pts[i][1]);
          const [x1, y1] = toScreen(pts[i + 1][0], pts[i + 1][1]);
          const dx = x1 - x0;
          const dy = y1 - y0;
          const len = Math.hypot(dx, dy) || 1;
          const ux = dx / len;
          const uy = dy / len;
          const arrowSize = 11;
          const mx = (x0 + x1) / 2;
          const my = (y0 + y1) / 2;
          const tipX = mx + arrowSize * 0.5 * ux;
          const tipY = my + arrowSize * 0.5 * uy;
          const baseX = tipX - arrowSize * ux;
          const baseY = tipY - arrowSize * uy;
          const px = -uy;
          const py = ux;
          const halfW = arrowSize * 0.5;
          const arrow = `${tipX},${tipY} ${baseX + px * halfW},${baseY + py * halfW} ${baseX - px * halfW},${baseY - py * halfW}`;
          segs.push({
            key: `${path.path_id}_${i}`, pathId: path.path_id, x0, y0, x1, y1, arrow, isStart: i === 0,
            // Colour by 7th-axis station so passes that run together share a colour.
            color: stationColor(path.station_index, path.axis7_position_mm),
            stationIndex: path.station_index ?? null,
          });
        }
        return segs;
      })
    : [];

  // Per-tool station summary for the legend: for each tool, its distinct 7th-axis stops
  // in run order, with the axis position and how many passes each stop runs. Built from
  // whatever pass groups belong to that tool. Only tools admitted by the filter appear.
  const toolStationLegend = React.useMemo(() => {
    if (!showToolpaths) return [];
    const groups = {
      tool_3: pocketToolpaths,
      tool_4: [...pocketZigzag, ...frameSectionPaths, ...frameZigzag],
      tool_1: contourToolpaths,
    };
    return TOOL_FILTER_ORDER
      .filter((tool) => (toolFilter === 'all' || toolFilter === tool) && (groups[tool] || []).length > 0)
      .map((tool) => {
        const byStation = new Map();
        let hasUnreachable = false;
        for (const seg of groups[tool]) {
          const key = stationLegendKey(seg);
          if (key === null) { hasUnreachable = true; continue; }
          if (!byStation.has(key)) {
            byStation.set(key, {
              count: 0,
              axis: seg.axis7_position_mm,
              stationIndex: seg.station_index,
              color: stationColor(seg.station_index, seg.axis7_position_mm),
            });
          }
          byStation.get(key).count += 1;
        }
        const stops = [...byStation.entries()]
          .sort((a, b) => {
            const ax = Number(a[1].axis);
            const bx = Number(b[1].axis);
            if (Number.isFinite(ax) && Number.isFinite(bx)) return ax - bx;
            return Number(a[1].stationIndex ?? 0) - Number(b[1].stationIndex ?? 0);
          })
          .map(([key, info]) => ({ key, si: info.stationIndex, count: info.count, axis: info.axis, color: info.color }));
        return { tool, label: TOOL_LABELS[tool] || tool, stops, hasUnreachable, planned: byStation.size > 0 };
      })
      .filter((entry) => entry.stops.length > 0 || entry.hasUnreachable);
  }, [showToolpaths, toolFilter, pocketToolpaths, pocketZigzag, frameSectionPaths, frameZigzag, contourToolpaths]);

  // The outer boundary fill should follow the real closed door loop when available.
  // Open guide paths can overhang the part edge; use them only as a fallback for DXFs
  // that do not provide any closed loop.
  const outerBoundaryRect = React.useMemo(() => {
    let minX = Infinity;
    let minY = Infinity;
    let maxX = -Infinity;
    let maxY = -Infinity;
    const consume = (pts) => {
      for (const p of pts || []) {
        if (p[0] < minX) minX = p[0];
        if (p[1] < minY) minY = p[1];
        if (p[0] > maxX) maxX = p[0];
        if (p[1] > maxY) maxY = p[1];
      }
    };
    if (loops.length > 0) {
      const largestLoop = [...loops].sort((a, b) => (b.area || 0) - (a.area || 0))[0];
      consume(largestLoop.points);
    } else if (framePolygons.length > 0) {
      framePolygons.forEach((poly) => consume(poly.exterior || []));
    } else {
      openPaths.forEach((path) => consume(path.points));
    }
    if (!Number.isFinite(minX)) return null;
    return [
      [minX, minY],
      [maxX, minY],
      [maxX, maxY],
      [minX, maxY],
    ];
  }, [loops, openPaths, framePolygons]);

  // Outer boundary is active once any loop is tagged 'outer' or a line-built
  // surface is assigned as the outer boundary.
  const outerBoundaryActive =
    Object.values(assignments).includes('outer') ||
    manualSurfaces.some((surface) => surface.assigned_operation === 'outer_boundary');

  // Is the region highlighted from the list an outer boundary? Its highlight is
  // handled by the background layer (bright leftover fill + outer-perimeter ring)
  // instead of stroking each region's outline — otherwise the outline would trace
  // the window holes and look like the inner regions are part of the outer boundary.
  const outerBoundaryHighlighted =
    highlightId != null &&
    (assignments[highlightId] === 'outer' ||
      manualSurfaces.some(
        (surface) => surface.id === highlightId && surface.assigned_operation === 'outer_boundary',
      ));

  return (
    <div style={{ display: 'flex', flexDirection: 'column', gap: '8px', width: '100%' }}>
      <div
        ref={containerRef}
        onMouseDown={startPan}
        onMouseMove={movePan}
        onMouseUp={endPan}
        onMouseLeave={handleMouseLeave}
        style={{
          position: 'relative',
          width: '100%',
          height: '100%',
          minHeight: '480px',
          border: '1px solid #cbd5e1',
          borderRadius: '12px',
          background: '#f8fafc',
          overflow: 'hidden',
          cursor: isPanning ? 'grabbing' : 'grab',
          touchAction: 'none',
          userSelect: 'none',
        }}
      >
        {loops.length === 0 && openPaths.length === 0 && framePolygons.length === 0 ? (
          <div
            style={{
              position: 'absolute',
              inset: 0,
              display: 'flex',
              alignItems: 'center',
              justifyContent: 'center',
              color: '#64748b',
              fontSize: '13px',
              textAlign: 'center',
              padding: '16px',
            }}
          >
            No DXF geometry to display yet. Upload a .dxf file in the DXF Upload panel.
          </div>
        ) : (
          <svg style={{ position: 'absolute', inset: 0, width: '100%', height: '100%', display: 'block' }}>
            {/* World-space layer: DXF geometry + frame fills (scale with zoom). */}
            <g transform={`translate(${transform.tx} ${transform.ty}) scale(${-transform.s} ${-transform.s})`}>              {/* Outer-boundary background fill: the full part extent, frame blue,
                  with every assigned region cut out. Drawn FIRST (bottom) so all
                  regions and outlines stay visible on top, and so the leftover
                  frame reads as one connected area out to the true model edge. */}
              {outerBoundaryActive &&
                outerBoundaryRect &&
                (() => {
                  const outerHoles = cleanHoles(
                    higherPriorityHoles(outerBoundaryRect, operationPriority('outer_boundary')),
                  );
                  const outerFillD =
                    ringPathD(outerBoundaryRect) + outerHoles.map((hole) => ' ' + ringPathD(hole)).join('');
                  return (
                    <>
                      <path
                        d={outerFillD}
                        fill={OPERATION_FILL.outer_boundary}
                        fillOpacity={outerBoundaryHighlighted ? 0.62 : 0.42}
                        fillRule="evenodd"
                        stroke="none"
                        pointerEvents="none"
                      />
                      {/* When highlighted from the list, trace the WHOLE leftover-frame
                          boundary — model perimeter plus the outer edge of every inner
                          ring region — so it's clear the outer boundary is everything
                          outside the inner sections, out to the model edge. */}
                      {outerBoundaryHighlighted && (
                        <path
                          d={outerFillD}
                          fill="none"
                          fillRule="evenodd"
                          stroke={HIGHLIGHT_STROKE}
                          strokeWidth={HIGHLIGHT_STROKE_WIDTH}
                          strokeLinejoin="round"
                          vectorEffect="non-scaling-stroke"
                          pointerEvents="none"
                        />
                      )}
                    </>
                  );
                })()}
              {/* Open guide lines are drawn under loops in closed-loop mode. In line-build
                  mode they are redrawn above loops below, so outer fills never hide them. */}
              {selectionMode !== 'line' && openPaths.map(renderOpenPath)}
              {/* Largest first (bottom) → smallest last (on top). Pointer events
                  are handled at the container level (smallest loop under cursor),
                  so the outer contour never blocks an inner pocket loop. */}
              {renderLoops.map((loop) => {
                const assignedType = assignments[loop.entity_id];
                const meta = assignedType ? DXF_REGION_META[assignedType] : null;
                const isSelected = selectedIds.includes(loop.entity_id);
                const isHovered = hoveredId === loop.entity_id;
                const pointsAttr = loop.points.map((point) => `${point[0]},${point[1]}`).join(' ');

                // Emphasize the OUTLINE for hover/selection (fill barely changes)
                // so the specific ring is clear even when rectangles are nested.
                const isOuterBoundary = assignedType === 'outer';
                // The outer-boundary fill is drawn once as the full-extent background
                // layer above, so the outer loop itself stays outline-only here (no
                // double-opacity stacking).
                // Outer-boundary loops are highlighted via the background layer, not
                // by stroking here (their outline would trace the window holes).
                const isHighlighted = highlightId != null && loop.entity_id === highlightId && !isOuterBoundary;
                const fill = meta ? meta.color : UNASSIGNED_FILL;
                const fillOpacity = isHighlighted ? 0.6 : isOuterBoundary ? 0 : meta ? 0.42 : isHovered ? 0.34 : 0.26;
                const stroke = isHighlighted
                  ? HIGHLIGHT_STROKE
                  : isSelected
                  ? SELECTED_STROKE
                  : isHovered
                  ? HOVER_STROKE
                  : meta
                  ? meta.color
                  : UNASSIGNED_STROKE;
                const strokeWidth = isHighlighted
                  ? HIGHLIGHT_STROKE_WIDTH
                  : isOuterBoundary
                  ? (isSelected ? 4 : isHovered ? 3 : 2.6)
                  : isSelected ? 4 : isHovered ? 3 : BASE_STROKE_WIDTH;
                // Any inner loop the user has selected or already assigned reads as
                // a hole in the surrounding loop. This makes selecting the outer
                // region of a ring show as a band around the inner region, instead
                // of the outer fill covering the whole area.
                const nestedHoles = loops
                  .filter(
                    (other) =>
                      other.entity_id !== loop.entity_id &&
                      other.area < loop.area &&
                      (selectedIds.includes(other.entity_id) || assignments[other.entity_id]) &&
                      polygonNested(other.points, loop.points),
                  )
                  .map((other) => other.points);
                const displayHoles = cleanHoles([
                  ...higherPriorityHoles(loop.points, regionPriority(assignedType)),
                  ...nestedHoles,
                ]);
                const pathD = ringPathD(loop.points) + displayHoles.map((hole) => ' ' + ringPathD(hole)).join('');

                return (
                  <path
                    key={loop.entity_id}
                    d={pathD}
                    fill={fill}
                    fillOpacity={fillOpacity}
                    fillRule="evenodd"
                    stroke={stroke}
                    strokeWidth={strokeWidth}
                    strokeLinejoin="round"
                    vectorEffect="non-scaling-stroke"
                    pointerEvents="none"
                  />
                );
              })}

              {showFrame &&
                frameRectangles.map((rect) => (
                  <polygon
                    key={`rect-${rect.rect_id}`}
                    points={`${rect.min_x},${rect.min_y} ${rect.max_x},${rect.min_y} ${rect.max_x},${rect.max_y} ${rect.min_x},${rect.max_y}`}
                    fill={RECT_FILL}
                    stroke={RECT_STROKE}
                    strokeWidth={1}
                    strokeDasharray="4 3"
                    vectorEffect="non-scaling-stroke"
                    pointerEvents="none"
                  />
                ))}

              {showFrame &&
                framePolygons.map((poly, index) => {
                  let d = ringPathD(poly.exterior || []);
                  for (const hole of poly.holes || []) d += ' ' + ringPathD(hole);
                  return (
                    <path
                      key={`frame-${index}`}
                      d={d}
                      fill={FRAME_FILL}
                      fillRule="evenodd"
                      stroke={FRAME_STROKE}
                      strokeWidth={1.5}
                      vectorEffect="non-scaling-stroke"
                      pointerEvents="none"
                    />
                  );
                })}

              {/* Backend-computed frame area (outer − pockets − 3D), curve preserved.
                  A verification overlay: dashed magenta outline, no fill, so it sits on
                  top of the existing geometry without obscuring it. */}
              {frameAreaPolygons.map((poly, index) => {
                let d = ringPathD(poly.exterior || []);
                for (const hole of poly.holes || []) d += ' ' + ringPathD(hole);
                return (
                  <path
                    key={`frame-area-${index}`}
                    d={d}
                    fill="none"
                    fillRule="evenodd"
                    stroke="#d946ef"
                    strokeWidth={2}
                    strokeDasharray="6 4"
                    vectorEffect="non-scaling-stroke"
                    pointerEvents="none"
                  />
                );
              })}

              {/* Confirmed manual surfaces.
                  Holes (subtracted inner regions) stay empty via fill-rule evenodd,
                  so no two confirmed surfaces overlap. */}
              {manualSurfaces.map((surface) => {
                const outer = surface.outer_points || surface.points || [];
                if (outer.length < 3) return null;
                const color = OPERATION_FILL[surface.assigned_operation] || '#64748b';
                const isOuterBoundary = surface.assigned_operation === 'outer_boundary';
                // Outer-boundary surfaces are highlighted via the background layer, not
                // by stroking here (their outline would trace the window holes).
                const isHighlighted = highlightId != null && surface.id === highlightId && !isOuterBoundary;
                const displayHoles = cleanHoles([
                  ...(surface.holes || []),
                  ...higherPriorityHoles(outer, operationPriority(surface.assigned_operation), surface.id),
                ]);
                let d = ringPathD(outer);
                for (const hole of displayHoles) d += ' ' + ringPathD(hole);
                return (
                  <path
                    key={`surface-${surface.id}`}
                    d={d}
                    // Outer-boundary fill comes from the full-extent background layer,
                    // so a line-built outer surface is outline-only here.
                    fill={isOuterBoundary ? 'none' : color}
                    fillOpacity={isOuterBoundary ? 0 : isHighlighted ? 0.6 : 0.4}
                    fillRule="evenodd"
                    stroke={isHighlighted ? HIGHLIGHT_STROKE : color}
                    strokeWidth={isHighlighted ? HIGHLIGHT_STROKE_WIDTH : isOuterBoundary ? 2.6 : 2}
                    strokeLinejoin="round"
                    vectorEffect="non-scaling-stroke"
                    pointerEvents="none"
                  />
                );
              })}

              {/* Temporary closed-surface preview (transparent yellow, not saved).
                  Contained regions are shown as empty holes via fill-rule evenodd. */}
              {surfacePreview && surfacePreview.outer && surfacePreview.outer.length >= 3 && (
                <path
                  d={
                    ringPathD(surfacePreview.outer) +
                    dedupeHoles(surfacePreview.holes || [])
                      .map((hole) => ' ' + ringPathD(hole))
                      .join('')
                  }
                  fill="rgba(250, 204, 21, 0.35)"
                  fillRule="evenodd"
                  stroke="#eab308"
                  strokeWidth={2}
                  strokeLinejoin="round"
                  vectorEffect="non-scaling-stroke"
                  pointerEvents="none"
                />
              )}

              {/* Temporary ring preview: outer ring filled yellow, inner hole
                  left empty via fill-rule evenodd (not saved). */}
              {ringPreview && ringPreview.outer && ringPreview.outer.length >= 3 && (
                <path
                  d={ringPathD(ringPreview.outer) + (ringPreview.hole ? ' ' + ringPathD(ringPreview.hole) : '')}
                  fill="rgba(250, 204, 21, 0.35)"
                  fillRule="evenodd"
                  stroke="#eab308"
                  strokeWidth={2}
                  strokeLinejoin="round"
                  vectorEffect="non-scaling-stroke"
                  pointerEvents="none"
                />
              )}

              {/* Remaining-frame reachable chunks (sections split by the robot reach
                  window), red dashed boxes for verification. Chunks needing an axis
                  move are tinted amber. Non-overlapping; never over a pocket/3D. */}
              {showToolpaths &&
                frameChunks.map((chunk) => {
                  const isSelected =
                    selectedFrameSectionId != null &&
                    (chunk.chunk_id === selectedFrameSectionId || chunk.parent_section_id === selectedFrameSectionId);
                  return (
                    <path
                      key={`fchunk-${chunk.chunk_id}`}
                      d={ringPathD(chunk.points)}
                      fill={isSelected ? 'rgba(244, 63, 94, 0.20)' : chunk.requires_axis_position ? 'rgba(245, 158, 11, 0.10)' : 'rgba(239, 68, 68, 0.06)'}
                      stroke={isSelected ? HIGHLIGHT_STROKE : TOOLPATH_COLOR}
                      strokeWidth={isSelected ? 3.2 : 1.4}
                      strokeDasharray={isSelected ? 'none' : '6 4'}
                      strokeLinejoin="round"
                      vectorEffect="non-scaling-stroke"
                      pointerEvents="none"
                    />
                  );
                })}

              {selectionMode === 'line' && openPaths.map(renderOpenPath)}
            </g>

            {/* Screen-space layer: toolpaths + arrows + markers (constant size).
                The legacy backend frame centerline (frameToolpaths) is intentionally
                not drawn — the computed frame-section chunks/passes replace it, and
                its dashed line down the middle only confused the operator. */}

            {/* Pocket toolpaths (Tool 3 rectangular contour): solid blue lines with
                a mid-segment travel arrow and a start marker at the first corner. */}
            {pocketToolpathOverlays.length > 0 && (
              <g pointerEvents="none">
                {pocketToolpathOverlays.map(({ seg, x0, y0, x1, y1, arrow, isStart }) => (
                  <g key={`ptp-${seg.id}`}>
                    {/* Coloured by 7th-axis station when the reach planner has run, so
                        splits are visible; falls back to the tool colour otherwise. */}
                    <line x1={x0} y1={y0} x2={x1} y2={y1} stroke={seg.station_index === undefined ? POCKET_TOOLPATH_COLOR : stationColor(seg.station_index, seg.axis7_position_mm)} strokeWidth={2} />
                    <polygon points={arrow} fill={seg.station_index === undefined ? POCKET_TOOLPATH_COLOR : stationColor(seg.station_index, seg.axis7_position_mm)} />
                    {isStart && (
                      <circle cx={x0} cy={y0} r={4.5} fill={START_MARKER_FILL} stroke={START_MARKER_STROKE} strokeWidth={1.8} />
                    )}
                  </g>
                ))}
              </g>
            )}

            {/* 3D-contour ring toolpaths: purple rectangular contour with a
                mid-segment travel arrow and a start marker at the bottom-right. */}
            {contourToolpathOverlays.length > 0 && (
              <g pointerEvents="none">
                {contourToolpathOverlays.map(({ seg, x0, y0, x1, y1, arrow, isStart }) => (
                  <g key={`ctp-${seg.id}`}>
                    <line x1={x0} y1={y0} x2={x1} y2={y1} stroke={seg.station_index === undefined ? CONTOUR_TOOLPATH_COLOR : stationColor(seg.station_index, seg.axis7_position_mm)} strokeWidth={2} />
                    <polygon points={arrow} fill={seg.station_index === undefined ? CONTOUR_TOOLPATH_COLOR : stationColor(seg.station_index, seg.axis7_position_mm)} />
                    {isStart && (
                      <circle cx={x0} cy={y0} r={4.5} fill={START_MARKER_FILL} stroke={START_MARKER_STROKE} strokeWidth={1.8} />
                    )}
                  </g>
                ))}
              </g>
            )}

            {/* Pocket zigzag fill (Tool 4): thin green passes with travel arrows and
                a start marker at the bottom-right start corner. */}
            {pocketZigzagOverlays.length > 0 && (
              <g pointerEvents="none">
                {pocketZigzagOverlays.map(({ seg, x0, y0, x1, y1, arrow, isStart }) => (
                  <g key={`pzz-${seg.id}`}>
                    <line x1={x0} y1={y0} x2={x1} y2={y1} stroke={seg.station_index === undefined ? POCKET_ZIGZAG_COLOR : stationColor(seg.station_index, seg.axis7_position_mm)} strokeWidth={1.5} />
                    <polygon points={arrow} fill={seg.station_index === undefined ? POCKET_ZIGZAG_COLOR : stationColor(seg.station_index, seg.axis7_position_mm)} />
                    {isStart && (
                      <circle cx={x0} cy={y0} r={4.5} fill={START_MARKER_FILL} stroke={START_MARKER_STROKE} strokeWidth={1.8} />
                    )}
                  </g>
                ))}
              </g>
            )}

            {/* Frame-section preview paths: red passes per chunk (centerline or a
                radius-based zigzag) with travel arrows and a start marker. */}
            {frameSectionPathOverlays.length > 0 && (
              <g pointerEvents="none">
                {frameSectionPathOverlays.map(({ key, pathId, x0, y0, x1, y1, arrow, isStart, color }) => {
                  const isSelected = selectedFramePathId != null && pathId === selectedFramePathId;
                  const drawColor = isSelected ? HIGHLIGHT_STROKE : color || TOOLPATH_COLOR;
                  return (
                    <g key={`fsp-${key}`}>
                      <line x1={x0} y1={y0} x2={x1} y2={y1} stroke={drawColor} strokeWidth={isSelected ? 4 : 2} />
                      <polygon points={arrow} fill={drawColor} />
                      {isStart && (
                        <circle cx={x0} cy={y0} r={isSelected ? 6 : 4.5} fill={START_MARKER_FILL} stroke={isSelected ? HIGHLIGHT_STROKE : START_MARKER_STROKE} strokeWidth={isSelected ? 2.4 : 1.8} />
                      )}
                    </g>
                  );
                })}
              </g>
            )}

            {/* Frame Tool 4 zigzag (no-pocket case): red passes filling the whole
                frame from the origin, with travel arrows and a start marker. */}
            {frameZigzagOverlays.length > 0 && (
              <g pointerEvents="none">
                {frameZigzagOverlays.map(({ seg, x0, y0, x1, y1, arrow, isStart }) => (
                  <g key={`fzz-${seg.id}`}>
                    {/* Colour by 7th-axis station like the other tools once planned. */}
                    <line x1={x0} y1={y0} x2={x1} y2={y1} stroke={seg.station_index === undefined ? TOOLPATH_COLOR : stationColor(seg.station_index, seg.axis7_position_mm)} strokeWidth={1.5} />
                    <polygon points={arrow} fill={seg.station_index === undefined ? TOOLPATH_COLOR : stationColor(seg.station_index, seg.axis7_position_mm)} />
                    {isStart && (
                      <circle cx={x0} cy={y0} r={4.5} fill={START_MARKER_FILL} stroke={START_MARKER_STROKE} strokeWidth={1.8} />
                    )}
                  </g>
                ))}
              </g>
            )}

            {/* Origin axes (screen-space, constant pixel size): +X red arrow, +Y
                green arrow, drawn at world origin (0,0). Y points up (screen -y). */}
            {(loops.length > 0 || openPaths.length > 0 || framePolygons.length > 0) &&
              (() => {
                const [ox, oy] = toScreen(0, 0);
                const len = 48;
                const head = 8;
                return (
                  <g pointerEvents="none">
                    {/* X axis (red, +X points LEFT in the machine frame) */}
                    <line x1={ox} y1={oy} x2={ox - len} y2={oy} stroke="#ef4444" strokeWidth={2} />
                    <polygon
                      points={`${ox - len},${oy} ${ox - len + head},${oy - head * 0.55} ${ox - len + head},${oy + head * 0.55}`}
                      fill="#ef4444"
                    />
                    <text x={ox - len - 12} y={oy + 4} fontSize={12} fontWeight={700} fill="#ef4444">
                      X
                    </text>
                    {/* Y axis (green, +y up = screen -y) */}
                    <line x1={ox} y1={oy} x2={ox} y2={oy - len} stroke="#22c55e" strokeWidth={2} />
                    <polygon
                      points={`${ox},${oy - len} ${ox - head * 0.55},${oy - len + head} ${ox + head * 0.55},${oy - len + head}`}
                      fill="#22c55e"
                    />
                    <text x={ox + 5} y={oy - len - 3} fontSize={12} fontWeight={700} fill="#22c55e">
                      Y
                    </text>
                    {/* Origin dot */}
                    <circle cx={ox} cy={oy} r={2.6} fill="#0f172a" />
                  </g>
                );
              })()}
          </svg>
        )}

        <div
          style={{ position: 'absolute', top: '8px', right: '8px', display: 'flex', gap: '6px' }}
          onMouseDown={(event) => event.stopPropagation()}
          onMouseUp={(event) => event.stopPropagation()}
        >
          <button type="button" title="Zoom in" onClick={() => zoomBy(1.2)} style={controlButtonStyle}>+</button>
          <button type="button" title="Zoom out" onClick={() => zoomBy(1 / 1.2)} style={controlButtonStyle}>−</button>
          <button type="button" title="Fit to screen" onClick={fitToScreen} style={controlButtonStyle}>Fit</button>
        </div>

        {/* Tool filter — show one tool's passes at a time so station colours are clear.
            Only appears while previewing toolpaths. */}
        {showToolpaths && (
          <div
            style={{ position: 'absolute', top: '44px', right: '8px', display: 'flex', gap: '4px' }}
            onMouseDown={(event) => event.stopPropagation()}
            onMouseUp={(event) => event.stopPropagation()}
          >
            {['all', ...TOOL_FILTER_ORDER].map((key) => {
              const active = toolFilter === key;
              const label = key === 'all' ? 'All' : (TOOL_LABELS[key] || key).replace(/^Tool \d+ · /, '').replace(' · ', ' ');
              const short = key === 'all' ? 'All' : key === 'tool_3' ? 'T3 edge' : key === 'tool_4' ? 'T4 frame' : '3D';
              return (
                <button
                  key={key}
                  type="button"
                  title={key === 'all' ? 'Show all tools' : `Show ${label} only`}
                  onClick={() => setToolFilter(key)}
                  style={{
                    ...controlButtonStyle,
                    padding: '3px 8px',
                    fontSize: '11px',
                    background: active ? '#0f172a' : '#ffffff',
                    color: active ? '#ffffff' : '#0f172a',
                    borderColor: active ? '#0f172a' : '#cbd5e1',
                  }}
                >
                  {short}
                </button>
              );
            })}
          </div>
        )}

        {/* Toolpath legend — grouped by tool, each tool's 7th-axis stops listed in run
            order. Colour = station, so each stop's swatch matches its passes on screen.
            The operator reads: which tool, how many repositions, and where the axis parks. */}
        {showToolpaths && toolStationLegend.length > 0 && (
          <div
            style={{
              position: 'absolute',
              top: `${legendPos.y}px`,
              left: `${legendPos.x}px`,
              maxHeight: 'calc(100% - 16px)',
              fontSize: '11px',
              color: '#0f172a',
              background: 'rgba(255, 255, 255, 0.96)',
              border: '1px solid #cbd5e1',
              borderRadius: '8px',
              boxShadow: '0 1px 4px rgba(15, 23, 42, 0.12)',
              pointerEvents: 'auto',
              display: 'flex',
              flexDirection: 'column',
              minWidth: legendCollapsed ? 'auto' : '190px',
              overflow: 'hidden',
            }}
            onMouseDown={(event) => event.stopPropagation()}
          >
            {/* Title bar: drag handle + collapse/expand toggle. */}
            <div
              onMouseDown={onLegendDragStart}
              style={{
                display: 'flex', alignItems: 'center', justifyContent: 'space-between', gap: '10px',
                padding: '5px 8px', cursor: 'move', userSelect: 'none',
                background: '#f1f5f9', borderBottom: legendCollapsed ? 'none' : '1px solid #e2e8f0',
              }}
            >
              <span style={{ fontWeight: 800, fontSize: '10px', textTransform: 'uppercase', letterSpacing: '0.04em', color: '#475569' }}>
                7th-axis stops
              </span>
              <button
                type="button"
                title={legendCollapsed ? 'Expand' : 'Collapse'}
                onMouseDown={(event) => event.stopPropagation()}
                onClick={() => setLegendCollapsed((v) => !v)}
                style={{ ...controlButtonStyle, padding: '0 6px', fontSize: '13px', lineHeight: '18px', minWidth: '22px' }}
              >
                {legendCollapsed ? '▢' : '—'}
              </button>
            </div>

            {!legendCollapsed && (
              <div style={{ padding: '7px 10px', display: 'flex', flexDirection: 'column', gap: '8px', overflowY: 'auto' }}>
            {toolStationLegend.map((tool) => {
              const totalPasses = tool.stops.reduce((s, x) => s + x.count, 0);
              return (
                <div key={tool.tool} style={{ display: 'flex', flexDirection: 'column', gap: '3px' }}>
                  <span style={{ fontWeight: 800, fontSize: '10px', textTransform: 'uppercase', letterSpacing: '0.04em', color: '#334155' }}>
                    {tool.label}
                    <span style={{ color: '#94a3b8', fontWeight: 600 }}>
                      {' · '}{tool.stops.length} stop{tool.stops.length === 1 ? '' : 's'} · {totalPasses} pass{totalPasses === 1 ? '' : 'es'}
                    </span>
                  </span>
                  {!tool.planned && (
                    <span style={{ color: '#94a3b8', fontStyle: 'italic' }}>reach plan pending…</span>
                  )}
                  {tool.stops.map((stop, i) => (
                    <span key={stop.key} style={{ display: 'flex', alignItems: 'center', gap: '6px', paddingLeft: '2px' }}>
                      <span style={{ width: '16px', height: '3px', borderRadius: '2px', background: stop.color, flex: '0 0 auto' }} />
                      <span>
                        Stop {i + 1}
                        {stop.axis !== undefined && stop.axis !== null ? ` @ X=${Math.round(stop.axis)}mm` : ''}
                        <span style={{ color: '#64748b' }}> · {stop.count} pass{stop.count === 1 ? '' : 'es'}</span>
                      </span>
                    </span>
                  ))}
                  {tool.hasUnreachable && (
                    <span style={{ display: 'flex', alignItems: 'center', gap: '6px', paddingLeft: '2px' }}>
                      <span style={{ width: '16px', height: '3px', borderRadius: '2px', background: STATION_UNREACHABLE_COLOR, flex: '0 0 auto' }} />
                      <span style={{ color: '#64748b' }}>not reachable</span>
                    </span>
                  )}
                </div>
              );
            })}
              </div>
            )}
          </div>
        )}

        {/* Live cursor coordinate in machine (world) units — visible at all times,
            including during toolpath preview. */}
        {cursorWorld && (
          <div
            style={{
              position: 'absolute',
              bottom: '8px',
              right: '8px',
              fontSize: '13px',
              fontFamily: 'monospace',
              fontWeight: 700,
              color: '#0f172a',
              background: 'rgba(255, 255, 255, 0.95)',
              border: '1px solid #cbd5e1',
              borderRadius: '8px',
              padding: '5px 10px',
              boxShadow: '0 1px 4px rgba(15, 23, 42, 0.12)',
              pointerEvents: 'none',
            }}
          >
            <span style={{ color: '#ef4444' }}>X</span> {cursorWorld[0].toFixed(2)}
            {'   '}
            <span style={{ color: '#22c55e' }}>Y</span> {cursorWorld[1].toFixed(2)}
          </div>
        )}

        <div
          style={{
            position: 'absolute',
            bottom: '8px',
            left: '8px',
            fontSize: '11px',
            color: '#334155',
            background: 'rgba(255, 255, 255, 0.9)',
            border: '1px solid #e2e8f0',
            borderRadius: '8px',
            padding: '4px 8px',
          }}
        >
          {selectionMode === 'line'
            ? hoveredLine
              ? `Selected lines: ${selectedLineIds.length} · Guide line ${hoveredLine.entity_id} · ${hoveredLine.dxf_type || 'line'} · layer ${hoveredLine.layer} · length ${Number(hoveredLine.length || 0).toFixed(1)}`
              : `Select Lines mode — selected lines: ${selectedLineIds.length} · hover a solid guide line to select it`
            : hoveredLoop
            ? `Loop ${hoveredLoop.loop_id || hoveredLoop.entity_id} · layer ${hoveredLoop.layer} · ` +
              `area ${Number(hoveredLoop.area).toFixed(1)} · ` +
              `${Number(hoveredLoop.width || 0).toFixed(0)}×${Number(hoveredLoop.height || 0).toFixed(0)}` +
              (assignments[hoveredLoop.entity_id] ? ` · ${DXF_REGION_META[assignments[hoveredLoop.entity_id]].label}` : '')
            : 'Hover a loop to inspect its id / layer / size'}
        </div>
      </div>
    </div>
  );
}
