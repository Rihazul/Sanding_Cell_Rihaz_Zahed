import React from 'react';
import { createPortal } from 'react-dom';
import { Badge } from '../ui/badge';
import { Button } from '../ui/button';
import Dxf2DViewer, { DXF_REGION_META, DXF_REGION_TYPES } from './Dxf2DViewer';
import {
  uploadTableBDxfFile,
  getTableBDxfParsedLoops,
  checkTableBDxfLinesClosed,
  detectTableBDxfLoops,
  computeTableBDxfFrameToolpaths,
  computeTableBDxfFrameZigzag,
  computeTableBDxfTool2Toolpaths,
  planTableBDxfReach,
  type TableBDxfFrameRing,
  type TableBDxfLoop,
  type TableBDxfOpenPath,
  type TableBDxfDetectedLoop,
  type TableBDxfFramePolygon,
  type TableBDxfFrameRectangle,
  type TableBDxfFrameToolpath,
  type TableBDxfFrameWarning,
} from '../../services/api';
import { type TableBFaceMetadata } from './FaceMetadataPanel';
import { type TableBCadRegion } from './RegionSelectionPanel';

export type TableBCadSelectionKey =
  | 'pocketBottomFace'
  | 'pocketBoundaryLoop'
  | 'bevelFaces'
  | 'frameOuterLoop'
  | 'sideReference';

export type TableBCadSelections = Record<TableBCadSelectionKey, boolean>;

export type TableBPreviewStatus = 'idle' | 'draft' | 'approved' | 'needs_revision';
export type TableBCadUploadStatus = 'idle' | 'uploading' | 'uploaded' | 'failed';
export type TableBCadConversionStatus = 'not started' | 'converting' | 'conversion_not_implemented' | 'converted' | 'failed';
export type TableBCadToolpathStatus = 'no toolpath' | 'generating' | 'generated' | 'failed';
export type TableBCadConfirmationStatus = 'not confirmed' | 'confirming' | 'confirmed' | 'failed';

export interface TableBPreviewOperation {
  id: string;
  label: string;
  tool: string;
  detail: string;
}

export interface TableBCadFileSummary {
  name: string;
  sizeLabel: string;
  uploadedAt: string;
}

interface TableBCadAssistedWorkspaceProps {
  mode: 'cad_assisted' | 'legacy';
  onModeChange: (mode: 'cad_assisted' | 'legacy') => void;
  isOperating: boolean;
  cadFile: TableBCadFileSummary | null;
  cadUploadStatus: TableBCadUploadStatus;
  cadJobId: string | null;
  onFileSelected: (file: File) => void;
  conversionStatus: TableBCadConversionStatus;
  conversionMessage: string;
  backendTestMode: boolean;
  onEnableBackendTestMode: () => void;
  onAddTestMesh: () => void;
  onConvertModel: () => void;
  convertedGlbUrl: string | null;
  faceMetadata: TableBFaceMetadata | null;
  faceMetadataStatus: string;
  toolpaths: any | null;
  toolpathStatus: TableBCadToolpathStatus;
  onGenerateToolpath: () => void;
  confirmationStatus: TableBCadConfirmationStatus;
  canConfirmToolpath: boolean;
  onConfirmToolpath: () => void;
  executionPreview: any | null;
  executionPreviewStatus: string;
  onLoadExecutionPreview: () => void;
  selectedMeshNames: string[];
  onMeshSelected: (meshName: string) => void;
  onClearMeshSelection: () => void;
  onRemoveLastMeshSelection: () => void;
  regions: TableBCadRegion[];
  mappingStatus: string;
  onAddRegion: (region: Omit<TableBCadRegion, 'id'>) => void;
  onSaveMapping: () => void;
  onLoadMapping: () => void;
  forceOptions: number[];
  cycleOptions: number[];
  activeSelection: TableBCadSelectionKey;
  onActiveSelectionChange: (key: TableBCadSelectionKey) => void;
  selections: TableBCadSelections;
  onToggleSelection: (key: TableBCadSelectionKey) => void;
  previewStatus: TableBPreviewStatus;
  previewGeneratedAt: string | null;
  previewStale: boolean;
  revisionNote: string;
  onRevisionNoteChange: (value: string) => void;
  onRequestRevision: () => void;
  onApprovePreview: () => void;
  previewOperations: TableBPreviewOperation[];
  // Reports the generated DXF toolpath payload up to the config screen so the
  // approve/start-task gate can use it. Called on every Preview Toolpath run.
  onPreviewGenerated?: (payload: DxfToolpathPreviewPayload | null) => void;
}

// One continuous robot path: an ordered list of [x, y] points (millimeters). The
// robot runs MoveL P1→P2→…→Pn; Z / Rx / Ry / Rz are fixed constants applied by the
// controller (defined per tool, NOT taken from the 2D drawing).
export interface DxfToolpathPath {
  path_id: string;
  tool: string; // tool_3 | tool_4 | tool_3d | tool_4_frame | frame_section
  operation: string; // human label, e.g. "Pocket contour", "Frame section pass"
  closed: boolean; // true = last point returns to the first (rectangular contours)
  points: number[][]; // [[x, y], [x, y], ...] in mm
  station_index?: number | null;
  axis7_position_mm?: number | null;
  reach_unreachable?: boolean;
  split_from_path_id?: string;
  reach_split_index?: number;
  reach_split_count?: number;
  chained_path_ids?: string[];
  cycle_window_id?: string;
  cycle_window_bounds?: { x_min: number; x_max: number; y_min: number; y_max: number };
}

type DxfPreviewSegment = {
  start: number[];
  end: number[];
  id: string;
  tool: string;
  seq: number;
  station_index?: number | null;
  axis7_position_mm?: number | null;
  reach_unreachable?: boolean;
  split_from_path_id?: string;
  reach_split_index?: number;
  reach_split_count?: number;
  cycle_window_id?: string;
  cycle_window_bounds?: { x_min: number; x_max: number; y_min: number; y_max: number };
};

// Corner-point geometry for one assigned region (what the Info panel shows).
export interface DxfRegionInfoPayload {
  region_id: string;
  label: string; // operator-friendly name, e.g. "Pocket 1"
  source_type: string; // line_surface | closed_loop | computed_frame
  operation: string;
  corner_shapes: { label: string; points: number[][] }[]; // e.g. outer/inner corners in mm
}

// A DXF toolpath preview handed to the config screen for the approval gate and the
// Start Task job payload.
export interface DxfToolpathPreviewPayload {
  job_id: string | null;
  file_name: string | null;
  scoped: boolean;
  units: 'mm';
  counts: {
    pocket_tool3: number;
    pocket_tool4_zigzag: number;
    contour_3d: number;
    frame_zigzag: number;
    frame_section_passes: number;
    frame_chunks: number;
  };
  settings?: {
    pocket_zigzag_orientation?: 'vertical' | 'horizontal' | 'rectspiral';
    pocket_overlap_mm?: number;
    frame_overlap_mm?: number;
    pocket_edge_margin_mm?: number;
    pocket_zigzag_cycle_patterns?: {
      selected_orientation: 'vertical' | 'horizontal' | 'rectspiral';
      alternate_orientation: 'vertical' | 'horizontal';
      vertical_paths: DxfToolpathPath[];
      horizontal_paths: DxfToolpathPath[];
      spiral_paths?: DxfToolpathPath[];
      spiral_out_paths?: DxfToolpathPath[];
    };
  };
  // Region corner points (geometry the operator assigned).
  regions: DxfRegionInfoPayload[];
  // MoveL paths (ordered points). This is what the robot executes.
  paths: DxfToolpathPath[];
}

const selectionLabels: Array<{
  key: TableBCadSelectionKey;
  title: string;
  subtitle: string;
}> = [
  {
    key: 'pocketBottomFace',
    title: 'Pocket floor',
    subtitle: 'Face used for Tool 4 zigzag fill.',
  },
  {
    key: 'pocketBoundaryLoop',
    title: 'Pocket contour',
    subtitle: 'Loop used to limit pocket clearing.',
  },
  {
    key: 'bevelFaces',
    title: '3D contour faces',
    subtitle: 'Slanted contour faces for the 3D edge pass.',
  },
  {
    key: 'frameOuterLoop',
    title: 'Outer frame contour',
    subtitle: 'Reference loop for frame sanding.',
  },
  {
    key: 'sideReference',
    title: 'Side and edge reference',
    subtitle: 'Outside edge / side pass reference.',
  },
];

const statusTone: Record<TableBPreviewStatus, string> = {
  idle: 'bg-slate-100 text-slate-700 border-slate-200',
  draft: 'bg-amber-100 text-amber-800 border-amber-200',
  approved: 'bg-emerald-100 text-emerald-800 border-emerald-200',
  needs_revision: 'bg-rose-100 text-rose-800 border-rose-200',
};

const statusLabel: Record<TableBPreviewStatus, string> = {
  idle: 'Waiting for preview',
  draft: 'Preview ready for review',
  approved: 'Preview approved',
  needs_revision: 'Revision requested',
};

const uploadStatusTone: Record<TableBCadUploadStatus, string> = {
  idle: 'bg-slate-100 text-slate-700 border-slate-200',
  uploading: 'bg-blue-100 text-blue-800 border-blue-200',
  uploaded: 'bg-emerald-100 text-emerald-800 border-emerald-200',
  failed: 'bg-rose-100 text-rose-800 border-rose-200',
};

// Inline status colors (frozen Tailwind can't be relied on for the tone classes).
const dxfUploadBadgeStyle = (status: TableBCadUploadStatus): React.CSSProperties => {
  switch (status) {
    case 'uploaded':
      return { background: '#dcfce7', color: '#166534', borderColor: '#86efac' };
    case 'uploading':
      return { background: '#dbeafe', color: '#1e40af', borderColor: '#93c5fd' };
    case 'failed':
      return { background: '#ffe4e6', color: '#9f1239', borderColor: '#fda4af' };
    default:
      return { background: '#f1f5f9', color: '#334155', borderColor: '#cbd5e1' };
  }
};

const conversionStatusTone: Record<TableBCadConversionStatus, string> = {
  'not started': 'bg-slate-100 text-slate-700 border-slate-200',
  converting: 'bg-blue-100 text-blue-800 border-blue-200',
  conversion_not_implemented: 'bg-amber-100 text-amber-800 border-amber-200',
  converted: 'bg-emerald-100 text-emerald-800 border-emerald-200',
  failed: 'bg-rose-100 text-rose-800 border-rose-200',
};

const toolpathStatusTone: Record<TableBCadToolpathStatus, string> = {
  'no toolpath': 'bg-slate-100 text-slate-700 border-slate-200',
  generating: 'bg-blue-100 text-blue-800 border-blue-200',
  generated: 'bg-emerald-100 text-emerald-800 border-emerald-200',
  failed: 'bg-rose-100 text-rose-800 border-rose-200',
};
const confirmationStatusTone: Record<TableBCadConfirmationStatus, string> = {
  'not confirmed': 'bg-slate-100 text-slate-700 border-slate-200',
  confirming: 'bg-blue-100 text-blue-800 border-blue-200',
  confirmed: 'bg-emerald-100 text-emerald-800 border-emerald-200',
  failed: 'bg-rose-100 text-rose-800 border-rose-200',
};

// Inline button styles for the DXF frame/toolpath controls (frozen Tailwind).
const dxfPlainButtonStyle: React.CSSProperties = {
  cursor: 'pointer',
  padding: '4px 9px',
  fontSize: '11px',
  fontWeight: 700,
  color: '#0f172a',
  background: '#ffffff',
  border: '1px solid #cbd5e1',
  borderRadius: '6px',
  lineHeight: 1.2,
  whiteSpace: 'nowrap',
};

const dxfToggleButtonStyle = (active: boolean): React.CSSProperties => ({
  ...dxfPlainButtonStyle,
  color: active ? '#ffffff' : '#334155',
  background: active ? '#0f172a' : '#ffffff',
  borderColor: active ? '#0f172a' : '#cbd5e1',
});

const dxfSurfaceConfirmStyle = (color: string): React.CSSProperties => ({
  cursor: 'pointer',
  padding: '4px 9px',
  fontSize: '11px',
  fontWeight: 700,
  color: '#ffffff',
  background: color,
  border: 'none',
  borderRadius: '6px',
  lineHeight: 1.2,
  whiteSpace: 'nowrap',
});

type DxfBBox = { min_x: number; min_y: number; max_x: number; max_y: number } | null;

// One unified surface model. Every surface has an outer boundary plus optional
// holes (subtracted inner regions) so no two confirmed surfaces overlap.
interface TableBDxfManualSurface {
  id: string;
  geometry_type: 'area_region' | 'area_region_with_hole';
  source: 'selected_lines' | 'selected_lines_detected_ring' | 'nested_closed_loops';
  outer_points: number[][];
  holes: number[][][];
  subtracted_region_ids: string[];
  source_entity_ids?: string[];
  outer_source_entity_ids?: string[];
  inner_source_entity_ids?: string[];
  ignored_source_entity_ids?: string[];
  outer_loop_id?: string;
  inner_loop_id?: string;
  area: number;
  bbox: DxfBBox;
  assigned_operation: string;
}

function dxfPolygonArea(points: number[][]): number {
  const n = points.length;
  if (n < 3) return 0;
  let total = 0;
  for (let i = 0; i < n; i++) {
    const [x1, y1] = points[i];
    const [x2, y2] = points[(i + 1) % n];
    total += x1 * y2 - x2 * y1;
  }
  return Math.abs(total) / 2;
}

// A bbox+area signature, independent of vertex order/count, so the same inner
// region detected via two representations maps to one key.
function dxfHoleSignature(points: number[][]): string {
  const xs = points.map((p) => p[0]);
  const ys = points.map((p) => p[1]);
  return [
    Math.round(Math.min(...xs)),
    Math.round(Math.min(...ys)),
    Math.round(Math.max(...xs)),
    Math.round(Math.max(...ys)),
    Math.round(dxfPolygonArea(points)),
  ].join(',');
}

function dxfDisplayCornerPoints(points: number[][]): number[][] {
  if (!points || points.length <= 4) return points || [];

  const cleaned: number[][] = [];
  for (const p of points) {
    const x = Number(p?.[0]);
    const y = Number(p?.[1]);
    if (!Number.isFinite(x) || !Number.isFinite(y)) continue;
    const prev = cleaned[cleaned.length - 1];
    if (!prev || Math.hypot(prev[0] - x, prev[1] - y) > 0.5) cleaned.push([x, y]);
  }
  if (cleaned.length >= 2 && Math.hypot(cleaned[0][0] - cleaned[cleaned.length - 1][0], cleaned[0][1] - cleaned[cleaned.length - 1][1]) <= 0.5) {
    cleaned.pop();
  }
  if (cleaned.length <= 4) return cleaned;

  const structural: number[][] = [];
  const n = cleaned.length;
  const angleThresholdDeg = 18;
  const minEdgeMm = 2;
  for (let i = 0; i < n; i++) {
    const a = cleaned[(i - 1 + n) % n];
    const b = cleaned[i];
    const c = cleaned[(i + 1) % n];
    const v1x = b[0] - a[0];
    const v1y = b[1] - a[1];
    const v2x = c[0] - b[0];
    const v2y = c[1] - b[1];
    const l1 = Math.hypot(v1x, v1y);
    const l2 = Math.hypot(v2x, v2y);
    if (l1 < minEdgeMm || l2 < minEdgeMm) continue;
    const dot = Math.max(-1, Math.min(1, (v1x * v2x + v1y * v2y) / (l1 * l2)));
    const turnDeg = Math.acos(dot) * 180 / Math.PI;
    if (turnDeg >= angleThresholdDeg) structural.push(b);
  }

  // A rectangular region with one curved side should reduce to its four real structure
  // corners: the two straight-side top corners plus the two curve endpoints. If angle
  // detection sees extra arc noise, keep the four strongest bbox-extreme candidates from
  // the original vertices instead of inventing a bbox rectangle.
  if (structural.length === 4) return structural;
  if (structural.length > 4) {
    const xs = cleaned.map((p) => p[0]);
    const ys = cleaned.map((p) => p[1]);
    const xLo = Math.min(...xs);
    const xHi = Math.max(...xs);
    const yLo = Math.min(...ys);
    const yHi = Math.max(...ys);
    const targets = [
      [xLo, yLo],
      [xLo, yHi],
      [xHi, yHi],
      [xHi, yLo],
    ];
    const picked: number[][] = [];
    for (const target of targets) {
      let best = structural[0];
      let bestDist = Number.POSITIVE_INFINITY;
      for (const p of structural) {
        const alreadyPicked = picked.some((q) => Math.hypot(q[0] - p[0], q[1] - p[1]) <= 0.5);
        if (alreadyPicked) continue;
        const d = Math.hypot(p[0] - target[0], p[1] - target[1]);
        if (d < bestDist) {
          best = p;
          bestDist = d;
        }
      }
      picked.push(best);
    }
    return picked;
  }

  // Fallback: use four real vertices nearest each bbox corner. This preserves actual
  // shape points and avoids displaying invented rectangular corners.
  const xs = cleaned.map((p) => p[0]);
  const ys = cleaned.map((p) => p[1]);
  const targets = [
    [Math.min(...xs), Math.min(...ys)],
    [Math.min(...xs), Math.max(...ys)],
    [Math.max(...xs), Math.max(...ys)],
    [Math.max(...xs), Math.min(...ys)],
  ];
  const picked: number[][] = [];
  for (const target of targets) {
    let best = cleaned[0];
    let bestDist = Number.POSITIVE_INFINITY;
    for (const p of cleaned) {
      const alreadyPicked = picked.some((q) => Math.hypot(q[0] - p[0], q[1] - p[1]) <= 0.5);
      if (alreadyPicked) continue;
      const d = Math.hypot(p[0] - target[0], p[1] - target[1]);
      if (d < bestDist) {
        best = p;
        bestDist = d;
      }
    }
    picked.push(best);
  }
  return picked;
}

// Drop duplicate holes (same region from multiple sources). Without this, an inner
// region counted twice over-subtracts area — which can make a valid ring look empty
// and refuse to confirm.
function dxfDedupeHoles(holes: number[][][]): number[][][] {
  const seen = new Set<string>();
  const out: number[][][] = [];
  for (const hole of holes) {
    if (!hole || hole.length < 3) continue;
    const key = dxfHoleSignature(hole);
    if (seen.has(key)) continue;
    seen.add(key);
    out.push(hole);
  }
  return out;
}

function dxfPointInPolygon(x: number, y: number, points: number[][]): boolean {
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

function dxfDistanceToSegment(px: number, py: number, ax: number, ay: number, bx: number, by: number): number {
  const dx = bx - ax;
  const dy = by - ay;
  const lenSq = dx * dx + dy * dy;
  let t = lenSq > 0 ? ((px - ax) * dx + (py - ay) * dy) / lenSq : 0;
  t = Math.max(0, Math.min(1, t));
  return Math.hypot(px - (ax + t * dx), py - (ay + t * dy));
}

function dxfPointOnPolygonBoundary(x: number, y: number, points: number[][]): boolean {
  const tolerance = 1e-4;
  for (let i = 0, j = points.length - 1; i < points.length; j = i++) {
    if (dxfDistanceToSegment(x, y, points[j][0], points[j][1], points[i][0], points[i][1]) <= tolerance) {
      return true;
    }
  }
  return false;
}

// True if every vertex of `inner` lies inside or on `outer` (shared DXF
// boundaries still count as contained for priority subtraction).
function dxfPolygonNested(inner: number[][], outer: number[][]): boolean {
  return (
    inner.length > 0 &&
    inner.every((p) => dxfPointInPolygon(p[0], p[1], outer) || dxfPointOnPolygonBoundary(p[0], p[1], outer))
  );
}

// Confirmed manual-surface operations → label + fill color.
const DXF_OPERATION_META: Record<string, { label: string; color: string }> = {
  outer_boundary: { label: 'Outer Boundary', color: '#3b82f6' }, // frame blue — leftover is frame too
  pocket_floor: { label: 'Pocket', color: '#f59e0b' },
  surface_3d_area: { label: '3D Contour', color: '#9333ea' },
  frame_level: { label: 'Frame Level', color: '#3b82f6' },
};

const dxfOperationMeta = (operation: string) =>
  DXF_OPERATION_META[operation] || { label: operation, color: '#64748b' };

// Special selection id for the computed frame (Outer − Pocket − 3D). It isn't a real
// assigned region, so a synthetic row carries it to let the operator scope a preview
// to just the frame sections.
const DXF_FRAME_SCOPE_ID = '__computed_frame__';
// Whole-door Frame Level entry: the frame zigzag covers the whole door, so a single
// synthetic row carries the door's 4 outer corners + the zigzag toolpath instead of
// repeating that toolpath on every individual frame-level region card.
const DXF_FRAME_LEVEL_DOOR_ID = '__frame_level_door__';
// A locked Lines-mode auto-confirm waits this long after the last line selection
// before committing the detected surface, so the operator can keep adding lines
// (e.g. a 2-line triangle vs. a 3+ line rectangle) before it commits. Kept generous
// so a 2-line V isn't locked as a triangle before the operator can add the 3rd/4th
// line to form a rectangle/square.
const DXF_AUTO_CONFIRM_DELAY_MS = 2000;
// DXF_REGION_META comes from the untyped .jsx viewer, so widen it to a string
// index before lookup to keep the region-assignment rows type-clean.
const DXF_REGION_META_MAP = DXF_REGION_META as Record<string, { label: string; color: string }>;
const dxfRegionMeta = (regionType: string) =>
  DXF_REGION_META_MAP[regionType] || { label: regionType, color: '#64748b' };
const DXF_OPERATION_PRIORITY: Record<string, number> = {
  pocket_floor: 1,
  surface_3d_area: 2,
  frame_level: 3,
};

const DXF_REGION_PRIORITY: Record<string, number> = {
  pocket: 1,
  surface3d: 2,
  frame: 3,
};

const dxfOperationPriority = (operation: string) => DXF_OPERATION_PRIORITY[operation] ?? 99;
const dxfRegionPriority = (regionType: string) => DXF_REGION_PRIORITY[regionType] ?? 99;

export function TableBCadAssistedWorkspace({
  mode,
  onModeChange,
  isOperating,
  cadFile,
  cadUploadStatus,
  cadJobId,
  onFileSelected,
  conversionStatus,
  conversionMessage,
  backendTestMode,
  onEnableBackendTestMode,
  onAddTestMesh,
  onConvertModel,
  convertedGlbUrl,
  faceMetadata,
  faceMetadataStatus,
  toolpaths,
  toolpathStatus,
  onGenerateToolpath,
  confirmationStatus,
  canConfirmToolpath,
  onConfirmToolpath,
  executionPreview,
  executionPreviewStatus,
  onLoadExecutionPreview,
  selectedMeshNames,
  onMeshSelected,
  onClearMeshSelection,
  onRemoveLastMeshSelection,
  regions,
  mappingStatus,
  onAddRegion,
  onSaveMapping,
  onLoadMapping,
  forceOptions,
  cycleOptions,
  activeSelection,
  onActiveSelectionChange,
  selections,
  onToggleSelection,
  previewStatus,
  previewGeneratedAt,
  previewStale,
  revisionNote,
  onRevisionNoteChange,
  onRequestRevision,
  onApprovePreview,
  previewOperations,
  onPreviewGenerated,
}: TableBCadAssistedWorkspaceProps) {
  const reachReport = toolpaths?.reach_report || null;
  const hasUnreachableSegments = Number(reachReport?.unreachable_segments || 0) > 0;

  // 2D DXF viewer state (independent of the STEP/3D pipeline above).
  const [dxfLoops, setDxfLoops] = React.useState<TableBDxfLoop[]>([]);
  const [dxfOpenPaths, setDxfOpenPaths] = React.useState<TableBDxfOpenPath[]>([]);
  const [dxfJobId, setDxfJobId] = React.useState<string | null>(null);
  const [dxfFileName, setDxfFileName] = React.useState<string | null>(null);
  const [dxfViewerStatus, setDxfViewerStatus] = React.useState<'idle' | 'loading' | 'ready' | 'error'>('idle');
  const [dxfViewerMessage, setDxfViewerMessage] = React.useState<string>('');
  const [dxfSelectedIds, setDxfSelectedIds] = React.useState<string[]>([]);
  // Region highlighted from the assignment list (row hover) → emphasized in the viewer.
  const [dxfHoveredRowId, setDxfHoveredRowId] = React.useState<string | null>(null);
  // Row whose corner points + toolpath points are shown in the info panel.
  const [dxfInfoRowId, setDxfInfoRowId] = React.useState<string | null>(null);
  // Which sub-sections of the Info panel are expanded. Collapsed by default so the operator
  // sees a plain-language summary first, then drills into raw points only if they want.
  const [dxfInfoExpanded, setDxfInfoExpanded] = React.useState<Record<string, boolean>>({});
  // Info panel position (draggable). null = default top-right; once dragged it stays put.
  const [dxfInfoPos, setDxfInfoPos] = React.useState<{ x: number; y: number } | null>(null);
  const dxfInfoDragRef = React.useRef<{ sx: number; sy: number; ox: number; oy: number } | null>(null);
  const dxfInfoPanelRef = React.useRef<HTMLDivElement | null>(null);
  const [dxfAssignments, setDxfAssignments] = React.useState<Record<string, string>>({});
  const [dxfSelectionMode, setDxfSelectionMode] = React.useState<'loop' | 'line' | 'ring'>('loop');
  const [dxfSelectedLineIds, setDxfSelectedLineIds] = React.useState<string[]>([]);
  // Sticky "locked" assign operation for Lines-to-Surface mode: while set, each newly
  // detected surface is auto-assigned this region type, so the operator can keep
  // selecting lines without re-clicking the button. Cleared on unlock / mode switch.
  const [dxfLockedOperation, setDxfLockedOperation] = React.useState<string | null>(null);
  // Detected surface from the selected guide lines (tolerant of extra lines).
  const [dxfDetectedSurface, setDxfDetectedSurface] = React.useState<{
    outer: number[][];
    holes: number[][][];
    area: number;
    bbox: DxfBBox;
    isRing: boolean;
    outer_loop_id: string;
    inner_loop_id: string | null;
    outer_source_entity_ids: string[];
    inner_source_entity_ids: string[];
    ignored_source_entity_ids: string[];
    subtracted_region_ids: string[];
  } | null>(null);
  const [dxfSurfaceMessage, setDxfSurfaceMessage] = React.useState<string>('');
  const dxfSurfaceReqRef = React.useRef(0);
  // Confirmed synthetic surfaces created from selected lines.
  const [dxfManualSurfaces, setDxfManualSurfaces] = React.useState<TableBDxfManualSurface[]>([]);
  const [dxfSelectedSurfaceIds, setDxfSelectedSurfaceIds] = React.useState<string[]>([]);
  const dxfSurfaceCounterRef = React.useRef(0);
  // Nested-loop ring selection (exactly two loops) + ring preview.
  const [dxfRingLoopIds, setDxfRingLoopIds] = React.useState<string[]>([]);
  const [dxfRingPreview, setDxfRingPreview] = React.useState<{
    outer: number[][];
    hole: number[][];
    area: number;
    outerId: string;
    innerId: string;
    bbox: { min_x: number; min_y: number; max_x: number; max_y: number } | null;
  } | null>(null);
  const [dxfRingMessage, setDxfRingMessage] = React.useState<string>('');

  // Compute the ring preview whenever the two nested-loop selection changes.
  React.useEffect(() => {
    if (dxfSelectionMode !== 'ring') {
      setDxfRingPreview(null);
      setDxfRingMessage('');
      if (dxfRingLoopIds.length) setDxfRingLoopIds([]);
      return;
    }
    if (dxfRingLoopIds.length !== 2) {
      setDxfRingPreview(null);
      setDxfRingMessage(
        dxfRingLoopIds.length === 1 ? 'Select one more loop.' : 'Select exactly two closed loops.',
      );
      return;
    }
    const a = dxfLoops.find((loop) => loop.entity_id === dxfRingLoopIds[0]);
    const b = dxfLoops.find((loop) => loop.entity_id === dxfRingLoopIds[1]);
    if (!a || !b) {
      setDxfRingPreview(null);
      setDxfRingMessage('');
      return;
    }
    const outer = a.area >= b.area ? a : b;
    const inner = a.area >= b.area ? b : a;
    if (dxfPolygonNested(inner.points, outer.points)) {
      setDxfRingPreview({
        outer: outer.points,
        hole: inner.points,
        area: Math.max(outer.area - inner.area, 0),
        outerId: outer.entity_id,
        innerId: inner.entity_id,
        bbox: (outer.bbox as any) ?? null,
      });
      setDxfRingMessage('Ring detected');
    } else {
      setDxfRingPreview(null);
      setDxfRingMessage('Selected loops do not form a ring.');
    }
  }, [dxfSelectionMode, dxfRingLoopIds, dxfLoops]);

  // Higher-priority regions become holes in lower-priority surfaces. Priority:
  // Pocket -> 3D Contour -> Frame Level -> computed remaining frame.
  const dxfHigherPriorityRegions = React.useCallback(
    (outerPoints: number[][], assignedOperation: string) => {
      const targetPriority = dxfOperationPriority(assignedOperation);
      const holes: number[][][] = [];
      const ids: string[] = [];

      for (const loop of dxfLoops) {
        const assignedType = dxfAssignments[loop.entity_id];
        if (dxfRegionPriority(assignedType) < targetPriority && dxfPolygonNested(loop.points, outerPoints)) {
          holes.push(loop.points);
          ids.push(loop.entity_id);
        }
      }

      for (const surface of dxfManualSurfaces) {
        if (
          surface.outer_points &&
          dxfOperationPriority(surface.assigned_operation) < targetPriority &&
          dxfPolygonNested(surface.outer_points, outerPoints)
        ) {
          holes.push(surface.outer_points, ...(surface.holes || []));
          ids.push(surface.id);
        }
      }

      return { holes, ids };
    },
    [dxfLoops, dxfAssignments, dxfManualSurfaces],
  );

  // The inner regions to carve out of a contour/ring so it fills the whole frame
  // down to the deepest pocket. For concentric bevel rings we keep only the
  // INNERMOST region (the one that contains no smaller region) — so `ring =
  // outer - innermost box` fills every bevel band, instead of just the outer
  // strip. Separate (non-nested) pockets are each kept.
  const dxfInnerRegions = React.useCallback(
    (
      outerPoints: number[][],
      outerArea: number,
      extraCandidates: { points: number[][]; id: string; area: number }[] = [],
    ) => {
      // Only exclude a candidate that is essentially the outer boundary itself.
      // A thin ring's inner rectangle (area close to the outer) must still count.
      const maxInnerArea = outerArea * 0.999;
      const candidates: { points: number[][]; id: string; area: number }[] = [];

      for (const loop of dxfLoops) {
        const area = dxfPolygonArea(loop.points);
        if (area > 1e-6 && area < maxInnerArea && dxfPolygonNested(loop.points, outerPoints)) {
          candidates.push({ points: loop.points, id: loop.entity_id, area });
        }
      }
      for (const surface of dxfManualSurfaces) {
        if (!surface.outer_points) continue;
        const area = dxfPolygonArea(surface.outer_points);
        if (area > 1e-6 && area < maxInnerArea && dxfPolygonNested(surface.outer_points, outerPoints)) {
          candidates.push({ points: surface.outer_points, id: surface.id, area });
        }
      }
      // Inner boundaries detected from the current line selection (may not be
      // parsed loops on their own).
      for (const extra of extraCandidates) {
        if (extra.area > 1e-6 && extra.area < maxInnerArea && dxfPolygonNested(extra.points, outerPoints)) {
          candidates.push(extra);
        }
      }

      // Keep only the innermost candidates (those that contain no smaller
      // candidate). Concentric bevel steps collapse to the single deepest box.
      const innermost = candidates.filter(
        (c) => !candidates.some((o) => o !== c && o.area < c.area - 1e-6 && dxfPolygonNested(o.points, c.points)),
      );

      const seen = new Set<string>();
      const holes: number[][][] = [];
      const ids: string[] = [];
      for (const c of innermost) {
        const key = c.points.map((p) => `${p[0].toFixed(3)},${p[1].toFixed(3)}`).join('|');
        if (seen.has(key)) continue;
        seen.add(key);
        holes.push(c.points);
        ids.push(c.id);
      }
      return { holes, ids };
    },
    [dxfLoops, dxfManualSurfaces],
  );

  const confirmDxfRingSurface = (assignedOperation: string) => {
    if (!dxfRingPreview) return;
    const { outer, hole, outerId, innerId, area, bbox } = dxfRingPreview;
    // Prevent duplicate ring surfaces from the same loop pair.
    if (dxfManualSurfaces.some((s) => s.outer_loop_id === outerId && s.inner_loop_id === innerId)) {
      setDxfRingMessage('This ring (same loop pair) is already saved.');
      return;
    }
    if (area <= 1e-6) {
      setDxfRingMessage('Resulting ring is empty.');
      return;
    }
    const nextIndex = dxfSurfaceCounterRef.current + 1;
    dxfSurfaceCounterRef.current = nextIndex;
    const surface: TableBDxfManualSurface = {
      id: `contour_surface_${String(nextIndex).padStart(3, '0')}`,
      geometry_type: 'area_region_with_hole',
      source: 'nested_closed_loops',
      outer_points: outer,
      holes: [hole], // inner loop stays empty
      subtracted_region_ids: [innerId],
      outer_loop_id: outerId,
      inner_loop_id: innerId,
      area,
      bbox,
      assigned_operation: assignedOperation,
    };
    console.log('[DXF Ring] confirmed', {
      id: surface.id,
      assigned_operation: assignedOperation,
      area,
      outer_loop_id: outerId,
      inner_loop_id: innerId,
    });
    setDxfManualSurfaces((prev) => [...prev, surface]);
    setDxfRingPreview(null);
    setDxfRingMessage('');
    setDxfRingLoopIds([]);
  };

  const cancelDxfRing = () => {
    console.log('[DXF Ring] preview cancelled');
    setDxfRingPreview(null);
    setDxfRingMessage('');
    setDxfRingLoopIds([]);
  };

  // On every selected-line change, detect candidate loops (tolerant of extra
  // lines) and build the best ring/contour surface from them.
  React.useEffect(() => {
    if (dxfSelectionMode !== 'line' || !dxfJobId || dxfSelectedLineIds.length === 0) {
      setDxfDetectedSurface(null);
      setDxfSurfaceMessage('');
      return;
    }
    const token = dxfSurfaceReqRef.current + 1;
    dxfSurfaceReqRef.current = token;
    (async () => {
      try {
        const res = await detectTableBDxfLoops(dxfJobId, dxfSelectedLineIds);
        if (token !== dxfSurfaceReqRef.current) return; // superseded by a newer request
        const loops = (res.loops || []).slice().sort((a, b) => b.area - a.area);
        const selected = res.selected_count ?? dxfSelectedLineIds.length;
        const candidates = res.candidate_count ?? loops.length;

        if (loops.length === 0) {
          setDxfDetectedSurface(null);
          setDxfSurfaceMessage(`${selected} line(s) selected · no closed boundary found — select lines that enclose an area.`);
          return;
        }

        // Outer boundary = the face that spans the whole selection (largest bounding
        // box), NOT the largest area. For a nested/bevel frame, polygonize returns
        // the outer boundary as a thin annulus whose area is smaller than the inner
        // box — so picking by area grabs the inner box and the ring only fills the
        // inner part. Picking by bbox extent always gets the true outer edge.
        const bboxExtent = (lp: TableBDxfDetectedLoop) => {
          const b = lp.bbox as DxfBBox | undefined;
          if (b && typeof b.max_x === 'number') return (b.max_x - b.min_x) * (b.max_y - b.min_y);
          const xs = lp.points.map((p) => p[0]);
          const ys = lp.points.map((p) => p[1]);
          return (Math.max(...xs) - Math.min(...xs)) * (Math.max(...ys) - Math.min(...ys));
        };
        // A REAL polygonized face always wins over the synthetic selection-bounding
        // rectangle (is_bbox_fallback). Otherwise a curved / triangular pocket, whose
        // true boundary is smaller in extent than its bounding box, would be replaced
        // by that rectangle and lose its actual shape. The bbox rectangle is used only
        // when no real face was detected.
        const isBboxFallback = (lp: TableBDxfDetectedLoop) =>
          (lp as { is_bbox_fallback?: boolean }).is_bbox_fallback === true;
        const realFaces = loops.filter((lp) => !isBboxFallback(lp));
        const outerPool = realFaces.length > 0 ? realFaces : loops;
        const outer = outerPool.reduce((best, lp) => (bboxExtent(lp) > bboxExtent(best) ? lp : best), outerPool[0]);

        // Carve down to the INNERMOST box so the whole frame fills, no matter how
        // polygonize split the bands. Every other face's exterior AND every face's
        // holes are candidates; dxfInnerRegions keeps the deepest nested one.
        const outerArea = dxfPolygonArea(outer.points);
        const detectedExtras = [
          ...loops
            // Never treat the synthetic bounding rectangle as an inner region.
            .filter((lp) => lp !== outer && !isBboxFallback(lp))
            .map((lp) => ({ points: lp.points, id: lp.loop_id, area: dxfPolygonArea(lp.points) })),
          ...loops
            .filter((lp) => !isBboxFallback(lp))
            .flatMap((lp) =>
              (lp.holes || []).map((h, i) => ({ points: h, id: `${lp.loop_id}_hole_${i}`, area: dxfPolygonArea(h) })),
            ),
        ];
        const contained = dxfInnerRegions(outer.points, outerArea, detectedExtras);
        const sourceDesc = 'auto';

        console.log('[DXF Contour] detect', {
          selected,
          detected_loops: loops.length,
          detected_areas: loops.map((lp) => Math.round(lp.area)),
          outer_area: Math.round(outerArea),
          outer_bbox_extent: Math.round(bboxExtent(outer)),
          outer_face_holes: outer.holes ? outer.holes.length : 0,
          inner_holes: contained.holes.length,
          hole_source: sourceDesc,
        });

        const used = new Set(outer.source_entity_ids);
        const ignored = dxfSelectedLineIds.filter((id) => !used.has(id));

        if (contained.holes.length > 0) {
          const netArea = dxfPolygonArea(outer.points) - contained.holes.reduce((s, h) => s + dxfPolygonArea(h), 0);
          setDxfDetectedSurface({
            outer: outer.points,
            holes: contained.holes,
            area: Math.max(netArea, 0),
            bbox: (outer.bbox as DxfBBox) ?? null,
            isRing: true,
            outer_loop_id: outer.loop_id,
            inner_loop_id: null,
            outer_source_entity_ids: outer.source_entity_ids,
            inner_source_entity_ids: [],
            ignored_source_entity_ids: ignored,
            subtracted_region_ids: contained.ids,
          });
          setDxfSurfaceMessage(
            `Ring contour · fills the frame, inner box kept empty` +
              (ignored.length ? ` · ${ignored.length} extra line(s) ignored` : ''),
          );
        } else {
          // Nothing inside this boundary — it can only be a solid area.
          setDxfDetectedSurface({
            outer: outer.points,
            holes: [],
            area: outer.area,
            bbox: (outer.bbox as DxfBBox) ?? null,
            isRing: false,
            outer_loop_id: outer.loop_id,
            inner_loop_id: null,
            outer_source_entity_ids: outer.source_entity_ids,
            inner_source_entity_ids: [],
            ignored_source_entity_ids: ignored,
            subtracted_region_ids: [],
          });
          setDxfSurfaceMessage('One solid boundary (no inner region inside it). For a ring, the boundary must enclose an inner box/pocket.');
        }
      } catch {
        if (token !== dxfSurfaceReqRef.current) return;
        setDxfDetectedSurface(null);
        setDxfSurfaceMessage('Could not detect a boundary from the selected lines.');
      }
    })();
  }, [dxfSelectedLineIds, dxfJobId, dxfSelectionMode, dxfInnerRegions]);

  const confirmDetectedSurface = (assignedOperation: string) => {
    const d = dxfDetectedSurface;
    if (!d) return;

    // Pocket Floor is a solid filled area (its geometry IS the floor); every
    // other operation keeps the detected inner regions as empty holes so the
    // ring/contour never overlaps the pocket. De-dupe first so a doubly-detected
    // inner region can't over-subtract and make a valid ring look empty.
    const holes = assignedOperation === 'pocket_floor' ? [] : dxfDedupeHoles(d.holes || []);
    const netArea = dxfPolygonArea(d.outer) - holes.reduce((total, hole) => total + dxfPolygonArea(hole), 0);

    if (netArea <= 1e-6) {
      setDxfSurfaceMessage('Resulting surface is empty after subtracting inner regions.');
      return;
    }

    // Prevent redundant duplicates: if a surface with the SAME outer footprint is
    // already assigned, update it in place (re-assign) instead of adding a copy —
    // otherwise the same region would generate the same toolpath twice.
    const label = dxfOperationMeta(assignedOperation).label;
    const newSig = dxfHoleSignature(d.outer);
    const existing = dxfManualSurfaces.find(
      (s) => s.outer_points && dxfHoleSignature(s.outer_points) === newSig,
    );
    if (existing) {
      setDxfManualSurfaces((prev) =>
        prev.map((s) =>
          s.id === existing.id
            ? {
                ...s,
                geometry_type: holes.length ? 'area_region_with_hole' : 'area_region',
                source: d.isRing ? 'selected_lines_detected_ring' : 'selected_lines',
                holes,
                subtracted_region_ids: holes.length ? d.subtracted_region_ids : [],
                area: netArea,
                bbox: d.bbox,
                assigned_operation: assignedOperation,
              }
            : s,
        ),
      );
      setDxfSurfaceMessage(
        existing.assigned_operation === assignedOperation
          ? `This region is already assigned as ${label} — kept as one (no duplicate).`
          : `Region re-assigned to ${label} (updated the existing one, no duplicate).`,
      );
      console.log('[DXF Surface] existing region updated (duplicate prevented)', {
        id: existing.id,
        assigned_operation: assignedOperation,
      });
      setDxfDetectedSurface(null);
      setDxfSelectedLineIds([]);
      setDxfSelectedSurfaceIds([]);
      return;
    }

    const nextIndex = dxfSurfaceCounterRef.current + 1;
    dxfSurfaceCounterRef.current = nextIndex;
    const prefix = assignedOperation === 'pocket_floor' ? 'pocket' : 'contour';
    const surface: TableBDxfManualSurface = {
      id: `${prefix}_surface_${String(nextIndex).padStart(3, '0')}`,
      geometry_type: holes.length ? 'area_region_with_hole' : 'area_region',
      source: d.isRing ? 'selected_lines_detected_ring' : 'selected_lines',
      outer_points: d.outer,
      holes,
      subtracted_region_ids: holes.length ? d.subtracted_region_ids : [],
      outer_source_entity_ids: d.outer_source_entity_ids,
      inner_source_entity_ids: d.inner_source_entity_ids,
      ignored_source_entity_ids: d.ignored_source_entity_ids,
      source_entity_ids: [...d.outer_source_entity_ids, ...d.inner_source_entity_ids],
      area: netArea,
      bbox: d.bbox,
      assigned_operation: assignedOperation,
    };
    console.log('[DXF Surface] confirmed detected', {
      id: surface.id,
      assigned_operation: assignedOperation,
      area: netArea,
      holes: holes.length,
      subtracted_region_ids: surface.subtracted_region_ids,
      ignored_source_entity_ids: d.ignored_source_entity_ids,
    });
    setDxfManualSurfaces((prev) => [...prev, surface]);
    setDxfDetectedSurface(null);
    setDxfSurfaceMessage('');
    setDxfSelectedLineIds([]);
    setDxfSelectedSurfaceIds([]);
  };

  const cancelDetectedSurface = () => {
    console.log('[DXF Surface] detection cancelled');
    setDxfDetectedSurface(null);
    setDxfSurfaceMessage('');
    setDxfSelectedLineIds([]);
  };

  type DxfFrameSection = {
    section_id: string;
    points: number[][];
    bbox: { min_x: number; min_y: number; max_x: number; max_y: number };
    width?: number;
    height?: number;
    orientation?: 'horizontal' | 'vertical';
    covered?: boolean;
    clipped?: boolean;
    area?: number;
    region_type?: string;
  };

  type DxfFrameChunk = DxfFrameSection & {
    chunk_id: string;
    parent_section_id: string;
    requires_axis_position: boolean;
  };

  type DxfFrameSectionPath = {
    path_id: string;
    source_section_id: string;
    source_chunk_id: string;
    region_type: 'computed_frame';
    operation_type: 'frame_section_pass';
    points: number[][];
    direction: 'X' | 'Y' | string;
    path_strategy?: string;
    start_point: number[];
    end_point: number[];
    station_index?: number | null;
    axis7_position_mm?: number | null;
    reach_unreachable?: boolean;
    split_from_path_id?: string;
    reach_split_index?: number;
    reach_split_count?: number;
    chained_path_ids?: string[];
  };

  // Frame + toolpath preview state.
  const [dxfFramePolygons, setDxfFramePolygons] = React.useState<TableBDxfFramePolygon[]>([]);
  const [dxfFrameRectangles, setDxfFrameRectangles] = React.useState<TableBDxfFrameRectangle[]>([]);
  const [dxfFrameToolpaths, setDxfFrameToolpaths] = React.useState<TableBDxfFrameToolpath[]>([]);
  // Pocket toolpaths (Tool 3 rectangular contour). Millimeters-per-drawing-unit
  // comes from the parse-time normalization so the fixed-mm offsets are exact.
  const [dxfPocketToolpaths, setDxfPocketToolpaths] = React.useState<
    { start: number[]; end: number[]; id: string; tool: string; seq: number }[]
  >([]);
  // Tool 4 zigzag (raster) fill segments.
  const [dxfPocketZigzag, setDxfPocketZigzag] = React.useState<
    { start: number[]; end: number[]; id: string; tool: string; seq: number }[]
  >([]);
  // Operator-set pocket overlap (mm, 0..100). Higher overlap → smaller pass step
  // → more passes; 0 → passes just touch (fewest passes).
  const [dxfPocketOverlap, setDxfPocketOverlap] = React.useState(0);
  // Operator-set Tool 3 (pocket edge) SAFETY MARGIN (mm), added to the fixed tool-size offset
  // (38.1 mm X / 50.8 mm Y). A larger margin keeps the pass farther from the pocket edge.
  // Default 4.5 mm matches the previous hardcoded value.
  const TOOL3_DEFAULT_EDGE_MARGIN_MM = 4.5;
  const [dxfPocketEdgeMargin, setDxfPocketEdgeMargin] = React.useState(TOOL3_DEFAULT_EDGE_MARGIN_MM);
  const [dxfPocketZigzagOrientation, setDxfPocketZigzagOrientation] = React.useState<'vertical' | 'horizontal' | 'rectspiral'>('vertical');
  // Operator-set FRAME overlap (mm, 0..100) — same idea but for the big frame
  // rectangle (chunk) zigzag passes. Independent of the pocket overlap.
  const [dxfFrameOverlap, setDxfFrameOverlap] = React.useState(0);
  // 3D-contour ring toolpaths (rectangular contour, Type 1 offset-from-outer or
  // Type 2 band-midline).
  const [dxf3dContourToolpaths, setDxf3dContourToolpaths] = React.useState<
    { start: number[]; end: number[]; id: string; tool: string; seq: number }[]
  >([]);
  // Frame Tool 4 zigzag — used when there is NO pocket (frame level + outer, or the
  // whole door is outer boundary). Fills the whole frame from the origin, no offset.
  const [dxfFrameZigzag, setDxfFrameZigzag] = React.useState<
    { start: number[]; end: number[]; id: string; tool: string; seq: number }[]
  >([]);
  // Tool 2 side/edge passes: the four outer door sides, reach-split and tagged with the
  // 7th-axis station by the backend Tool 2 reach model (execution/tool2_side_geometry), so
  // the viewer shows the same paths + J7 stations the robot runs. No assign type — derived
  // only from the door outer corners. Populated on Preview Toolpath.
  const [dxfTool2Sides, setDxfTool2Sides] = React.useState<
    { start: number[]; end: number[]; id: string; tool: string; seq: number; side_label: string; station_index: number | null; axis7_position_mm: number | null }[]
  >([]);
  // The true frame surface polygon(s) = outer door − pockets − 3D, computed by the
  // backend (shapely) with curved edges preserved. Rendered as an overlay to verify
  // the frame region before the section/pass pipeline is built on top of it.
  const [dxfFrameArea, setDxfFrameArea] = React.useState<TableBDxfFrameRing[]>([]);

  // Remaining frame surface (Outer Boundary − Pocket − 3D Contour) decomposed into
  // non-overlapping rectangular sections. Toolpaths for these come in a later task.
  const [dxfFrameSections, setDxfFrameSections] = React.useState<DxfFrameSection[]>([]);
  // Frame sections split into robot-reachable chunks (no chunk wider than the X
  // reach window or taller than the Y reach window).
  const [dxfFrameChunks, setDxfFrameChunks] = React.useState<DxfFrameChunk[]>([]);
  // Backend Tool 4 frame sanding paths for each reachable chunk.
  const [dxfFrameSectionPaths, setDxfFrameSectionPaths] = React.useState<DxfFrameSectionPath[]>([]);
  const [dxfMmPerUnit, setDxfMmPerUnit] = React.useState(1);
  // The part's extent in the machine frame, from parse-time normalization. It is
  // derived from the outline layer alone, so it stays correct when other layers
  // (e.g. grooves) overhang the part edge. Null until a DXF reports one.
  const [dxfPartBBox, setDxfPartBBox] = React.useState<DxfBBox>(null);
  // The door outline stitched from line segments at parse time (largest closed
  // polygon). Excludes dangling fragments that share the outline's layer (e.g.
  // prongs on layer "0"), which part_bbox cannot. Null when nothing closed cleanly.
  const [dxfOutlineBBox, setDxfOutlineBBox] = React.useState<DxfBBox>(null);
  // The door outline as an ordered exterior ring (curved edges kept as flattened
  // segments). Frame sections are clipped to this so they follow a curved door edge
  // instead of the bounding rectangle. Null when no clean outline closed.
  const [dxfOutlinePolygon, setDxfOutlinePolygon] = React.useState<number[][] | null>(null);
  const [dxfFrameWarnings, setDxfFrameWarnings] = React.useState<TableBDxfFrameWarning[]>([]);
  const [dxfShowFrame, setDxfShowFrame] = React.useState(true);
  const [dxfShowToolpaths, setDxfShowToolpaths] = React.useState(true);
  const [dxfSelectedToolpathId, setDxfSelectedToolpathId] = React.useState<string | null>(null);
  const [dxfSelectedFrameSectionId, setDxfSelectedFrameSectionId] = React.useState<string | null>(null);
  const [dxfSelectedFramePathId, setDxfSelectedFramePathId] = React.useState<string | null>(null);
  const [dxfSelectedOperationToolpathId, setDxfSelectedOperationToolpathId] = React.useState<string | null>(null);
  const [dxfFrameStatus, setDxfFrameStatus] = React.useState<'idle' | 'loading' | 'ready' | 'error'>('idle');
  const [dxfFrameMessage, setDxfFrameMessage] = React.useState<string>('');
  const [isDxfViewerOpen, setIsDxfViewerOpen] = React.useState(false);

  // Map the DXF viewer status to the shared upload-status vocabulary/badge.
  const dxfUploadStatusLabel: TableBCadUploadStatus =
    dxfViewerStatus === 'loading'
      ? 'uploading'
      : dxfViewerStatus === 'ready'
      ? 'uploaded'
      : dxfViewerStatus === 'error'
      ? 'failed'
      : 'idle';
  const canOpenDxfViewer = dxfUploadStatusLabel === 'uploaded' && !!dxfJobId;

  const handleOpenDxfViewer = () => {
    console.log('[DXF Viewer] View 2D CAD clicked');
    setIsDxfViewerOpen(true);
    console.log('[DXF Viewer] modal opened');
  };

  const handleCloseDxfViewer = () => {
    setIsDxfViewerOpen(false);
    console.log('[DXF Viewer] modal closed');
  };

  const resetDxfFramePreview = () => {
    setDxfFramePolygons([]);
    setDxfFrameRectangles([]);
    setDxfFrameToolpaths([]);
    setDxfPocketToolpaths([]);
    setDxfPocketZigzag([]);
    setDxf3dContourToolpaths([]);
    setDxfFrameZigzag([]);
    setDxfTool2Sides([]);
    setDxfFrameArea([]);
    setDxfFrameSections([]);
    setDxfFrameChunks([]);
    setDxfFrameSectionPaths([]);
    setDxfFrameWarnings([]);
    setDxfSelectedToolpathId(null);
    setDxfSelectedFrameSectionId(null);
    setDxfSelectedFramePathId(null);
    setDxfSelectedOperationToolpathId(null);
    setDxfFrameStatus('idle');
    setDxfFrameMessage('');
  };

  React.useEffect(() => {
    if (!isDxfViewerOpen) return;
    const handleKeyDown = (event: KeyboardEvent) => {
      if (event.key === 'Escape') handleCloseDxfViewer();
    };
    window.addEventListener('keydown', handleKeyDown);
    return () => window.removeEventListener('keydown', handleKeyDown);
  }, [isDxfViewerOpen]);

  // Close the 2D CAD viewer automatically the moment the operator APPROVES (the status
  // TRANSITIONS to 'approved'), so they land back on the Table B config without an extra
  // Close click. We watch the transition — not the mere fact that it is approved — so
  // re-opening the viewer afterwards to review or change something does NOT auto-close it.
  const prevPreviewStatusRef = React.useRef(previewStatus);
  React.useEffect(() => {
    const justApproved =
      prevPreviewStatusRef.current !== 'approved' && previewStatus === 'approved' && !previewStale;
    prevPreviewStatusRef.current = previewStatus;
    if (justApproved && isDxfViewerOpen) {
      setIsDxfViewerOpen(false);
    }
  }, [previewStatus, previewStale, isDxfViewerOpen]);
  const handleLoadDxfIntoViewer = async (file: File) => {
    setDxfFileName(file.name);
    setDxfViewerStatus('loading');
    setDxfViewerMessage(`Uploading ${file.name}...`);
    try {
      const upload = await uploadTableBDxfFile(file);
      if (!upload.success || !upload.job_id) {
        throw new Error(upload.error || 'DXF upload did not return a job id.');
      }
      const parsed = await getTableBDxfParsedLoops(upload.job_id);
      const loops = parsed.loops || [];
      const openPaths = parsed.open_paths || [];
      console.log('[DXF Viewer] parse result', {
        total_loops: parsed.summary?.closed_loops_found ?? loops.length,
        narrow_loops: parsed.summary?.narrow_loops_found ?? 0,
        loops_filtered_out: parsed.summary?.loops_filtered_out ?? 0,
        open_paths: openPaths.length,
      });
      // Geometry is normalized to millimeters at parse time (inch drawings are
      // converted), so 1 drawing unit = 1 mm and the coordinate readout shows mm.
      const normalization = (parsed as {
        normalization?: {
          mm_per_unit?: number;
          origin_source?: string;
          part_bbox?: { min_x: number; min_y: number; max_x: number; max_y: number };
          outline_bbox?: { min_x: number; min_y: number; max_x: number; max_y: number } | null;
          outline_polygon?: number[][] | null;
        };
      }).normalization;
      const mmPerUnit = normalization?.mm_per_unit;
      setDxfMmPerUnit(mmPerUnit ?? 1);
      // Authoritative part extent (outline only). Frame toolpaths must use this rather
      // than a bbox over all geometry, which overhanging layers would inflate.
      setDxfPartBBox(normalization?.part_bbox ?? null);
      // Stitched door outline (largest closed polygon); excludes prongs sharing the
      // outline's layer. Preferred over part_bbox for the frame when present.
      setDxfOutlineBBox(normalization?.outline_bbox ?? null);
      setDxfOutlinePolygon(
        Array.isArray(normalization?.outline_polygon) && normalization.outline_polygon.length >= 3
          ? normalization.outline_polygon
          : null,
      );
      console.log('[DXF Viewer] normalization', {
        mm_per_unit: mmPerUnit,
        origin_source: normalization?.origin_source,
        part_bbox: normalization?.part_bbox,
        outline_bbox: normalization?.outline_bbox,
      });
      setDxfJobId(upload.job_id);
      setDxfLoops(loops);
      setDxfOpenPaths(openPaths);
      setDxfSelectedIds([]);
      setDxfSelectedLineIds([]);
      setDxfManualSurfaces([]);
      setDxfSelectedSurfaceIds([]);
      dxfSurfaceCounterRef.current = 0;
      setDxfRingLoopIds([]);
      setDxfAssignments({});
      resetDxfFramePreview();
      setDxfViewerStatus('ready');
      setDxfViewerMessage(
        `${parsed.summary?.closed_loops_found ?? loops.length} closed loop(s) found` +
          (openPaths.length ? `, ${openPaths.length} open path(s) shown for context.` : '.'),
      );
    } catch (error) {
      setDxfViewerStatus('error');
      setDxfViewerMessage(error instanceof Error ? error.message : 'Failed to load DXF.');
    }
  };

  // Tool 3 (rectangular contour) TCP offset from each pocket edge, in millimeters.
  // The tool center sits 38.1 mm (X) / 50.8 mm (Y) in from the pocket edge — the fixed tool
  // size — PLUS the operator-set safety margin (dxfPocketEdgeMargin, default 4.5 mm).
  const TOOL3_SIZE_X_MM = 38.1;
  const TOOL3_SIZE_Y_MM = 50.8;
  const TOOL3_OFFSET_X_MM = TOOL3_SIZE_X_MM + dxfPocketEdgeMargin;
  const TOOL3_OFFSET_Y_MM = TOOL3_SIZE_Y_MM + dxfPocketEdgeMargin;

  // Build the Tool 3 rectangular-contour toolpath for every assigned pocket. The
  // path is an inner rectangle offset in from the 4 corners, traced as 4 lines
  // counter-clockwise starting at the bottom-right corner (nearest the machine
  // origin, which in the normalized frame is min-x / min-y).
  const computeDxfPocketToolpaths = (scope: Set<string> | null) => {
    const offX = TOOL3_OFFSET_X_MM / dxfMmPerUnit;
    const offY = TOOL3_OFFSET_Y_MM / dxfMmPerUnit;
    const pockets: { id: string; pts: number[][] }[] = [
      ...dxfManualSurfaces
        .filter((s) => s.assigned_operation === 'pocket_floor' && s.outer_points && (!scope || scope.has(s.id)))
        .map((s) => ({ id: s.id, pts: s.outer_points as number[][] })),
      ...dxfLoops
        .filter((l) => dxfAssignments[l.entity_id] === 'pocket' && (!scope || scope.has(l.entity_id)))
        .map((l) => ({ id: l.entity_id, pts: l.points })),
    ];

    const segments: { start: number[]; end: number[]; id: string; tool: string; seq: number }[] = [];
    for (const pk of pockets) {
      const uniquePts = pk.pts.slice(
        0,
        pk.pts.length > 1 && Math.hypot(pk.pts[0][0] - pk.pts[pk.pts.length - 1][0], pk.pts[0][1] - pk.pts[pk.pts.length - 1][1]) <= 1e-6
          ? pk.pts.length - 1
          : pk.pts.length,
      );
      if (uniquePts.length === 3) {
        console.log('[Pocket Toolpath] triangular pocket skipped for Tool 3 rectangular contour', { region: pk.id });
        continue;
      }
      const xs = pk.pts.map((p) => p[0]);
      const ys = pk.pts.map((p) => p[1]);
      const minX = Math.min(...xs);
      const maxX = Math.max(...xs);
      const minY = Math.min(...ys);
      const maxY = Math.max(...ys);
      const ixMin = minX + offX;
      const ixMax = maxX - offX;
      const iyMin = minY + offY;
      const iyMax = maxY - offY;
      if (ixMin >= ixMax || iyMin >= iyMax) {
        console.warn('[Pocket Toolpath] pocket too small for Tool 3 offset, skipped', pk.id);
        continue;
      }
      // CCW from bottom-right: up the right edge, across the top, down the left
      // edge, back across the bottom.
      const corners = [
        [ixMin, iyMin],
        [ixMin, iyMax],
        [ixMax, iyMax],
        [ixMax, iyMin],
      ];
      for (let i = 0; i < 4; i++) {
        segments.push({
          start: corners[i],
          end: corners[(i + 1) % 4],
          id: `${pk.id}_tool3_${i}`,
          tool: 'tool_3',
          seq: i,
        });
      }
    }
    console.log('[Pocket Toolpath] Tool 3 generated', {
      pockets: pockets.length,
      segments: segments.length,
      offset_units: { x: offX, y: offY },
    });
    return segments;
  };

  // Tool 4 (zigzag / raster fill) fixed offset in from each pocket edge, in mm.
  const TOOL4_OFFSET_MM = 72;
  // Pass width at zero overlap (= step spacing when overlap is 0). NOTE: inferred
  // as twice the 72 mm offset (tool coverage); adjust if the real Tool 4 width differs.
  const TOOL4_PASS_WIDTH_MM = 144;
  // Horizontal pocket zigzag is split into full X-window sections before reach
  // planning, so individual rows are not cut midway by the backend splitter.
  const TOOL4_HORIZONTAL_WINDOW_MM = 515;

  // Distribute passes evenly across a span so the first and last passes land on the
  // two ends (matches _calculate_zigzag_pass_spacing: passes = num_steps + 1).
  const calcZigzagPassSpacing = (span: number, stepMm: number) => {
    const step = stepMm > 1e-9 ? stepMm : span;
    const numSteps = Math.max(1, Math.round(span / step));
    return { numSteps, adjustedStep: span / numSteps };
  };

  const TOOL4_REACH_RECT_BELOW_Y_MM = 305;
  const TOOL4_REACH_ROWS_MM = [
    [305, -1100, 630],
    [515, -1050, 585],
    [740, -940, 420],
    [916, -690, 205],
  ] as const;
  const TOOL4_AXIS7_MIN_MM = 0;
  const TOOL4_AXIS7_MAX_MM = 2310;

  const tool4ReachSpanAtY = (yMm: number): [number, number] | null => {
    if (yMm < TOOL4_REACH_RECT_BELOW_Y_MM) return [0, 630];
    const rows = TOOL4_REACH_ROWS_MM;
    if (yMm <= rows[0][0]) return [rows[0][1], rows[0][2]];
    if (yMm >= rows[rows.length - 1][0]) {
      return yMm <= rows[rows.length - 1][0] + 6 ? [rows[rows.length - 1][1], rows[rows.length - 1][2]] : null;
    }
    for (let i = 0; i < rows.length - 1; i++) {
      const [y0, lo0, hi0] = rows[i];
      const [y1, lo1, hi1] = rows[i + 1];
      if (y0 <= yMm && yMm <= y1) {
        const t = (yMm - y0) / Math.max(y1 - y0, 1e-9);
        return [lo0 + (lo1 - lo0) * t, hi0 + (hi1 - hi0) * t];
      }
    }
    return null;
  };

  const tool4StationWindowForRectBounds = (bounds: { xMin: number; xMax: number; yMin: number; yMax: number }) => {
    const samples = [
      [bounds.xMin, bounds.yMin],
      [bounds.xMax, bounds.yMin],
      [bounds.xMin, bounds.yMax],
      [bounds.xMax, bounds.yMax],
    ];
    let lo = TOOL4_AXIS7_MIN_MM;
    let hi = TOOL4_AXIS7_MAX_MM;
    for (const [x, y] of samples) {
      const span = tool4ReachSpanAtY(y * dxfMmPerUnit);
      if (!span) return null;
      const xMm = x * dxfMmPerUnit;
      lo = Math.max(lo, xMm - span[1]);
      hi = Math.min(hi, xMm - span[0]);
      if (lo > hi) return null;
    }
    return [lo, hi] as [number, number];
  };

  const splitBoundsByTool4Reach = (bxLo: number, bxHi: number, byLo: number, byHi: number) => {
    const full = { xMin: bxLo, xMax: bxHi, yMin: byLo, yMax: byHi };
    if (tool4StationWindowForRectBounds(full)) return [full];

    const sections: { xMin: number; xMax: number; yMin: number; yMax: number }[] = [];
    const minWidth = 10 / dxfMmPerUnit;
    let xMin = bxLo;
    for (let guard = 0; guard < 100 && xMin < bxHi - 1e-6; guard++) {
      let low = xMin;
      let high = bxHi;
      let best = xMin;
      for (let i = 0; i < 48; i++) {
        const mid = (low + high) / 2;
        const candidate = { xMin, xMax: mid, yMin: byLo, yMax: byHi };
        if (tool4StationWindowForRectBounds(candidate)) {
          best = mid;
          low = mid;
        } else {
          high = mid;
        }
      }
      if (best <= xMin + minWidth) {
        const fallbackXMax = Math.min(bxHi, xMin + minWidth);
        sections.push({ xMin, xMax: fallbackXMax, yMin: byLo, yMax: byHi });
        xMin = fallbackXMax;
      } else {
        sections.push({ xMin, xMax: best, yMin: byLo, yMax: byHi });
        xMin = best;
      }
    }
    return sections;
  };

  const trimRectSpiralSharedBoundaries = (
    sections: { xMin: number; xMax: number; yMin: number; yMax: number }[],
    step: number,
  ) => {
    if (sections.length <= 1) return sections;

    const minWidth = 10 / dxfMmPerUnit;
    return sections
      .map((section, index) => {
        const out = { ...section };
        if (index > 0) {
          const width = out.xMax - out.xMin;
          const inset = Math.min(step, Math.max(0, width - minWidth));
          out.xMin += inset;
        }
        return out;
      })
      .filter((section) => section.xMax - section.xMin > minWidth);
  };

  // Build the Tool 4 zigzag fill for every assigned pocket. Vertical mode spans Y
  // and steps across X. Horizontal mode spans X and steps across Y. Both start from
  // the bottom-right convention used by the DXF workspace, and both use
  // step = pass width - pocket overlap.
  const dxfUniquePolygonPoints = (pts: number[][]) => {
    const out = pts.map((p) => [p[0], p[1]]);
    if (out.length > 1) {
      const first = out[0];
      const last = out[out.length - 1];
      if (Math.hypot(first[0] - last[0], first[1] - last[1]) <= 1e-6) out.pop();
    }
    return out;
  };

  const dxfTriangleIncenter = (tri: number[][]) => {
    const [a, b, c] = tri;
    const lenA = Math.hypot(b[0] - c[0], b[1] - c[1]);
    const lenB = Math.hypot(a[0] - c[0], a[1] - c[1]);
    const lenC = Math.hypot(a[0] - b[0], a[1] - b[1]);
    const perimeter = lenA + lenB + lenC;
    const area = dxfPolygonArea(tri);
    if (perimeter <= 1e-9 || area <= 1e-9) return null;
    return {
      point: [
        (lenA * a[0] + lenB * b[0] + lenC * c[0]) / perimeter,
        (lenA * a[1] + lenB * b[1] + lenC * c[1]) / perimeter,
      ],
      radius: (2 * area) / perimeter,
    };
  };

  const dxfTriangleAngle = (tri: number[][], index: number) => {
    const p = tri[index];
    const a = tri[(index + 1) % 3];
    const b = tri[(index + 2) % 3];
    const va = [a[0] - p[0], a[1] - p[1]];
    const vb = [b[0] - p[0], b[1] - p[1]];
    const denom = Math.max(Math.hypot(va[0], va[1]) * Math.hypot(vb[0], vb[1]), 1e-9);
    const cos = Math.max(-1, Math.min(1, (va[0] * vb[0] + va[1] * vb[1]) / denom));
    return Math.acos(cos);
  };

  const buildTriangleTool4SpiralSegments = (
    regionId: string,
    pts: number[][],
    edgeOffset: number,
    step: number,
  ) => {
    const tri = dxfUniquePolygonPoints(pts);
    if (tri.length !== 3) return null;

    const incenter = dxfTriangleIncenter(tri);
    if (!incenter || incenter.radius <= edgeOffset + 1e-6) {
      console.warn('[Pocket Toolpath] triangle too small for Tool 4 offset, skipped', {
        region: regionId,
        inradius: incenter?.radius,
        required_offset: edgeOffset,
      });
      return [];
    }

    const widestIndex = [0, 1, 2].sort((a, b) => dxfTriangleAngle(tri, b) - dxfTriangleAngle(tri, a))[0];
    const ordered = [0, 1, 2].map((n) => tri[(widestIndex + n) % 3]);
    const points: number[][] = [];
    let inset = edgeOffset;
    while (inset < incenter.radius - 1e-6) {
      const scale = Math.max(0, 1 - inset / incenter.radius);
      const layer = ordered.map((p) => [
        incenter.point[0] + (p[0] - incenter.point[0]) * scale,
        incenter.point[1] + (p[1] - incenter.point[1]) * scale,
      ]);
      points.push(...layer, layer[0]);
      inset += step;
    }
    points.push(incenter.point);

    const segments: { start: number[]; end: number[]; id: string; tool: string; seq: number }[] = [];
    for (let i = 0; i < points.length - 1; i++) {
      if (Math.hypot(points[i][0] - points[i + 1][0], points[i][1] - points[i + 1][1]) <= 1e-6) continue;
      segments.push({
        start: points[i],
        end: points[i + 1],
        id: `${regionId}_tool4_tri_${i}`,
        tool: 'tool_4',
        seq: i,
      });
    }
    console.log('[Pocket Toolpath] Tool 4 triangle spiral', {
      region: regionId,
      inradius: incenter.radius,
      offset: edgeOffset,
      step,
      segments: segments.length,
    });
    return segments;
  };

  // Rectangular spiral fill for a rectangular (sub-)box. Starts at the BOTTOM-RIGHT corner of
  // the box and winds inward CCW (up the right edge, across the top, down the left, across the
  // bottom), stepping in by `step` (= pocket pass spacing) each ring until the inset box
  // collapses, then finishes at the center. One continuous corner->center path. `idPrefix`
  // already carries the `_tool4_rect` marker (and a `_sec{n}` window tag for wide pockets);
  // segment ids append `_${i}`. `seqBase` offsets seq across sections.
  const buildRectSpiralTool4Segments = (
    idPrefix: string,
    bxLo: number,
    bxHi: number,
    byLo: number,
    byHi: number,
    step: number,
    seqBase = 0,
    insetStart = 0,
  ) => {
    const s = Math.max(step, 1e-6);
    // insetStart lets the OUTWARD cycle spiral on rings offset by half a step so it threads
    // between the inward cycle's rings (a different path, not a reversed retrace). Shrink the
    // starting box by insetStart on all sides; bail if that collapses the box.
    bxLo += insetStart; bxHi -= insetStart; byLo += insetStart; byHi -= insetStart;
    const points: number[][] = [];
    if (bxHi - bxLo <= 1e-6 || byHi - byLo <= 1e-6) return [] as { start: number[]; end: number[]; id: string; tool: string; seq: number }[];
    const pushPt = (p: number[]) => {
      const last = points[points.length - 1];
      if (!last || Math.hypot(last[0] - p[0], last[1] - p[1]) > 1e-6) points.push(p);
    };

    // Pure rectangular spiral — straight lines only, no curves, no diagonals. From the
    // bottom-right corner, walk each edge once (up right, left across top, down left, right
    // across bottom) and step that bound inward by `s`, forming evenly spaced concentric
    // rectangles. When the remaining region becomes a thin corridor (one dimension can no
    // longer take a full ring), stop spiralling and finish with a SINGLE straight pass along
    // the center of that corridor to its far end. This avoids the skinny up/down zigzag a tall
    // pocket would otherwise leave in the middle, and the stacked bottom hops at high overlap.
    let left = bxLo;
    let right = bxHi;
    let bottom = byLo;
    let top = byHi;
    let cur = [right, bottom];
    pushPt(cur); // start at the bottom-right corner

    const finishCorridor = () => {
      const w = right - left;
      const h = top - bottom;
      if (w <= 1e-9 || h <= 1e-9) return;
      if (h >= w) {
        // Vertical corridor: hop to its center X (straight, at the current Y) then run one
        // straight pass to the far Y end. Skip the hop if already at center (avoids a tiny
        // back-and-forth jog).
        const cx = (left + right) / 2;
        if (Math.abs(cur[0] - cx) > 1e-6) pushPt([cx, cur[1]]);
        const farY = Math.abs(cur[1] - bottom) < Math.abs(cur[1] - top) ? top : bottom;
        pushPt([cx, farY]);
      } else {
        // Horizontal corridor: mirror on X.
        const cy = (bottom + top) / 2;
        if (Math.abs(cur[1] - cy) > 1e-6) pushPt([cur[0], cy]);
        const farX = Math.abs(cur[0] - left) < Math.abs(cur[0] - right) ? right : left;
        pushPt([farX, cy]);
      }
    };

    // Spiral one edge at a time, checking AFTER each edge whether the remaining region is now
    // a thin corridor (one dimension can no longer take a full ring). Breaking mid-ring the
    // moment a corridor appears — rather than only at the ring top — prevents the last ring's
    // "across" move from overshooting into the corridor and then reversing (the back-and-forth
    // jog). The corridor is then finished with one straight center pass.
    const isCorridor = () => right - left <= s + 1e-9 || top - bottom <= s + 1e-9;
    while (!isCorridor()) {
      pushPt([right, top]); cur = [right, top]; right -= s; // up the right edge
      if (isCorridor()) break;
      pushPt([left, top]); cur = [left, top]; top -= s; // across the top
      if (isCorridor()) break;
      pushPt([left, bottom]); cur = [left, bottom]; left += s; // down the left edge
      if (isCorridor()) break;
      pushPt([right, bottom]); cur = [right, bottom]; bottom += s; // across the bottom
    }
    finishCorridor();

    const segments: { start: number[]; end: number[]; id: string; tool: string; seq: number }[] = [];
    for (let i = 0; i < points.length - 1; i++) {
      if (Math.hypot(points[i][0] - points[i + 1][0], points[i][1] - points[i + 1][1]) <= 1e-6) continue;
      segments.push({
        start: points[i],
        end: points[i + 1],
        id: `${idPrefix}_${i}`,
        tool: 'tool_4',
        seq: seqBase + i,
      });
    }
    return segments;
  };

  const computeDxfPocketZigzag = (
    scope: Set<string> | null,
    orientation: 'vertical' | 'horizontal' | 'rectspiral' = dxfPocketZigzagOrientation,
    forceWindowSections = false,
    spiralOut = false,
  ) => {
    const off = TOOL4_OFFSET_MM / dxfMmPerUnit;
    const overlap = Math.max(0, Math.min(100, dxfPocketOverlap));
    const stepMmEffective = Math.max(TOOL4_PASS_WIDTH_MM - overlap, 1);
    const step = stepMmEffective / dxfMmPerUnit;

    const regions: { id: string; pts: number[][]; operation: 'pocket' | 'surface3d' }[] = [
      ...dxfManualSurfaces
        .filter(
          (s) =>
            (s.assigned_operation === 'pocket_floor' || s.assigned_operation === 'surface_3d_area') &&
            s.outer_points &&
            (!scope || scope.has(s.id)),
        )
        .map((s) => ({
          id: s.id,
          pts: s.outer_points as number[][],
          operation: s.assigned_operation === 'surface_3d_area' ? 'surface3d' as const : 'pocket' as const,
        })),
      ...dxfLoops
        .filter(
          (l) =>
            (dxfAssignments[l.entity_id] === 'pocket' || dxfAssignments[l.entity_id] === 'surface3d') &&
            (!scope || scope.has(l.entity_id)),
        )
        .map((l) => ({
          id: l.entity_id,
          pts: l.points,
          operation: dxfAssignments[l.entity_id] === 'surface3d' ? 'surface3d' as const : 'pocket' as const,
        })),
    ];

    const segments: { start: number[]; end: number[]; id: string; tool: string; seq: number }[] = [];
    for (const pk of regions) {
      const triangleSegments = buildTriangleTool4SpiralSegments(pk.id, pk.pts, off, step);
      if (triangleSegments) {
        segments.push(...triangleSegments);
        continue;
      }

      // Non-triangular 3D contour regions are handled by computeDxf3dContourToolpaths().
      if (pk.operation === 'surface3d') continue;

      const xs = pk.pts.map((p) => p[0]);
      const ys = pk.pts.map((p) => p[1]);
      const bxLo = Math.min(...xs) + off;
      const bxHi = Math.max(...xs) - off;
      const byLo = Math.min(...ys) + off;
      const byHi = Math.max(...ys) - off;
      if (bxHi <= bxLo || byHi <= byLo) {
        console.warn('[Pocket Toolpath] pocket too small for Tool 4 offset, skipped', pk.id);
        continue;
      }

      // Rectangular spiral. A spiral can't be split mid-ring, so a pocket wider than the arm
      // reach window is divided into reach-safe X-sections like the
      // horizontal pattern, and EACH section gets its own corner->center spiral at its own J7
      // station. Each section id carries both markers: `_tool4_rect` (spiral handling) and
      // `_sec{n}` (per-window grouping / reach), so the backend treats each as an independent
      // spiral window. Narrow pockets are a single section = one clean spiral.
      if (orientation === 'rectspiral') {
        const spiralSections = trimRectSpiralSharedBoundaries(
          splitBoundsByTool4Reach(bxLo, bxHi, byLo, byHi),
          step,
        );
        // spiralOut builds the OUTWARD variant: rings inset by half a step so multi-cycle runs
        // thread between the inward rings (a different path, not a reversed retrace). The id
        // marker `_tool4_rectout` still contains `_tool4_rect` so the backend spiral logic sees
        // it, but distinguishes in-vs-out grouping.
        const marker = spiralOut ? 'tool4_rectout' : 'tool4_rect';
        const insetStart = spiralOut ? step / 2 : 0;
        console.log('[Pocket Toolpath] Tool 4 rectangular spiral', {
          pocket: pk.id,
          overlap_mm: overlap,
          step_mm: stepMmEffective,
          sections: spiralSections.length,
          spiralOut,
        });
        for (let sectionIndex = 0; sectionIndex < spiralSections.length; sectionIndex++) {
          const section = spiralSections[sectionIndex];
          const sxLo = section.xMin;
          const sxHi = section.xMax;
          const cycleWindowId = `${pk.id}_tool4_rect_sec${sectionIndex + 1}`;
          segments.push(
            ...buildRectSpiralTool4Segments(`${pk.id}_${marker}_sec${sectionIndex + 1}`, sxLo, sxHi, byLo, byHi, step, sectionIndex * 10000, insetStart),
          );
          for (let i = segments.length - 1; i >= 0; i--) {
            if (!segments[i].id.startsWith(`${pk.id}_${marker}_sec${sectionIndex + 1}_`)) break;
            segments[i].cycle_window_id = cycleWindowId;
            segments[i].cycle_window_bounds = { x_min: sxLo, x_max: sxHi, y_min: byLo, y_max: byHi };
          }
        }
        continue;
      }

      const pushPointPath = (pathPoints: number[][], idPrefix: string, seqBase: number) => {
        for (let i = 0; i < pathPoints.length - 1; i++) {
          segments.push({
            start: pathPoints[i],
            end: pathPoints[i + 1],
            id: `${idPrefix}_${i}`,
            tool: 'tool_4',
            seq: seqBase + i,
          });
        }
      };

      if (orientation === 'horizontal') {
        const xSpan = bxHi - bxLo;
        const ySpan = byHi - byLo;
        const { numSteps, adjustedStep } = calcZigzagPassSpacing(ySpan, step);
        const maxWindow = Math.max(1, TOOL4_HORIZONTAL_WINDOW_MM / dxfMmPerUnit);
        const sectionCount = Math.max(1, Math.ceil(xSpan / maxWindow));
        const sectionWidth = xSpan / sectionCount;
        console.log('[Pocket Toolpath] Tool 4 zigzag', {
          pocket: pk.id,
          orientation,
          overlap_mm: overlap,
          step_mm: stepMmEffective,
          passes: numSteps + 1,
          sections: sectionCount,
        });

        for (let sectionIndex = 0; sectionIndex < sectionCount; sectionIndex++) {
          const sxLo = bxLo + sectionIndex * sectionWidth;
          const sxHi = sectionIndex === sectionCount - 1 ? bxHi : sxLo + sectionWidth;
          const cycleWindowId = `${pk.id}_tool4_sec${sectionIndex + 1}`;
          const points: number[][] = [];
          let offset = 0;
          let toggle = 0;
          while (offset <= ySpan + 1e-9) {
            const y = byLo + offset;
            const row = [
              [sxLo, y],
              [sxHi, y],
            ];
            if (toggle) row.reverse();
            points.push(...row);
            offset += adjustedStep;
            toggle = 1 - toggle;
          }
          pushPointPath(points, `${pk.id}_tool4_sec${sectionIndex + 1}`, sectionIndex * 10000);
          for (let i = segments.length - 1; i >= 0; i--) {
            if (!segments[i].id.startsWith(`${pk.id}_tool4_sec${sectionIndex + 1}_`)) break;
            segments[i].cycle_window_id = cycleWindowId;
            segments[i].cycle_window_bounds = { x_min: sxLo, x_max: sxHi, y_min: byLo, y_max: byHi };
          }
        }
        continue;
      }

      const xSpan = bxHi - bxLo;
      const maxWindow = Math.max(1, TOOL4_HORIZONTAL_WINDOW_MM / dxfMmPerUnit);
      const sectionCount = forceWindowSections ? Math.max(1, Math.ceil(xSpan / maxWindow)) : 1;
      const sectionWidth = xSpan / sectionCount;
      console.log('[Pocket Toolpath] Tool 4 zigzag', {
        pocket: pk.id,
        orientation,
        overlap_mm: overlap,
        step_mm: stepMmEffective,
        sections: sectionCount,
        cycle_windowed: forceWindowSections,
      });

      // Flat point list: consecutive points also form the step-over moves. Normal
      // one-cycle preview keeps one continuous vertical path. Hidden cycle patterns
      // use matching X-window sections so vertical/horizontal alternation can stay
      // inside the same reach window.
      for (let sectionIndex = 0; sectionIndex < sectionCount; sectionIndex++) {
        const sxLo = bxLo + sectionIndex * sectionWidth;
        const sxHi = sectionIndex === sectionCount - 1 ? bxHi : sxLo + sectionWidth;
        const sectionXSpan = sxHi - sxLo;
        const { adjustedStep } = calcZigzagPassSpacing(sectionXSpan, step);
        const points: number[][] = [];
        let offset = 0;
        let toggle = 0;
        // Start at the bottom-right corner of this reach window.
        while (offset <= sectionXSpan + 1e-9) {
          const row = [
            [sxLo + offset, byLo],
            [sxLo + offset, byHi],
          ];
          if (toggle) row.reverse();
          points.push(...row);
          offset += adjustedStep;
          toggle = 1 - toggle;
        }
        const idPrefix = forceWindowSections ? `${pk.id}_tool4_sec${sectionIndex + 1}` : `${pk.id}_tool4`;
        pushPointPath(points, idPrefix, sectionIndex * 10000);
        if (forceWindowSections) {
          const cycleWindowId = `${pk.id}_tool4_sec${sectionIndex + 1}`;
          for (let i = segments.length - 1; i >= 0; i--) {
            if (!segments[i].id.startsWith(`${idPrefix}_`)) break;
            segments[i].cycle_window_id = cycleWindowId;
            segments[i].cycle_window_bounds = { x_min: sxLo, x_max: sxHi, y_min: byLo, y_max: byHi };
          }
        }
      }
    }
    return segments;
  };

  // 3D-contour Type 1: rectangular contour offset this far in from the ring's
  // outer edge (mm).
  const TOOL_3D_OFFSET_MM = 27;

  // CCW rectangle path from the bottom-right corner (min-x / min-y), matching the
  // Tool 3 direction: up the right edge, across the top, down the left, back across.
  const rectToolpathCCW = (
    ixMin: number,
    ixMax: number,
    iyMin: number,
    iyMax: number,
    idPrefix: string,
    tool: string,
  ) => {
    const corners = [
      [ixMin, iyMin],
      [ixMin, iyMax],
      [ixMax, iyMax],
      [ixMax, iyMin],
    ];
    const segs: DxfPreviewSegment[] = [];
    for (let i = 0; i < 4; i++) {
      segs.push({ start: corners[i], end: corners[(i + 1) % 4], id: `${idPrefix}_${i}`, tool, seq: i });
    }
    return segs;
  };

  const dxfBboxOf = (pts: number[][]) => {
    const xs = pts.map((p) => p[0]);
    const ys = pts.map((p) => p[1]);
    return { minX: Math.min(...xs), maxX: Math.max(...xs), minY: Math.min(...ys), maxY: Math.max(...ys) };
  };

  // Build the 3D-contour ring toolpaths. Each 3D-contour surface is a ring (outer
  // boundary + inner hole). The tool motion depends on what the ring encloses:
  //   Type 1 (encloses a pocket): rectangular contour 15 mm inside the outer edge.
  //   Type 2 (encloses a frame level): rectangle on the ring band's midline.
  // Both trace CCW from the bottom-right corner.
  const computeDxf3dContourToolpaths = (scope: Set<string> | null) => {
    const off15 = TOOL_3D_OFFSET_MM / dxfMmPerUnit;

    const pocketBoxes = [
      ...dxfManualSurfaces.filter((s) => s.assigned_operation === 'pocket_floor' && s.outer_points).map((s) => dxfBboxOf(s.outer_points as number[][])),
      ...dxfLoops.filter((l) => dxfAssignments[l.entity_id] === 'pocket').map((l) => dxfBboxOf(l.points)),
    ];
    const frameBoxes = [
      ...dxfManualSurfaces.filter((s) => s.assigned_operation === 'frame_level' && s.outer_points).map((s) => dxfBboxOf(s.outer_points as number[][])),
      ...dxfLoops.filter((l) => dxfAssignments[l.entity_id] === 'frame').map((l) => dxfBboxOf(l.points)),
    ];

    const contained = (inner: ReturnType<typeof dxfBboxOf>, outer: ReturnType<typeof dxfBboxOf>) =>
      inner.minX >= outer.minX - 1e-6 &&
      inner.maxX <= outer.maxX + 1e-6 &&
      inner.minY >= outer.minY - 1e-6 &&
      inner.maxY <= outer.maxY + 1e-6 &&
      (inner.maxX - inner.minX) * (inner.maxY - inner.minY) < (outer.maxX - outer.minX) * (outer.maxY - outer.minY) - 1e-6;

    const rings = dxfManualSurfaces.filter(
      (s) =>
        s.assigned_operation === 'surface_3d_area' &&
        s.outer_points &&
        (s.holes?.length ?? 0) > 0 &&
        (!scope || scope.has(s.id)),
    );

    const segments: { start: number[]; end: number[]; id: string; tool: string; seq: number }[] = [];
    for (const ring of rings) {
      if (dxfUniquePolygonPoints(ring.outer_points as number[][]).length === 3) {
        console.log('[3D Contour Toolpath] triangular ring uses Tool 4 triangle spiral path', { region: ring.id });
        continue;
      }
      const outerB = dxfBboxOf(ring.outer_points as number[][]);
      const innerB = dxfBboxOf((ring.holes as number[][][])[0]);

      const hasPocket = pocketBoxes.some((b) => contained(b, outerB));
      const hasFrame = frameBoxes.some((b) => contained(b, outerB));
      // Default to Type 1 (pocket) when nothing is detected inside the ring.
      const type = hasFrame && !hasPocket ? 'type2' : 'type1';

      let bounds: { ixMin: number; ixMax: number; iyMin: number; iyMax: number };
      if (type === 'type1') {
        bounds = {
          ixMin: outerB.minX + off15,
          ixMax: outerB.maxX - off15,
          iyMin: outerB.minY + off15,
          iyMax: outerB.maxY - off15,
        };
      } else {
        // Midline of the ring band = average of outer and inner edges.
        bounds = {
          ixMin: (outerB.minX + innerB.minX) / 2,
          ixMax: (outerB.maxX + innerB.maxX) / 2,
          iyMin: (outerB.minY + innerB.minY) / 2,
          iyMax: (outerB.maxY + innerB.maxY) / 2,
        };
      }
      if (bounds.ixMin >= bounds.ixMax || bounds.iyMin >= bounds.iyMax) {
        console.warn('[3D Contour Toolpath] ring too small for offset, skipped', ring.id);
        continue;
      }
      console.log('[3D Contour Toolpath]', { ring: ring.id, type });
      segments.push(
        ...rectToolpathCCW(bounds.ixMin, bounds.ixMax, bounds.iyMin, bounds.iyMax, `${ring.id}_3d`, `tool_3d_${type}`),
      );
    }
    return segments;
  };

  // Layer selection for the part bounds. Other layers (e.g. grooves) can draw lines
  // that overhang the part edge; those must never define the bounds, or the origin /
  // corner points / toolpaths pick up the overhang. Mirrors the backend
  // _select_origin_layers rule. All matching is case-insensitive.
  //   - allowlist: layers that ARE the outline ("0" is AutoCAD's default layer).
  //   - exclusion: layers that are never the outline.
  const DXF_CONTOUR_LAYERS = new Set(['contour', 'outer', 'outline', '0']);
  const DXF_NON_OUTLINE_LAYERS = new Set([
    'groove', 'notes', 'dimensions', 'dim', 'center', 'centerline',
    'construction', 'hidden', 'text', 'annotation',
  ]);

  // Part-boundary fallback for drawings whose parse reports no part_bbox. Rule: if any
  // allowlisted layer is present, use only those; otherwise use every layer EXCEPT the
  // excluded ones; last resort, use all geometry.
  const dxfBoundsOfAllGeometry = (): DxfBBox => {
    const norm = (layer?: string) => (layer ?? '').trim().toLowerCase();
    const all = [
      ...dxfLoops.map((l) => ({ layer: l.layer, points: l.points })),
      ...dxfOpenPaths.map((p) => ({ layer: p.layer, points: p.points })),
    ];
    const allow = all.filter((e) => DXF_CONTOUR_LAYERS.has(norm(e.layer)));
    const nonExcluded = all.filter((e) => !DXF_NON_OUTLINE_LAYERS.has(norm(e.layer)));
    const selected = allow.length > 0 ? allow : (nonExcluded.length > 0 ? nonExcluded : all);
    const pts: number[][] = selected.flatMap((e) => e.points);
    if (pts.length === 0) return null;
    const xs = pts.map((p) => p[0]);
    const ys = pts.map((p) => p[1]);
    return { min_x: Math.min(...xs), min_y: Math.min(...ys), max_x: Math.max(...xs), max_y: Math.max(...ys) };
  };

  const dxfBBoxOfPoints = (pts: number[][]): DxfBBox => {
    if (!pts.length) return null;
    const xs = pts.map((p) => p[0]);
    const ys = pts.map((p) => p[1]);
    return { min_x: Math.min(...xs), min_y: Math.min(...ys), max_x: Math.max(...xs), max_y: Math.max(...ys) };
  };

  const dxfBBoxSize = (bbox: DxfBBox) => ({
    width: bbox ? bbox.max_x - bbox.min_x : 0,
    height: bbox ? bbox.max_y - bbox.min_y : 0,
  });

  const dxfWorkRegionPoints = (): number[][][] => [
    ...dxfManualSurfaces
      .filter((s) => (s.assigned_operation === 'pocket_floor' || s.assigned_operation === 'surface_3d_area') && s.outer_points)
      .map((s) => s.outer_points as number[][]),
    ...dxfLoops
      .filter((loop) => dxfAssignments[loop.entity_id] === 'pocket' || dxfAssignments[loop.entity_id] === 'surface3d')
      .map((loop) => loop.points || []),
  ].filter((pts) => pts.length >= 3);

  // Pocket / 3D-contour region polygons, kept separate for the backend frame-area
  // computation (frame = outer − pockets − 3D).
  const dxfPocketPolygons = (): number[][][] =>
    [
      ...dxfManualSurfaces
        .filter((s) => s.assigned_operation === 'pocket_floor' && s.outer_points)
        .map((s) => s.outer_points as number[][]),
      ...dxfLoops
        .filter((loop) => dxfAssignments[loop.entity_id] === 'pocket')
        .map((loop) => loop.points || []),
    ].filter((pts) => pts.length >= 3);

  const dxfSurface3dPolygons = (): number[][][] =>
    [
      ...dxfManualSurfaces
        .filter((s) => s.assigned_operation === 'surface_3d_area' && s.outer_points)
        .map((s) => s.outer_points as number[][]),
      ...dxfLoops
        .filter((loop) => dxfAssignments[loop.entity_id] === 'surface3d')
        .map((loop) => loop.points || []),
    ].filter((pts) => pts.length >= 3);

  const dxfBBoxContains = (outer: DxfBBox, inner: DxfBBox, tolerance = 1e-3) =>
    !!outer &&
    !!inner &&
    outer.min_x <= inner.min_x + tolerance &&
    outer.max_x >= inner.max_x - tolerance &&
    outer.min_y <= inner.min_y + tolerance &&
    outer.max_y >= inner.max_y - tolerance;

  const dxfAutoOuterBoundaryLoopId = (): string | null => {
    const workRegions = dxfWorkRegionPoints();
    if (!workRegions.length) return null;
    const workBounds = dxfBBoxOfPoints(workRegions.flat());
    const excludedWorkLoopIds = new Set(
      dxfLoops
        .filter((loop) => dxfAssignments[loop.entity_id] === 'pocket' || dxfAssignments[loop.entity_id] === 'surface3d')
        .map((loop) => loop.entity_id),
    );
    const candidates = dxfLoops
      .filter((loop) => (loop.points || []).length >= 3 && !excludedWorkLoopIds.has(loop.entity_id))
      .map((loop) => {
        const bbox = dxfBBoxOfPoints(loop.points || []);
        const area = Number.isFinite((loop as any).area) ? Number((loop as any).area) : dxfPolygonArea(loop.points || []);
        const size = dxfBBoxSize(bbox);
        const containsWork = workBounds ? dxfBBoxContains(bbox, workBounds, 1e-2) : true;
        const containsByPolygon = workRegions.length > 0 && workRegions.every((region) => dxfPolygonNested(region, loop.points || []));
        return { loop, bbox, area, size, containsWork, containsByPolygon };
      })
      .filter((candidate) => candidate.bbox && candidate.area > 1e-6 && candidate.containsWork)
      .sort((a, b) => {
        // Same intent as manual Outer Boundary: choose the closed loop that wraps
        // the selected work. BBox containment is intentionally tolerant because
        // line-built surfaces can sit exactly on edges and fail polygon nesting.
        if (a.containsByPolygon !== b.containsByPolygon) return a.containsByPolygon ? -1 : 1;
        return a.area - b.area;
      });

    if (candidates.length) {
      const selected = candidates[0];
      console.table(
        candidates.slice(0, 10).map((c, index) => ({
          rank: index + 1,
          selected: c.loop.entity_id === selected.loop.entity_id,
          entity_id: c.loop.entity_id,
          layer: c.loop.layer,
          area: Math.round(c.area * 1000) / 1000,
          min_y: c.bbox ? Math.round(c.bbox.min_y * 1000) / 1000 : null,
          max_y: c.bbox ? Math.round(c.bbox.max_y * 1000) / 1000 : null,
          width: Math.round(c.size.width * 1000) / 1000,
          height: Math.round(c.size.height * 1000) / 1000,
          containsByPolygon: c.containsByPolygon,
        })),
      );
      console.log('[DXF Auto Outer] selected', { entity_id: selected.loop.entity_id, layer: selected.loop.layer, bbox: selected.bbox });
      return selected.loop.entity_id;
    }

    return null;
  };

  const dxfComputedFrameLoopBounds = (): DxfBBox => {
    const workRegions = [
      ...dxfManualSurfaces
        .filter((s) => (s.assigned_operation === 'pocket_floor' || s.assigned_operation === 'surface_3d_area') && s.outer_points)
        .map((s) => s.outer_points as number[][]),
      ...dxfLoops
        .filter((loop) => dxfAssignments[loop.entity_id] === 'pocket' || dxfAssignments[loop.entity_id] === 'surface3d')
        .map((loop) => loop.points || []),
    ].filter((pts) => pts.length >= 3);

    if (!workRegions.length) return null;
    const maxWorkArea = Math.max(...workRegions.map((pts) => dxfPolygonArea(pts)), 0);
    const excludedWorkLoopIds = new Set(
      dxfLoops
        .filter((loop) => dxfAssignments[loop.entity_id] === 'pocket' || dxfAssignments[loop.entity_id] === 'surface3d')
        .map((loop) => loop.entity_id),
    );

    // The computed frame must use the real door closed loop. Previously this
    // picked the smallest parent loop around the work regions, which could be a
    // local pocket/frame contour. Manual "Outer Boundary" worked because it
    // forced the correct loop. Use the outermost containing loop automatically.
    const candidates = dxfLoops
      .filter((loop) => (loop.points || []).length >= 3 && !excludedWorkLoopIds.has(loop.entity_id))
      .map((loop) => ({ loop, area: Number.isFinite((loop as any).area) ? Number((loop as any).area) : dxfPolygonArea(loop.points || []) }))
      .filter(({ loop, area }) => area > maxWorkArea * 1.05 && workRegions.every((region) => dxfPolygonNested(region, loop.points || [])))
      .sort((a, b) => b.area - a.area);

    const selected = candidates[0]?.loop;
    return selected ? dxfBBoxOfPoints(selected.points || []) : null;
  };
  // Bounds used for the frame (outer boundary, sections, zigzag).
  //
  const hasDxfFrameObstacles = () =>
    dxfManualSurfaces.some((s) => s.assigned_operation === 'pocket_floor' || s.assigned_operation === 'surface_3d_area') ||
    dxfLoops.some((l) => dxfAssignments[l.entity_id] === 'pocket' || dxfAssignments[l.entity_id] === 'surface3d');

  // Only a POCKET carves the frame into separate rails (→ Computed Frame sections).
  // 3D contour rings sit ON TOP of a frame-level surface: the frame zigzag still covers
  // the whole door and the contour rings are an additional, overlaid toolpath. So they
  // must NOT switch the preview to Computed Frame.
  const hasDxfPocketRegion = () =>
    dxfManualSurfaces.some((s) => s.assigned_operation === 'pocket_floor') ||
    dxfLoops.some((l) => dxfAssignments[l.entity_id] === 'pocket');

  const dxfFrameObstaclePolygons = () =>
    [
      ...dxfManualSurfaces
        .filter((s) => (s.assigned_operation === 'pocket_floor' || s.assigned_operation === 'surface_3d_area') && s.outer_points)
        .map((s) => s.outer_points as number[][]),
      ...dxfLoops
        .filter((l) => dxfAssignments[l.entity_id] === 'pocket' || dxfAssignments[l.entity_id] === 'surface3d')
        .map((l) => l.points),
    ].filter((pts) => pts.length >= 3);

  const dxfFrameOuterPolygon = (): number[][] | null => {
    // The parsed outline is the authoritative door boundary. Manual Frame Level /
    // Outer Boundary selections can be partial helper regions; using them first can
    // shrink the computed frame and hide curved lower sections.
    if (dxfOutlinePolygon && dxfOutlinePolygon.length >= 3) return dxfOutlinePolygon;

    const candidates: number[][][] = [];
    dxfManualSurfaces
      .filter((s) => (s.assigned_operation === 'frame_level' || s.assigned_operation === 'outer_boundary') && s.outer_points)
      .forEach((s) => candidates.push(s.outer_points as number[][]));

    const autoOuterBoundaryLoopId = dxfAutoOuterBoundaryLoopId();
    dxfLoops
      .filter(
        (loop) =>
          dxfAssignments[loop.entity_id] === 'frame' ||
          dxfAssignments[loop.entity_id] === 'outer' ||
          loop.entity_id === autoOuterBoundaryLoopId,
      )
      .forEach((loop) => candidates.push(loop.points || []));

    const selected = candidates
      .filter((pts) => pts.length >= 3)
      .sort((a, b) => Math.abs(dxfPolygonArea(b)) - Math.abs(dxfPolygonArea(a)))[0];
    return selected || null;
  };

  // Bounds used for computed frame. Prefer the parsed closed door outline when
  // available so curved door sections remain part of the frame area.
  const dxfFrameBounds = (): DxfBBox => {
    if (dxfOutlineBBox) return dxfOutlineBBox;

    // 1. Fallback to the operator's selected Frame Level / Outer Boundary surfaces.
    const frameSurfaces = dxfManualSurfaces.filter(
      (s) =>
        (s.assigned_operation === 'frame_level' || s.assigned_operation === 'outer_boundary') &&
        s.outer_points,
    );
    const framePts: number[][] = frameSurfaces.flatMap((s) => s.outer_points as number[][]);

    // 2. Honor closed loops explicitly assigned as Frame / Outer Boundary, plus
    // the auto-detected outer loop. The auto path intentionally feeds the selected
    // entity_id through the same points branch as manual Outer Boundary.
    const autoOuterBoundaryLoopId = dxfAutoOuterBoundaryLoopId();
    const assignedFrameLoopPts: number[][] = dxfLoops
      .filter(
        (loop) =>
          dxfAssignments[loop.entity_id] === 'frame' ||
          dxfAssignments[loop.entity_id] === 'outer' ||
          loop.entity_id === autoOuterBoundaryLoopId,
      )
      .flatMap((loop) => loop.points || []);

    const selectedFrameBounds = dxfBBoxOfPoints([...framePts, ...assignedFrameLoopPts]);
    if (selectedFrameBounds) return selectedFrameBounds;

    // 3. Fallbacks only when no manual or automatic boundary loop is available.
    const closedLoopBounds = dxfComputedFrameLoopBounds();
    return dxfOutlineBBox ?? dxfPartBBox ?? closedLoopBounds ?? dxfBoundsOfAllGeometry();
  };
  // NOTE: the frame-level zigzag is now computed entirely in the backend
  // (compute_frame_zigzag_fill via useBackendFrameZigzag) so it can clip to the real
  // door outline (curve-aware). The old frontend bbox-based computeDxfFrameZigzag was
  // removed — it could not follow a curved edge.

  // Robot reach window (mm). The backend uses these values to split computed
  // frame sections into reachable chunks before returning preview toolpaths.
  const REACH_X_MM = 515;
  const REACH_Y_MM = 750;

  // Tool 4 frame pass settings sent to the backend frame-toolpath route.
  const TOOL_FRAME_OFFSET_MM = 50;
  const FRAME_PASS_WIDTH_MM = 75;

  const handleGenerateFramePreview = async () => {
    if (!dxfJobId) return;
    setDxfFrameStatus('loading');

    // Scope the preview: if the operator has selected specific region(s) — surfaces
    // in the list or loops in the drawing — generate toolpaths ONLY for those, and
    // skip the auto-computed frame. With nothing selected, generate everything.
    const scopeIds = new Set<string>([...dxfSelectedSurfaceIds, ...dxfSelectedIds]);
    const scope = scopeIds.size > 0 ? scopeIds : null;

    const segmentsToPaths = (
      segs: DxfPreviewSegment[],
      operation: string,
      closed: boolean,
    ): DxfToolpathPath[] => {
      const groups = new Map<string, DxfPreviewSegment[]>();
      for (const s of segs) {
        const key = s.id.replace(/_\d+$/, ''); // drop the trailing segment index
        if (!groups.has(key)) groups.set(key, []);
        groups.get(key)!.push(s);
      }
      const out: DxfToolpathPath[] = [];
      for (const [key, list] of groups) {
        list.sort((a, b) => a.seq - b.seq);
        const points = [list[0].start, ...list.map((s) => s.end)];
        const first = list[0];
        out.push({
          path_id: key,
          tool: first.tool,
          operation,
          closed,
          points,
          station_index: first.station_index ?? null,
          axis7_position_mm: first.axis7_position_mm ?? null,
          reach_unreachable: list.some((s) => !!s.reach_unreachable),
          split_from_path_id: first.split_from_path_id,
          reach_split_index: first.reach_split_index,
          reach_split_count: first.reach_split_count,
          cycle_window_id: first.cycle_window_id,
          cycle_window_bounds: first.cycle_window_bounds,
        });
      }
      return out;
    };

    const pathsToSegments = (paths: DxfToolpathPath[]): DxfPreviewSegment[] => {
      const out: DxfPreviewSegment[] = [];
      paths.forEach((path, pathIndex) => {
        const pts = path.points ?? [];
        for (let i = 0; i < pts.length - 1; i += 1) {
          out.push({
            start: pts[i],
            end: pts[i + 1],
            id: `${path.path_id}_${i}`,
            tool: path.tool,
            seq: pathIndex * 10000 + i,
            station_index: path.station_index ?? null,
            axis7_position_mm: path.axis7_position_mm ?? null,
            reach_unreachable: path.reach_unreachable,
            split_from_path_id: path.split_from_path_id,
            reach_split_index: path.reach_split_index,
            reach_split_count: path.reach_split_count,
            cycle_window_id: path.cycle_window_id,
            cycle_window_bounds: path.cycle_window_bounds,
          });
        }
      });
      return out;
    };

    const planSegmentsForTool = async (
      segs: DxfPreviewSegment[],
      operation: string,
      closed: boolean,
      plannerTool: 'tool_1' | 'tool_3' | 'tool_4',
    ): Promise<{ segments: DxfPreviewSegment[]; paths: DxfToolpathPath[] }> => {
      const rawPaths = segmentsToPaths(segs, operation, closed);
      if (!rawPaths.length) return { segments: segs, paths: rawPaths };
      try {
        const result = await planTableBDxfReach(
          dxfJobId,
          plannerTool,
          rawPaths.map((path) => ({
            path_id: path.path_id,
            tool: path.tool,
            operation: path.operation,
            operation_type: operation,
            closed: path.closed,
            points: path.points,
            cycle_window_id: path.cycle_window_id,
            cycle_window_bounds: path.cycle_window_bounds,
          })),
        );
        const responsePaths = result.toolpaths ?? [];
        if (!responsePaths.length) return { segments: segs, paths: rawPaths };
        const plannedPaths = responsePaths.map((path, plannedIndex) => {
          const source = rawPaths.find(
            (raw) => raw.path_id === path.path_id || raw.path_id === path.split_from_path_id,
          );
          return {
            path_id: path.path_id ?? source?.path_id ?? `${operation}_${plannedIndex}`,
            tool: path.tool ?? source?.tool ?? plannerTool,
            operation: path.operation ?? source?.operation ?? operation,
            closed: path.closed ?? source?.closed ?? closed,
            points: path.points,
            station_index: path.station_index ?? null,
            axis7_position_mm: path.axis7_position_mm ?? null,
            reach_unreachable: path.reach_unreachable,
            split_from_path_id: path.split_from_path_id,
            reach_split_index: path.reach_split_index,
            reach_split_count: path.reach_split_count,
            chained_path_ids: path.chained_path_ids,
            cycle_window_id: path.cycle_window_id ?? source?.cycle_window_id,
            cycle_window_bounds: path.cycle_window_bounds ?? source?.cycle_window_bounds,
          };
        }) as DxfToolpathPath[];
        const stationOrder = (result.reach_plan?.stations ?? []).flatMap((station) => station.path_indices ?? []);
        const orderedIndexes = new Set<number>();
        const orderedPaths = [
          ...stationOrder.flatMap((index) => {
            if (index < 0 || index >= plannedPaths.length || orderedIndexes.has(index)) return [];
            orderedIndexes.add(index);
            return [plannedPaths[index]];
          }),
          ...plannedPaths.filter((_, index) => !orderedIndexes.has(index)),
        ];
        return { segments: pathsToSegments(orderedPaths), paths: orderedPaths };
      } catch (error) {
        console.warn(`[DXF Reach] ${operation} planning failed; using raw browser paths`, error);
        return { segments: segs, paths: rawPaths };
      }
    };

    const alternatePocketZigzagOrientation: 'vertical' | 'horizontal' = dxfPocketZigzagOrientation === 'vertical' ? 'horizontal' : 'vertical';
    const rawPocketTp = computeDxfPocketToolpaths(scope);
    const rawPocketZz = computeDxfPocketZigzag(scope, dxfPocketZigzagOrientation);
    const rawPocketZzCycleVertical = computeDxfPocketZigzag(scope, 'vertical', true);
    const rawPocketZzCycleHorizontal = computeDxfPocketZigzag(scope, 'horizontal', true);
    const rawPocketZzCycleSpiral = computeDxfPocketZigzag(scope, 'rectspiral', true);
    const rawPocketZzCycleSpiralOut = computeDxfPocketZigzag(scope, 'rectspiral', true, true);
    const rawContourTp = computeDxf3dContourToolpaths(scope);
    const plannedPocketTp = await planSegmentsForTool(rawPocketTp, 'Pocket contour (Tool 3)', true, 'tool_3');
    const plannedPocketZz = await planSegmentsForTool(rawPocketZz, 'Pocket zigzag (Tool 4)', false, 'tool_4');
    const plannedPocketZzCycleVertical = await planSegmentsForTool(rawPocketZzCycleVertical, 'Pocket zigzag vertical cycle pattern (Tool 4)', false, 'tool_4');
    const plannedPocketZzCycleHorizontal = await planSegmentsForTool(rawPocketZzCycleHorizontal, 'Pocket zigzag horizontal cycle pattern (Tool 4)', false, 'tool_4');
    const plannedPocketZzCycleSpiral = await planSegmentsForTool(rawPocketZzCycleSpiral, 'Pocket rectangular spiral cycle pattern (Tool 4)', false, 'tool_4');
    const plannedPocketZzCycleSpiralOut = await planSegmentsForTool(rawPocketZzCycleSpiralOut, 'Pocket rectangular spiral OUT cycle pattern (Tool 4)', false, 'tool_4');
    const plannedContourTp = await planSegmentsForTool(rawContourTp, '3D contour ring', true, 'tool_1');
    const pocketTp = plannedPocketTp.segments;
    const pocketZz = plannedPocketZz.segments;
    const contourTp = plannedContourTp.segments;
    const pocketTpPaths = plannedPocketTp.paths;
    const pocketZzPaths = plannedPocketZz.paths;
    const pocketZigzagCyclePatterns = {
      selected_orientation: dxfPocketZigzagOrientation,
      alternate_orientation: alternatePocketZigzagOrientation,
      vertical_paths: plannedPocketZzCycleVertical.paths,
      horizontal_paths: plannedPocketZzCycleHorizontal.paths,
      spiral_paths: plannedPocketZzCycleSpiral.paths,
      spiral_out_paths: plannedPocketZzCycleSpiralOut.paths,
    };
    const contourTpPaths = plannedContourTp.paths;
    setDxfPocketToolpaths(pocketTp);
    setDxfPocketZigzag(pocketZz);
    setDxf3dContourToolpaths(contourTp);

    // Tool 2 sides: fetch the reach-split contact toolpaths + 7th-axis stations from the
    // backend Tool 2 reach model (the SAME model the robot runs), so the viewer colours each
    // piece by station like tools 1/3/4. Derived only from the door outer corners — no assign
    // type. Distinct axis7 positions become station indices (sorted) for the viewer's colour.
    const t2Bounds = dxfFrameBounds();
    if (t2Bounds) {
      try {
        const t2 = await computeTableBDxfTool2Toolpaths(dxfJobId, {
          min_x: t2Bounds.min_x,
          min_y: t2Bounds.min_y,
          max_x: t2Bounds.max_x,
          max_y: t2Bounds.max_y,
        });
        // Cluster near-equal 7th-axis positions into one VISUAL station so the operator reads
        // them as the same 7th-axis region. The robot's own grouping (_segments_by_station)
        // separates positions >1mm apart, but on a door two sides can plan J7 stops a few mm
        // apart (e.g. top@1397 vs bottom@1400) — colouring those differently looked like
        // arbitrary grouping. A ~20mm cluster keeps genuinely-different stops distinct while
        // merging ones that are effectively the same 7th-axis position.
        const STATION_CLUSTER_MM = 20;
        const rawPositions = [...(t2.axis7_positions_mm ?? [])].sort((a, b) => a - b);
        const clusters: number[] = [];
        for (const p of rawPositions) {
          if (clusters.length === 0 || Math.abs(p - clusters[clusters.length - 1]) > STATION_CLUSTER_MM) {
            clusters.push(p);
          }
        }
        const stationOf = (axis: number) => {
          const idx = clusters.findIndex((a) => Math.abs(a - axis) <= STATION_CLUSTER_MM);
          return idx >= 0 ? idx : null;
        };
        // Backend returns them already ordered by ascending 7th-axis (run order) — Tool 2 is
        // not a loop, so seq = run order for correct start markers and info ordering.
        setDxfTool2Sides(
          (t2.toolpaths ?? []).map((tp, i) => ({
            start: tp.points[0],
            end: tp.points[tp.points.length - 1],
            id: tp.path_id,
            tool: 'tool_2',
            seq: tp.run_index ?? i,
            side_label: tp.side_label,
            station_index: stationOf(tp.axis7_position_mm),
            axis7_position_mm: tp.axis7_position_mm,
          })),
        );
        console.log('[DXF Tool2] backend sides', { count: t2.toolpaths?.length ?? 0, stations: clusters.length });
      } catch (error) {
        console.error('[DXF Tool2] backend fetch failed', error);
        setDxfTool2Sides([]);
      }
    } else {
      setDxfTool2Sides([]);
    }

    const outlineForFrame = dxfOutlinePolygon ?? (dxfPartBBox
      ? [
          [dxfPartBBox.min_x, dxfPartBBox.min_y],
          [dxfPartBBox.max_x, dxfPartBBox.min_y],
          [dxfPartBBox.max_x, dxfPartBBox.max_y],
          [dxfPartBBox.min_x, dxfPartBBox.max_y],
        ]
      : null);

    let frameZig: DxfPreviewSegment[] = [];
    let frameChunks: DxfFrameChunk[] = [];
    let frameSectionPaths: DxfFrameSectionPath[] = [];

    const useBackendFrameToolpaths = async (): Promise<boolean> => {
      try {
        const result = await computeTableBDxfFrameToolpaths(
          dxfJobId,
          outlineForFrame,
          dxfPocketPolygons(),
          dxfSurface3dPolygons(),
          {
            passWidthMm: FRAME_PASS_WIDTH_MM,
            offsetMm: TOOL_FRAME_OFFSET_MM,
            overlapMm: dxfFrameOverlap,
            reachXMm: REACH_X_MM,
            reachYMm: REACH_Y_MM,
          },
        );
        setDxfFrameArea(result.rings ?? []);
        const backendSections = (result.sections ?? []) as any[];
        const backendChunks = (result.chunks ?? []) as DxfFrameChunk[];
        const backendPaths = (result.toolpaths ?? []) as DxfFrameSectionPath[];
        setDxfFrameSections(backendSections.map((section) => ({ ...section, covered: true })));
        setDxfFrameChunks(backendChunks);
        setDxfFrameSectionPaths(backendPaths);
        frameChunks = backendChunks;
        frameSectionPaths = backendPaths;
        console.log('[DXF FrameToolpaths] backend generated', {
          ok: result.ok,
          rings: result.rings?.length ?? 0,
          sections: backendSections.length,
          chunks: backendChunks.length,
          paths: backendPaths.length,
          frame_area: result.frame_area,
        });
        return true;
      } catch (error) {
        console.error('[DXF FrameToolpaths] backend failed', error);
        setDxfFrameArea([]);
        return false;
      }
    };

    // Whole-door frame (no pocket): curve-aware zigzag FILL from the backend. Each
    // returned pass is a [start,end] point path; convert to the segment shape the
    // frame-zigzag renderer uses. Returns the segments (or [] on failure).
    const useBackendFrameZigzag = async (): Promise<DxfPreviewSegment[]> => {
      try {
        // Frame-level surface = the whole door is FLAT with no pocket assigned. It is a
        // pure linear zigzag over the full outline (curve-aware), so pass NO obstacles —
        // nothing is subtracted. (Pockets/3D are separate operations, not part of this.)
        // Step spacing = tool diameter − overlap (same rule as the pocket zigzag). The
        // tool is 144 mm wide, so overlap=0 → 144 mm step, overlap=100 → 44 mm step.
        // (Do NOT use the legacy 75 mm FRAME_PASS_WIDTH_MM — that made step collapse to
        // 1 mm at high overlap and condensed the passes.)
        const result = await computeTableBDxfFrameZigzag(
          dxfJobId,
          outlineForFrame,
          [],
          [],
          { passWidthMm: TOOL4_PASS_WIDTH_MM, overlapMm: dxfFrameOverlap },
        );
        setDxfFrameArea(result.rings ?? []);
        // The backend returns one or more frame-zigzag polylines. It connects passes
        // only when the connector hardline stays inside the computed frame surface.
        // The frontend must not invent step-over lines here, because those can cross
        // pockets or leave the valid frame.
        const passes = result.toolpaths ?? [];
        const segs: DxfPreviewSegment[] = [];
        let seq = 0;
        passes.forEach((tp, passIndex) => {
          const pts = tp.points ?? [];
          const tool = tp.tool ?? 'tool_4_frame';
          const baseId = tp.path_id ?? `frame_zigzag_${passIndex}`;
          for (let i = 0; i < pts.length - 1; i += 1) {
            segs.push({
              start: pts[i],
              end: pts[i + 1],
              id: `${baseId}_${i}`,
              tool,
              seq: seq++,
              station_index: tp.station_index ?? null,
              axis7_position_mm: tp.axis7_position_mm ?? null,
              reach_unreachable: tp.reach_unreachable,
              split_from_path_id: tp.split_from_path_id,
              reach_split_index: tp.reach_split_index,
              reach_split_count: tp.reach_split_count,
            });
          }
        });
        console.log('[DXF FrameZigzag] backend generated', {
          ok: result.ok,
          passes: result.pass_count ?? segs.length,
          frame_area: result.frame_area,
        });
        return segs;
      } catch (error) {
        console.error('[DXF FrameZigzag] backend failed', error);
        setDxfFrameArea([]);
        return [];
      }
    };

    if (scope) {
      // TWO DISTINCT frame operations — never mix them:
      //  • Frame Level: an ASSIGNED frame_level / outer_boundary region → a single
      //    curve-aware zigzag fill over the flat door outline (useBackendFrameZigzag).
      //  • Computed Frame: the synthetic "__computed_frame__" row (frame = outer −
      //    pockets − 3D, split into reachable sections) → the section pipeline
      //    (useBackendFrameToolpaths). It has NO zigzag fill.
      const scopeHasFrameLevel =
        scope.has(DXF_FRAME_LEVEL_DOOR_ID) ||
        dxfManualSurfaces.some(
          (s) => scope.has(s.id) && (s.assigned_operation === 'frame_level' || s.assigned_operation === 'outer_boundary'),
        ) ||
        dxfLoops.some(
          (l) => scope.has(l.entity_id) && (dxfAssignments[l.entity_id] === 'frame' || dxfAssignments[l.entity_id] === 'outer'),
        );
      const scopeHasComputedFrame = scope.has(DXF_FRAME_SCOPE_ID);

      if (scopeHasComputedFrame) {
        // Computed Frame: section toolpaths only, NO frame-level zigzag.
        setDxfFrameZigzag([]);
        await useBackendFrameToolpaths();
      } else if (scopeHasFrameLevel) {
        // Frame Level: zigzag fill only, NO sections. Origin start, bottom→top, right→left.
        frameZig = await useBackendFrameZigzag();
        setDxfFrameZigzag(frameZig);
        setDxfFrameSections([]);
        setDxfFrameChunks([]);
        setDxfFrameSectionPaths([]);
      } else {
        setDxfFrameZigzag([]);
        setDxfFrameSections([]);
        setDxfFrameChunks([]);
        setDxfFrameSectionPaths([]);
      }
      setDxfSelectedToolpathId(null);
      setDxfFrameStatus('ready');
      const total = pocketTp.length + pocketZz.length + contourTp.length + frameZig.length + frameSectionPaths.length;
      setDxfFrameMessage(
        total > 0
          ? `Toolpath preview for ${scopeIds.size} selected region(s).`
          : `Selected region(s) produced no toolpath — the region may be smaller than the tool offset.`,
      );
      console.log('[DXF Toolpath] scoped preview generated', { regions: scopeIds.size, scopeHasFrameLevel, scopeHasComputedFrame, segments: total });
    } else {
      // "All regions" preview. Frame is ONE of two mutually-exclusive operations:
      //  • A POCKET is assigned → Computed Frame: leftover frame split into reachable
      //    sections (useBackendFrameToolpaths). No frame-level zigzag.
      //  • No pocket, frame_level assigned → Frame Level: zigzag fill over the whole
      //    door outline. 3D contour rings do NOT change this — they overlay on top, so
      //    a frame-level surface + contour rings shows BOTH the zigzag and the rings.
      const hasObstacles = hasDxfPocketRegion();
      const hasFrameOrOuter =
        dxfManualSurfaces.some(
          (s) => s.assigned_operation === 'frame_level' || s.assigned_operation === 'outer_boundary',
        ) ||
        dxfLoops.some((l) => dxfAssignments[l.entity_id] === 'frame' || dxfAssignments[l.entity_id] === 'outer');

      if (hasObstacles) {
        // Computed Frame: sections only, never the zigzag.
        setDxfFrameZigzag([]);
        await useBackendFrameToolpaths();
      } else if (hasFrameOrOuter) {
        // Frame Level: zigzag fill only. Origin start, bottom→top, right→left.
        frameZig = await useBackendFrameZigzag();
        setDxfFrameZigzag(frameZig);
        setDxfFrameSections([]);
        setDxfFrameChunks([]);
        setDxfFrameSectionPaths([]);
      } else {
        setDxfFrameZigzag([]);
        setDxfFrameArea([]);
        setDxfFrameSections([]);
        setDxfFrameChunks([]);
        setDxfFrameSectionPaths([]);
      }
      setDxfSelectedToolpathId(null);
      setDxfFrameStatus('ready');
      const total = pocketTp.length + pocketZz.length + contourTp.length + frameZig.length + frameSectionPaths.length;
      setDxfFrameMessage(
        total > 0
          ? 'Toolpath preview generated (all regions).'
          : 'No regions assigned — assign a region, then press Preview.',
      );
      console.log('[DXF Toolpath] preview generated (all regions)', { hasObstacles, hasFrameOrOuter, segments: total });
    }

    // Report the toolpath payload up so the config screen can gate approve / Start Task.
    // Segments are stitched back into ordered [x, y] point paths for MoveL execution:
    // consecutive segments chain (end of one = start of the next), so a path is the
    // start of its first segment followed by every segment's end.
    const paths: DxfToolpathPath[] = [
      ...pocketTpPaths,
      ...pocketZzPaths,
      ...contourTpPaths,
      ...segmentsToPaths(frameZig, 'Frame zigzag (Tool 4)', false),
      ...frameSectionPaths.map((p) => ({
        path_id: p.path_id,
        tool: 'frame_section',
        operation: 'Frame toolpath',
        closed: false,
        points: p.points,
        station_index: p.station_index ?? null,
        axis7_position_mm: p.axis7_position_mm ?? null,
        reach_unreachable: p.reach_unreachable,
        split_from_path_id: p.split_from_path_id,
        reach_split_index: p.reach_split_index,
        reach_split_count: p.reach_split_count,
        chained_path_ids: p.chained_path_ids,
      })),
    ];

    // Region corner points, one entry per assigned region (plus the computed frame).
    // Mirrors what the Info panel shows so the backend JSON records both the geometry
    // the operator chose and the toolpath that will run over it.
    const regions: DxfRegionInfoPayload[] = dxfAssignmentRows.map((row) => {
      const info = computeDxfRegionInfo(row.id, row.sourceType);
      return {
        region_id: row.id,
        label: row.displayId,
        source_type: row.sourceType,
        operation: row.assignedLabel,
        corner_shapes: info.shapes.map((s) => ({ label: s.label, points: s.points })),
      };
    });

    onPreviewGenerated?.({
      job_id: dxfJobId,
      file_name: dxfFileName,
      scoped: !!scope,
      units: 'mm',
      counts: {
        pocket_tool3: pocketTp.length,
        pocket_tool4_zigzag: pocketZz.length,
        contour_3d: contourTp.length,
        frame_zigzag: frameZig.length,
        frame_section_passes: frameSectionPaths.length,
        frame_chunks: frameChunks.length,
      },
      settings: {
        pocket_zigzag_orientation: dxfPocketZigzagOrientation,
        pocket_overlap_mm: dxfPocketOverlap,
        frame_overlap_mm: dxfFrameOverlap,
        pocket_edge_margin_mm: dxfPocketEdgeMargin,
        pocket_zigzag_cycle_patterns: pocketZigzagCyclePatterns,
      },
      regions,
      paths,
    });
  };

  const reverseSelectedToolpath = () => {
    if (!dxfSelectedToolpathId) return;
    setDxfFrameToolpaths((prev) =>
      prev.map((path) =>
        path.rect_id === dxfSelectedToolpathId ? { ...path, start: path.end, end: path.start } : path,
      ),
    );
    console.log('[DXF Frame] toolpath direction reversed', { rect_id: dxfSelectedToolpathId });
  };

  const deleteSelectedToolpath = () => {
    if (!dxfSelectedToolpathId) return;
    setDxfFrameToolpaths((prev) => prev.filter((path) => path.rect_id !== dxfSelectedToolpathId));
    console.log('[DXF Frame] toolpath deleted', { rect_id: dxfSelectedToolpathId });
    setDxfSelectedToolpathId(null);
  };

  const handleToggleDxfLoop = (loop: TableBDxfLoop) => {
    // In ring mode, loop clicks build the two-loop nested-ring selection.
    if (dxfSelectionMode === 'ring') {
      setDxfRingLoopIds((prev) => {
        if (prev.includes(loop.entity_id)) {
          console.log('[DXF Ring] loop deselected', { entity_id: loop.entity_id });
          return prev.filter((id) => id !== loop.entity_id);
        }
        console.log('[DXF Ring] loop selected', { entity_id: loop.entity_id, area: loop.area });
        // Keep at most two — the most recently clicked loops.
        return [...prev, loop.entity_id].slice(-2);
      });
      return;
    }
    setDxfSelectedIds((prev) => {
      if (prev.includes(loop.entity_id)) {
        console.log('[DXF Viewer] loop deselected', { loop_id: loop.loop_id ?? loop.entity_id, layer: loop.layer });
        return prev.filter((id) => id !== loop.entity_id);
      }
      console.log('[DXF Viewer] loop selected', {
        loop_id: loop.loop_id ?? loop.entity_id,
        layer: loop.layer,
        area: loop.area,
        size: `${Math.round(loop.width ?? 0)}x${Math.round(loop.height ?? 0)}`,
      });
      return [...prev, loop.entity_id];
    });
  };

  const clearDxfSelection = () => {
    console.log('[DXF Viewer] selection cleared');
    setDxfSelectedIds([]);
  };

  const handleToggleDxfLine = (line: TableBDxfOpenPath) => {
    setDxfSelectedLineIds((prev) => {
      if (prev.includes(line.entity_id)) {
        console.log('[DXF Viewer] guide line deselected', { entity_id: line.entity_id, layer: line.layer });
        return prev.filter((id) => id !== line.entity_id);
      }
      console.log('[DXF Viewer] guide line selected', {
        entity_id: line.entity_id,
        dxf_type: line.dxf_type,
        layer: line.layer,
        length: line.length,
      });
      // includes() check above prevents duplicate selected line entries.
      return [...prev, line.entity_id];
    });
  };

  const clearDxfLineSelection = () => {
    console.log('[DXF Viewer] guide line selection cleared');
    setDxfSelectedLineIds([]);
  };

  const assignDxfRegion = (regionType: string) => {
    if (dxfSelectedIds.length === 0) return;
    setDxfAssignments((prev) => {
      const next = { ...prev };
      // Only one Outer allowed by default: drop any existing Outer first.
      if (regionType === 'outer') {
        for (const id of Object.keys(next)) {
          if (next[id] === 'outer') delete next[id];
        }
      }
      for (const id of dxfSelectedIds) {
        next[id] = regionType;
      }
      return next;
    });
    console.log('[DXF Region] assigned', { type: regionType, entity_ids: dxfSelectedIds });
    setDxfSelectedIds([]);
  };

  const dxfToolbarAssignments = [
    { type: 'pocket', label: DXF_REGION_META.pocket.label, color: DXF_REGION_META.pocket.color },
    { type: 'surface3d', label: DXF_REGION_META.surface3d.label, color: DXF_REGION_META.surface3d.color },
    { type: 'frame', label: DXF_REGION_META.frame.label, color: DXF_REGION_META.frame.color },
  ].filter((item) => DXF_REGION_TYPES.includes(item.type));

  const dxfActiveModeLabel = dxfSelectionMode === 'line' ? 'Select Lines to Build Surface' : 'Select Closed Loop';
  const dxfSelectedCount = dxfSelectionMode === 'line' ? dxfSelectedLineIds.length : dxfSelectedIds.length;

  const dxfRegionToOperation = (regionType: string) => {
    if (regionType === 'outer') return 'outer_boundary';
    if (regionType === 'pocket') return 'pocket_floor';
    if (regionType === 'surface3d') return 'surface_3d_area';
    return 'frame_level';
  };

  // Frame Level is only relevant on a flat door. Once a pocket or 3D contour is
  // assigned, the frame is computed automatically (Outer − Pocket − 3D) on Preview
  // Toolpath, so assigning Frame Level would be redundant/confusing.
  // A Computed Frame only exists when a POCKET carves the frame into rails. 3D contour
  // rings overlay a frame-level surface without carving it, so they must not create a
  // synthetic "Computed Frame" row.
  const dxfHasPocketOrContour =
    dxfManualSurfaces.some((s) => s.assigned_operation === 'pocket_floor') ||
    dxfLoops.some((l) => dxfAssignments[l.entity_id] === 'pocket');

  // Any frame-level / outer-boundary region assigned → the whole-door zigzag applies.
  const dxfHasFrameLevelRegion =
    dxfManualSurfaces.some(
      (s) => s.assigned_operation === 'frame_level' || s.assigned_operation === 'outer_boundary',
    ) ||
    dxfLoops.some((l) => dxfAssignments[l.entity_id] === 'frame' || dxfAssignments[l.entity_id] === 'outer');

  const assignDxfToolbarRegion = (regionType: string) => {
    // Reassign confirmed surfaces picked from the list first (works in any mode).
    if (dxfSelectedSurfaceIds.length > 0) {
      const operation = dxfRegionToOperation(regionType);
      setDxfManualSurfaces((prev) =>
        prev.map((surface) =>
          dxfSelectedSurfaceIds.includes(surface.id) ? { ...surface, assigned_operation: operation } : surface,
        ),
      );
      console.log('[DXF Region] selected surface assignment updated', {
        surface_ids: dxfSelectedSurfaceIds,
        assigned_operation: operation,
      });
      setDxfSelectedSurfaceIds([]);
      return;
    }

    if (dxfSelectionMode === 'line') {
      // Clicking an assign button toggles a sticky LOCK for that operation: same op
      // → unlock, different op → lock it. While locked, each newly detected surface
      // is auto-assigned (see effect below), so the operator just keeps selecting
      // lines. If a preview is already showing, confirm it now with this operation.
      const nowLocked = dxfLockedOperation !== regionType;
      setDxfLockedOperation(nowLocked ? regionType : null);
      const label = dxfRegionMeta(regionType).label;
      if (dxfDetectedSurface) {
        confirmDetectedSurface(dxfRegionToOperation(regionType));
      }
      if (!dxfDetectedSurface) {
        setDxfSurfaceMessage(
          nowLocked
            ? `${label} locked — keep selecting lines to assign; click ${label} again to unlock.`
            : `${label} unlocked.`,
        );
      }
      return;
    }

    assignDxfRegion(regionType);
  };

  // While an operation is locked in Lines mode, auto-confirm the detected surface so
  // the operator never has to re-click the button — but DEBOUNCED: it fires only after
  // line selection has been stable for a moment. This lets a shape keep gaining lines
  // (select 2 lines = triangle preview; add a 3rd before the pause = rectangle) and
  // commits whatever is detected once the operator stops selecting. Any change to the
  // selection or preview resets the timer. Clicking the locked button still confirms
  // immediately.
  const dxfAutoConfirmTimerRef = React.useRef<ReturnType<typeof setTimeout> | null>(null);
  React.useEffect(() => {
    if (dxfAutoConfirmTimerRef.current) {
      clearTimeout(dxfAutoConfirmTimerRef.current);
      dxfAutoConfirmTimerRef.current = null;
    }
    if (dxfSelectionMode === 'line' && dxfLockedOperation && dxfDetectedSurface) {
      const operation = dxfRegionToOperation(dxfLockedOperation);
      dxfAutoConfirmTimerRef.current = setTimeout(() => {
        dxfAutoConfirmTimerRef.current = null;
        confirmDetectedSurface(operation);
      }, DXF_AUTO_CONFIRM_DELAY_MS);
    }
    return () => {
      if (dxfAutoConfirmTimerRef.current) {
        clearTimeout(dxfAutoConfirmTimerRef.current);
        dxfAutoConfirmTimerRef.current = null;
      }
    };
    // confirmDetectedSurface reads live state; re-running on selection/preview change
    // is enough and avoids adding an unstable function dep.
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [dxfDetectedSurface, dxfSelectedLineIds, dxfLockedOperation, dxfSelectionMode]);

  const clearDxfToolbarSelection = () => {
    console.log('[DXF Toolbar] Clear Selection clicked');
    clearDxfSelection();
    clearDxfLineSelection();
    setDxfDetectedSurface(null);
    setDxfSurfaceMessage('');
    setDxfRingLoopIds([]);
    setDxfRingPreview(null);
    setDxfRingMessage('');
    setDxfSelectedSurfaceIds([]);
    setDxfLockedOperation(null);
  };

  const deleteSelectedDxfRegion = () => {
    if (dxfSelectedSurfaceIds.length > 0) {
      setDxfManualSurfaces((prev) => prev.filter((surface) => !dxfSelectedSurfaceIds.includes(surface.id)));
      console.log('[DXF Region] selected line surface deleted', { surface_ids: dxfSelectedSurfaceIds });
      setDxfSelectedSurfaceIds([]);
      return;
    }

    if (dxfSelectedIds.length > 0) {
      setDxfAssignments((prev) => {
        const next = { ...prev };
        for (const id of dxfSelectedIds) delete next[id];
        return next;
      });
      console.log('[DXF Region] selected closed-loop region assignment deleted', { entity_ids: dxfSelectedIds });
      setDxfSelectedIds([]);
      return;
    }

    if (dxfSelectedLineIds.length > 0 || dxfDetectedSurface) {
      console.log('[DXF Region] selected line-built surface cleared', { entity_ids: dxfSelectedLineIds });
      cancelDetectedSurface();
      return;
    }

    if (dxfManualSurfaces.length > 0) {
      const removed = dxfManualSurfaces[dxfManualSurfaces.length - 1];
      setDxfManualSurfaces((prev) => prev.slice(0, -1));
      console.log('[DXF Region] last manual surface deleted', { surface_id: removed.id });
    }
  };

  const dxfAssignmentRowsRaw = [
    ...dxfLoops
      .filter((loop) => dxfAssignments[loop.entity_id])
      .map((loop) => {
        const assignedType = dxfAssignments[loop.entity_id];
        const meta = dxfRegionMeta(assignedType);
        return {
          id: loop.entity_id,
          displayId: loop.entity_id,
          sourceType: 'closed_loop',
          assignedType,
          assignedLabel: meta.label,
          color: meta.color,
        };
      }),
    ...dxfManualSurfaces.map((surface) => {
      const meta = dxfOperationMeta(surface.assigned_operation);
      return {
        id: surface.id,
        displayId: surface.id,
        sourceType: 'line_surface',
        assignedType: surface.assigned_operation,
        assignedLabel: meta.label,
        color: meta.color,
      };
    }),
    // When the frame is auto-computed (a pocket carves it), add a synthetic row so the
    // operator can select and re-preview only the frame sections.
    ...(dxfHasPocketOrContour
      ? [
          {
            id: DXF_FRAME_SCOPE_ID,
            displayId: 'Computed Frame (all sections)',
            sourceType: 'computed_frame',
            assignedType: 'frame_level',
            assignedLabel: 'Frame (computed)',
            color: DXF_OPERATION_META.frame_level.color,
          },
        ]
      : []),
    // When a frame-level region is assigned (no pocket), the zigzag covers the WHOLE
    // door. Add ONE synthetic row carrying the door outline + zigzag toolpath, so each
    // individual frame-level region card can stay lean (just its 4 corners).
    ...(dxfHasFrameLevelRegion && !dxfHasPocketOrContour
      ? [
          {
            id: DXF_FRAME_LEVEL_DOOR_ID,
            displayId: 'Frame Level (whole door)',
            sourceType: 'frame_level_door',
            assignedType: 'frame_level',
            assignedLabel: 'Frame Level (door)',
            color: DXF_OPERATION_META.frame_level.color,
          },
        ]
      : []),
  ];

  // Operator-friendly, per-type numbered names ("Pocket 1", "3D Contour 2") instead
  // of raw DXF entity ids like "LWPOLYLINE-165".
  const dxfFriendlyBase = (label: string) => (label === 'Frame Level' ? 'Frame' : label);
  const dxfTypeCounters: Record<string, number> = {};
  const dxfAssignmentRows = dxfAssignmentRowsRaw.map((row) => {
    if (row.sourceType === 'computed_frame' || row.sourceType === 'frame_level_door') return row;
    const base = dxfFriendlyBase(row.assignedLabel);
    dxfTypeCounters[base] = (dxfTypeCounters[base] || 0) + 1;
    return { ...row, displayId: `${base} ${dxfTypeCounters[base]}` };
  });

  // Stitch a region's toolpath segments back into ordered [x,y] polylines (matches
  // the MoveL payload order): a path = first segment's start + every segment's end.
  const dxfSegmentsToPolylines = (
    segs: { start: number[]; end: number[]; id: string; tool: string; seq: number }[],
  ) => {
    // Frame-zigzag arrives from the backend as one or more already-safe polylines.
    // Rejoin each returned segment group for the info/export payload without adding
    // any extra connector that the backend did not approve.
    const frameZig = segs.filter((s) => /^frame_zigzag_/.test(s.id));
    const others = segs.filter((s) => !/^frame_zigzag_/.test(s.id));

    const out: { id?: string; points: number[][] }[] = [];
    const frameGroups = new Map<string, typeof segs>();
    for (const s of frameZig) {
      const key = s.id.replace(/_\d+$/, '');
      if (!frameGroups.has(key)) frameGroups.set(key, []);
      frameGroups.get(key)!.push(s);
    }
    for (const list of frameGroups.values()) {
      list.sort((a, b) => a.seq - b.seq);
      const id = list[0].id.replace(/_\d+$/, '');
      out.push({ id, points: [list[0].start, ...list.map((s) => s.end)] });
    }

    // Pocket/contour toolpaths split one continuous path into sub-segments id_0,
    // id_1, ... where each end == the next start; rejoin as first.start + every end.
    const groups = new Map<string, typeof segs>();
    for (const s of others) {
      const key = s.id.replace(/_\d+$/, '');
      if (!groups.has(key)) groups.set(key, []);
      groups.get(key)!.push(s);
    }
    for (const list of groups.values()) {
      list.sort((a, b) => a.seq - b.seq);
      const id = list[0].id.replace(/_\d+$/, '');
      out.push({ id, points: [list[0].start, ...list.map((s) => s.end)] });
    }
    return out;
  };

  // Region info for the inspector: corner-point shapes + ordered toolpath point lists.
  const computeDxfRegionInfo = (rowId: string, sourceType: string) => {
    const shapes: { id?: string; label: string; points: number[][] }[] = [];
    const toolpaths: { id?: string; label: string; points: number[][] }[] = [];
    const forRegion = (segs: { id: string }[]) => segs.filter((s) => s.id.startsWith(`${rowId}_`));

    if (sourceType === 'line_surface' || sourceType === 'closed_loop') {
      // The region's assigned operation decides which toolpaths belong to it.
      let assignedOperation: string | undefined;
      if (sourceType === 'line_surface') {
        const s = dxfManualSurfaces.find((x) => x.id === rowId);
        assignedOperation = s?.assigned_operation;
        if (s?.outer_points) shapes.push({ label: 'Outer corners', points: dxfDisplayCornerPoints(s.outer_points) });
        if (s?.holes?.[0]) shapes.push({ label: 'Inner corners', points: dxfDisplayCornerPoints(s.holes[0]) });
      } else {
        const l = dxfLoops.find((x) => x.entity_id === rowId);
        assignedOperation = dxfAssignments[rowId];
        if (l) shapes.push({ label: 'Corners', points: dxfDisplayCornerPoints(l.points) });
      }
      dxfSegmentsToPolylines(forRegion(dxfPocketToolpaths) as never).forEach((p) =>
        toolpaths.push({ id: p.id, label: 'Pocket contour · Tool 3', points: p.points }),
      );
      dxfSegmentsToPolylines(forRegion(dxfPocketZigzag) as never).forEach((p) =>
        toolpaths.push({ id: p.id, label: 'Pocket zigzag · Tool 4', points: p.points }),
      );
      dxfSegmentsToPolylines(forRegion(dxf3dContourToolpaths) as never).forEach((p) =>
        toolpaths.push({ id: p.id, label: '3D contour ring', points: p.points }),
      );
      // The frame-level zigzag is computed over the WHOLE door, not per-region, so it is
      // NOT attached to each individual frame-level region card (that repeated the whole
      // door toolpath on every card and confused the operator). Each frame-level region
      // card shows only its own 4 corner points; the whole-door zigzag lives on the
      // dedicated 'frame_level_door' synthetic row below.
    } else if (sourceType === 'frame_level_door') {
      // Whole-door Frame Level entry: the door's 4 outer corners + the single frame
      // zigzag toolpath, shown ONCE (not repeated per region).
      const outerBounds = dxfFrameBounds();
      if (outerBounds) {
        const { min_x: xLo, max_x: xHi, min_y: yLo, max_y: yHi } = outerBounds;
        shapes.push({
          label: 'Door outer corners',
          points: [
            [xLo, yLo],
            [xLo, yHi],
            [xHi, yHi],
            [xHi, yLo],
          ],
        });
      }
      dxfSegmentsToPolylines(dxfFrameZigzag).forEach((p) =>
        toolpaths.push({ label: 'Frame zigzag · Tool 4', points: p.points }),
      );
    } else if (sourceType === 'computed_frame') {
      // Overall door outer boundary = the door outline (bottom-right is the
      // machine-frame origin), listed CCW from the origin corner. Uses the selected
      // Frame region when present so it matches the frame toolpath exactly, else the
      // parse-time part_bbox.
      const outerBounds = dxfFrameBounds();
      if (outerBounds) {
        const { min_x: xLo, max_x: xHi, min_y: yLo, max_y: yHi } = outerBounds;
        shapes.push({
          label: 'Door outer corners',
          points: [
            [xLo, yLo],
            [xLo, yHi],
            [xHi, yHi],
            [xHi, yLo],
          ],
        });
      }
      dxfFrameChunks.forEach((c, i) => shapes.push({ id: c.chunk_id, label: `Section ${i + 1} corners`, points: dxfDisplayCornerPoints(c.points) }));
      dxfFrameSectionPaths.forEach((p, i) => toolpaths.push({ id: p.path_id, label: `Frame toolpath ${i + 1}`, points: p.points }));
      dxfSegmentsToPolylines(dxfFrameZigzag).forEach((p) => toolpaths.push({ label: 'Frame zigzag', points: p.points }));
    }
    return { shapes, toolpaths };
  };

  const deleteDxfAssignmentRow = (sourceType: string, id: string) => {
    if (sourceType === 'closed_loop') {
      setDxfAssignments((prev) => {
        const next = { ...prev };
        delete next[id];
        return next;
      });
      setDxfSelectedIds((prev) => prev.filter((selectedId) => selectedId !== id));
      console.log('[DXF Assignment] closed loop deleted', { id });
      return;
    }

    if (sourceType === 'line_surface') {
      setDxfManualSurfaces((prev) => prev.filter((surface) => surface.id !== id));
      setDxfSelectedSurfaceIds((prev) => prev.filter((surfaceId) => surfaceId !== id));
      console.log('[DXF Assignment] line surface deleted', { id });
      return;
    }
  };
  const handleSaveDxfMapping = () => {
    const mapping = dxfLoops
      .filter((loop) => dxfAssignments[loop.entity_id])
      .map((loop) => ({
        entity_id: loop.entity_id,
        layer: loop.layer,
        assigned_type: dxfAssignments[loop.entity_id],
        area: loop.area,
      }));
    // Backend save is intentionally not implemented yet.
    console.log('[DXF Region] Save Mapping clicked (backend save not implemented yet)', { mapping });
  };


  return (
    <div className="space-y-3">
      {/* --- CAD Assisted Mode: active workspace (Table B is always DXF Assisted) --- */}
      <>
        <div
          className="rounded-2xl border border-slate-200 bg-white shadow-sm"
          style={{ padding: '10px 12px', display: 'flex', flexWrap: 'wrap', alignItems: 'center', gap: '8px 12px' }}
        >
          {/* Scoped hover styles (frozen Tailwind can't add :hover utilities). */}
          <style>{`
            .dxf-upload-btn { background:#0f172a; color:#fff; border:1px solid #0f172a; border-radius:10px;
              padding:6px 14px; font-size:13px; font-weight:700; cursor:pointer; white-space:nowrap;
              display:inline-flex; align-items:center; transition:background .15s, border-color .15s; }
            .dxf-upload-btn:hover { background:#2563eb; border-color:#2563eb; }
            .dxf-upload-btn.is-disabled { opacity:.5; cursor:not-allowed; }
            .dxf-upload-btn.is-disabled:hover { background:#0f172a; border-color:#0f172a; }
          `}</style>
          <span className="text-sm font-semibold text-slate-900">DXF Upload</span>
          <span
            style={{
              ...dxfUploadBadgeStyle(dxfUploadStatusLabel),
              borderRadius: '999px',
              border: '1px solid',
              padding: '3px 10px',
              fontSize: '12px',
              fontWeight: 700,
              textTransform: 'capitalize',
            }}
          >
            {dxfUploadStatusLabel}
          </span>
          <label className={`dxf-upload-btn ${isOperating || dxfViewerStatus === 'loading' ? 'is-disabled' : ''}`}>
            {dxfViewerStatus === 'loading' ? 'Uploading…' : 'Upload File'}
            <input
              type="file"
              accept=".dxf"
              disabled={isOperating || dxfViewerStatus === 'loading'}
              onChange={(event) => {
                const file = event.target.files?.[0];
                if (file) handleLoadDxfIntoViewer(file);
                event.target.value = '';
              }}
              style={{ display: 'none' }}
            />
          </label>
          <button
            type="button"
            onClick={handleOpenDxfViewer}
            disabled={!canOpenDxfViewer || isOperating}
            style={{
              padding: '6px 12px',
              borderRadius: '10px',
              border: '1px solid #0f172a',
              background: canOpenDxfViewer && !isOperating ? '#0f172a' : '#e2e8f0',
              color: canOpenDxfViewer && !isOperating ? '#ffffff' : '#64748b',
              fontSize: '13px',
              fontWeight: 700,
              cursor: canOpenDxfViewer && !isOperating ? 'pointer' : 'not-allowed',
              whiteSpace: 'nowrap',
            }}
          >
            View 2D CAD
          </button>
          {/* One-line file summary (no parse details — they confuse the operator). */}
          <div
            className="text-xs text-slate-600"
            style={{ flexBasis: '100%', wordBreak: 'break-word', minHeight: '14px' }}
          >
            {dxfFileName
              ? `${dxfFileName} · Job ${dxfJobId || 'pending'}`
              : 'No DXF uploaded yet — press Upload File to select a .dxf.'}
          </div>
        </div>

        {isDxfViewerOpen &&
          createPortal(
            <div
              role="dialog"
              aria-modal="true"
              aria-label="2D DXF Viewer Workspace"
              style={{
                position: 'fixed',
                inset: 0,
                zIndex: 9999,
                background: 'rgba(255, 255, 255, 0.98)',
                display: 'flex',
                alignItems: 'center',
                justifyContent: 'center',
                padding: '24px',
              }}
            >
              <div
                style={{
                  width: '95vw',
                  height: '90vh',
                  border: '1px solid #cbd5e1',
                  borderRadius: '18px',
                  background: '#ffffff',
                  boxShadow: '0 24px 80px rgba(15,23,42,0.18)',
                  display: 'flex',
                  flexDirection: 'column',
                  overflow: 'hidden',
                }}
              >
                <div style={{ display: 'flex', alignItems: 'center', justifyContent: 'space-between', gap: '16px', padding: '14px 18px', borderBottom: '1px solid #e2e8f0', background: '#f8fafc' }}>
                  <div>
                    <div style={{ fontSize: '15px', fontWeight: 700, color: '#0f172a' }}>2D DXF Viewer Workspace</div>
                    <div style={{ fontSize: '12px', color: '#64748b', wordBreak: 'break-all' }}>{dxfFileName || 'DXF file'} · Job ID: {dxfJobId || 'pending'}</div>
                  </div>
                  <button type="button" onClick={handleCloseDxfViewer} style={{ padding: '8px 14px', borderRadius: '10px', border: '1px solid #cbd5e1', background: '#ffffff', color: '#0f172a', fontSize: '13px', fontWeight: 700, cursor: 'pointer' }}>
                    Close
                  </button>
                </div>
                <div
                  style={{
                    display: 'flex',
                    alignItems: 'center',
                    gap: '8px',
                    padding: '7px 12px',
                    borderBottom: '1px solid #dbe3ef',
                    background: '#f8fafc',
                    overflowX: 'auto',
                    whiteSpace: 'nowrap',
                  }}
                >
                  <span style={{ fontSize: '10px', fontWeight: 800, color: '#64748b', textTransform: 'uppercase', letterSpacing: '0.04em' }}>
                    Select
                  </span>
                  <button
                    type="button"
                    onClick={() => {
                      setDxfSelectionMode('loop');
                      setDxfSelectedLineIds([]);
                      setDxfDetectedSurface(null);
                      setDxfSurfaceMessage('');
                      setDxfLockedOperation(null);
                      console.log('[DXF Toolbar] selection mode changed', { mode: 'loop' });
                    }}
                    style={dxfToggleButtonStyle(dxfSelectionMode !== 'line')}
                  >
                    Closed Loop
                  </button>
                  <button
                    type="button"
                    onClick={() => {
                      setDxfSelectionMode('line');
                      setDxfSelectedIds([]);
                      setDxfRingLoopIds([]);
                      setDxfRingPreview(null);
                      console.log('[DXF Toolbar] selection mode changed', { mode: 'line' });
                    }}
                    style={dxfToggleButtonStyle(dxfSelectionMode === 'line')}
                  >
                    Lines to Surface
                  </button>

                  <span style={{ width: '1px', height: '24px', background: '#cbd5e1', flex: '0 0 auto' }} />
                  <span style={{ fontSize: '10px', fontWeight: 800, color: '#64748b', textTransform: 'uppercase', letterSpacing: '0.04em' }}>
                    Assign
                  </span>
                  {dxfToolbarAssignments.map((assignment) => {
                    // Region assignment is a design-time mapping action (no robot
                    // movement), so it is NOT gated on isOperating — only on whether
                    // there is something to assign (a detected surface, selected
                    // lines/loops, or a selected confirmed surface).
                    // Assign buttons stay enabled in Lines mode so the operator can
                    // lock/unlock the operation even with no preview.
                    const disabled =
                      dxfSelectedSurfaceIds.length === 0 &&
                      (dxfSelectionMode === 'line' ? false : dxfSelectedIds.length === 0);
                    const isLocked = dxfSelectionMode === 'line' && dxfLockedOperation === assignment.type;
                    return (
                      <button
                        key={assignment.type}
                        type="button"
                        disabled={disabled}
                        onClick={() => assignDxfToolbarRegion(assignment.type)}
                        title={
                          isLocked
                            ? `${assignment.label} locked — keep selecting lines; click again to unlock`
                            : dxfSelectionMode === 'line'
                            ? `Click to lock ${assignment.label} for repeated line-surface assignment`
                            : assignment.label
                        }
                        style={{
                          ...dxfSurfaceConfirmStyle(assignment.color),
                          opacity: disabled ? 0.45 : 1,
                          cursor: disabled ? 'not-allowed' : 'pointer',
                          outline: isLocked ? '2px solid #0f172a' : 'none',
                          outlineOffset: '1px',
                        }}
                      >
                        {isLocked ? `🔒 ${assignment.label}` : assignment.label}
                      </button>
                    );
                  })}

                  <span style={{ width: '1px', height: '24px', background: '#cbd5e1', flex: '0 0 auto' }} />
                  <span style={{ fontSize: '10px', fontWeight: 800, color: '#64748b', textTransform: 'uppercase', letterSpacing: '0.04em' }}>
                    Actions
                  </span>
                  <button type="button" onClick={clearDxfToolbarSelection} style={dxfPlainButtonStyle}>
                    Clear
                  </button>
                  <button
                    type="button"
                    onClick={deleteSelectedDxfRegion}
                    disabled={
                      dxfSelectedIds.length === 0 &&
                      dxfSelectedLineIds.length === 0 &&
                      !dxfDetectedSurface &&
                      dxfManualSurfaces.length === 0
                    }
                    style={{
                      ...dxfPlainButtonStyle,
                      opacity:
                        dxfSelectedIds.length === 0 &&
                        dxfSelectedLineIds.length === 0 &&
                        !dxfDetectedSurface &&
                        dxfManualSurfaces.length === 0
                          ? 0.45
                          : 1,
                      cursor:
                        dxfSelectedIds.length === 0 &&
                        dxfSelectedLineIds.length === 0 &&
                        !dxfDetectedSurface &&
                        dxfManualSurfaces.length === 0
                          ? 'not-allowed'
                          : 'pointer',
                    }}
                  >
                    Delete Region
                  </button>
                  <label style={{ display: 'inline-flex', alignItems: 'center', gap: '5px', fontSize: '11px', color: '#334155', flex: '0 0 auto' }}>
                    Pocket Overlap
                    <input
                      type="number"
                      min={0}
                      max={100}
                      step={1}
                      value={dxfPocketOverlap}
                      onChange={(event) => {
                        const v = Number(event.target.value);
                        setDxfPocketOverlap(Number.isFinite(v) ? Math.max(0, Math.min(100, v)) : 0);
                      }}
                      style={{ width: '52px', padding: '3px 6px', border: '1px solid #cbd5e1', borderRadius: '6px', fontSize: '11px' }}
                    />
                    mm
                  </label>
                  <label
                    title="Tool 3 pocket-edge safety margin, added to the fixed tool size (38.1 X / 50.8 Y). Larger = pass sits farther from the pocket edge. Applied on the next Preview Toolpath."
                    style={{ display: 'inline-flex', alignItems: 'center', gap: '5px', fontSize: '11px', color: '#334155', flex: '0 0 auto' }}
                  >
                    Pocket Edge Offset
                    <input
                      type="number"
                      min={0}
                      max={100}
                      step={0.5}
                      value={dxfPocketEdgeMargin}
                      onChange={(event) => {
                        const v = Number(event.target.value);
                        setDxfPocketEdgeMargin(Number.isFinite(v) ? Math.max(0, Math.min(100, v)) : TOOL3_DEFAULT_EDGE_MARGIN_MM);
                      }}
                      style={{ width: '52px', padding: '3px 6px', border: '1px solid #cbd5e1', borderRadius: '6px', fontSize: '11px' }}
                    />
                    mm
                  </label>
                  <label style={{ display: 'inline-flex', alignItems: 'center', gap: '5px', fontSize: '11px', color: '#334155', flex: '0 0 auto' }}>
                    Pocket ZigZag
                    <select
                      value={dxfPocketZigzagOrientation}
                      onChange={(event) => {
                        const v = event.target.value;
                        setDxfPocketZigzagOrientation(
                          v === 'horizontal' ? 'horizontal' : v === 'rectspiral' ? 'rectspiral' : 'vertical',
                        );
                      }}
                      title="Tool 4 pocket fill pattern"
                      style={{ padding: '3px 6px', border: '1px solid #cbd5e1', borderRadius: '6px', fontSize: '11px', background: '#ffffff' }}
                    >
                      <option value="vertical">Vertical</option>
                      <option value="horizontal">Horizontal</option>
                      <option value="rectspiral">Rectangular Spiral</option>
                    </select>
                  </label>
                  <label style={{ display: 'inline-flex', alignItems: 'center', gap: '5px', fontSize: '11px', color: '#334155', flex: '0 0 auto' }}>
                    Frame Overlap
                    <input
                      type="number"
                      min={0}
                      max={100}
                      step={1}
                      value={dxfFrameOverlap}
                      onChange={(event) => {
                        const v = Number(event.target.value);
                        setDxfFrameOverlap(Number.isFinite(v) ? Math.max(0, Math.min(100, v)) : 0);
                      }}
                      style={{ width: '52px', padding: '3px 6px', border: '1px solid #cbd5e1', borderRadius: '6px', fontSize: '11px' }}
                    />
                    mm
                  </label>
                  {(() => {
                    const scopeCount = dxfSelectedSurfaceIds.length + dxfSelectedIds.length;
                    return (
                      <button
                        type="button"
                        onClick={handleGenerateFramePreview}
                        disabled={!dxfJobId || dxfFrameStatus === 'loading'}
                        title={
                          scopeCount > 0
                            ? `Preview toolpath for the ${scopeCount} selected region(s) only`
                            : 'Preview toolpath for all regions. Select region rows / loops first to preview only those.'
                        }
                        style={{
                          ...dxfPlainButtonStyle,
                          background: '#0f172a',
                          color: '#ffffff',
                          opacity: !dxfJobId || dxfFrameStatus === 'loading' ? 0.45 : 1,
                          cursor: !dxfJobId || dxfFrameStatus === 'loading' ? 'not-allowed' : 'pointer',
                        }}
                      >
                        {scopeCount > 0 ? `Preview Toolpath (${scopeCount})` : 'Preview Toolpath'}
                      </button>
                    );
                  })()}
                  <button
                    type="button"
                    onClick={onApprovePreview}
                    disabled={dxfFrameStatus !== 'ready' || (previewStatus === 'approved' && !previewStale)}
                    style={{
                      ...dxfPlainButtonStyle,
                      background: '#16a34a',
                      color: '#ffffff',
                      borderColor: '#16a34a',
                      opacity: dxfFrameStatus !== 'ready' || (previewStatus === 'approved' && !previewStale) ? 0.45 : 1,
                      cursor: dxfFrameStatus !== 'ready' || (previewStatus === 'approved' && !previewStale) ? 'not-allowed' : 'pointer',
                    }}
                  >
                    {previewStatus === 'approved' && !previewStale ? 'Approved ✓' : previewStale ? 'Re-approve' : 'Approve'}
                  </button>
                  <span style={{ marginLeft: 'auto', fontSize: '11px', color: '#64748b', flex: '0 0 auto' }}>
                    {dxfFrameStatus !== 'idle' ? `Preview: ${dxfFrameStatus}` : ''}
                  </span>
                </div>
                <div style={{ borderBottom: '1px solid #e2e8f0', background: '#ffffff', maxHeight: '104px', overflow: 'auto' }}>
                  <table style={{ width: '100%', borderCollapse: 'collapse', fontSize: '11px', color: '#334155' }}>
                    <thead style={{ position: 'sticky', top: 0, background: '#f8fafc', zIndex: 1 }}>
                      <tr>
                        <th style={{ width: '34%', padding: '5px 10px', textAlign: 'left', borderBottom: '1px solid #e2e8f0', fontWeight: 800 }}>id</th>
                        <th style={{ width: '20%', padding: '5px 10px', textAlign: 'left', borderBottom: '1px solid #e2e8f0', fontWeight: 800 }}>source type</th>
                        <th style={{ width: '28%', padding: '5px 10px', textAlign: 'left', borderBottom: '1px solid #e2e8f0', fontWeight: 800 }}>assigned type</th>
                        <th style={{ width: '18%', padding: '5px 10px', textAlign: 'right', borderBottom: '1px solid #e2e8f0', fontWeight: 800 }}>delete</th>
                      </tr>
                    </thead>
                    <tbody>
                      {dxfAssignmentRows.length === 0 ? (
                        <tr>
                          <td colSpan={4} style={{ padding: '7px 10px', color: '#94a3b8' }}>
                            No DXF assignments yet.
                          </td>
                        </tr>
                      ) : (
                        dxfAssignmentRows.map((row) => {
                          const isSurfaceRow = row.sourceType === 'line_surface';
                          const isLoopRow = row.sourceType === 'closed_loop';
                          // Both synthetic rows (Computed Frame, whole-door Frame Level) behave the
                          // same in the table: selectable, no delete, Info shows the door-level view.
                          const isFrameRow =
                            row.sourceType === 'computed_frame' || row.sourceType === 'frame_level_door';
                          const isSelectable = isSurfaceRow || isLoopRow || isFrameRow;
                          const selected =
                            (isSurfaceRow && dxfSelectedSurfaceIds.includes(row.id)) ||
                            (isFrameRow && dxfSelectedSurfaceIds.includes(row.id)) ||
                            (isLoopRow && dxfSelectedIds.includes(row.id));
                          const hovered = dxfHoveredRowId === row.id;
                          return (
                            <tr
                              key={`${row.sourceType}-${row.id}`}
                              onMouseEnter={() => setDxfHoveredRowId(row.id)}
                              onMouseLeave={() => setDxfHoveredRowId((prev) => (prev === row.id ? null : prev))}
                              onClick={() => {
                                // Selecting a row scopes the toolpath preview to that region.
                                // The synthetic "Computed Frame" row uses the surface-id set too.
                                if (isSurfaceRow || isFrameRow) {
                                  setDxfSelectedSurfaceIds((prev) =>
                                    prev.includes(row.id) ? prev.filter((id) => id !== row.id) : [...prev, row.id],
                                  );
                                } else if (isLoopRow) {
                                  setDxfSelectedIds((prev) =>
                                    prev.includes(row.id) ? prev.filter((id) => id !== row.id) : [...prev, row.id],
                                  );
                                }
                              }}
                              style={{
                                background: hovered ? '#fff7ed' : selected ? '#e0f2fe' : '#ffffff',
                                outline: hovered ? '2px solid #f97316' : 'none',
                                outlineOffset: '-2px',
                                cursor: isSelectable ? 'pointer' : 'default',
                              }}
                            >
                              <td style={{ padding: '5px 10px', borderBottom: '1px solid #f1f5f9', fontFamily: 'monospace', whiteSpace: 'nowrap', overflow: 'hidden', textOverflow: 'ellipsis', maxWidth: '240px' }}>
                                {row.displayId}
                              </td>
                              <td style={{ padding: '5px 10px', borderBottom: '1px solid #f1f5f9' }}>{row.sourceType}</td>
                              <td style={{ padding: '5px 10px', borderBottom: '1px solid #f1f5f9' }}>
                                <span style={{ display: 'inline-flex', alignItems: 'center', gap: '6px' }}>
                                  <span style={{ width: '9px', height: '9px', borderRadius: '999px', background: row.color, border: row.assignedType === 'outer' || row.assignedType === 'outer_boundary' ? `2px solid ${row.color}` : 'none' }} />
                                  {row.assignedLabel}
                                </span>
                              </td>
                              <td style={{ padding: '4px 10px', borderBottom: '1px solid #f1f5f9', textAlign: 'right', whiteSpace: 'nowrap' }}>
                                <button
                                  type="button"
                                  onClick={(event) => {
                                    event.stopPropagation();
                                    setDxfInfoExpanded({}); // new region opens collapsed
                                    setDxfSelectedToolpathId(null);
                                    setDxfSelectedFrameSectionId(null);
                                    setDxfSelectedFramePathId(null);
                                    setDxfSelectedOperationToolpathId(null);
                                    setDxfHoveredRowId(null);
                                    setDxfInfoRowId((prev) => (prev === row.id ? null : row.id));
                                  }}
                                  style={{ ...dxfPlainButtonStyle, padding: '3px 7px', fontSize: '10px', marginRight: '5px' }}
                                >
                                  Info
                                </button>
                                {/* The computed-frame row isn't a real assignment, so it can't be deleted. */}
                                {!isFrameRow && (
                                  <button
                                    type="button"
                                    onClick={(event) => {
                                      event.stopPropagation();
                                      deleteDxfAssignmentRow(row.sourceType, row.id);
                                    }}
                                    style={{ ...dxfPlainButtonStyle, padding: '3px 7px', fontSize: '10px' }}
                                  >
                                    Delete
                                  </button>
                                )}
                              </td>
                            </tr>
                          );
                        })
                      )}
                    </tbody>
                  </table>
                </div>
                <div style={{ flex: 1, minHeight: 0, padding: '14px', background: '#f8fafc', position: 'relative' }}>
                  <div style={{ width: '100%', height: '100%', minHeight: 0, border: '1px solid #e2e8f0', borderRadius: '14px', overflow: 'hidden', background: '#ffffff' }}>
                    <Dxf2DViewer
                      loops={dxfLoops}
                      openPaths={dxfOpenPaths}
                      selectedIds={dxfSelectedIds}
                      highlightId={dxfHoveredRowId ?? dxfInfoRowId}
                      assignments={dxfAssignments}
                      onToggleSelect={handleToggleDxfLoop}
                      selectionMode={dxfSelectionMode}
                      selectedLineIds={dxfSelectedLineIds}
                      onToggleLine={handleToggleDxfLine}
                      surfacePreview={dxfDetectedSurface ? { outer: dxfDetectedSurface.outer, holes: dxfDetectedSurface.holes } : null}
                      manualSurfaces={dxfManualSurfaces}
                      framePolygons={dxfFramePolygons}
                      frameAreaPolygons={dxfFrameArea}
                      frameRectangles={dxfFrameRectangles}
                      frameToolpaths={dxfFrameToolpaths}
                      pocketToolpaths={dxfPocketToolpaths}
                      pocketZigzag={dxfPocketZigzag}
                      contourToolpaths={dxf3dContourToolpaths}
                      frameZigzag={dxfFrameZigzag}
                      frameSections={dxfFrameSections}
                      frameChunks={dxfFrameChunks}
                      frameSectionPaths={dxfFrameSectionPaths}
                      tool2Sides={dxfTool2Sides}
                      showFrame={dxfShowFrame}
                      showToolpaths={dxfShowToolpaths}
                      selectedToolpathId={dxfSelectedToolpathId}
                      selectedFrameSectionId={dxfSelectedFrameSectionId}
                      selectedFramePathId={dxfSelectedFramePathId}
                      selectedOperationToolpathId={dxfSelectedOperationToolpathId}
                      onSelectToolpath={(rectId: string) => {
                        setDxfSelectedToolpathId((prev) => (prev === rectId ? null : rectId));
                        console.log('[DXF Frame] toolpath selected', { rect_id: rectId });
                      }}
                    />
                  </div>
                  {dxfInfoRowId &&
                    (() => {
                      const row = dxfAssignmentRows.find((r) => r.id === dxfInfoRowId);
                      if (!row) return null;
                      const info = computeDxfRegionInfo(row.id, row.sourceType);
                      const fmt = (pts: number[][]) =>
                        pts.map((p) => `(${p[0].toFixed(1)}, ${p[1].toFixed(1)})`).join('  ');
                      const pathLen = (pts: number[][]) => {
                        let d = 0;
                        for (let i = 1; i < pts.length; i++) d += Math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1]);
                        return d;
                      };
                      // What this region IS, in one plain sentence.
                      const opLabel =
                        row.sourceType === 'frame_level_door' ? 'Frame Level — the whole door surface'
                        : row.sourceType === 'computed_frame' ? 'Computed Frame — leftover frame split into reachable sections'
                        : row.assignedLabel || 'Region';
                      const totalToolpathLen = info.toolpaths.reduce((s, t) => s + pathLen(t.points), 0);
                      const isOpen = (key: string) => !!dxfInfoExpanded[key];
                      const toggle = (key: string) => setDxfInfoExpanded((prev) => ({ ...prev, [key]: !prev[key] }));
                      const Row = ({ label, value, hint }: { label: string; value: string; hint?: string }) => (
                        <div style={{ display: 'flex', justifyContent: 'space-between', gap: '10px', padding: '3px 0' }}>
                          <span style={{ color: '#64748b' }} title={hint}>{label}{hint ? ' ⓘ' : ''}</span>
                          <span style={{ fontWeight: 700, color: '#0f172a', textAlign: 'right' }}>{value}</span>
                        </div>
                      );
                      const Section = ({ id, title, count, active = false, onActivate, children }: { id: string; title: string; count: string; active?: boolean; onActivate?: () => void; children: React.ReactNode }) => (
                        <div style={{ borderTop: '1px solid #e2e8f0', paddingTop: '8px', marginTop: '8px' }}>
                          <button
                            type="button"
                            onClick={(event) => {
                              const panel = dxfInfoPanelRef.current;
                              const scrollTop = panel?.scrollTop ?? 0;
                              onActivate?.();
                              toggle(id);
                              event.currentTarget.blur();
                              window.requestAnimationFrame(() => {
                                if (dxfInfoPanelRef.current) dxfInfoPanelRef.current.scrollTop = scrollTop;
                              });
                            }}
                            style={{ ...dxfPlainButtonStyle, width: '100%', display: 'flex', justifyContent: 'space-between', alignItems: 'center', padding: '4px 6px', fontSize: '11px', fontWeight: 800, color: active ? '#be123c' : '#334155', background: active ? '#ffe4e6' : isOpen(id) ? '#f1f5f9' : '#ffffff', borderColor: active ? '#fb7185' : '#cbd5e1' }}
                          >
                            <span>{title} <span style={{ color: active ? '#e11d48' : '#94a3b8', fontWeight: 600 }}>· {count}</span></span>
                            <span style={{ color: active ? '#e11d48' : '#94a3b8' }}>{isOpen(id) ? '▾' : '▸'}</span>
                          </button>
                          {isOpen(id) && <div style={{ padding: '6px 4px 2px' }}>{children}</div>}
                        </div>
                      );
                      const onInfoDragStart = (event: React.MouseEvent) => {
                        // Start from the panel's current on-screen box so the first drag
                        // doesn't jump, whether it's still at the default corner or already moved.
                        const box = (event.currentTarget.parentElement as HTMLElement)?.getBoundingClientRect();
                        const origin = dxfInfoPos ?? (box ? { x: box.left, y: box.top } : { x: 0, y: 0 });
                        dxfInfoDragRef.current = { sx: event.clientX, sy: event.clientY, ox: origin.x, oy: origin.y };
                        const move = (e: MouseEvent) => {
                          if (!dxfInfoDragRef.current) return;
                          const { sx, sy, ox, oy } = dxfInfoDragRef.current;
                          setDxfInfoPos({
                            x: Math.max(0, Math.min(window.innerWidth - 80, ox + (e.clientX - sx))),
                            y: Math.max(0, Math.min(window.innerHeight - 40, oy + (e.clientY - sy))),
                          });
                        };
                        const up = () => {
                          dxfInfoDragRef.current = null;
                          window.removeEventListener('mousemove', move);
                          window.removeEventListener('mouseup', up);
                        };
                        window.addEventListener('mousemove', move);
                        window.addEventListener('mouseup', up);
                      };
                      return (
                        <div
                          ref={dxfInfoPanelRef}
                          style={{
                            position: 'fixed',
                            ...(dxfInfoPos
                              ? { top: `${dxfInfoPos.y}px`, left: `${dxfInfoPos.x}px` }
                              : { top: '24px', right: '24px' }),
                            width: '340px',
                            maxHeight: 'calc(100vh - 48px)',
                            overflowY: 'auto',
                            background: '#ffffff',
                            border: '1px solid #cbd5e1',
                            borderRadius: '12px',
                            boxShadow: '0 10px 30px rgba(15,23,42,0.18)',
                            paddingBottom: '14px',
                            fontSize: '11px',
                            zIndex: 40,
                          }}
                        >
                          {/* Draggable title bar. */}
                          <div
                            onMouseDown={onInfoDragStart}
                            style={{
                              display: 'flex', justifyContent: 'space-between', alignItems: 'center',
                              padding: '10px 14px 8px', cursor: 'move', userSelect: 'none',
                              borderBottom: '1px solid #e2e8f0', marginBottom: '10px',
                              position: 'sticky', top: 0, background: '#ffffff', zIndex: 1,
                            }}
                          >
                            <span style={{ display: 'flex', flexDirection: 'column' }}>
                              <strong style={{ fontSize: '13px' }}>{row.displayId}</strong>
                              <span style={{ color: '#94a3b8', fontSize: '10px' }}>drag to move</span>
                            </span>
                            <button
                              type="button"
                              onMouseDown={(e) => e.stopPropagation()}
                              onClick={() => {
                                setDxfInfoRowId(null);
                                setDxfHoveredRowId(null);
                                setDxfSelectedToolpathId(null);
                                setDxfSelectedFrameSectionId(null);
                                setDxfSelectedFramePathId(null);
                                setDxfSelectedOperationToolpathId(null);
                              }}
                              style={{ ...dxfPlainButtonStyle, padding: '2px 8px', fontSize: '11px' }}
                            >
                              Close
                            </button>
                          </div>
                          <div style={{ padding: '0 14px' }}>
                          <div style={{ color: '#64748b', marginBottom: '8px', lineHeight: 1.4 }}>{opLabel}</div>

                          {/* Plain-language summary — the numbers that matter, always visible. */}
                          <div style={{ background: '#f8fafc', border: '1px solid #e2e8f0', borderRadius: '8px', padding: '8px 10px' }}>
                            <Row label="Corner shapes" value={`${info.shapes.length}`} hint="Outlines that bound this region." />
                            <Row label="Sanding passes" value={`${info.toolpaths.length}`} hint="Separate strokes the tool will run here." />
                            <Row label="Total pass length" value={`${totalToolpathLen.toFixed(0)} mm`} hint="Sum of all pass lengths — how far the tool travels sanding." />
                            {info.toolpaths.length === 0 && (
                              <div style={{ color: '#94a3b8', paddingTop: '4px' }}>No toolpath yet — run Preview Toolpath.</div>
                            )}
                          </div>

                          {/* Corner shapes - collapsed; each with its own point count. */}
                          {info.shapes.map((s, i) => {
                            const isFrameSection = row.sourceType === 'computed_frame' && !!s.id;
                            return (
                              <Section
                                key={`shape-${i}`}
                                id={`shape-${i}`}
                                title={s.label}
                                count={`${s.points.length} corner${s.points.length === 1 ? '' : 's'}`}
                                active={isFrameSection && dxfSelectedFrameSectionId === s.id}
                                onActivate={isFrameSection ? () => {
                                  setDxfSelectedFrameSectionId(s.id || null);
                                  setDxfSelectedFramePathId(null);
                                  setDxfSelectedOperationToolpathId(null);
                                } : undefined}
                              >
                                <div style={{ fontFamily: 'monospace', color: '#0f172a', wordBreak: 'break-word', fontSize: '10.5px', lineHeight: 1.5 }}>{fmt(s.points)}</div>
                                <div style={{ color: '#94a3b8', marginTop: '4px' }}>(x, y) in mm from the UCS origin.</div>
                              </Section>
                            );
                          })}

                          {/* Each pass — its length and point list, expandable. */}
                          {info.toolpaths.map((t, i) => {
                            const len = pathLen(t.points);
                            const isFramePath = row.sourceType === 'computed_frame' && !!t.id;
                            const isOperationPath = row.sourceType !== 'computed_frame' && !!t.id;
                            return (
                              <Section
                                key={`tp-${i}`}
                                id={`tp-${i}`}
                                title={t.label}
                                count={`${len.toFixed(0)} mm · ${t.points.length} pts`}
                                active={(isFramePath && dxfSelectedFramePathId === t.id) || (isOperationPath && dxfSelectedOperationToolpathId === t.id)}
                                onActivate={isFramePath ? () => {
                                  setDxfSelectedFramePathId(t.id || null);
                                  setDxfSelectedFrameSectionId(null);
                                  setDxfSelectedOperationToolpathId(null);
                                } : isOperationPath ? () => {
                                  setDxfSelectedOperationToolpathId(t.id || null);
                                  setDxfSelectedFramePathId(null);
                                  setDxfSelectedFrameSectionId(null);
                                } : undefined}
                              >
                                <div style={{ fontFamily: 'monospace', color: '#0f172a', wordBreak: 'break-word', fontSize: '10.5px', lineHeight: 1.5 }}>{fmt(t.points)}</div>
                                <div style={{ color: '#94a3b8', marginTop: '4px' }}>Points in order - the tool moves through them as straight MoveL segments.</div>
                              </Section>
                            );
                          })}
                          </div>
                        </div>
                      );
                    })()}
                </div>
              </div>
            </div>,
            document.body,
          )}
      </>
      {/* --- CAD Assisted Mode: active workspace ends --- */}
    </div>
  );
}
