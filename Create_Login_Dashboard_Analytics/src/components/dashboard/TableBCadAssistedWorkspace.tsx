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
}

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
        const outer = loops.reduce((best, lp) => (bboxExtent(lp) > bboxExtent(best) ? lp : best), loops[0]);

        // Carve down to the INNERMOST box so the whole frame fills, no matter how
        // polygonize split the bands. Every other face's exterior AND every face's
        // holes are candidates; dxfInnerRegions keeps the deepest nested one.
        const outerArea = dxfPolygonArea(outer.points);
        const detectedExtras = [
          ...loops
            .filter((lp) => lp !== outer)
            .map((lp) => ({ points: lp.points, id: lp.loop_id, area: dxfPolygonArea(lp.points) })),
          ...loops.flatMap((lp) =>
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
  // Remaining frame surface (Outer Boundary − Pocket − 3D Contour) decomposed into
  // non-overlapping rectangular sections. Toolpaths for these come in a later task.
  const [dxfFrameSections, setDxfFrameSections] = React.useState<
    {
      section_id: string;
      points: number[][];
      bbox: { min_x: number; min_y: number; max_x: number; max_y: number };
      width: number;
      height: number;
      orientation: 'horizontal' | 'vertical';
      covered: boolean;
    }[]
  >([]);
  // Frame sections split into robot-reachable chunks (no chunk wider than the X
  // reach window or taller than the Y reach window).
  const [dxfFrameChunks, setDxfFrameChunks] = React.useState<
    {
      chunk_id: string;
      parent_section_id: string;
      bbox: { min_x: number; min_y: number; max_x: number; max_y: number };
      points: number[][];
      requires_axis_position: boolean;
    }[]
  >([]);
  // One preview sanding path per reachable chunk (centerline along its long axis).
  const [dxfFrameSectionPaths, setDxfFrameSectionPaths] = React.useState<
    {
      path_id: string;
      source_section_id: string;
      source_chunk_id: string;
      region_type: 'computed_frame';
      operation_type: 'frame_section_pass';
      points: number[][];
      direction: 'X' | 'Y';
      start_point: number[];
      end_point: number[];
    }[]
  >([]);
  const [dxfMmPerUnit, setDxfMmPerUnit] = React.useState(1);
  // The part's extent in the machine frame, from parse-time normalization. It is
  // derived from the outline layer alone, so it stays correct when other layers
  // (e.g. grooves) overhang the part edge. Null until a DXF reports one.
  const [dxfPartBBox, setDxfPartBBox] = React.useState<DxfBBox>(null);
  // The door outline stitched from line segments at parse time (largest closed
  // polygon). Excludes dangling fragments that share the outline's layer (e.g.
  // prongs on layer "0"), which part_bbox cannot. Null when nothing closed cleanly.
  const [dxfOutlineBBox, setDxfOutlineBBox] = React.useState<DxfBBox>(null);
  const [dxfFrameWarnings, setDxfFrameWarnings] = React.useState<TableBDxfFrameWarning[]>([]);
  const [dxfShowFrame, setDxfShowFrame] = React.useState(true);
  const [dxfShowToolpaths, setDxfShowToolpaths] = React.useState(true);
  const [dxfSelectedToolpathId, setDxfSelectedToolpathId] = React.useState<string | null>(null);
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
    setDxfFrameSections([]);
    setDxfFrameChunks([]);
    setDxfFrameSectionPaths([]);
    setDxfFrameWarnings([]);
    setDxfSelectedToolpathId(null);
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
  // The tool center sits 38.1/50.8 mm in from the pocket edge, plus a 2.5 mm margin.
  const TOOL3_OFFSET_X_MM = 38.1 + 2.5;
  const TOOL3_OFFSET_Y_MM = 50.8 + 2.5;

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

  // Distribute passes evenly across a span so the first and last passes land on the
  // two ends (matches _calculate_zigzag_pass_spacing: passes = num_steps + 1).
  const calcZigzagPassSpacing = (span: number, stepMm: number) => {
    const step = stepMm > 1e-9 ? stepMm : span;
    const numSteps = Math.max(1, Math.round(span / step));
    return { numSteps, adjustedStep: span / numSteps };
  };

  // Build the Tool 4 zigzag fill for every assigned pocket: vertical passes that
  // span the pocket's Y and step across in X, alternating up/down (zigzag), bounded
  // 72 mm in from each edge, starting at the bottom-right corner. Step spacing comes
  // from the operator's pocket overlap: step = pass width - overlap.
  const computeDxfPocketZigzag = (scope: Set<string> | null) => {
    const off = TOOL4_OFFSET_MM / dxfMmPerUnit;
    const overlap = Math.max(0, Math.min(100, dxfPocketOverlap));
    const stepMmEffective = Math.max(TOOL4_PASS_WIDTH_MM - overlap, 1);
    const step = stepMmEffective / dxfMmPerUnit;

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
      const xinner = bxHi - bxLo;
      const { numSteps, adjustedStep } = calcZigzagPassSpacing(xinner, step);
      console.log('[Pocket Toolpath] Tool 4 zigzag', {
        pocket: pk.id,
        overlap_mm: overlap,
        step_mm: stepMmEffective,
        passes: numSteps + 1,
      });

      // Flat point list: each vertical pass is (x, byLo)->(x, byHi), reversed on
      // alternate passes; consecutive points also form the horizontal step-overs.
      const points: number[][] = [];
      let offset = 0;
      let toggle = 0;
      // Start at the bottom-right corner: physical right = min X, bottom = min Y.
      while (offset <= xinner + 1e-9) {
        const x = bxLo + offset;
        const row = [
          [x, byLo],
          [x, byHi],
        ];
        if (toggle) row.reverse();
        points.push(...row);
        offset += adjustedStep;
        toggle = 1 - toggle;
      }
      for (let i = 0; i < points.length - 1; i++) {
        segments.push({
          start: points[i],
          end: points[i + 1],
          id: `${pk.id}_tool4_${i}`,
          tool: 'tool_4',
          seq: i,
        });
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
    const segs: { start: number[]; end: number[]; id: string; tool: string; seq: number }[] = [];
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
  // Prefer the operator's selected Frame Level / Outer Boundary geometry. That
  // is the only reliable way to prevent stray DXF fragments from stretching the
  // computed frame outside the door closed loop.
  const dxfFrameBounds = (): DxfBBox => {
    // 1. Prefer the operator's selected Frame Level / Outer Boundary surfaces.
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
  // Frame Tool 4 zigzag: only when the part has NO pocket (just frame level +
  // outer boundary, or the whole door is outer boundary). Unlike the pocket zigzag
  // it has NO offset and starts at the machine origin (0,0 = bottom-right corner),
  // filling the full frame extent with the operator's step size.
  const computeDxfFrameZigzag = () => {
    const hasPocket =
      dxfManualSurfaces.some((s) => s.assigned_operation === 'pocket_floor') ||
      dxfLoops.some((l) => dxfAssignments[l.entity_id] === 'pocket');
    const hasFrameOrOuter =
      dxfManualSurfaces.some((s) => s.assigned_operation === 'frame_level' || s.assigned_operation === 'outer_boundary') ||
      dxfLoops.some((l) => dxfAssignments[l.entity_id] === 'frame' || dxfAssignments[l.entity_id] === 'outer');
    if (hasPocket || !hasFrameOrOuter) return [];

    // Full frame extent = the part's outline. Prefer the backend's part_bbox: it is
    // derived from the outline layer alone, so layers that overhang the part edge
    // (e.g. grooves) cannot inflate it, and the selected Frame region trims stray
    // fragments that share the outline's layer.
    const bounds = dxfFrameBounds();
    if (!bounds) return [];
    const bxLo = bounds.min_x;
    const bxHi = bounds.max_x;
    const byLo = bounds.min_y;
    const byHi = bounds.max_y;
    const xinner = bxHi - bxLo;
    if (xinner <= 0 || byHi <= byLo) return [];

    const overlap = Math.max(0, Math.min(100, dxfPocketOverlap));
    const stepMmEffective = Math.max(TOOL4_PASS_WIDTH_MM - overlap, 1);
    const step = stepMmEffective / dxfMmPerUnit;
    const { numSteps, adjustedStep } = calcZigzagPassSpacing(xinner, step);
    console.log('[Frame Toolpath] Tool 4 zigzag (no offset, from origin)', {
      overlap_mm: overlap,
      step_mm: stepMmEffective,
      passes: numSteps + 1,
    });

    const points: number[][] = [];
    let offset = 0;
    let toggle = 0;
    while (offset <= xinner + 1e-9) {
      const x = bxLo + offset;
      const row = [
        [x, byLo],
        [x, byHi],
      ];
      if (toggle) row.reverse();
      points.push(...row);
      offset += adjustedStep;
      toggle = 1 - toggle;
    }
    const segments: { start: number[]; end: number[]; id: string; tool: string; seq: number }[] = [];
    for (let i = 0; i < points.length - 1; i++) {
      segments.push({ start: points[i], end: points[i + 1], id: `frame_tool4_${i}`, tool: 'tool_4_frame', seq: i });
    }
    return segments;
  };

  // Compute the remaining frame surface and split it into non-overlapping rectangles:
  //   remaining_frame = Outer Boundary − Pocket − 3D Contour
  // The Outer Boundary is the full part extent; Pockets and 3D-contour ring footprints
  // are the obstacles removed from it. The leftover rectilinear region is decomposed
  // with a coordinate grid + greedy maximal-rectangle merge, so sections are unique,
  // non-overlapping, and never cover a Pocket or 3D Contour. No toolpaths yet.
  const computeDxfFrameSections = () => {
    // The frame's outer boundary is the door outline, not a bbox over every entity:
    // overhanging layers and stray fragments would otherwise stretch the sections
    // past the part edge and shift every section corner.
    const outerBounds = dxfFrameBounds();
    if (!outerBounds) return [];
    const outer = { x0: outerBounds.min_x, x1: outerBounds.max_x, y0: outerBounds.min_y, y1: outerBounds.max_y };

    const boxOf = (pts: number[][]) => {
      const xs = pts.map((p) => p[0]);
      const ys = pts.map((p) => p[1]);
      return { x0: Math.min(...xs), x1: Math.max(...xs), y0: Math.min(...ys), y1: Math.max(...ys) };
    };
    // Obstacles removed from the frame: pocket footprints + 3D-contour ring footprints.
    const obstacles = [
      ...dxfManualSurfaces.filter((s) => s.assigned_operation === 'pocket_floor' && s.outer_points).map((s) => boxOf(s.outer_points as number[][])),
      ...dxfLoops.filter((l) => dxfAssignments[l.entity_id] === 'pocket').map((l) => boxOf(l.points)),
      ...dxfManualSurfaces.filter((s) => s.assigned_operation === 'surface_3d_area' && s.outer_points).map((s) => boxOf(s.outer_points as number[][])),
      ...dxfLoops.filter((l) => dxfAssignments[l.entity_id] === 'surface3d').map((l) => boxOf(l.points)),
    ];

    // Coordinate grid from all rectangle edges, clamped to the outer boundary.
    const uniq = (arr: number[]) =>
      Array.from(new Set(arr.map((v) => Math.round(v * 1e4) / 1e4))).sort((a, b) => a - b);
    const xs = uniq([outer.x0, outer.x1, ...obstacles.flatMap((o) => [o.x0, o.x1])].filter((v) => v >= outer.x0 - 1e-9 && v <= outer.x1 + 1e-9));
    const ys = uniq([outer.y0, outer.y1, ...obstacles.flatMap((o) => [o.y0, o.y1])].filter((v) => v >= outer.y0 - 1e-9 && v <= outer.y1 + 1e-9));
    const cols = xs.length - 1;
    const rows = ys.length - 1;
    if (cols <= 0 || rows <= 0) return [];

    const inObstacle = (cx: number, cy: number) =>
      obstacles.some((o) => cx > o.x0 + 1e-9 && cx < o.x1 - 1e-9 && cy > o.y0 + 1e-9 && cy < o.y1 - 1e-9);

    // frame[j][i] = true when the grid cell is part of the remaining frame.
    const frame: boolean[][] = [];
    for (let j = 0; j < rows; j++) {
      frame[j] = [];
      for (let i = 0; i < cols; i++) {
        const cx = (xs[i] + xs[i + 1]) / 2;
        const cy = (ys[j] + ys[j + 1]) / 2;
        frame[j][i] = !inObstacle(cx, cy);
      }
    }

    const used = frame.map((row) => row.map(() => false));
    const sections: {
      section_id: string;
      points: number[][];
      bbox: { min_x: number; min_y: number; max_x: number; max_y: number };
      width: number;
      height: number;
      orientation: 'horizontal' | 'vertical';
      covered: boolean;
    }[] = [];
    let counter = 0;
    for (let j = 0; j < rows; j++) {
      for (let i = 0; i < cols; i++) {
        if (!frame[j][i] || used[j][i]) continue;
        // Grow right across contiguous frame cells, then down while the full span stays frame.
        let i2 = i;
        while (i2 + 1 < cols && frame[j][i2 + 1] && !used[j][i2 + 1]) i2++;
        let j2 = j;
        let canExtend = true;
        while (canExtend && j2 + 1 < rows) {
          for (let k = i; k <= i2; k++) {
            if (!frame[j2 + 1][k] || used[j2 + 1][k]) {
              canExtend = false;
              break;
            }
          }
          if (canExtend) j2++;
        }
        for (let jj = j; jj <= j2; jj++) for (let ii = i; ii <= i2; ii++) used[jj][ii] = true;

        const x0 = xs[i];
        const x1 = xs[i2 + 1];
        const y0 = ys[j];
        const y1 = ys[j2 + 1];
        const width = x1 - x0;
        const height = y1 - y0;
        if (width <= 1e-6 || height <= 1e-6) continue;
        counter += 1;
        sections.push({
          section_id: `frame_section_${String(counter).padStart(3, '0')}`,
          points: [
            [x0, y0],
            [x1, y0],
            [x1, y1],
            [x0, y1],
          ],
          bbox: { min_x: x0, min_y: y0, max_x: x1, max_y: y1 },
          width,
          height,
          orientation: width >= height ? 'horizontal' : 'vertical',
          covered: false,
        });
      }
    }
    console.log('[Frame Sections] remaining_frame decomposed', {
      sections: sections.length,
      obstacles: obstacles.length,
    });
    return sections;
  };

  // Robot reach window (mm). A single sanding pass can only span this far in one
  // base position; larger frame sections must be split into reachable chunks.
  const REACH_X_MM = 515;
  const REACH_Y_MM = 750;

  type DxfFrameSection = (typeof dxfFrameSections)[number];
  type DxfFrameChunk = (typeof dxfFrameChunks)[number];

  // Split each frame section into robot-reachable chunks. A section wider than the
  // X reach is divided along X, taller than the Y reach along Y, into evenly-sized
  // chunks (each within the window). Chunks tile the section exactly, so they don't
  // overlap, stay inside the remaining frame, and never cover a pocket / 3D contour.
  const splitFrameSectionsByReach = (sections: DxfFrameSection[]): DxfFrameChunk[] => {
    const reachX = REACH_X_MM / dxfMmPerUnit;
    const reachY = REACH_Y_MM / dxfMmPerUnit;
    const chunks: DxfFrameChunk[] = [];
    let counter = 0;
    for (const section of sections) {
      const { min_x, min_y, max_x, max_y } = section.bbox;
      const w = max_x - min_x;
      const h = max_y - min_y;
      const nx = Math.max(1, Math.ceil(w / reachX - 1e-9));
      const ny = Math.max(1, Math.ceil(h / reachY - 1e-9));
      const wasSplit = nx > 1 || ny > 1;
      for (let gx = 0; gx < nx; gx++) {
        for (let gy = 0; gy < ny; gy++) {
          const cx0 = min_x + (w * gx) / nx;
          const cx1 = gx === nx - 1 ? max_x : min_x + (w * (gx + 1)) / nx;
          const cy0 = min_y + (h * gy) / ny;
          const cy1 = gy === ny - 1 ? max_y : min_y + (h * (gy + 1)) / ny;
          counter += 1;
          chunks.push({
            chunk_id: `frame_chunk_${String(counter).padStart(3, '0')}`,
            parent_section_id: section.section_id,
            bbox: { min_x: cx0, min_y: cy0, max_x: cx1, max_y: cy1 },
            points: [
              [cx0, cy0],
              [cx1, cy0],
              [cx1, cy1],
              [cx0, cy1],
            ],
            // Split chunks sit outside a single reach window → the robot must move
            // its base/axis to reach them.
            requires_axis_position: wasSplit,
          });
        }
      }
    }
    return chunks;
  };

  // Sanding tool radius (mm). One pass covers a 2·radius wide strip; a chunk wider
  // than that on its short axis needs multiple passes (a zigzag) to cover it.
  // Tool 4 on the frame: 50 mm offset (inset of the pass ends from the rail ends),
  // and a 75 mm single-pass width — a rectangle wider than 75 mm on its short side
  // gets multiple overlapping passes.
  const TOOL_FRAME_OFFSET_MM = 50;
  const FRAME_PASS_WIDTH_MM = 75;

  // One preview sanding path per reachable chunk. Passes run along the chunk's long
  // axis and step across its short axis. A chunk ≤ 75 mm on its short side gets one
  // centerline pass; wider than that gets overlapping zigzag passes whose step is the
  // 75 mm pass width reduced by the operator's FRAME overlap (denser with overlap).
  // Paths stay inside the chunk, so they never overlap or enter a pocket / 3D contour.
  const computeDxfFrameSectionPaths = (chunks: DxfFrameChunk[]) => {
    const offset = TOOL_FRAME_OFFSET_MM / dxfMmPerUnit;
    const halfPass = FRAME_PASS_WIDTH_MM / 2 / dxfMmPerUnit;
    const singlePassMax = FRAME_PASS_WIDTH_MM / dxfMmPerUnit;
    const frameOverlap = Math.max(0, Math.min(100, dxfFrameOverlap));
    const stepMm = Math.max(FRAME_PASS_WIDTH_MM - frameOverlap, 10);
    const step = stepMm / dxfMmPerUnit;

    const paths: {
      path_id: string;
      source_section_id: string;
      source_chunk_id: string;
      region_type: 'computed_frame';
      operation_type: 'frame_section_pass';
      points: number[][];
      direction: 'X' | 'Y';
      start_point: number[];
      end_point: number[];
    }[] = [];

    chunks.forEach((chunk, index) => {
      const { min_x, min_y, max_x, max_y } = chunk.bbox;
      const horizontal = max_x - min_x >= max_y - min_y;
      // Long-axis pass endpoints, inset by the 50 mm offset (clamped so they stay ordered).
      const loMin = horizontal ? min_x : min_y;
      const loMax = horizontal ? max_x : max_y;
      const longInset = Math.min(offset, (loMax - loMin) / 2);
      const pa0 = loMin + longInset;
      const pa1 = loMax - longInset;
      // Short axis (stepping direction).
      const shMin = horizontal ? min_y : min_x;
      const shMax = horizontal ? max_y : max_x;
      const shLen = shMax - shMin;
      const shortC = (shMin + shMax) / 2;

      const points: number[][] = [];
      if (shLen <= singlePassMax + 1e-9) {
        // ≤ 75 mm wide: one centerline pass.
        points.push(horizontal ? [pa0, shortC] : [shortC, pa0]);
        points.push(horizontal ? [pa1, shortC] : [shortC, pa1]);
      } else {
        // > 75 mm wide: overlapping passes. Pass centers span [shMin+halfPass,
        // shMax-halfPass] so the 75 mm strips tile the short side; ceil guarantees
        // full coverage and the frame overlap tightens the step further.
        const s0 = shMin + halfPass;
        const s1 = shMax - halfPass;
        const span = s1 - s0;
        const numSteps = Math.max(1, Math.ceil(span / step - 1e-9));
        const adjustedStep = span / numSteps;
        let toggle = 0;
        for (let k = 0; k <= numSteps; k++) {
          const s = s0 + k * adjustedStep;
          const row = horizontal
            ? [
                [pa0, s],
                [pa1, s],
              ]
            : [
                [s, pa0],
                [s, pa1],
              ];
          if (toggle) row.reverse();
          points.push(...row);
          toggle = 1 - toggle;
        }
      }

      paths.push({
        path_id: `frame_path_${String(index + 1).padStart(3, '0')}`,
        source_section_id: chunk.parent_section_id,
        source_chunk_id: chunk.chunk_id,
        region_type: 'computed_frame',
        operation_type: 'frame_section_pass',
        points,
        direction: horizontal ? 'X' : 'Y',
        start_point: points[0],
        end_point: points[points.length - 1],
      });
    });
    console.log('[Frame Section Paths] generated', {
      paths: paths.length,
      chunks: chunks.length,
      total_passes: paths.reduce((n, p) => n + Math.max(1, Math.floor(p.points.length / 2)), 0),
    });
    return paths;
  };

  const handleGenerateFramePreview = () => {
    if (!dxfJobId) return;

    // Scope the preview: if the operator has selected specific region(s) — surfaces
    // in the list or loops in the drawing — generate toolpaths ONLY for those, and
    // skip the auto-computed frame. With nothing selected, generate everything.
    const scopeIds = new Set<string>([...dxfSelectedSurfaceIds, ...dxfSelectedIds]);
    const scope = scopeIds.size > 0 ? scopeIds : null;

    const pocketTp = computeDxfPocketToolpaths(scope);
    const pocketZz = computeDxfPocketZigzag(scope);
    const contourTp = computeDxf3dContourToolpaths(scope);
    setDxfPocketToolpaths(pocketTp);
    setDxfPocketZigzag(pocketZz);
    setDxf3dContourToolpaths(contourTp);

    let frameZig: { start: number[]; end: number[]; id: string; tool: string; seq: number }[] = [];
    let frameChunks: DxfFrameChunk[] = [];
    let frameSectionPaths: ReturnType<typeof computeDxfFrameSectionPaths> = [];

    if (scope) {
      // The frame is a GLOBAL region (Outer − Pocket − 3D), not a per-region toolpath.
      // If the scope includes a frame / outer region, generate the full frame toolpath
      // so selecting it in the list shows the frame; otherwise the frame stays blank.
      const scopeHasFrame =
        scope.has(DXF_FRAME_SCOPE_ID) ||
        dxfManualSurfaces.some(
          (s) => scope.has(s.id) && (s.assigned_operation === 'frame_level' || s.assigned_operation === 'outer_boundary'),
        ) ||
        dxfLoops.some(
          (l) => scope.has(l.entity_id) && (dxfAssignments[l.entity_id] === 'frame' || dxfAssignments[l.entity_id] === 'outer'),
        );
      const hasPocket =
        dxfManualSurfaces.some((s) => s.assigned_operation === 'pocket_floor') ||
        dxfLoops.some((l) => dxfAssignments[l.entity_id] === 'pocket');

      if (scopeHasFrame) {
        frameZig = computeDxfFrameZigzag();
        setDxfFrameZigzag(frameZig);
        if (hasPocket) {
          const frameSections = computeDxfFrameSections();
          frameChunks = splitFrameSectionsByReach(frameSections);
          frameSectionPaths = computeDxfFrameSectionPaths(frameChunks);
          setDxfFrameSections(frameSections.map((section) => ({ ...section, covered: true })));
          setDxfFrameChunks(frameChunks);
          setDxfFrameSectionPaths(frameSectionPaths);
        } else {
          setDxfFrameSections([]);
          setDxfFrameChunks([]);
          setDxfFrameSectionPaths([]);
        }
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
      console.log('[DXF Toolpath] scoped preview generated', { regions: scopeIds.size, scopeHasFrame, segments: total });
    } else {
      frameZig = computeDxfFrameZigzag();
      setDxfFrameZigzag(frameZig);
      // Frame-section boxes/chunks are only meaningful when a pocket carves the frame
      // into separate rails. For a flat door (no pocket — frame level, or nothing but
      // frame) the whole-model zigzag covers everything, so skip the boxes entirely.
      const hasPocket =
        dxfManualSurfaces.some((s) => s.assigned_operation === 'pocket_floor') ||
        dxfLoops.some((l) => dxfAssignments[l.entity_id] === 'pocket');
      if (hasPocket) {
        const frameSections = computeDxfFrameSections();
        frameChunks = splitFrameSectionsByReach(frameSections);
        frameSectionPaths = computeDxfFrameSectionPaths(frameChunks);
        // Every section is fully covered once its chunks each get a path.
        setDxfFrameSections(frameSections.map((section) => ({ ...section, covered: true })));
        setDxfFrameChunks(frameChunks);
        setDxfFrameSectionPaths(frameSectionPaths);
      } else {
        setDxfFrameSections([]);
        setDxfFrameChunks([]);
        setDxfFrameSectionPaths([]);
      }
      setDxfSelectedToolpathId(null);
      setDxfFrameStatus('ready');
      setDxfFrameMessage('Toolpath preview generated (all regions).');
      console.log('[DXF Toolpath] preview generated (all regions)');
    }

    // Report the toolpath payload up so the config screen can gate approve / Start Task.
    // Segments are stitched back into ordered [x, y] point paths for MoveL execution:
    // consecutive segments chain (end of one = start of the next), so a path is the
    // start of its first segment followed by every segment's end.
    const segmentsToPaths = (
      segs: { start: number[]; end: number[]; id: string; tool: string; seq: number }[],
      operation: string,
      closed: boolean,
    ): DxfToolpathPath[] => {
      const groups = new Map<string, typeof segs>();
      for (const s of segs) {
        const key = s.id.replace(/_\d+$/, ''); // drop the trailing segment index
        if (!groups.has(key)) groups.set(key, []);
        groups.get(key)!.push(s);
      }
      const out: DxfToolpathPath[] = [];
      for (const [key, list] of groups) {
        list.sort((a, b) => a.seq - b.seq);
        const points = [list[0].start, ...list.map((s) => s.end)];
        out.push({ path_id: key, tool: list[0].tool, operation, closed, points });
      }
      return out;
    };

    const paths: DxfToolpathPath[] = [
      ...segmentsToPaths(pocketTp, 'Pocket contour (Tool 3)', true),
      ...segmentsToPaths(pocketZz, 'Pocket zigzag (Tool 4)', false),
      ...segmentsToPaths(contourTp, '3D contour ring', true),
      ...segmentsToPaths(frameZig, 'Frame zigzag (Tool 4)', false),
      ...frameSectionPaths.map((p) => ({
        path_id: p.path_id,
        tool: 'frame_section',
        operation: 'Frame section pass',
        closed: false,
        points: p.points,
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
  const dxfHasPocketOrContour =
    dxfManualSurfaces.some(
      (s) => s.assigned_operation === 'pocket_floor' || s.assigned_operation === 'surface_3d_area',
    ) ||
    dxfLoops.some(
      (l) => dxfAssignments[l.entity_id] === 'pocket' || dxfAssignments[l.entity_id] === 'surface3d',
    );

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

  // While an operation is locked in Lines mode, auto-confirm each detected surface
  // so the operator never has to re-click the button. Confirming clears the preview
  // and selection, which prevents any re-fire loop.
  React.useEffect(() => {
    if (dxfSelectionMode === 'line' && dxfLockedOperation && dxfDetectedSurface) {
      confirmDetectedSurface(dxfRegionToOperation(dxfLockedOperation));
    }
    // confirmDetectedSurface reads live state; re-running on detected-surface change
    // is enough and avoids adding an unstable function dep.
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [dxfDetectedSurface, dxfLockedOperation, dxfSelectionMode]);

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
    // When the frame is auto-computed (pockets / 3D present), add a synthetic row so
    // the operator can select and re-preview only the frame sections.
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
  ];

  // Operator-friendly, per-type numbered names ("Pocket 1", "3D Contour 2") instead
  // of raw DXF entity ids like "LWPOLYLINE-165".
  const dxfFriendlyBase = (label: string) => (label === 'Frame Level' ? 'Frame' : label);
  const dxfTypeCounters: Record<string, number> = {};
  const dxfAssignmentRows = dxfAssignmentRowsRaw.map((row) => {
    if (row.sourceType === 'computed_frame') return row;
    const base = dxfFriendlyBase(row.assignedLabel);
    dxfTypeCounters[base] = (dxfTypeCounters[base] || 0) + 1;
    return { ...row, displayId: `${base} ${dxfTypeCounters[base]}` };
  });

  // Stitch a region's toolpath segments back into ordered [x,y] polylines (matches
  // the MoveL payload order): a path = first segment's start + every segment's end.
  const dxfSegmentsToPolylines = (
    segs: { start: number[]; end: number[]; id: string; tool: string; seq: number }[],
  ) => {
    const groups = new Map<string, typeof segs>();
    for (const s of segs) {
      const key = s.id.replace(/_\d+$/, '');
      if (!groups.has(key)) groups.set(key, []);
      groups.get(key)!.push(s);
    }
    return [...groups.values()].map((list) => {
      list.sort((a, b) => a.seq - b.seq);
      return { points: [list[0].start, ...list.map((s) => s.end)] };
    });
  };

  // Region info for the inspector: corner-point shapes + ordered toolpath point lists.
  const computeDxfRegionInfo = (rowId: string, sourceType: string) => {
    const shapes: { label: string; points: number[][] }[] = [];
    const toolpaths: { label: string; points: number[][] }[] = [];
    const forRegion = (segs: { id: string }[]) => segs.filter((s) => s.id.startsWith(`${rowId}_`));

    if (sourceType === 'line_surface' || sourceType === 'closed_loop') {
      // The region's assigned operation decides which toolpaths belong to it.
      let assignedOperation: string | undefined;
      if (sourceType === 'line_surface') {
        const s = dxfManualSurfaces.find((x) => x.id === rowId);
        assignedOperation = s?.assigned_operation;
        if (s?.outer_points) shapes.push({ label: 'Outer corners', points: s.outer_points });
        if (s?.holes?.[0]) shapes.push({ label: 'Inner corners', points: s.holes[0] });
      } else {
        const l = dxfLoops.find((x) => x.entity_id === rowId);
        assignedOperation = dxfAssignments[rowId];
        if (l) shapes.push({ label: 'Corners', points: l.points });
      }
      dxfSegmentsToPolylines(forRegion(dxfPocketToolpaths) as never).forEach((p) =>
        toolpaths.push({ label: 'Pocket contour · Tool 3', points: p.points }),
      );
      dxfSegmentsToPolylines(forRegion(dxfPocketZigzag) as never).forEach((p) =>
        toolpaths.push({ label: 'Pocket zigzag · Tool 4', points: p.points }),
      );
      dxfSegmentsToPolylines(forRegion(dxf3dContourToolpaths) as never).forEach((p) =>
        toolpaths.push({ label: '3D contour ring', points: p.points }),
      );
      // Frame Level / Outer Boundary regions drive the global frame toolpath (it is
      // computed over the whole part, not per-region), so surface it on this region.
      const isFrameRegion =
        assignedOperation === 'frame_level' ||
        assignedOperation === 'outer_boundary' ||
        assignedOperation === 'frame' ||
        assignedOperation === 'outer';
      if (isFrameRegion) {
        dxfFrameSectionPaths.forEach((p, i) =>
          toolpaths.push({ label: `Frame section pass ${i + 1}`, points: p.points }),
        );
        dxfSegmentsToPolylines(dxfFrameZigzag).forEach((p) =>
          toolpaths.push({ label: 'Frame zigzag · Tool 4', points: p.points }),
        );
      }
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
      dxfFrameChunks.forEach((c, i) => shapes.push({ label: `Section ${i + 1} corners`, points: c.points }));
      dxfFrameSectionPaths.forEach((p, i) => toolpaths.push({ label: `Frame section pass ${i + 1}`, points: p.points }));
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
                          const isFrameRow = row.sourceType === 'computed_frame';
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
                      highlightId={dxfHoveredRowId}
                      assignments={dxfAssignments}
                      onToggleSelect={handleToggleDxfLoop}
                      selectionMode={dxfSelectionMode}
                      selectedLineIds={dxfSelectedLineIds}
                      onToggleLine={handleToggleDxfLine}
                      surfacePreview={dxfDetectedSurface ? { outer: dxfDetectedSurface.outer, holes: dxfDetectedSurface.holes } : null}
                      manualSurfaces={dxfManualSurfaces}
                      framePolygons={dxfFramePolygons}
                      frameRectangles={dxfFrameRectangles}
                      frameToolpaths={dxfFrameToolpaths}
                      pocketToolpaths={dxfPocketToolpaths}
                      pocketZigzag={dxfPocketZigzag}
                      contourToolpaths={dxf3dContourToolpaths}
                      frameZigzag={dxfFrameZigzag}
                      frameSections={dxfFrameSections}
                      frameChunks={dxfFrameChunks}
                      frameSectionPaths={dxfFrameSectionPaths}
                      showFrame={dxfShowFrame}
                      showToolpaths={dxfShowToolpaths}
                      selectedToolpathId={dxfSelectedToolpathId}
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
                      return (
                        <div
                          style={{
                            position: 'absolute',
                            top: '24px',
                            right: '24px',
                            width: '340px',
                            maxHeight: 'calc(100% - 48px)',
                            overflowY: 'auto',
                            background: '#ffffff',
                            border: '1px solid #cbd5e1',
                            borderRadius: '12px',
                            boxShadow: '0 10px 30px rgba(15,23,42,0.18)',
                            padding: '14px',
                            fontSize: '11px',
                            zIndex: 20,
                          }}
                        >
                          <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: '10px' }}>
                            <strong style={{ fontSize: '13px' }}>{row.displayId}</strong>
                            <button
                              type="button"
                              onClick={() => setDxfInfoRowId(null)}
                              style={{ ...dxfPlainButtonStyle, padding: '2px 8px', fontSize: '11px' }}
                            >
                              Close
                            </button>
                          </div>
                          <div style={{ fontWeight: 800, color: '#334155', marginBottom: '4px' }}>Corner points (mm)</div>
                          {info.shapes.length === 0 ? (
                            <div style={{ color: '#94a3b8', marginBottom: '8px' }}>No corner data.</div>
                          ) : (
                            info.shapes.map((s, i) => (
                              <div key={`shape-${i}`} style={{ marginBottom: '8px' }}>
                                <div style={{ color: '#64748b', marginBottom: '2px' }}>
                                  {s.label} · {s.points.length} pts
                                </div>
                                <div style={{ fontFamily: 'monospace', color: '#0f172a', wordBreak: 'break-word' }}>{fmt(s.points)}</div>
                              </div>
                            ))
                          )}
                          <div style={{ fontWeight: 800, color: '#334155', margin: '10px 0 4px' }}>Toolpath points (in order, mm)</div>
                          {info.toolpaths.length === 0 ? (
                            <div style={{ color: '#94a3b8' }}>No toolpath generated yet — run Preview Toolpath.</div>
                          ) : (
                            info.toolpaths.map((t, i) => (
                              <div key={`tp-${i}`} style={{ marginBottom: '8px' }}>
                                <div style={{ color: '#64748b', marginBottom: '2px' }}>
                                  {t.label} · {t.points.length} pts
                                </div>
                                <div style={{ fontFamily: 'monospace', color: '#0f172a', wordBreak: 'break-word' }}>{fmt(t.points)}</div>
                              </div>
                            ))
                          )}
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



















