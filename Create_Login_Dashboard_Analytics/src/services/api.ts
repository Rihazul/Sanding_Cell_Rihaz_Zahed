// API service for robot control dashboard

// DEV: pointed at the local backend for testing on this machine.
// ⚠️ REVERT TO 'http://192.168.0.230:5100' (robot PC) before deploying to the robot.
export const API_BASE_URL = 'http://192.168.0.230:5100';

// Generic API call function
async function apiCall(endpoint: string, method: 'GET' | 'POST', payload?: any) {
  try {
    const options: RequestInit = {
      method,
      headers: {
        'Content-Type': 'application/json',
      },
      cache: method === 'GET' ? 'no-store' : undefined,
    };

    if (method === 'POST' && payload) {
      options.body = JSON.stringify(payload);
    }

    const response = await fetch(`${API_BASE_URL}${endpoint}`, options);

    if (!response.ok) {
      let details = '';
      try {
        const data = await response.json();
        details = data?.message || data?.error || JSON.stringify(data);
      } catch {
        try {
          details = await response.text();
        } catch {
          details = '';
        }
      }
      const suffix = details ? ` - ${details}` : '';
      throw new Error(`API call failed: ${response.status} ${response.statusText}${suffix}`);
    }

    return await response.json();
  } catch (error) {
    console.error(`Error calling ${endpoint}:`, error);
    throw error;
  }
}

// Trigger robot sanding process
export async function triggerRobotProcess(message: string) {
  return apiCall('/trigger', 'POST', { message });
}

// Get saved modal/config data (used by backend to combine settings)
export async function getModalData() {
  return apiCall('/get_modal_data', 'GET');
}

// Save modal data for Table A
export async function saveModalData(tableAData: {
  frameSandCount: number;
  pocketZigSandCnt: number;
  robotSpeed: number;
  scanSpeed: number;
  model: string;
}) {
  return apiCall('/save_modal_data', 'POST', {
    tableA: {
      UI: tableAData
    }
  });
}

// Door configuration type for Table A
export interface DoorConfig {
  doorNumber: number;
  model: string;
  rows: {
    label: string;
    selection: string;
    force: number;
    cycle: number;
    // Pocket ZigZag specific options
    verticalSpiral?: boolean;
    horizontalSpiral?: boolean;
    edgeCoverage?: boolean;
    // Derived helpers
    orientation?: 'vertical' | 'horizontal' | 'both';
    edge?: boolean;
  }[];
}

export type SpiralSettingsPayload = {
  enabled?: boolean;
  speedPercent: number;
  radiusMm: number;
  linearSpeedMmS: number;
};

// Start Table A process with door configurations
export async function startTableAProcess(data: {
  doorConfigs: DoorConfig[];
  robotSpeed: string;
  sandingSpeed: string;
  inverseOverlapping: number;
  spiralSettings?: SpiralSettingsPayload;
  tableAFrameSize?: { x: number | null; y: number | null };
}) {
  const inferredModel =
    data.doorConfigs.find(d => d.model && d.model !== '')?.model || '';

  const rowKeyMap: Record<string, string> = {
    'Frame': 'frame',
    'Pocket ZigZag': 'pocketzigzag',
    'Pocket Edge': 'pocketedge',
    'Pocket Square': 'pocketsquare',
    '3D': '3D',
    'Edge Inside': 'edgeInside',
    'Edge Outside': 'edgeOutside',
    'Side': 'side',
  };

  const getDoorsForRow = (label: string) =>
    data.doorConfigs
      .filter(door => {
        const row = door.rows.find(r => r.label === label);
        return row && (row.force > 0 || row.cycle > 0);
      })
      .map(door => door.doorNumber);

  const getRowValues = (label: string) => {
    for (const door of data.doorConfigs) {
      const row = door.rows.find(r => r.label === label && (r.force > 0 || r.cycle > 0));
      if (row) {
        return { cycle: row.cycle || 0, force: row.force || 0 };
      }
    }
    for (const door of data.doorConfigs) {
      const row = door.rows.find(r => r.label === label);
      if (row) {
        return { cycle: row.cycle || 0, force: row.force || 0 };
      }
    }
    return { cycle: 0, force: 0 };
  };

  const buildRowPayload = (label: string) => {
    const base = getRowValues(label);
    return { ...base, doors: getDoorsForRow(label) };
  };

  const getPocketZigZagMeta = () => {
    const derive = (row?: DoorConfig['rows'][number]) => {
      const verticalSpiral = !!row?.verticalSpiral;
      const horizontalSpiral = !!row?.horizontalSpiral;
      const orientation: 'vertical' | 'horizontal' | 'both' =
        verticalSpiral && horizontalSpiral
          ? 'both'
          : horizontalSpiral
            ? 'horizontal'
            : 'vertical';
      const edge = false;
      return { orientation, edge };
    };

    for (const door of data.doorConfigs) {
      const row = door.rows.find(r => r.label === 'Pocket ZigZag' && (r.force > 0 || r.cycle > 0));
      if (row) return derive(row);
    }
    for (const door of data.doorConfigs) {
      const row = door.rows.find(r => r.label === 'Pocket ZigZag');
      if (row) return derive(row);
    }
    return { orientation: 'vertical' as const, edge: false };
  };

  const buildDoorTasks = (door: DoorConfig) => {
    const tasks: Record<string, any> = {};
    for (const row of door.rows) {
      const key = rowKeyMap[row.label];
      if (!key) continue;
      const base = { cycle: row.cycle || 0, force: row.force || 0 };
      if (row.label === 'Pocket ZigZag') {
        const verticalSpiral = !!row.verticalSpiral;
        const horizontalSpiral = !!row.horizontalSpiral;
        const orientation: 'vertical' | 'horizontal' | 'both' =
          verticalSpiral && horizontalSpiral
            ? 'both'
            : horizontalSpiral
              ? 'horizontal'
              : 'vertical';
        const edgeCoverage = false;
        tasks[key] = {
          ...base,
          verticalSpiral,
          horizontalSpiral,
          edgeCoverage,
          orientation,
          edge: edgeCoverage,
        };
      } else {
        tasks[key] = base;
      }
    }
    return tasks;
  };

  const doorsPayload = data.doorConfigs.map(door => ({
    doorNumber: door.doorNumber,
    model: door.model,
    tasks: buildDoorTasks(door),
  }));

  // Build the payload matching the Flask backend format
  const payload = {
    TableA: {
      // Keep legacy compatibility: backend expects TableA.model
      model: inferredModel,
      frame: buildRowPayload('Frame'),
      pocketzigzag: { ...buildRowPayload('Pocket ZigZag'), ...getPocketZigZagMeta() },
      pocketedge: buildRowPayload('Pocket Edge'),
      pocketsquare: buildRowPayload('Pocket Square'),
      '3D': buildRowPayload('3D'),
      edgeInside: buildRowPayload('Edge Inside'),
      edgeOutside: buildRowPayload('Edge Outside'),
      side: buildRowPayload('Side'),
      // New per-door format (supports unique settings per door)
      doors: doorsPayload,
      tableAFrameSize: data.tableAFrameSize,
    },
    robotSpeed: data.robotSpeed,
    sandingSpeed: data.sandingSpeed,
    inverseOverlapping: data.inverseOverlapping,
    spiralSettings: data.spiralSettings,
  };

  return apiCall('/start_TableA_process', 'POST', payload);
}

// Start Table A process (legacy format - single model)
export async function startTableAProcessLegacy(data: {
  model: string;
  frame: { cycle: number; force: number; doors?: number[] };
  pocketzigzag: {
    cycle: number;
    force: number;
    doors?: number[];
    Operation?: boolean;
    edgeCoverage?: boolean;
  };
  pocketsquare?: { cycle: number; force: number; doors?: number[] };
  '3D': { cycle: number; force: number; doors?: number[] };
  edgeInside?: { cycle: number; force: number; doors?: number[] };
  edgeOutside: { cycle: number; force: number; doors?: number[] };
  side: { cycle: number; force: number; doors?: number[] };
  robotSpeed?: string;
  sandingSpeed?: string;
  inverseOverlapping?: number;
  spiralSettings?: SpiralSettingsPayload;
}) {
  const payload = {
    TableA: {
      model: data.model,
      frame: data.frame,
      pocketzigzag: data.pocketzigzag,
      pocketsquare: data.pocketsquare || { cycle: 0, force: 0 },
      '3D': data['3D'],
      edgeInside: data.edgeInside || { cycle: 0, force: 0 },
      edgeOutside: data.edgeOutside,
      side: data.side
    },
    robotSpeed: data.robotSpeed || '1.00',
    sandingSpeed: data.sandingSpeed || '0.50',
    inverseOverlapping: data.inverseOverlapping || 50,
    spiralSettings: data.spiralSettings,
  };

  return apiCall('/start_TableA_process', 'POST', payload);
}

// Start Table B process
export async function startTableBProcess(data: {
  // Table B runs the operator-approved 2D DXF toolpath: the backend loads
  // approved_toolpath.json for this job. The legacy `model` preset is gone.
  job_id: string;
  frame: { cycle: number; force: number };
  pocketzigzag: {
    cycle: number;
    force: number;
    verticalSpiral?: boolean;
    horizontalSpiral?: boolean;
    edgeCoverage?: boolean;
  };
  // Pocket Edge drives Tool 3; it is configured separately from Pocket ZigZag (Tool 4).
  pocketedge?: { cycle: number; force: number };
  pocketsquare?: { cycle: number; force: number };
  '3D': { cycle: number; force: number };
  edgeInside?: { cycle: number; force: number };
  edgeOutside: { cycle: number; force: number };
  side: { cycle: number; force: number };
  robotSpeed: string;
  sandingSpeed: string;
  inverseOverlapping: number;
  spiralSettings?: SpiralSettingsPayload;
}) {
  return apiCall('/start_TableB_process', 'POST', {
    TableB: {
      // Identifies the approved DXF toolpath the backend loads from disk. This is
      // what defines the run now — the legacy `model` preset is gone.
      job_id: data.job_id,
      frame: data.frame,
      pocketzigzag: data.pocketzigzag,
      pocketedge: data.pocketedge || { cycle: 0, force: 0 },
      pocketsquare: data.pocketsquare || { cycle: 0, force: 0 },
      '3D': data['3D'],
      edgeInside: data.edgeInside || { cycle: 0, force: 0 },
      edgeOutside: data.edgeOutside,
      side: data.side
    },
    robotSpeed: data.robotSpeed,
    sandingSpeed: data.sandingSpeed,
    inverseOverlapping: data.inverseOverlapping,
    spiralSettings: data.spiralSettings,
  });
}

// Upload 3D file for Table B
export async function upload3DFile(file: File) {
  const formData = new FormData();
  formData.append('file', file);

  try {
    const response = await fetch(`${API_BASE_URL}/upload_3d_file`, {
      method: 'POST',
      body: formData
    });

    if (!response.ok) {
      throw new Error(`Upload failed: ${response.statusText}`);
    }

    return await response.json();
  } catch (error) {
    console.error('Error uploading 3D file:', error);
    throw error;
  }
}

// Tool toggle operations
export async function toolToggle(toolNumber: 1 | 2 | 3 | 4, action: 'pick' | 'keep') {
  const endpoint = toolNumber === 1 ? '/tool_toggle1' :
    toolNumber === 2 ? '/tool_toggle2' :
      '/tool_toggle';
  return apiCall(endpoint, 'POST', { toolNumber, action });
}

// Action operations
export async function performAction(
  action: 'stopperUp' | 'stopperDown' | 'stopperUpB' | 'stopperDownB' | 'homing' | 'enable' | 'disable' | 'scan' | 'stop' | 'toolLift' | 'toolDrop' | 'laserOn' | 'laserOff',
  extraPayload?: Record<string, unknown>
) {
  return apiCall('/action', 'POST', { action, ...(extraPayload || {}) });
}

// Toggle state for Table open/close
export async function toggleTableState(
  tableId: 'tableAOpenClose' | 'tableBOpenClose',
  desiredState?: 'Open' | 'Close'
) {
  const desired = desiredState ? `?desired=${desiredState}` : '';
  return apiCall(`/toggle_state/${tableId}${desired}`, 'GET');
}

// Get current state for Table open/close
export async function getTableState(tableId: 'tableAOpenClose' | 'tableBOpenClose') {
  return apiCall(`/table_state/${tableId}`, 'GET');
}

// Get current stopper state
export async function getStopperState(stopperId: 'A' | 'B') {
  return apiCall(`/stopper_state/${stopperId}`, 'GET');
}

// Get robot status flags
export async function getRobotStatus() {
  return apiCall('/robot_status', 'GET');
}

// Get process status (used for homing completion)
export async function getProcessStatus() {
  return apiCall('/process_status', 'GET');
}

export async function getHomingStatus() {
  return apiCall('/homing_status', 'GET');
}

export async function getScanStatus() {
  return apiCall(`/scan_status?ts=${Date.now()}`, 'GET');
}

// Tool attachment status checks (returns shouldBlink boolean)
export async function checkToolStatus(toolNumber: 1 | 2 | 3 | 4) {
  const endpoint =
    toolNumber === 1 ? '/check_tool1_status' :
      toolNumber === 2 ? '/check_tool2_status' :
        toolNumber === 3 ? '/check_tool3_status' :
          '/check_tool4_status';
  return apiCall(endpoint, 'GET');
}

export interface HistoricalLogEntry {
  id: number;
  timestamp: string;
  message: string;
  type: 'info' | 'success' | 'warning' | 'error';
}

export interface HistoricalLogDay {
  date: string;
  displayDate: string;
  entries: HistoricalLogEntry[];
}

export async function getLogsHistory(days = 14, perFileLines = 2000, includeAll = false): Promise<{
  logs: HistoricalLogDay[];
  source?: string;
  message?: string;
}> {
  const query = `?days=${encodeURIComponent(days)}&per_file_lines=${encodeURIComponent(perFileLines)}&all=${includeAll ? 'true' : 'false'}`;
  return apiCall(`/logs/history${query}`, 'GET');
}

// The legacy STEP/3D "CAD Assisted" backend (table_b_3d) has been removed — the 2D
// DXF Assisted flow fully replaces it. These wrappers are kept only so any leftover
// STEP UI still compiles; they make no backend call and return a disabled response.
const TABLE_B_3D_DISABLED = 'The STEP/3D CAD flow has been removed. Use the 2D DXF Assisted mode instead.';

export interface TableB3DUploadResponse {
  success: boolean;
  job_id?: string;
  source_file?: string;
  status?: 'uploaded' | string;
  error?: string;
}

export async function uploadTableB3DStepFile(_file: File): Promise<TableB3DUploadResponse> {
  return { success: false, status: 'disabled', error: TABLE_B_3D_DISABLED };
}
// --- Table B DXF (2D CAD Assisted) ---
export interface TableBDxfUploadResponse {
  success: boolean;
  job_id?: string;
  status?: 'uploaded' | string;
  error?: string;
}

export async function uploadTableBDxfFile(file: File): Promise<TableBDxfUploadResponse> {
  const formData = new FormData();
  formData.append('file', file);

  try {
    const response = await fetch(`${API_BASE_URL}/api/table-b-dxf/upload`, {
      method: 'POST',
      body: formData,
    });
    const data = await response.json().catch(() => ({}));

    if (!response.ok) {
      throw new Error(data?.error || data?.message || `DXF upload failed: ${response.statusText}`);
    }

    return data;
  } catch (error) {
    console.error('Error uploading Table B DXF file:', error);
    throw error;
  }
}

export interface TableBDxfLoop {
  entity_id: string;
  loop_id?: string;
  type: string;
  layer: string;
  points: number[][];
  closed: boolean;
  bbox: { min_x: number; min_y: number; max_x: number; max_y: number } | null;
  area: number;
  width?: number;
  height?: number;
  aspect_ratio?: number;
}

export interface TableBDxfOpenPath {
  entity_id: string;
  type: string; // "line_entity"
  dxf_type?: string;
  layer: string;
  points: number[][];
  bbox?: { min_x: number; min_y: number; max_x: number; max_y: number } | null;
  length?: number;
  closed?: boolean;
}

export interface TableBDxfParsedResponse {
  success: boolean;
  job_id?: string;
  status?: string;
  loops?: TableBDxfLoop[];
  open_paths?: TableBDxfOpenPath[];
  summary?: {
    total_entities_scanned: number;
    closed_loops_found: number;
    narrow_loops_found?: number;
    open_paths_found?: number;
    loops_filtered_out?: number;
    open_entities_ignored: number;
  };
  message?: string;
}

export async function getTableBDxfParsedLoops(jobId: string): Promise<TableBDxfParsedResponse> {
  try {
    const response = await fetch(`${API_BASE_URL}/api/table-b-dxf/parsed/${encodeURIComponent(jobId)}`, {
      method: 'GET',
    });
    const data = await response.json().catch(() => ({}));

    if (!response.ok) {
      throw new Error(data?.message || data?.error || `DXF parse failed: ${response.statusText}`);
    }

    return data;
  } catch (error) {
    console.error('Error parsing Table B DXF file:', error);
    throw error;
  }
}

export interface TableBDxfFramePolygon {
  exterior: number[][];
  holes: number[][][];
}

export interface TableBDxfFrameResponse {
  success: boolean;
  job_id?: string;
  status?: string;
  frame?: { type: string; polygons: TableBDxfFramePolygon[] };
  summary?: { outer_area: number; pocket_count: number; surface_3d_count: number; frame_area: number };
  message?: string;
}

// The frame is now computed in the frontend (Outer − Pocket − 3D Contour); the
// backend compute-frame / generate-frame-toolpath routes were removed. The frame
// data types below are kept only for the inert frame-overlay state.

export interface TableBDxfFrameRectangle {
  rect_id: string;
  min_x: number;
  min_y: number;
  max_x: number;
  max_y: number;
  width: number;
  height: number;
}

export interface TableBDxfFrameToolpath {
  rect_id: string;
  orientation: 'horizontal' | 'vertical' | string;
  start: number[];
  end: number[];
  length: number;
  tool_diameter_mm: number;
}

export interface TableBDxfFrameWarning {
  rect_id: string;
  type: string;
  message: string;
}

export interface TableBDxfFrameToolpathResponse {
  success: boolean;
  job_id?: string;
  status?: string;
  tool_diameter_mm?: number;
  end_margin_mm?: number;
  frame_rectangles?: TableBDxfFrameRectangle[];
  frame_toolpaths?: TableBDxfFrameToolpath[];
  warnings?: TableBDxfFrameWarning[];
  message?: string;
}

export interface TableBDxfLinesClosedResponse {
  success: boolean;
  job_id?: string;
  closed?: boolean;
  points?: number[][];
  area?: number;
  bbox?: { min_x: number; min_y: number; max_x: number; max_y: number };
  message?: string;
  open_endpoints?: number[][];
  line_count?: number;
  status?: string;
}

export async function checkTableBDxfLinesClosed(
  jobId: string,
  selectedEntityIds: string[],
): Promise<TableBDxfLinesClosedResponse> {
  try {
    const response = await fetch(`${API_BASE_URL}/api/table-b-dxf/check-lines-closed/${encodeURIComponent(jobId)}`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ selected_entity_ids: selectedEntityIds }),
    });
    const data = await response.json().catch(() => ({}));

    if (!response.ok) {
      throw new Error(data?.message || data?.error || `Line-closure check failed: ${response.statusText}`);
    }

    return data;
  } catch (error) {
    console.error('Error checking Table B DXF line closure:', error);
    throw error;
  }
}

export interface TableBDxfDetectedLoop {
  loop_id: string;
  points: number[][];
  holes?: number[][][];
  area: number;
  net_area?: number;
  bbox: { min_x: number; min_y: number; max_x: number; max_y: number };
  source_entity_ids: string[];
}

export interface TableBDxfDetectLoopsResponse {
  success: boolean;
  job_id?: string;
  loops?: TableBDxfDetectedLoop[];
  selected_count?: number;
  candidate_count?: number;
  message?: string;
  status?: string;
}

// --- Frame area (outer door − pockets − 3D contours) ---
export interface TableBDxfFrameRing {
  exterior: number[][];
  holes: number[][][];
  area: number;
}
export interface TableBDxfFrameAreaResponse {
  success: boolean;
  ok?: boolean;
  rings?: TableBDxfFrameRing[];
  outer_area?: number;
  frame_area?: number;
  obstacle_count?: number;
  reason?: string;
  message?: string;
}

export interface TableBDxfFrameToolpathsBackendResponse extends TableBDxfFrameAreaResponse {
  sections?: any[];
  chunks?: any[];
  toolpaths?: any[];
}

export async function computeTableBDxfFrameToolpaths(
  jobId: string,
  outlinePolygon: number[][] | null,
  pocketPolygons: number[][][],
  surface3dPolygons: number[][][],
  options?: {
    passWidthMm?: number;
    offsetMm?: number;
    overlapMm?: number;
    reachXMm?: number;
    reachYMm?: number;
  },
): Promise<TableBDxfFrameToolpathsBackendResponse> {
  try {
    const response = await fetch(`${API_BASE_URL}/api/table-b-dxf/frame-toolpaths/${encodeURIComponent(jobId)}`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({
        outline_polygon: outlinePolygon,
        pocket_polygons: pocketPolygons,
        surface3d_polygons: surface3dPolygons,
        pass_width_mm: options?.passWidthMm ?? 75,
        offset_mm: options?.offsetMm ?? 50,
        overlap_mm: options?.overlapMm ?? 0,
        reach_x_mm: options?.reachXMm ?? 515,
        reach_y_mm: options?.reachYMm ?? 750,
      }),
    });
    const data = await response.json().catch(() => ({}));
    if (!response.ok) {
      throw new Error(data?.message || data?.error || `Frame toolpath failed: ${response.statusText}`);
    }
    return data;
  } catch (error) {
    console.error('Error computing Table B DXF frame toolpaths:', error);
    throw error;
  }
}

export interface TableBDxfFrameZigzagResponse {
  success: boolean;
  ok?: boolean;
  rings?: { exterior: number[][]; holes: number[][][]; area: number }[];
  toolpaths?: { path_id: string; points: number[][]; tool: string; operation_type: string; direction: string }[];
  frame_area?: number;
  pass_count?: number;
  reason?: string;
  message?: string;
}

// Curve-aware zigzag FILL of the whole frame surface (Frame Level on the whole door).
export async function computeTableBDxfFrameZigzag(
  jobId: string,
  outlinePolygon: number[][] | null,
  pocketPolygons: number[][][],
  surface3dPolygons: number[][][],
  options?: { passWidthMm?: number; overlapMm?: number },
): Promise<TableBDxfFrameZigzagResponse> {
  try {
    const response = await fetch(`${API_BASE_URL}/api/table-b-dxf/frame-zigzag/${encodeURIComponent(jobId)}`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({
        outline_polygon: outlinePolygon,
        pocket_polygons: pocketPolygons,
        surface3d_polygons: surface3dPolygons,
        pass_width_mm: options?.passWidthMm ?? 75,
        overlap_mm: options?.overlapMm ?? 0,
      }),
    });
    const data = await response.json().catch(() => ({}));
    if (!response.ok) {
      throw new Error(data?.message || data?.error || `Frame zigzag failed: ${response.statusText}`);
    }
    return data;
  } catch (error) {
    console.error('Error computing Table B DXF frame zigzag:', error);
    throw error;
  }
}

export async function detectTableBDxfLoops(
  jobId: string,
  selectedEntityIds: string[],
): Promise<TableBDxfDetectLoopsResponse> {
  try {
    const response = await fetch(`${API_BASE_URL}/api/table-b-dxf/detect-loops/${encodeURIComponent(jobId)}`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ selected_entity_ids: selectedEntityIds }),
    });
    const data = await response.json().catch(() => ({}));

    if (!response.ok) {
      throw new Error(data?.message || data?.error || `Loop detection failed: ${response.statusText}`);
    }

    return data;
  } catch (error) {
    console.error('Error detecting Table B DXF loops:', error);
    throw error;
  }
}

export interface TableBDxfSaveToolpathResponse {
  success: boolean;
  job_id?: string;
  saved_path?: string;
  message?: string;
  error?: string;
}

// Persist the approved DXF toolpath + region corner points as a JSON file on the
// backend job. Re-approving overwrites the same file so the operation always runs
// the operator's latest choice.
export async function saveTableBDxfApprovedToolpath(
  jobId: string,
  payload: unknown,
): Promise<TableBDxfSaveToolpathResponse> {
  try {
    const response = await fetch(`${API_BASE_URL}/api/table-b-dxf/approved-toolpath/${encodeURIComponent(jobId)}`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(payload),
    });
    const data = await response.json().catch(() => ({}));
    if (!response.ok) {
      throw new Error(data?.message || data?.error || `Saving approved toolpath failed: ${response.statusText}`);
    }
    return data;
  } catch (error) {
    console.error('Error saving approved Table B DXF toolpath:', error);
    throw error;
  }
}

export interface TableB3DConversionResponse {
  success: boolean;
  job_id?: string;
  status?: 'conversion_not_implemented' | 'converted' | string;
  message?: string;
  glb_url?: string;
  metadata_url?: string;
}

export async function convertTableB3DModel(_jobId: string): Promise<TableB3DConversionResponse> {
  return { success: false, status: 'disabled', message: TABLE_B_3D_DISABLED };
}
export interface TableB3DFaceMetadataFace {
  face_id: string;
  normal: number[];
  center: number[];
  area: number;
  z_level: number;
  type_hint: 'frame_top' | 'pocket_bottom' | 'pocket_bevel_3d' | 'vertical_edge' | 'unknown' | string;
}

export interface TableB3DFaceMetadata {
  job_id: string;
  source_file?: string;
  created_at?: string;
  status?: string;
  extractor?: string;
  todo?: string;
  message?: string;
  faces: TableB3DFaceMetadataFace[];
}

export interface TableB3DFaceMetadataResponse {
  success: boolean;
  job_id?: string;
  status?: 'loaded' | string;
  message?: string;
  metadata?: TableB3DFaceMetadata;
}

export async function loadTableB3DFaceMetadata(_jobId: string): Promise<TableB3DFaceMetadataResponse> {
  return { success: false, status: 'disabled', message: TABLE_B_3D_DISABLED };
}
export interface TableB3DMappingResponse {
  success: boolean;
  job_id?: string;
  status?: 'saved' | 'loaded' | string;
  message?: string;
  mapping?: {
    job_id: string;
    source_file?: string;
    created_at?: string;
    regions?: any[];
  };
}

export async function saveTableB3DMapping(_jobId: string, _regions: any[]): Promise<TableB3DMappingResponse> {
  return { success: false, status: 'disabled', message: TABLE_B_3D_DISABLED };
}

export async function loadTableB3DMapping(_jobId: string): Promise<TableB3DMappingResponse> {
  return { success: false, status: 'disabled', message: TABLE_B_3D_DISABLED };
}
export interface TableB3DToolpathSegment {
  type: 'sanding' | 'retract' | string;
  points: number[][];
}

export interface TableB3DReachReport {
  total_segments: number;
  reachable_segments: number;
  unreachable_segments: number;
  group_count: number;
  retract_count: number;
}

export interface TableB3DReachGroup {
  group_id: string;
  path_index: number;
  region_id: string;
  region_type: string;
  tool?: number;
  force?: number;
  cycle?: number;
  segments: TableB3DToolpathSegment[];
}

export interface TableB3DGeneratedPath {
  region_id: string;
  region_type: 'pocket_bottom' | 'frame_top' | string;
  tool: number;
  force: number;
  cycle: number;
  segments: TableB3DToolpathSegment[];
}

export interface TableB3DGeneratedToolpaths {
  job_id: string;
  paths: TableB3DGeneratedPath[];
  reach_report?: TableB3DReachReport;
  reach_groups?: TableB3DReachGroup[];
  unreachable_segments?: any[];
  workspace_limits?: Record<string, number>;
}

export interface TableB3DGenerateToolpathResponse {
  success: boolean;
  job_id?: string;
  status?: 'generated' | string;
  message?: string;
  toolpaths?: TableB3DGeneratedToolpaths;
}

export async function generateTableB3DToolpath(_jobId: string): Promise<TableB3DGenerateToolpathResponse> {
  return { success: false, status: 'disabled', message: TABLE_B_3D_DISABLED };
}

export interface TableB3DConfirmToolpathResponse {
  success: boolean;
  job_id?: string;
  status?: 'confirmed' | string;
  confirmed?: boolean;
  confirmed_at?: string;
  message?: string;
  reach_report?: TableB3DReachReport;
  metadata?: Record<string, any>;
}

export async function confirmTableB3DToolpath(_jobId: string): Promise<TableB3DConfirmToolpathResponse> {
  return { success: false, status: 'disabled', message: TABLE_B_3D_DISABLED };
}

export interface TableB3DExecutionPreviewSummary {
  group_count: number;
  sanding_moves: number;
  retract_moves: number;
  tools_required: Array<number | string>;
}

export interface TableB3DExecutionPreview {
  job_id: string;
  status: 'execution_preview_ready' | string;
  preview_only: boolean;
  generated_at?: string;
  summary: TableB3DExecutionPreviewSummary;
  groups: any[];
}

export interface TableB3DExecutionPreviewResponse {
  success: boolean;
  job_id?: string;
  status?: 'execution_preview_ready' | string;
  message?: string;
  execution_preview?: TableB3DExecutionPreview;
}

export async function getTableB3DExecutionPreview(_jobId: string): Promise<TableB3DExecutionPreviewResponse> {
  return { success: false, status: 'disabled', message: TABLE_B_3D_DISABLED };
}

// Export all API functions
export const api = {
  triggerRobotProcess,
  saveModalData,
  getModalData,
  startTableAProcess,
  startTableAProcessLegacy,
  startTableBProcess,
  upload3DFile,
  uploadTableB3DStepFile,
  convertTableB3DModel,
  loadTableB3DFaceMetadata,
  saveTableB3DMapping,
  loadTableB3DMapping,
  generateTableB3DToolpath,
  confirmTableB3DToolpath,
  getTableB3DExecutionPreview,
  toolToggle,
  performAction,
  toggleTableState,
  getTableState,
  getStopperState,
  getRobotStatus,
  getProcessStatus,
  getHomingStatus,
  getScanStatus,
  checkToolStatus,
  getLogsHistory,
};

