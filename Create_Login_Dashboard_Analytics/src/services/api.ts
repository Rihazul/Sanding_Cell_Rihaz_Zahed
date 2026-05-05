// API service for robot control dashboard

const API_BASE_URL = 'http://192.168.0.230:5100';

// Generic API call function
async function apiCall(endpoint: string, method: 'GET' | 'POST', payload?: any) {
  try {
    const options: RequestInit = {
      method,
      headers: {
        'Content-Type': 'application/json',
      },
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
    orientation?: 'vertical' | 'horizontal';
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
}) {
  const inferredModel =
    data.doorConfigs.find(d => d.model && d.model !== '')?.model || '';

  const rowKeyMap: Record<string, string> = {
    'Frame': 'frame',
    'Pocket ZigZag': 'pocketzigzag',
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
      const orientation: 'vertical' | 'horizontal' =
        horizontalSpiral && !verticalSpiral ? 'horizontal' : 'vertical';
      const edge = !!row?.edgeCoverage;
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
        const orientation: 'vertical' | 'horizontal' =
          horizontalSpiral && !verticalSpiral ? 'horizontal' : 'vertical';
        const edgeCoverage = !!row.edgeCoverage;
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
      pocketsquare: buildRowPayload('Pocket Square'),
      '3D': buildRowPayload('3D'),
      edgeInside: buildRowPayload('Edge Inside'),
      edgeOutside: buildRowPayload('Edge Outside'),
      side: buildRowPayload('Side'),
      // New per-door format (supports unique settings per door)
      doors: doorsPayload,
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
  model: string;
  frame: { cycle: number; force: number };
  pocketzigzag: {
    cycle: number;
    force: number;
    verticalSpiral?: boolean;
    horizontalSpiral?: boolean;
    edgeCoverage?: boolean;
  };
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
      model: data.model,
      frame: data.frame,
      pocketzigzag: data.pocketzigzag,
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
export async function performAction(action: 'stopperUp' | 'stopperDown' | 'stopperUpB' | 'stopperDownB' | 'homing' | 'enable' | 'disable' | 'scan' | 'stop' | 'toolLift' | 'toolDrop' | 'laserOn' | 'laserOff') {
  return apiCall('/action', 'POST', { action });
}

// Toggle state for Table open/close
export async function toggleTableState(tableId: 'tableAOpenClose' | 'tableBOpenClose') {
  return apiCall(`/toggle_state/${tableId}`, 'GET');
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

// Export all API functions
export const api = {
  triggerRobotProcess,
  saveModalData,
  getModalData,
  startTableAProcess,
  startTableAProcessLegacy,
  startTableBProcess,
  upload3DFile,
  toolToggle,
  performAction,
  toggleTableState,
  getTableState,
  getStopperState,
  getRobotStatus,
  getProcessStatus,
  checkToolStatus,
  getLogsHistory,
};
