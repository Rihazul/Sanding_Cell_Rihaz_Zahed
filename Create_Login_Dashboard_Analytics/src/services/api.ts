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
      throw new Error(`API call failed: ${response.statusText}`);
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

  // Build the payload matching the Flask backend format
  const payload = {
    TableA: {
      // Keep legacy compatibility: backend expects TableA.model
      model: inferredModel,
      doors: data.doorConfigs.map(door => ({
        doorNumber: door.doorNumber,
        model: door.model,
        frame: { cycle: door.rows[0]?.cycle || 0, force: door.rows[0]?.force || 0 },
        pocketzigzag: {
          cycle: door.rows[1]?.cycle || 0,
          force: door.rows[1]?.force || 0,
          verticalSpiral: !!door.rows[1]?.verticalSpiral,
          horizontalSpiral: !!door.rows[1]?.horizontalSpiral,
          edgeCoverage: !!door.rows[1]?.edgeCoverage,
        },
        '3D': { cycle: door.rows[2]?.cycle || 0, force: door.rows[2]?.force || 0 },
        edgeOutside: { cycle: door.rows[3]?.cycle || 0, force: door.rows[3]?.force || 0 },
        side: { cycle: door.rows[4]?.cycle || 0, force: door.rows[4]?.force || 0 },
      }))
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
    verticalSpiral?: boolean;
    horizontalSpiral?: boolean;
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
                   toolNumber === 3 ? '/tool_toggle' :
                   '/tool_toggle4';
  return apiCall(endpoint, 'POST', { toolNumber, action });
}

// Action operations
export async function performAction(action: 'stopperUp' | 'stopperDown' | 'stopperUpB' | 'stopperDownB' | 'homing' | 'enable' | 'disable' | 'scan' | 'stop' | 'toolLift' | 'toolDrop') {
  return apiCall('/action', 'POST', { action });
}

// Toggle state for Table open/close
export async function toggleTableState(tableId: 'tableAOpenClose' | 'tableBOpenClose') {
  return apiCall(`/toggle_state/${tableId}`, 'GET');
}

// Get current state for Table open/close
export async function getTableState(tableId: 'tableAOpenClose' | 'tableBOpenClose') {
  return apiCall(`/get_state/${tableId}`, 'GET');
}

// Tool attachment status checks (returns shouldBlink boolean)
export async function checkToolStatus(toolNumber: 1 | 2 | 3) {
  const endpoint =
    toolNumber === 1 ? '/check_tool1_status' :
    toolNumber === 2 ? '/check_tool2_status' :
    '/check_tool3_status';
  return apiCall(endpoint, 'GET');
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
  checkToolStatus,
};
