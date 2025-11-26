// API service for robot control dashboard

const API_BASE_URL = 'http://localhost:5100';

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

// Start Table A process
export async function startTableAProcess(data: {
  model: string;
  frame: { cycle: number; force: number };
  pocketzigzag: { cycle: number; force: number };
  pocketsquare: { cycle: number; force: number };
  '3D': { cycle: number; force: number };
  edgeInside: { cycle: number; force: number };
  edgeOutside: { cycle: number; force: number };
  side: { cycle: number; force: number };
}) {
  return apiCall('/start_TableA_process', 'POST', { TableA: data });
}

// Start Table B process
export async function startTableBProcess(data: {
  model: string;
  frame: { cycle: number; force: number };
  pocketzigzag: { cycle: number; force: number };
  pocketsquare: { cycle: number; force: number };
  '3D': { cycle: number; force: number };
  edgeInside: { cycle: number; force: number };
  edgeOutside: { cycle: number; force: number };
  side: { cycle: number; force: number };
  robotSpeed: string;
  sandingSpeed: string;
  inverseOverlapping: number;
}) {
  return apiCall('/start_TableB_process', 'POST', { 
    TableB: {
      model: data.model,
      frame: data.frame,
      pocketzigzag: data.pocketzigzag,
      pocketsquare: data.pocketsquare,
      '3D': data['3D'],
      edgeInside: data.edgeInside,
      edgeOutside: data.edgeOutside,
      side: data.side
    },
    robotSpeed: data.robotSpeed,
    sandingSpeed: data.sandingSpeed,
    inverseOverlapping: data.inverseOverlapping
  });
}

// Tool toggle operations
export async function toolToggle(toolNumber: 1 | 2 | 3, action: 'pick' | 'keep') {
  const endpoint = `/tool_toggle${toolNumber}`;
  return apiCall(endpoint, 'POST', { toolNumber, action });
}

// Action operations
export async function performAction(action: 'stopperUp' | 'stopperDown' | 'homing' | 'enable'| 'disable' | 'scan') {
  return apiCall('/action', 'POST', { action });
}

// Toggle state for Table A open/close
export async function getTableAOpenCloseState() {
  return apiCall('/toggle_state/tableAOpenClose', 'GET');
}

// Export all API functions
export const api = {
  triggerRobotProcess,
  saveModalData,
  startTableAProcess,
  startTableBProcess,
  toolToggle,
  performAction,
  getTableAOpenCloseState,
};
