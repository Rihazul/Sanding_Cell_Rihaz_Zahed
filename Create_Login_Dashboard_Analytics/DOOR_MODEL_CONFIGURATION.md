# Door-Based Model Configuration Layout

## Overview
The Table A configuration now supports **independent model selection for each door (1-4)**, allowing different models (A-E) to be assigned to each door without assuming consistency.

## Key Features

### 1. **Door Tabs Interface**
- Four tabs representing Door 1, Door 2, Door 3, and Door 4
- Visual indicator (green dot) shows which doors have models configured
- Active tab is highlighted with blue underline
- Easy switching between door configurations

### 2. **Per-Door Model Selection**
Each door can have:
- **Independent Model Selection**: Choose from Model A, B, C, D, or E
- **Separate Configuration**: Each door maintains its own:
  - Frame settings (Force & Cycle)
  - Pocket ZigZag settings (Force & Cycle)
  - 3D settings (Force & Cycle)
  - Edge Outside settings (Force & Cycle)
  - Side settings (Force & Cycle)

### 3. **Configuration Structure**

```
Table A Configuration
├── Door 1 (Tab)
│   ├── Model: [Select A/B/C/D/E]
│   ├── Frame: Force [0-25], Cycle [0-25]
│   ├── Pocket ZigZag: Force [0-25], Cycle [0-25]
│   ├── 3D: Force [0-25], Cycle [0-25]
│   ├── Edge Outside: Force [0-25], Cycle [0-25]
│   └── Side: Force [0-25], Cycle [0-25]
│
├── Door 2 (Tab)
│   ├── Model: [Select A/B/C/D/E]
│   └── ... (same structure)
│
├── Door 3 (Tab)
│   ├── Model: [Select A/B/C/D/E]
│   └── ... (same structure)
│
└── Door 4 (Tab)
    ├── Model: [Select A/B/C/D/E]
    └── ... (same structure)
```

### 4. **Table B Configuration**
Table B maintains the original single-model configuration:
- One model selection for the entire table
- Standard Force & Cycle settings for each function

## User Workflow

### Configuring Table A:
1. **Navigate** to Table A Configuration panel
2. **Select** a door tab (1, 2, 3, or 4)
3. **Choose** the appropriate model (A-E) for that door
4. **Configure** Force and Cycle values for each function
5. **Repeat** for other doors as needed
6. **Start Scan** or **Start Task** when ready

### Visual Indicators:
- **Blue underline**: Currently selected door
- **Green dot**: Door has a model configured
- **No dot**: Door has no model selected

## Benefits

✅ **Flexibility**: Each door can use a different model  
✅ **No Assumptions**: System doesn't assume consistency across doors  
✅ **Clear Organization**: Tab-based interface keeps configurations organized  
✅ **Visual Feedback**: Easy to see which doors are configured  
✅ **Independent Settings**: Each door's parameters are isolated  

## Technical Implementation

### Data Structure:
```typescript
type DoorConfig = {
  doorNumber: number;
  model: string;
  rows: RowConfig[];
};

// Each door maintains:
doorConfigs = [
  { doorNumber: 1, model: 'modelA', rows: [...] },
  { doorNumber: 2, model: 'modelC', rows: [...] },
  { doorNumber: 3, model: 'modelB', rows: [...] },
  { doorNumber: 4, model: 'modelE', rows: [...] },
]
```

### State Management:
- `doorConfigs`: Array of configurations for all 4 doors
- `selectedDoor`: Currently active door tab (1-4)
- Each door's data is independently managed and persisted

## Example Scenarios

### Scenario 1: Different Models per Door
- Door 1: Model A - for large frames
- Door 2: Model C - for pocket designs
- Door 3: Model B - for 3D operations
- Door 4: Model E - for edge finishing

### Scenario 2: Partial Configuration
- Door 1: Model A (configured) ✅
- Door 2: Not configured yet
- Door 3: Model D (configured) ✅
- Door 4: Not configured yet

The system allows flexible partial configurations without requiring all doors to be set up.

---

**Note**: This design ensures maximum flexibility for production environments where different doors may require different processing models based on the specific operations being performed.
