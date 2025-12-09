# Door-Based Model Configuration Layout

## Overview
Table A now uses **one model selection for all four doors**. You pick the model once, and each door keeps its own force/cycle values for every function (Frame, Pocket ZigZag, 3D, Edge Outside, Side).

## Key Features

### 1. **Global Model + Door Tabs**
- One model dropdown applies to all doors (Model A-E)
- Tabs for Door 1-4 to tweak per-door settings
- Green dot shows a door is configured with the chosen model
- Active tab is highlighted for clarity

### 2. **Per-Door Function Settings**
- Each door keeps independent Force/Cycle for every function
- Switching tabs changes the function values shown, not the model

### 3. **Configuration Structure**

```
Table A Configuration
├── Model (single select A/B/C/D/E for all doors)
├── Door 1 (Tab)
│   ├── Frame: Force [0-25], Cycle [0-25]
│   ├── Pocket ZigZag: Force [0-25], Cycle [0-25]
│   ├── 3D: Force [0-25], Cycle [0-25]
│   ├── Edge Outside: Force [0-25], Cycle [0-25]
│   └── Side: Force [0-25], Cycle [0-25]
│
├── Door 2 (Tab)
│   └── ... (same structure)
│
├── Door 3 (Tab)
│   └── ... (same structure)
│
└── Door 4 (Tab)
  └── ... (same structure)
```

### 4. **Table B Configuration**
Table B maintains the original single-model configuration:
- One model selection for the entire table
- Standard Force & Cycle settings for each function

## User Workflow

### Configuring Table A:
1. **Choose** a model (A-E) once at the top (applies to all doors)
2. **Select** a door tab (1, 2, 3, or 4)
3. **Configure** Force and Cycle values for each function
4. **Repeat** for other doors as needed
5. **Start Scan** or **Start Task** when ready

### Visual Indicators:
- **Blue underline**: Currently selected door
- **Green dot**: Door has a model configured
- **No dot**: Door has no model selected

## Benefits

✅ **Simplicity**: Choose one model once for all doors  
✅ **Per-Door Precision**: Each door keeps its own force/cycle values  
✅ **Clear Organization**: Tab-based interface keeps configurations organized  
✅ **Visual Feedback**: Easy to see which doors are configured  
✅ **Consistent Runs**: All doors operate with the same model choice  

## Technical Implementation

### Data Structure:
```typescript
type DoorConfig = {
  doorNumber: number;
  model: string; // auto-populated from the global model choice
  rows: RowConfig[];
};

// One model applied to all doors, with door-specific force/cycle rows
const tableAModel = 'modelB';
doorConfigs = [
  { doorNumber: 1, model: tableAModel, rows: [...] },
  { doorNumber: 2, model: tableAModel, rows: [...] },
  { doorNumber: 3, model: tableAModel, rows: [...] },
  { doorNumber: 4, model: tableAModel, rows: [...] },
]
```

### State Management:
- `tableAModel`: Single model selection that applies to all doors
- `doorConfigs`: Array of configurations for all 4 doors
- `selectedDoor`: Currently active door tab (1-4)
- Each door's data is independently managed and persisted

## Example Scenarios

### Scenario: One model, varied force/cycle
- Model: B (applies to all doors)
- Door 1: Frame 10/5, Pocket 8/4, 3D 6/2, Edge 7/3, Side 5/2
- Door 2: Frame 12/6, Pocket 9/5, 3D 7/3, Edge 8/4, Side 6/3
- Door 3: Frame 9/4, Pocket 7/3, 3D 5/2, Edge 6/2, Side 4/1
- Door 4: Frame 11/5, Pocket 10/5, 3D 8/4, Edge 9/4, Side 7/3

---

**Note**: This design ensures maximum flexibility for production environments where different doors may require different processing models based on the specific operations being performed.
