import React from 'react';
import { createPortal } from 'react-dom';
import { Card, CardContent, CardHeader, CardTitle } from '../ui/card';
import { Button } from '../ui/button';
import { Badge } from '../ui/badge';
import { startTableAProcess, startTableBProcess, performAction, upload3DFile, getProcessStatus, getScanStatus } from '../../services/api';

export type RowConfig = {
  label: string;
  selection: string;
  force: number;
  cycle: number;
  // Pocket ZigZag specific options
  verticalSpiral?: boolean;
  horizontalSpiral?: boolean;
  edgeCoverage?: boolean;
};

export type DoorConfig = {
  doorNumber: number;
  model: string;
  rows: RowConfig[];
};

interface CompactTableConfigProps {
  tableName: 'A' | 'B';
  model: string;
  setModel: (model: string) => void;
  rows: RowConfig[];
  setRows: React.Dispatch<React.SetStateAction<RowConfig[]>>;
  isActive: boolean;
  isOperating: boolean;
  setIsOperating: (operating: boolean) => void;
  addActivity: (message: string, type?: 'info' | 'success' | 'warning' | 'error') => void;
  robotSpeed: number[];
  sandingSpeed: number[];
  inverseOverlapping: number[];
  spiralSettings?: {
    enabled: boolean;
    speedPercent: number;
    radiusMm: number;
    linearSpeedMmS: number;
  };
  doorConfigs?: DoorConfig[];
  setDoorConfigs?: React.Dispatch<React.SetStateAction<DoorConfig[]>>;
}

export function CompactTableConfig({
  tableName,
  model,
  setModel,
  rows,
  setRows,
  isActive,
  isOperating,
  setIsOperating,
  addActivity,
  robotSpeed,
  sandingSpeed,
  inverseOverlapping,
  spiralSettings,
  doorConfigs,
  setDoorConfigs,
}: CompactTableConfigProps) {
  console.log('CompactTableConfig rendering:', tableName, 'rows:', rows.length, 'addActivity:', !!addActivity);
  const POCKET_MAX_OVERLAP_MM = 100;
  
  const [selectedDoor, setSelectedDoor] = React.useState<number>(1);
  const [scanCompleted, setScanCompleted] = React.useState<boolean>(false);
  const [lastScanSignature, setLastScanSignature] = React.useState<string | null>(null);
  const [lastScannedAt, setLastScannedAt] = React.useState<string | null>(null);
  const [isScanning, setIsScanning] = React.useState<boolean>(false);
  const [completionPopup, setCompletionPopup] = React.useState<{ title: string; subtitle?: string } | null>(null);
  const completionTimerRef = React.useRef<number | null>(null);
  const isModelF = model === 'modelF';
  const isModelFAllowedRow = (label: string) => label === 'Pocket ZigZag';
  const rowDisplayLabel = (label: string) =>
    isModelF && label === 'Pocket ZigZag' ? 'Flat ZigZag' : label;

  const [rowDoorSelections, setRowDoorSelections] = React.useState<Record<string, number[]>>({
    Frame: [],
    'Pocket ZigZag': [],
    '3D': [],
    'Edge Outside': [],
    Side: [],
  });

  const formatModelName = (value: string) => {
    if (value === 'modelA') return 'Model A - Shaker A';
    if (value === 'modelB') return 'Model B - Shaker B';
    if (value === 'modelC') return 'Model C - Moulure Externe';
    if (value === 'modelD') return 'Model D - Moulure Interne';
    if (value === 'modelE') return 'Model E - Moulure Interne et Externe';
    if (value === 'modelF') return 'Model F - Flat';
    return value || 'No model selected';
  };
  
  React.useEffect(() => {
    console.log(`Table ${tableName}: addActivity prop changed:`, !!addActivity);
  }, [addActivity, tableName]);

  React.useEffect(() => {
    return () => {
      if (completionTimerRef.current !== null) {
        window.clearTimeout(completionTimerRef.current);
      }
    };
  }, []);

  const showCompletionPopup = (title: string, subtitle?: string) => {
    if (tableName !== 'A') return;
    setCompletionPopup({ title, subtitle });
    if (completionTimerRef.current !== null) {
      window.clearTimeout(completionTimerRef.current);
    }
    completionTimerRef.current = window.setTimeout(() => {
      setCompletionPopup(null);
    }, 2600);
  };

  const waitForBackendProcessCompletion = async (
    timeoutMs = 6 * 60 * 60 * 1000,
    pollMs = 2000
  ) => {
    const start = Date.now();
    while (Date.now() - start < timeoutMs) {
      const statusRes = await getProcessStatus();
      const status = String(statusRes?.status || '').toLowerCase();
      if (status && status !== 'in_progress') {
        return status;
      }
      await new Promise((resolve) => window.setTimeout(resolve, pollMs));
    }
    throw new Error(`Timed out waiting for task completion after ${Math.round(timeoutMs / 1000)}s`);
  };

  const getSwal = () => (window as any).Swal;
  const getTableADoorModels = () => {
    if (tableName !== 'A') return [] as string[];
    return (doorConfigs || [])
      .map((d) => `${d.doorNumber}:${(d.model || '').trim()}`)
      .filter(Boolean);
  };

  const getTableAScanSignature = () => {
    if (tableName !== 'A') return '';
    const baseModel = (model || '').trim();
    const doorModelSig = (doorConfigs || [])
      .map((d) => `${d.doorNumber}:${(d.model || '').trim()}`)
      .join('|');
    return `${baseModel}::${doorModelSig}`;
  };

  React.useEffect(() => {
    let cancelled = false;
    if (tableName !== 'A') return;

    (async () => {
      try {
        const status = await getScanStatus();
        if (cancelled) return;
        const hasScan = !!status?.hasScan;
        setScanCompleted(hasScan);
        setLastScannedAt(status?.scannedAt || null);
        const signature = (status?.signature || '').trim();
        setLastScanSignature(signature || null);
      } catch {
        // Non-blocking; fallback to in-memory scan status.
      }
    })();

    return () => {
      cancelled = true;
    };
  }, [tableName]);

  const confirmScanForTableA = async () => {
    if (tableName !== 'A') return true;
    const hasAnyModel =
      !!(model || '').trim() ||
      (doorConfigs || []).some((d) => !!(d.model || '').trim());
    if (!hasAnyModel) {
      const warning = 'Select a model before scanning Table A.';
      addActivity(`Table ${tableName}: ${warning}`, 'warning');
      const swal = getSwal();
      if (swal?.fire) {
        await swal.fire({
          title: 'Model Required',
          text: warning,
          icon: 'warning',
          timer: 1800,
          showConfirmButton: false,
        });
      }
      return false;
    }

    const swal = getSwal();
    const selectedModelText = (model || '').trim()
      ? `Selected model: ${formatModelName(model)}.`
      : 'Model selected per-door configuration.';
    const rescanText = scanCompleted
      ? `A previous scan exists${lastScannedAt ? ` (${lastScannedAt})` : ''}. Do you want to run scan again?`
      : 'No previous scan found. Start a new scan?';
    const text = scanCompleted
      ? `${rescanText} ${selectedModelText}`
      : `Confirm scan for Table A. ${selectedModelText} Ensure the area is clear and setup is correct.`;
    if (!swal?.fire) {
      return window.confirm(text);
    }
    const result = await swal.fire({
      title: 'Confirm Scan',
      text,
      icon: 'warning',
      showCancelButton: true,
      confirmButtonText: 'Start Scan',
      cancelButtonText: 'Cancel',
      reverseButtons: true,
    });
    return !!result.isConfirmed;
  };

  const confirmStartTask = async () => {
    const swal = getSwal();
    const scanState =
      tableName === 'A'
        ? (scanCompleted ? 'Scan status: Completed.' : 'Scan status: Not marked completed.')
        : 'Scan status: Not required for Table B.';
    const modelState = model?.trim() ? `Model: ${formatModelName(model)}.` : 'Model: Not selected at table level.';
    const reminder = 'Please verify door selection, force/cycle values, and safety before continuing.';
    const text = `${scanState} ${modelState} ${reminder}`;
    const title = tableName === 'A' ? 'Confirm Start Task (Table A)' : 'Confirm Start Task (Table B)';
    if (!swal?.fire) {
      return window.confirm(`${title}\n\n${text}`);
    }
    const result = await swal.fire({
      title,
      text,
      icon: 'warning',
      showCancelButton: true,
      confirmButtonText: 'Start Task',
      cancelButtonText: 'Review Settings',
      reverseButtons: true,
    });
    return !!result.isConfirmed;
  };
  
  const handleStartScan = async () => {
    console.log('Start Scan clicked for Table', tableName);
    if (isOperating || isScanning) return;
    const confirmed = await confirmScanForTableA();
    if (!confirmed) {
      addActivity(`Table ${tableName}: Scan cancelled by user for safety check`, 'warning');
      return;
    }
    setIsOperating(true);
    setIsScanning(true);
      addActivity(`Table ${tableName}: Scan in progress...`, 'warning');
    try {
      await performAction('scan', {
        tableAScanSignature: getTableAScanSignature(),
        tableAModel: model || '',
        tableADoorModels: getTableADoorModels(),
      });
      setScanCompleted(true);
      const signature = getTableAScanSignature();
      setLastScanSignature(signature);
      setLastScannedAt(new Date().toISOString());
      addActivity(`Table ${tableName}: Scan completed successfully`, 'success');
      showCompletionPopup('Scan Completed', 'Table A scan completed');
    } catch (error) {
      addActivity(`Table ${tableName}: Scan failed - ${error}`, 'error');
    } finally {
      setIsScanning(false);
      setIsOperating(false);
    }
  };
  
  const handleStartTask = async () => {
    console.log('Start Task clicked for Table', tableName);

    if (isOperating || isScanning) return;
    if (tableName === 'A') {
      let effectiveScanCompleted = scanCompleted;
      let effectiveLastSignature = lastScanSignature;
      try {
        const status = await getScanStatus();
        const hasScan = !!status?.hasScan;
        if (!hasScan) {
          const warningText = 'Scan required before starting Table A task.';
          const swal = getSwal();
          if (swal?.fire) {
            await swal.fire({
              title: 'Scan Required',
              text: warningText,
              icon: 'warning',
              timer: 2000,
              showConfirmButton: false,
            });
          }
          addActivity(`Table ${tableName}: ${warningText}`, 'warning');
          return;
        }
        effectiveScanCompleted = true;
        setScanCompleted(true);
        if (status?.signature) {
          effectiveLastSignature = String(status.signature);
          setLastScanSignature(effectiveLastSignature);
        }
        if (status?.scannedAt) {
          setLastScannedAt(String(status.scannedAt));
        }
      } catch {
        // keep local checks as fallback
      }

      const scanSignatureNow = getTableAScanSignature();
      const scanInvalid =
        !effectiveScanCompleted ||
        !effectiveLastSignature ||
        scanSignatureNow !== effectiveLastSignature;
      if (scanInvalid) {
        const swal = getSwal();
        const warningText = 'Model/configuration changed or scan missing. Please run Scan first before Start Task.';
        if (swal?.fire) {
          await swal.fire({
            title: 'Scan Required',
            text: warningText,
            icon: 'warning',
            timer: 2000,
            showConfirmButton: false,
          });
        }
        addActivity(`Table ${tableName}: Scan required before starting task (model/config changed or no valid scan).`, 'warning');
        return;
      }
    }
    const confirmed = await confirmStartTask();
    if (!confirmed) {
      addActivity(`Table ${tableName}: Start task cancelled to review configuration`, 'warning');
      return;
    }

    setIsOperating(true);
    
    // For Table A, process all doors with their configurations
    if (tableName === 'A' && doorConfigs) {
      const configuredDoors = doorConfigs.filter(d => d.model && d.model !== '');
      const totalDoors = doorConfigs.length;
      const modelName = formatModelName(model);
      const hasTableModel = !!model && model.trim() !== '';
      const hasDoorModel = configuredDoors.length > 0;
      if (!hasTableModel && !hasDoorModel) {
        addActivity(`Table ${tableName}: Select a model before starting the task.`, 'warning');
        setIsOperating(false);
        return;
      }
      
      try {
        const selectedDoorsByRow = Object.fromEntries(
          Object.entries(rowDoorSelections).map(([label, doors]) => [label, [...new Set(doors)].sort()])
        ) as Record<string, number[]>;

        const normalizedDoorConfigs = doorConfigs.map(dc => ({
          ...dc,
          rows: dc.rows.map(r => ({ ...r })),
        }));

        // Ensure all selected doors for a row share the same task values unless explicitly set.
        // This prevents partial payloads like only one selected door being serialized with cycle/force.
        for (const [label, selectedDoors] of Object.entries(selectedDoorsByRow)) {
          if (!selectedDoors.length) continue;
          const rowIndex = rows.findIndex(r => r.label === label);
          if (rowIndex < 0) continue;

          const rowsForSelectedDoors = selectedDoors
            .map(doorNumber => normalizedDoorConfigs.find(dc => dc.doorNumber === doorNumber)?.rows[rowIndex])
            .filter((row): row is RowConfig => !!row);

          const templateRow =
            rowsForSelectedDoors.find(r =>
              r.force > 0 ||
              r.cycle > 0 ||
              !!r.verticalSpiral ||
              !!r.horizontalSpiral ||
              !!r.edgeCoverage
            ) || rowsForSelectedDoors[0];

          if (!templateRow) continue;

          normalizedDoorConfigs.forEach(dc => {
            if (!selectedDoors.includes(dc.doorNumber)) return;
            const currentRow = dc.rows[rowIndex];
            if (!currentRow) return;
            const shouldPopulateValues = currentRow.force <= 0 && currentRow.cycle <= 0;
            if (!shouldPopulateValues) return;
            dc.rows[rowIndex] = {
              ...currentRow,
              force: templateRow.force,
              cycle: templateRow.cycle,
              verticalSpiral: templateRow.verticalSpiral,
              horizontalSpiral: templateRow.horizontalSpiral,
              edgeCoverage: templateRow.edgeCoverage,
            };
          });
        }

        const effectiveDoorConfigs = normalizedDoorConfigs.map(dc => ({
          ...dc,
          rows: dc.rows.map(r => {
            if (isModelF && !isModelFAllowedRow(r.label)) {
              return {
                ...r,
                force: 0,
                cycle: 0,
                verticalSpiral: false,
                horizontalSpiral: false,
                edgeCoverage: false,
              };
            }
            const allowed = selectedDoorsByRow[r.label] || [];
            if (allowed.includes(dc.doorNumber)) return r;
            return { ...r, force: 0, cycle: 0 };
          })
        }));

        const validationErrors: string[] = [];
        let hasTaskSelection = false;
        effectiveDoorConfigs.forEach(dc => {
          dc.rows.forEach(r => {
            if (r.force > 0 || r.cycle > 0) {
              hasTaskSelection = true;
              if (r.force <= 0 || r.cycle <= 0) {
                validationErrors.push(`Door ${dc.doorNumber} ${r.label}: set force & cycle`);
              }
            }
          });
        });

        if (!hasTaskSelection) {
          validationErrors.push('Select at least one task with force and cycle.');
        }

        if (validationErrors.length) {
          const preview = validationErrors.slice(0, 3).join(' | ');
          const suffix = validationErrors.length > 3 ? ' ...' : '';
          addActivity(`Table ${tableName}: Fix task settings: ${preview}${suffix}`, 'warning');
          return;
        }

        addActivity(`Table ${tableName}: Starting task for all doors with ${modelName} (${configuredDoors.length} configured, ${totalDoors - configuredDoors.length} unconfigured)...`, 'info');

        // Build payload with all door configurations
        const overlapMm = Math.max(0, Math.min(POCKET_MAX_OVERLAP_MM, inverseOverlapping[0] ?? 0));
        const taskData = {
          doorConfigs: effectiveDoorConfigs,
          robotSpeed: (robotSpeed[0] / 100).toFixed(2),
          sandingSpeed: (sandingSpeed[0] / 100).toFixed(2),
          inverseOverlapping: overlapMm,
          spiralSettings,
        };
        
        // Send all door configurations to the backend
        const result = await startTableAProcess(taskData);

        if (!result?.success) {
          addActivity(`Table ${tableName}: Task failed to start (${result?.status || 'unknown'})`, 'error');
          return;
        }

        addActivity(`Table ${tableName}: Task started`, 'info');
        const finalStatus = await waitForBackendProcessCompletion();
        if (finalStatus === 'completed') {
          addActivity(`Table ${tableName}: Task completed successfully`, 'success');
          showCompletionPopup('Task Completed', 'Table A task completed');
        } else {
          addActivity(`Table ${tableName}: Task finished with status: ${finalStatus}`, 'warning');
        }
        
      } catch (error) {
        addActivity(`Table ${tableName}: Task failed - ${error}`, 'error');
      } finally {
        setIsOperating(false);
      }
    } else {
      // Table B logic (single model)
      if (!model || model.trim() === '') {
        addActivity(`Table ${tableName}: Select a model before starting the task.`, 'warning');
        setIsOperating(false);
        return;
      }
      if (!['modelA', 'modelB', 'modelC', 'modelD', 'modelE'].includes(model)) {
        addActivity(`Table ${tableName}: Model ${formatModelName(model)} is not supported by backend yet.`, 'warning');
        setIsOperating(false);
        return;
      }

      const modelName = formatModelName(model);
      
      addActivity(`Table ${tableName}: Starting task with ${modelName}`, 'info');
      
      try {
        // Build payload from rows
        const overlapMm = Math.max(0, Math.min(POCKET_MAX_OVERLAP_MM, inverseOverlapping[0] ?? 0));
        const taskData = {
          model,
          frame: { cycle: rows[0].cycle, force: rows[0].force },
          pocketzigzag: {
            cycle: rows[1].cycle,
            force: rows[1].force,
            verticalSpiral: !!rows[1].verticalSpiral,
            horizontalSpiral: !!rows[1].horizontalSpiral,
            edgeCoverage: !!rows[1].edgeCoverage,
          },
          '3D': { cycle: rows[2].cycle, force: rows[2].force },
          edgeOutside: { cycle: rows[3].cycle, force: rows[3].force },
          side: { cycle: rows[4].cycle, force: rows[4].force },
          robotSpeed: (robotSpeed[0] / 100).toFixed(2),
          sandingSpeed: (sandingSpeed[0] / 100).toFixed(2),
          inverseOverlapping: overlapMm,
          spiralSettings,
        };
        
        const result = await startTableBProcess(taskData);

        if (!result?.success) {
          addActivity(`Table ${tableName}: Task failed to start (${result?.status || 'unknown'})`, 'error');
          return;
        }

        addActivity(`Table ${tableName}: Task started with ${modelName}`, 'info');
        const finalStatus = await waitForBackendProcessCompletion();
        if (finalStatus === 'completed') {
          addActivity(`Table ${tableName}: Task completed successfully with ${modelName}`, 'success');
        } else {
          addActivity(`Table ${tableName}: Task finished with status: ${finalStatus}`, 'warning');
        }
        
      } catch (error) {
        addActivity(`Table ${tableName}: Task failed - ${error}`, 'error');
      } finally {
        setIsOperating(false);
      }
    }
  };
  
  const handleUpload3DFile = async () => {
    console.log('Upload 3D File clicked for Table', tableName);
    
    // Create file input element
    const input = document.createElement('input');
    input.type = 'file';
    input.accept = '.stp,.step';
    
    input.onchange = async (e) => {
      const file = (e.target as HTMLInputElement).files?.[0];
      if (!file) return;
      
      setIsOperating(true);
      addActivity(`Table ${tableName}: Uploading 3D file "${file.name}"...`, 'info');
      
      try {
        const result = await upload3DFile(file);
        if (result.success) {
          addActivity(`Table ${tableName}: 3D file uploaded successfully`, 'success');
        } else {
          addActivity(`Table ${tableName}: Upload failed - ${result.message || 'Unknown error'}`, 'error');
        }
      } catch (error) {
        addActivity(`Table ${tableName}: Upload failed - ${error}`, 'error');
      } finally {
        setIsOperating(false);
      }
    };
    
    input.click();
  };
  
  // Get current door configuration
  const currentDoorConfig = doorConfigs?.find(d => d.doorNumber === selectedDoor);
  const currentRows = tableName === 'A' && doorConfigs ? (currentDoorConfig?.rows || rows) : rows;
  const buildDisplayRows = (sourceRows: RowConfig[]) =>
    sourceRows
      .map((row, idx) => ({ row, idx }))
      .filter(({ row }) => !isModelF || isModelFAllowedRow(row.label));
  const currentDisplayRows = buildDisplayRows(currentRows);
  const tableBDisplayRows = buildDisplayRows(rows);

  const handleModelChange = (newModel: string) => {
    if (tableName === 'A' && newModel !== model) {
      if (scanCompleted) {
        addActivity(`Table ${tableName}: Model changed. Previous scan is now invalid; run Scan again.`, 'warning');
      }
      setScanCompleted(false);
      setLastScanSignature(null);
    }
    setModel(newModel);

    if (tableName === 'A' && doorConfigs && setDoorConfigs) {
      setDoorConfigs(prev => prev.map(cfg => ({ ...cfg, model: newModel })));
    }
  };

  const handleRowChange = (idx: number, field: 'selection' | 'force' | 'cycle', value: any) => {
    if (tableName === 'A' && doorConfigs && setDoorConfigs) {
      const rowLabel = rows[idx]?.label;
      if (!rowLabel) return;
      setRowDoorSelections(prev => {
        const current = prev[rowLabel] || [];
        if (current.includes(selectedDoor)) {
          return prev;
        }
        return { ...prev, [rowLabel]: [...current, selectedDoor].sort() };
      });

      setDoorConfigs(prev =>
        prev.map(dc => {
          if (dc.doorNumber !== selectedDoor) return dc;
          const newRows = [...dc.rows];
          newRows[idx] = { ...newRows[idx], [field]: value };
          return { ...dc, rows: newRows };
        })
      );
    } else {
      setRows((prev: RowConfig[]) => {
        const next = [...prev];
        next[idx] = { ...next[idx], [field]: value };
        return next;
      });
    }
  };

  const toggleRowDoor = (label: string, doorNumber: number) => {
    const rowIndex = rows.findIndex(r => r.label === label);
    const sourceDoorNumber = selectedDoor;
    const currentSelection = rowDoorSelections[label] || [];
    const wasSelected = currentSelection.includes(doorNumber);

    if (wasSelected) {
      // Allow door deselection without page refresh.
      const nextSelection = currentSelection.filter(d => d !== doorNumber);
      setRowDoorSelections(prev => ({ ...prev, [label]: nextSelection }));
      if (selectedDoor === doorNumber) {
        // Keep editing context on another selected door when possible.
        setSelectedDoor(nextSelection[0] ?? 1);
      }
      return;
    }

    setRowDoorSelections(prev => {
      const current = prev[label] || [];
      const exists = current.includes(doorNumber);
      const next = exists ? current : [...current, doorNumber].sort();
      return { ...prev, [label]: next };
    });
    setSelectedDoor(doorNumber);

    if (!wasSelected && tableName === 'A' && doorConfigs && setDoorConfigs && rowIndex >= 0) {
      setDoorConfigs(prev =>
        prev.map(dc => {
          if (dc.doorNumber !== doorNumber) return dc;
          const newRows = [...dc.rows];
          const targetRow = newRows[rowIndex];
          if (!targetRow) return dc;
          const hasValues =
            targetRow.force > 0 ||
            targetRow.cycle > 0 ||
            !!targetRow.verticalSpiral ||
            !!targetRow.horizontalSpiral ||
            !!targetRow.edgeCoverage;
          if (hasValues) return dc;
          const sourceDoor = prev.find(d => d.doorNumber === sourceDoorNumber);
          const sourceRow = sourceDoor?.rows[rowIndex];
          if (!sourceRow) return dc;
          newRows[rowIndex] = { ...targetRow, ...sourceRow };
          return { ...dc, rows: newRows };
        })
      );
    }

  };

  const handlePocketZigZagOptionChange = (idx: number, option: 'verticalSpiral' | 'horizontalSpiral' | 'edgeCoverage', checked: boolean) => {
    if (tableName === 'A' && doorConfigs && setDoorConfigs) {
      const rowLabel = rows[idx]?.label;
      if (!rowLabel) return;
      setRowDoorSelections(prev => {
        const current = prev[rowLabel] || [];
        if (current.includes(selectedDoor)) {
          return prev;
        }
        return { ...prev, [rowLabel]: [...current, selectedDoor].sort() };
      });

      setDoorConfigs(prev =>
        prev.map(dc => {
          if (dc.doorNumber !== selectedDoor) return dc;
          const newRows = [...dc.rows];
          const nextRow = { ...newRows[idx], [option]: checked };
          if (option === 'verticalSpiral' && checked) {
            nextRow.horizontalSpiral = false;
          }
          if (option === 'horizontalSpiral' && checked) {
            nextRow.verticalSpiral = false;
          }
          newRows[idx] = nextRow;
          return { ...dc, rows: newRows };
        })
      );
    } else {
      setRows((prev: RowConfig[]) => {
        const next = [...prev];
        const nextRow = { ...next[idx], [option]: checked };
        if (option === 'verticalSpiral' && checked) {
          nextRow.horizontalSpiral = false;
        }
        if (option === 'horizontalSpiral' && checked) {
          nextRow.verticalSpiral = false;
        }
        next[idx] = nextRow;
        return next;
      });
    }
  };

  return (
    <>
      {completionPopup &&
        createPortal(
          <div className="fixed inset-0 z-[9999] flex items-center justify-center pointer-events-none">
            <div className="absolute inset-0 bg-black/35 backdrop-blur-[2px]" />
            <div className="relative bg-white border-2 border-green-200 ring-1 ring-green-100 shadow-[0_32px_120px_rgba(0,0,0,0.35)] rounded-3xl px-12 py-10 text-center min-w-[360px] max-w-[460px]">
              <div className="absolute left-8 top-6 h-1.5 w-12 rounded-full bg-green-300" />
              <div className="absolute right-8 top-10 h-1.5 w-16 rounded-full bg-green-200" />
              <div
                className="mx-auto mb-5 flex h-24 w-24 items-center justify-center rounded-full border-[6px] text-4xl shadow-[0_10px_28px_rgba(34,197,94,0.35)]"
                style={{ borderColor: '#22c55e', color: '#16a34a', backgroundColor: '#f0fdf4' }}
              >
                ✓
              </div>
              <div className="text-2xl font-bold text-gray-900 tracking-tight">{completionPopup.title}</div>
              {completionPopup.subtitle && (
                <div className="text-sm text-gray-600 mt-2 leading-relaxed">{completionPopup.subtitle}</div>
              )}
            </div>
          </div>,
          document.body
        )}
      <Card className="shadow-xl border border-slate-300 bg-white/95 backdrop-blur-sm">
      <CardHeader className="bg-gradient-to-r from-indigo-50 to-cyan-50">
        <CardTitle className="flex items-center justify-between">
          Table {tableName} Configuration
          <Badge variant={isActive ? 'default' : 'secondary'} className={isActive ? 'bg-green-500' : ''}>
            {isActive ? 'Active' : 'Inactive'}
          </Badge>
        </CardTitle>
      </CardHeader>
      <CardContent className="pt-6 bg-white rounded-b-lg">
        <div className="border-2 border-indigo-300 rounded-xl p-5 bg-white shadow-[0_8px_24px_rgba(30,64,175,0.08)]">
          {tableName === 'A' && doorConfigs ? (
            <>
              {/* Door Selection Tabs */}
              <div className="bg-white rounded-md p-4 border border-gray-200 mb-4">
                <label className="text-sm text-gray-600 mb-2 flex items-center gap-1">
                  Model for all doors
                  <span className="text-gray-400 text-xs">ⓘ</span>
                </label>
                <select
                  value={model}
                  onChange={(e) => handleModelChange(e.target.value)}
                  disabled={isOperating}
                  className="w-full px-3 py-2 border border-gray-300 rounded-md text-sm bg-white focus:ring-2 focus:ring-blue-500 focus:border-transparent disabled:bg-gray-100 disabled:cursor-not-allowed"
                >
                  <option value="">Select a Model</option>
                  <option value="modelA">Model A - Shaker A</option>
                  <option value="modelB">Model B - Shaker B</option>
                  <option value="modelC">Model C - Moulure Externe</option>
                  <option value="modelD">Model D - Moulure Interne</option>
                  <option value="modelE">Model E - Moulure Interne et Externe</option>
                  <option value="modelF">Model F - Flat</option>
                </select>
                <div className="mt-2 text-xs">
                  {scanCompleted ? (
                    <span className="text-green-700">
                      Scan on record{lastScannedAt ? ` (${lastScannedAt})` : ''}. Re-scan if model/door setup changed.
                    </span>
                  ) : (
                    <span className="text-amber-700">No scan on record yet.</span>
                  )}
                </div>
              </div>

              <div className="bg-gray-50 rounded-lg p-4 mb-4">
                <div className="mt-2 space-y-3">
                  {currentDisplayRows.map(({ row, idx }) => (
                    <div key={row.label} className={`bg-white rounded-md p-3 border ${row.label === 'Pocket ZigZag' ? 'border-indigo-300 shadow-sm' : 'border-gray-200'}`}>
                      {/* Main row: Label + Door buttons + Force + Cycle */}
                      <div className="flex flex-wrap items-center gap-3 justify-between">
                        <div className="text-sm font-medium text-gray-700 flex items-center gap-1 whitespace-nowrap">
                          {row.label === 'Pocket ZigZag' && (
                            <span className="text-indigo-500 mr-1">⬡</span>
                          )}
                          {rowDisplayLabel(row.label)}
                          <span className="text-gray-400 text-xs">ⓘ</span>
                        </div>

                        <div className="flex items-center gap-3 flex-wrap">
                          <div className="flex items-center gap-2 flex-wrap">
                            {[1, 2, 3, 4].map((doorNum) => {
                              const doorConfig = doorConfigs.find(d => d.doorNumber === doorNum);
                              const hasModel = doorConfig?.model && doorConfig.model !== '';
                              const isSelected = (rowDoorSelections[row.label] || []).includes(doorNum);
                              return (
                                <button
                                  key={doorNum}
                                  type="button"
                                  onClick={(e) => {
                                    e.preventDefault();
                                    toggleRowDoor(row.label, doorNum);
                                  }}
                                  disabled={isOperating}
                                  className={`min-w-[78px] px-3 py-1 text-xs font-semibold text-center transition-colors relative disabled:cursor-not-allowed disabled:opacity-100 rounded-md border ${
                                    isSelected
                                      ? 'text-white bg-blue-600 border-blue-600 hover:bg-blue-700'
                                      : 'text-gray-900 bg-white border-gray-500 hover:bg-gray-50'
                                  }`}
                                  style={{
                                    opacity: 1,
                                    color: isSelected ? '#ffffff' : '#111827',
                                    backgroundColor: isSelected ? '#2563eb' : '#ffffff',
                                    borderColor: isSelected ? '#2563eb' : '#6b7280',
                                    fontWeight: 600,
                                  }}
                                >
                                  Door {doorNum}
                                  {hasModel && (
                                    <span className="absolute -top-1 -right-1 w-2.5 h-2.5 bg-green-500 rounded-full ring-2 ring-white"></span>
                                  )}
                                </button>
                              );
                            })}
                          </div>

                          <div className="flex items-center gap-3 flex-nowrap">
                            <div className="flex items-center gap-1.5">
                              <label className="text-xs text-gray-500 font-medium whitespace-nowrap">Force:</label>
                              <select
                                value={row.force}
                                disabled={isOperating}
                                onChange={(e: React.ChangeEvent<HTMLSelectElement>) => {
                                  handleRowChange(idx, 'force', Number(e.target.value));
                                }}
                                className="px-2 py-1 border border-gray-300 rounded-md text-sm w-16 focus:ring-2 focus:ring-blue-500 focus:border-transparent disabled:bg-gray-100 disabled:cursor-not-allowed"
                              >
                                <option value={0}>-</option>
                                {Array.from({ length: 25 }, (_, i) => i + 1).map((n) => (
                                  <option key={n} value={n}>
                                    {n}
                                  </option>
                                ))}
                              </select>
                            </div>

                            <div className="flex items-center gap-1.5">
                              <label className="text-xs text-gray-500 font-medium whitespace-nowrap">Cycle:</label>
                              <select
                                value={row.cycle}
                                disabled={isOperating}
                                onChange={(e: React.ChangeEvent<HTMLSelectElement>) => {
                                  handleRowChange(idx, 'cycle', Number(e.target.value));
                                }}
                                className="px-2 py-1 border border-gray-300 rounded-md text-sm w-16 focus:ring-2 focus:ring-blue-500 focus:border-transparent disabled:bg-gray-100 disabled:cursor-not-allowed"
                              >
                                <option value={0}>-</option>
                                {Array.from({ length: 25 }, (_, i) => i + 1).map((n) => (
                                  <option key={n} value={n}>
                                    {n}
                                  </option>
                                ))}
                              </select>
                            </div>
                          </div>
                        </div>
                      </div>

                      {/* Pocket ZigZag Options - Second line below */}
                      {row.label === 'Pocket ZigZag' && (
                        <div className="mt-4 pt-6 border-t border-indigo-100 flex items-center justify-center gap-3">
                          <span className="text-sm text-gray-500 font-medium">Pattern:</span>
                          <div className="flex items-center gap-3">
                            <label className={`flex items-center gap-2 px-4 py-2 rounded-lg border-2 cursor-pointer transition-colors ${
                              row.verticalSpiral 
                                ? 'bg-blue-500 border-blue-500 text-white' 
                                : 'bg-white border-gray-200 text-gray-700 hover:border-blue-400'
                            }`}>
                              <input
                                type="checkbox"
                                checked={row.verticalSpiral || false}
                                onChange={(e) => handlePocketZigZagOptionChange(idx, 'verticalSpiral', e.target.checked)}
                                disabled={isOperating}
                                className="sr-only"
                              />
                              <span className="text-sm font-medium">↕ Vertical</span>
                            </label>
                            <label className={`flex items-center gap-2 px-4 py-2 rounded-lg border-2 cursor-pointer transition-colors ${
                              row.horizontalSpiral 
                                ? 'bg-blue-500 border-blue-500 text-white' 
                                : 'bg-white border-gray-200 text-gray-700 hover:border-blue-400'
                            }`}>
                              <input
                                type="checkbox"
                                checked={row.horizontalSpiral || false}
                                onChange={(e) => handlePocketZigZagOptionChange(idx, 'horizontalSpiral', e.target.checked)}
                                disabled={isOperating}
                                className="sr-only"
                              />
                              <span className="text-sm font-medium">↔ Horizontal</span>
                            </label>
                          </div>
                        </div>
                      )}
                    </div>
                  ))}
                </div>
              </div>
            </>
          ) : (
            <div className="bg-gray-50 rounded-lg p-4 mb-4">
              <label className="text-sm text-gray-500 mb-2 flex items-center gap-1">
                Model
                <span className="text-gray-400 text-xs">ⓘ</span>
              </label>
              <select
                value={model}
                onChange={(e) => setModel(e.target.value)}
                disabled={isOperating}
                className="w-full px-3 py-2 border border-gray-300 rounded-md text-sm bg-white focus:ring-2 focus:ring-blue-500 focus:border-transparent disabled:bg-gray-100 disabled:cursor-not-allowed"
              >
                <option value="">Select Model</option>
                <option value="modelA">Model A</option>
                <option value="modelB">Model B</option>
                <option value="modelC">Model C</option>
                <option value="modelD">Model D</option>
                <option value="modelE">Model E</option>
                <option value="modelF">Model F</option>
              </select>

              <div className="mt-6 space-y-3">
                {tableBDisplayRows.map(({ row, idx }) => (
                  <div key={row.label} className={`bg-white rounded-md p-3 border ${row.label === 'Pocket ZigZag' ? 'border-indigo-300 shadow-sm' : 'border-gray-200'}`}>
                    {/* Main row: Label + Force + Cycle */}
                    <div className="flex items-center justify-between gap-4">
                      <div className="text-sm font-medium text-gray-700 flex items-center gap-1">
                        {row.label === 'Pocket ZigZag' && (
                          <span className="text-indigo-500 mr-1">⬡</span>
                        )}
                        {rowDisplayLabel(row.label)}
                        <span className="text-gray-400 text-xs">ⓘ</span>
                      </div>

                      <div className="flex items-center gap-3">
                        <div className="flex items-center gap-1.5">
                          <label className="text-xs text-gray-500 font-medium whitespace-nowrap">Force:</label>
                          <select
                            value={row.force}
                            disabled={isOperating}
                            onChange={(e: React.ChangeEvent<HTMLSelectElement>) => {
                              const v = Number(e.target.value);
                              setRows((prev: RowConfig[]) => {
                                const next = [...prev];
                                next[idx] = { ...next[idx], force: v };
                                return next;
                              });
                            }}
                            className="px-2 py-1 border border-gray-300 rounded-md text-sm w-16 focus:ring-2 focus:ring-blue-500 focus:border-transparent disabled:bg-gray-100 disabled:cursor-not-allowed"
                          >
                            <option value={0}>-</option>
                            {Array.from({ length: 25 }, (_, i) => i + 1).map((n) => (
                              <option key={n} value={n}>
                                {n}
                              </option>
                            ))}
                          </select>
                        </div>

                        <div className="flex items-center gap-1.5">
                          <label className="text-xs text-gray-500 font-medium whitespace-nowrap">Cycle:</label>
                          <select
                            value={row.cycle}
                            disabled={isOperating}
                            onChange={(e: React.ChangeEvent<HTMLSelectElement>) => {
                              const v = Number(e.target.value);
                              setRows((prev: RowConfig[]) => {
                                const next = [...prev];
                                next[idx] = { ...next[idx], cycle: v };
                                return next;
                              });
                            }}
                            className="px-2 py-1 border border-gray-300 rounded-md text-sm w-16 focus:ring-2 focus:ring-blue-500 focus:border-transparent disabled:bg-gray-100 disabled:cursor-not-allowed"
                          >
                            <option value={0}>-</option>
                            {Array.from({ length: 25 }, (_, i) => i + 1).map((n) => (
                              <option key={n} value={n}>
                                {n}
                              </option>
                            ))}
                          </select>
                        </div>
                      </div>
                    </div>

                    {/* Pocket ZigZag Options - Second line below */}
                    {row.label === 'Pocket ZigZag' && (
                      <div className="mt-4 pt-6 border-t border-indigo-100 flex items-center justify-center gap-3">
                        <span className="text-sm text-gray-500 font-medium">Pattern:</span>
                        <div className="flex items-center gap-3">
                          <label className={`flex items-center gap-2 px-4 py-2 rounded-lg border-2 cursor-pointer transition-colors ${
                            row.verticalSpiral 
                              ? 'bg-blue-500 border-blue-500 text-white' 
                              : 'bg-white border-gray-200 text-gray-700 hover:border-blue-400'
                          }`}>
                            <input
                              type="checkbox"
                              checked={row.verticalSpiral || false}
                              onChange={(e) => handlePocketZigZagOptionChange(idx, 'verticalSpiral', e.target.checked)}
                              disabled={isOperating}
                              className="sr-only"
                            />
                            <span className="text-sm font-medium">↕ Vertical</span>
                          </label>
                          <label className={`flex items-center gap-2 px-4 py-2 rounded-lg border-2 cursor-pointer transition-colors ${
                            row.horizontalSpiral 
                              ? 'bg-blue-500 border-blue-500 text-white' 
                              : 'bg-white border-gray-200 text-gray-700 hover:border-blue-400'
                          }`}>
                            <input
                              type="checkbox"
                              checked={row.horizontalSpiral || false}
                              onChange={(e) => handlePocketZigZagOptionChange(idx, 'horizontalSpiral', e.target.checked)}
                              disabled={isOperating}
                              className="sr-only"
                            />
                            <span className="text-sm font-medium">↔ Horizontal</span>
                          </label>
                        </div>
                      </div>
                    )}
                  </div>
                ))}
              </div>
            </div>
          )}

          <div className="mt-5 border-2 border-slate-200 pt-4 bg-white rounded-xl px-4 pb-4 shadow-sm">
            <div className={`grid gap-3 ${tableName === 'A' ? 'grid-cols-2' : 'grid-cols-2'}`}>
              {tableName === 'A' ? (
                <>
                  <Button 
                    onClick={handleStartScan} 
                    disabled={isOperating || isScanning}
                    className={`scan-button ${isScanning ? 'bg-green-600 hover:bg-green-700' : 'bg-green-500 hover:bg-purple-600'} text-white disabled:bg-green-600 disabled:text-white disabled:opacity-100 disabled:brightness-95 disabled:cursor-not-allowed`}
                  >
                    {isScanning ? 'Scanning...' : 'Scan'}
                  </Button>
                  <Button 
                    onClick={handleStartTask} 
                    disabled={isOperating}
                    className="bg-blue-500 hover:bg-purple-600 text-white disabled:opacity-100 disabled:brightness-95 disabled:cursor-not-allowed"
                  >
                    {isOperating ? 'Operating...' : 'Start Task'}
                  </Button>
                </>
              ) : (
                <>
                  <Button 
                    onClick={handleUpload3DFile} 
                    disabled={isOperating}
                    className="bg-pink-500 hover:bg-pink-600 text-white w-full disabled:opacity-100 disabled:brightness-95 disabled:cursor-not-allowed"
                  >
                    {isOperating ? 'Operating...' : 'Upload 3D File'}
                  </Button>
                  <Button 
                    onClick={handleStartTask} 
                    disabled={isOperating}
                    className="bg-blue-500 hover:bg-purple-600 text-white w-full disabled:opacity-100 disabled:brightness-95 disabled:cursor-not-allowed"
                  >
                    {isOperating ? 'Operating...' : 'Start Task'}
                  </Button>
                </>
              )}
            </div>
          </div>
        </div>
      </CardContent>
      </Card>
    </>
  );
}
