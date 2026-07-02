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

const MODEL_IMAGE_MAP: Record<'A' | 'B', Record<string, string[]>> = {
  A: {
    modelA: ['table_1/model1.jpg'],
    modelD: ['table_1/model4.jpg'],
    modelE: ['table_1/model5.jpeg', 'table_1/model_5.jpg', 'table_1/model5_a.jpg', 'table_1/model5_c.jpeg'],
    modelF: ['table_1/modelf.jpg', 'table_1/modelF.jpg', 'table_1/modelf.jpeg'],
  },
  B: {
    modelA: ['table_2/model1.jpeg'],
    modelB: ['table_2/model2.jpeg'],
    modelC: ['table_2/model3.jpeg'],
    modelD: ['table_2/model4.jpg'],
    modelE: ['table_2/model5.jpeg'],
  },
};

const MODEL_KEY_ALIAS: Record<string, string> = {
  modela: 'modelA',
  modelb: 'modelB',
  modelc: 'modelC',
  modeld: 'modelD',
  modele: 'modelE',
  modelf: 'modelF',
};

const PREVIEW_CACHE_BUST = 'v=20260514_modelfix';

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
  homingRequired?: boolean;
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
  homingRequired = false,
  doorConfigs,
  setDoorConfigs,
}: CompactTableConfigProps) {
  console.log('CompactTableConfig rendering:', tableName, 'rows:', rows.length, 'addActivity:', !!addActivity);
  const POCKET_MAX_OVERLAP_MM = 100;

  const [selectedDoor, setSelectedDoor] = React.useState<number>(1);
  const [scanCompleted, setScanCompleted] = React.useState<boolean>(false);
  const [lastScanSignature, setLastScanSignature] = React.useState<string | null>(null);
  const [lastScannedAt, setLastScannedAt] = React.useState<string | null>(null);
  const [detectedDoorNumbers, setDetectedDoorNumbers] = React.useState<number[] | null>(null);
  const [isScanning, setIsScanning] = React.useState<boolean>(false);
  const [completionPopup, setCompletionPopup] = React.useState<{ title: string; subtitle?: string } | null>(null);
  const [previewAttemptIndexA, setPreviewAttemptIndexA] = React.useState<number>(0);
  const [previewAttemptIndexB, setPreviewAttemptIndexB] = React.useState<number>(0);
  const [tableAFrameSizeX, setTableAFrameSizeX] = React.useState<string>('57');
  const [tableAFrameSizeY, setTableAFrameSizeY] = React.useState<string>('57');
  const [tableAFrameSizeConfirmed, setTableAFrameSizeConfirmed] = React.useState<boolean>(false);
  const completionTimerRef = React.useRef<number | null>(null);
  const isModelF = model === 'modelF';
  const parsePositiveFrameSize = (value: string) => {
    const parsed = Number(value);
    return Number.isFinite(parsed) && parsed > 0 && parsed <= 200 ? parsed : null;
  };
  const getTableAFrameSizeValues = () => ({
    x: parsePositiveFrameSize(tableAFrameSizeX),
    y: parsePositiveFrameSize(tableAFrameSizeY),
  });
  const getTableAFrameSizeSignature = () => {
    const values = getTableAFrameSizeValues();
    return values.x !== null && values.y !== null
      ? `frameX=${values.x};frameY=${values.y}`
      : 'frameX=invalid;frameY=invalid';
  };
  const isModelFAllowedRow = (label: string) => label === 'Pocket ZigZag';
  const isTableAOperationAllowed = (selectedModel: string, operationLabel: string) => {
    if (selectedModel === 'modelA' || selectedModel === 'modelB') {
      return operationLabel !== '3D';
    }
    if (selectedModel === 'modelC' || selectedModel === 'modelE') {
      return operationLabel !== 'Edge Outside';
    }
    return true;
  };
  const rowDisplayLabel = (label: string) =>
    isModelF && label === 'Pocket ZigZag' ? 'Flat ZigZag' : label;

  const [rowDoorSelections, setRowDoorSelections] = React.useState<Record<string, number[]>>({
    Frame: [],
    'Pocket ZigZag': [],
    '3D': [],
    'Edge Outside': [],
    Side: [],
  });
  const [rowActiveDoor, setRowActiveDoor] = React.useState<Record<string, number>>({
    Frame: 1,
    'Pocket ZigZag': 1,
    '3D': 1,
    'Edge Outside': 1,
    Side: 1,
  });

  const formatModelName = (value: string) => {
    if (tableName === 'A') {
      if (value === 'modelA' || value === 'modelB') return 'Model A - Shaker';
      if (value === 'modelC') return 'Model B - Moulure Externe';
      if (value === 'modelD') return 'Model C - Moulure Interne';
      if (value === 'modelE') return 'Model D - Moulure Interne et Externe';
      if (value === 'modelF') return 'Model E - Flat';
    }
    if (value === 'modelA') return 'Model A';
    if (value === 'modelB') return 'Model B';
    if (value === 'modelC') return 'Model C';
    if (value === 'modelD') return 'Model D';
    if (value === 'modelE') return 'Model E';
    if (value === 'modelF') return 'Model F';
    return value || 'No model selected';
  };

  const getCanonicalModelKey = (selectedModel: string) => {
    const raw = (selectedModel || '').trim();
    if (!raw) return '';
    const normalized = raw.toLowerCase().replace(/[^a-z0-9]/g, '');
    let canonicalKey = MODEL_KEY_ALIAS[normalized] || raw;
    if (typeof canonicalKey === 'string') {
      const probe = canonicalKey.toLowerCase().replace(/[^a-z0-9]/g, '');
      if (probe.includes('modela')) canonicalKey = 'modelA';
      else if (probe.includes('modelb')) canonicalKey = 'modelB';
      else if (probe.includes('modelc')) canonicalKey = 'modelC';
      else if (probe.includes('modeld')) canonicalKey = 'modelD';
      else if (probe.includes('modele')) canonicalKey = 'modelE';
      else if (probe.includes('modelf')) canonicalKey = 'modelF';
    }
    return canonicalKey;
  };

  const getModelPreviewCandidates = (table: 'A' | 'B', selectedModel: string) => {
    const raw = (selectedModel || '').trim();
    if (!raw) return [];
    const canonicalKey = getCanonicalModelKey(raw);
    const candidates = MODEL_IMAGE_MAP[table][canonicalKey] || [];
    if (candidates.length) return candidates;

    const probe = raw.toLowerCase().replace(/[^a-z0-9]/g, '');
    const looksLikeModelF = probe.includes('modelf') || probe.includes('flat');
    if (table === 'A' && looksLikeModelF) {
      return ['table_1/modelf.jpg'];
    }
    return [];
  };

  const getModelPreviewSrc = (table: 'A' | 'B', selectedModel: string, attemptIndex: number) => {
    const candidates = getModelPreviewCandidates(table, selectedModel);
    const relativePath = candidates[attemptIndex] || '';
    if (!relativePath) return '';
    return `/${relativePath}?${PREVIEW_CACHE_BUST}`;
  };

  const getModelPreviewDisplaySrcs = (
    table: 'A' | 'B',
    selectedModel: string,
    attemptIndex: number
  ) => {
    if (table === 'A' && getCanonicalModelKey(selectedModel) === 'modelD') {
      return ['table_1/model5_c.jpeg', 'table_1/model4.jpg', 'table_1/model5_a.jpg'].map(
        (relativePath) => `/${relativePath}?${PREVIEW_CACHE_BUST}`
      );
    }
    const src = getModelPreviewSrc(table, selectedModel, attemptIndex);
    return src ? [src] : [];
  };

  const selectedDoorModel =
    tableName === 'A'
      ? (doorConfigs || []).find((d) => d.doorNumber === selectedDoor)?.model || ''
      : '';
  const tableAPreviewModel =
    (selectedDoorModel || '').trim() ||
    (model || '').trim() ||
    (tableName === 'A' ? (doorConfigs || []).find((d) => (d.model || '').trim())?.model || '' : '');
  const tableBPreviewModel = (model || '').trim();

  React.useEffect(() => {
    setPreviewAttemptIndexA(0);
  }, [tableAPreviewModel]);

  React.useEffect(() => {
    setPreviewAttemptIndexB(0);
  }, [tableBPreviewModel]);

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
    if (tableName !== 'A') return [] as { doorNumber: number; model: string }[];
    return (doorConfigs || [])
      .map((d) => ({ doorNumber: d.doorNumber, model: (d.model || '').trim() }))
      .filter((d) => !!d.model);
  };

  const getTableAScanSignature = () => {
    if (tableName !== 'A') return '';
    const baseModel = (model || '').trim();
    const doorModelSig = (doorConfigs || [])
      .map((d) => `${d.doorNumber}:${(d.model || '').trim()}`)
      .join('|');
    return `${baseModel}::${doorModelSig}::${getTableAFrameSizeSignature()}`;
  };

  const formatScanSignatureSummary = (signature: string) => {
    const [baseModel = '', doorSig = '', frameSig = ''] = String(signature || '').split('::');
    const firstDoorModel = doorSig
      .split('|')
      .map((part) => part.split(':')[1])
      .find(Boolean);
    const modelText = baseModel || firstDoorModel ? formatModelName(baseModel || firstDoorModel || '') : 'No model recorded';
    const frameText = frameSig
      ? ` ${frameSig.replace('frameX=', 'X frame ').replace(';frameY=', ' mm, Y frame ')} mm`
      : '';
    return `${modelText}${frameText}`;
  };

  const buildScanMismatchWarning = (scanSignature: string, _currentSignature: string) =>
    `Saved scan was made for ${formatScanSignatureSummary(scanSignature)}. ` +
    'Re-scan or select the scanned setup before starting.';

  const applyDetectedDoorsFromScanStatus = (status: any) => {
    if (!status?.hasScan || !status?.doorDetectionAvailable) {
      setDetectedDoorNumbers(null);
      return;
    }

    const detected = Array.from<number>(
      new Set<number>(
        (Array.isArray(status.detectedDoorNumbers) ? status.detectedDoorNumbers : [])
          .map((value: unknown) => Number(value))
          .filter((value: number): value is number => Number.isInteger(value) && value >= 1 && value <= 4)
      )
    ).sort((a, b) => a - b);

    setDetectedDoorNumbers(detected);
    setRowDoorSelections((prev: Record<string, number[]>) =>
      Object.fromEntries(
        Object.entries(prev).map(([label, doors]: [string, number[]]) => [
          label,
          doors.filter((doorNumber: number) => detected.includes(doorNumber)),
        ])
      ) as Record<string, number[]>
    );
    setRowActiveDoor((prev: Record<string, number>) => {
      const fallbackDoor = detected[0] ?? 1;
      return Object.fromEntries(
        Object.entries(prev).map(([label, doorNumber]: [string, number]) => [
          label,
          detected.includes(doorNumber) ? doorNumber : fallbackDoor,
        ])
      ) as Record<string, number>;
    });
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
        applyDetectedDoorsFromScanStatus(status);
      } catch {
        // Non-blocking; fallback to in-memory scan status.
      }
    })();

    return () => {
      cancelled = true;
    };
  }, [tableName]);

  const confirmTableAFrameSizes = async () => {
    if (tableName !== 'A') return true;
    const frameSizes = getTableAFrameSizeValues();
    const warning = 'Enter valid Table A frame sizes for X and Y between 1 mm and 200 mm.';
    if (frameSizes.x === null || frameSizes.y === null) {
      addActivity(`Table ${tableName}: ${warning}`, 'warning');
      const swal = getSwal();
      if (swal?.fire) {
        await swal.fire({
          title: 'Frame Size Required',
          text: warning,
          icon: 'warning',
          confirmButtonText: 'OK',
        });
      }
      return false;
    }

    const text = `Confirm Table A frame sizes: X = ${frameSizes.x} mm, Y = ${frameSizes.y} mm. These values will be used to calculate scan frame and inner corner points.`;
    const swal = getSwal();
    const confirmed = swal?.fire
      ? !!(await swal.fire({
          title: 'Confirm Frame Sizes',
          text,
          icon: 'warning',
          showCancelButton: true,
          confirmButtonText: 'Confirm Values',
          cancelButtonText: 'Edit Values',
          reverseButtons: true,
        })).isConfirmed
      : window.confirm(text);
    if (confirmed) {
      setTableAFrameSizeConfirmed(true);
      addActivity(`Table ${tableName}: Frame sizes confirmed X=${frameSizes.x} mm, Y=${frameSizes.y} mm`, 'success');
    }
    return confirmed;
  };

  const confirmScanForTableA = async () => {
    if (tableName !== 'A') return true;
    const frameSizes = getTableAFrameSizeValues();
    if (frameSizes.x === null || frameSizes.y === null) {
      return confirmTableAFrameSizes();
    }
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
    const frameSizeText = `Frame sizes: X = ${frameSizes.x} mm, Y = ${frameSizes.y} mm.`;
    const rescanText = scanCompleted
      ? `A previous scan exists${lastScannedAt ? ` (${lastScannedAt})` : ''}. Do you want to run scan again?`
      : 'No previous scan found. Start a new scan?';
    const text = scanCompleted
      ? `${rescanText} ${selectedModelText} ${frameSizeText}`
      : `Confirm scan for Table A. ${selectedModelText} ${frameSizeText} Ensure the area is clear and setup is correct.`;
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
    if (tableName === 'A' && homingRequired) {
      const warning = 'Homing is required before scan. Please run Homing to calibrate the 7th axis.';
      addActivity(`Table ${tableName}: ${warning}`, 'warning');
      const swal = getSwal();
      if (swal?.fire) {
        await swal.fire({
          title: 'Homing Required',
          text: warning,
          icon: 'warning',
          timer: 2200,
          showConfirmButton: false,
        });
      }
      return;
    }
    const confirmed = await confirmScanForTableA();
    if (!confirmed) {
      addActivity(`Table ${tableName}: Scan cancelled by user for safety check`, 'warning');
      return;
    }
    setIsOperating(true);
    setIsScanning(true);
    addActivity(`Table ${tableName}: Scan in progress...`, 'warning');
    try {
      const frameSizes = getTableAFrameSizeValues();
      const scanResponse = await performAction('scan', {
        tableAScanSignature: getTableAScanSignature(),
        tableAModel: model || '',
        tableADoorModels: getTableADoorModels(),
        tableAFrameSize: {
          x: frameSizes.x,
          y: frameSizes.y,
        },
      });
      if (scanResponse?.status === 'cancelled') {
        addActivity(`Table ${tableName}: Scan stopped by user`, 'warning');
        return;
      }
      const status = scanResponse?.scanStatus;
      if (!status?.hasScan) {
        throw new Error('Scan action completed, but no scan result was returned.');
      }
      setScanCompleted(!!status?.hasScan);
      setLastScanSignature((status?.signature || getTableAScanSignature()).trim());
      setLastScannedAt(status?.scannedAt || new Date().toISOString());
      applyDetectedDoorsFromScanStatus(status);
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
    const confirmed = await confirmStartTask();
    if (!confirmed) {
      addActivity(`Table ${tableName}: Start task cancelled to review configuration`, 'warning');
      return;
    }

    if (tableName === 'A') {
      const scanSignatureNow = getTableAScanSignature();
      const scanInvalid =
        !scanCompleted ||
        (!!lastScanSignature && scanSignatureNow !== lastScanSignature);
      if (scanInvalid) {
        const swal = getSwal();
        const warningText = lastScanSignature
          ? buildScanMismatchWarning(lastScanSignature, scanSignatureNow)
          : 'No Table A scan is recorded. Run Scan for the selected model and detected doors before starting the task.';
        if (swal?.fire) {
          await swal.fire({
            title: 'Scan Required',
            text: warningText,
            icon: 'warning',
            confirmButtonText: 'OK',
          });
        }
        addActivity(`Table ${tableName}: ${warningText}`, 'warning');
        return;
      }
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
          Object.entries(rowDoorSelections).map(([label, doors]) => [
            label,
            [...new Set(doors)]
              .filter(doorNumber => {
                const selectedModel = doorConfigs.find(dc => dc.doorNumber === doorNumber)?.model || model;
                return !selectedModel || isTableAOperationAllowed(selectedModel, label);
              })
              .sort(),
          ])
        ) as Record<string, number[]>;

        const normalizedDoorConfigs = doorConfigs.map(dc => ({
          ...dc,
          model: normalizeTableAModelKey(dc.model || model),
          rows: dc.rows.map(r => ({ ...r })),
        }));

        // A row selected for multiple doors is one shared operation. Apply the
        // active door's values to every selected door before serializing it.
        for (const [label, selectedDoors] of Object.entries(selectedDoorsByRow)) {
          if (!selectedDoors.length) continue;
          const rowIndex = rows.findIndex(r => r.label === label);
          if (rowIndex < 0) continue;

          const rowsForSelectedDoors = selectedDoors
            .map(doorNumber => normalizedDoorConfigs.find(dc => dc.doorNumber === doorNumber)?.rows[rowIndex])
            .filter((row): row is RowConfig => !!row);

          const activeDoorNumber = rowActiveDoor[label];
          const activeDoorRow = selectedDoors.includes(activeDoorNumber)
            ? normalizedDoorConfigs.find(dc => dc.doorNumber === activeDoorNumber)?.rows[rowIndex]
            : undefined;
          const templateRow =
            activeDoorRow ||
            rowsForSelectedDoors.find(r => r.force > 0 && r.cycle > 0) ||
            rowsForSelectedDoors[0];

          if (!templateRow) continue;

          normalizedDoorConfigs.forEach(dc => {
            if (!selectedDoors.includes(dc.doorNumber)) return;
            const currentRow = dc.rows[rowIndex];
            if (!currentRow) return;
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
            if (!isTableAOperationAllowed(dc.model || model, r.label)) {
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
          tableAFrameSize: getTableAFrameSizeValues(),
        };

        // Send all door configurations to the backend
        const result = await startTableAProcess(taskData);

        if (!result?.success) {
          addActivity(`Table ${tableName}: Task failed to start (${result?.status || 'unknown'})`, 'error');
          return;
        }

        const timings = result?.timings && typeof result.timings === 'object'
          ? result.timings as Record<string, unknown>
          : null;
        if (timings) {
          const numericTimings = Object.entries(timings)
            .filter(([, value]) => typeof value === 'number' && Number.isFinite(value))
            .map(([key, value]) => [key, value as number] as const)
            .sort((a, b) => b[1] - a[1]);
          const routeTotal = typeof timings.routeTotalSeconds === 'number'
            ? timings.routeTotalSeconds
            : undefined;
          const slowest = numericTimings.find(([key]) => key !== 'routeTotalSeconds');
          if (routeTotal !== undefined && slowest) {
            const level = routeTotal > 2 ? 'warning' : 'info';
            addActivity(
              `Table ${tableName}: Start route ${routeTotal.toFixed(2)}s, slowest ${slowest[0]}=${slowest[1].toFixed(2)}s`,
              level
            );
          }
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
  const resolveActiveDoorForRow = (
    rowLabel: string,
    preferredDoor: number
  ): number => {
    if (tableName !== 'A' || !doorConfigs) return preferredDoor;
    const selectedForRow = rowDoorSelections[rowLabel] || [];
    if (!selectedForRow.length) return preferredDoor;
    if (selectedForRow.includes(preferredDoor)) return preferredDoor;

    const rowIndex = rows.findIndex(r => r.label === rowLabel);
    if (rowIndex >= 0) {
      const withValues = selectedForRow.find((doorNumber) => {
        const row = doorConfigs.find(dc => dc.doorNumber === doorNumber)?.rows?.[rowIndex];
        return !!row && (
          (row.force ?? 0) > 0 ||
          (row.cycle ?? 0) > 0 ||
          !!row.verticalSpiral ||
          !!row.horizontalSpiral ||
          !!row.edgeCoverage
        );
      });
      if (withValues !== undefined) return withValues;
    }
    return selectedForRow[0];
  };

  const getRowForTableA = (idx: number, rowLabel: string): RowConfig => {
    if (tableName !== 'A' || !doorConfigs) return rows[idx];
    const activeDoor = rowActiveDoor[rowLabel] ?? selectedDoor;
    const activeDoorConfig = doorConfigs.find(d => d.doorNumber === activeDoor);
    return activeDoorConfig?.rows?.[idx] || rows[idx];
  };
  const currentRows = tableName === 'A' && doorConfigs
    ? rows.map((r, idx) => getRowForTableA(idx, r.label))
    : rows;
  const buildDisplayRows = (sourceRows: RowConfig[]) =>
    sourceRows
      .map((row, idx) => ({ row, idx }))
      .filter(({ row }) => {
        if (isModelF && !isModelFAllowedRow(row.label)) return false;
        if (tableName !== 'A') return true;

        const configuredModels = (doorConfigs || [])
          .map(door => door.model)
          .filter(selectedModel => !!selectedModel);
        const candidateModels = configuredModels.length ? configuredModels : [model];
        return candidateModels.some(selectedModel =>
          !selectedModel || isTableAOperationAllowed(selectedModel, row.label)
        );
      });
  const currentDisplayRows = buildDisplayRows(currentRows);
  const tableBDisplayRows = buildDisplayRows(rows);
  const scanConfigMismatch =
    tableName === 'A' &&
    scanCompleted &&
    !!lastScanSignature &&
    getTableAScanSignature() !== lastScanSignature;

  const normalizeTableAModelKey = (value: string) =>
    tableName === 'A' && value === 'modelB' ? 'modelA' : value;
  const handleModelChange = (newModel: string) => {
    const normalizedModel = normalizeTableAModelKey(newModel);
    if (tableName === 'A' && normalizedModel !== model && scanCompleted) {
      addActivity(
        `Table ${tableName}: Model changed. Saved scan remains on record and will be validated before the task starts.`,
        'warning'
      );
    }
    setModel(normalizedModel);

    if (tableName === 'A' && doorConfigs && setDoorConfigs) {
      setDoorConfigs(prev => prev.map(cfg => ({ ...cfg, model: normalizedModel })));
    }
  };

  const handleRowChange = (idx: number, field: 'selection' | 'force' | 'cycle', value: any) => {
    if (tableName === 'A' && doorConfigs && setDoorConfigs) {
      const rowLabel = rows[idx]?.label;
      if (!rowLabel) return;
      const targetDoor = rowActiveDoor[rowLabel] ?? selectedDoor;
      const selectedForRow = rowDoorSelections[rowLabel] || [];
      const targetDoors = selectedForRow.length ? selectedForRow : [targetDoor];

      setDoorConfigs(prev =>
        prev.map(dc => {
          if (!targetDoors.includes(dc.doorNumber)) return dc;
          const newRows = [...dc.rows];
          const updatedRow = { ...newRows[idx], [field]: value };
          if (rowLabel === 'Pocket ZigZag' && field === 'cycle' && Number(value) > 1) {
            updatedRow.verticalSpiral = true;
            updatedRow.horizontalSpiral = true;
          }
          newRows[idx] = updatedRow;
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
    if (detectedDoorNumbers !== null && !detectedDoorNumbers.includes(doorNumber)) {
      return;
    }
    const selectedDoorModel = doorConfigs?.find(d => d.doorNumber === doorNumber)?.model || model;
    if (tableName === 'A' && selectedDoorModel && !isTableAOperationAllowed(selectedDoorModel, label)) {
      return;
    }
    const rowIndex = rows.findIndex(r => r.label === label);
    const sourceDoorNumber = rowActiveDoor[label] ?? selectedDoor;
    const currentSelection = rowDoorSelections[label] || [];
    const wasSelected = currentSelection.includes(doorNumber);

    if (wasSelected) {
      // Allow door deselection without page refresh.
      const nextSelection = currentSelection.filter(d => d !== doorNumber);
      setRowDoorSelections(prev => ({ ...prev, [label]: nextSelection }));
      // Keep active door unchanged while toggling membership to avoid visual flips.
      setRowActiveDoor(prev => ({ ...prev }));
      return;
    }

    setRowDoorSelections(prev => {
      const current = prev[label] || [];
      const exists = current.includes(doorNumber);
      const next = exists ? current : [...current, doorNumber].sort();
      return { ...prev, [label]: next };
    });
    // Only initialize active door when row had no selected doors before.
    if (currentSelection.length === 0) {
      setSelectedDoor(doorNumber);
      setRowActiveDoor(prev => ({ ...prev, [label]: doorNumber }));
    }

    if (!wasSelected && tableName === 'A' && doorConfigs && setDoorConfigs && rowIndex >= 0) {
      setDoorConfigs(prev =>
        prev.map(dc => {
          if (dc.doorNumber !== doorNumber) return dc;
          const newRows = [...dc.rows];
          const targetRow = newRows[rowIndex];
          if (!targetRow) return dc;
          const sourceDoor = prev.find(d => d.doorNumber === sourceDoorNumber);
          const sourceRow = sourceDoor?.rows[rowIndex];
          if (!sourceRow) return dc;
          // A multi-door row is one shared operation. When a door is added,
          // copy the visible row intensity so it does not silently keep 5/1.
          newRows[rowIndex] = {
            ...targetRow,
            force: sourceRow.force,
            cycle: sourceRow.cycle,
          };
          return { ...dc, rows: newRows };
        })
      );
    }

  };

  const toggleAllScannedDoorsForRow = (label: string) => {
    if (
      tableName !== 'A' ||
      detectedDoorNumbers?.length !== 4 ||
      !doorConfigs ||
      !setDoorConfigs
    ) {
      return;
    }

    const rowIndex = rows.findIndex(r => r.label === label);
    if (rowIndex < 0) return;

    const compatibleDoors = detectedDoorNumbers.filter(doorNumber => {
      const selectedModel =
        doorConfigs.find(d => d.doorNumber === doorNumber)?.model || model;
      return !selectedModel || isTableAOperationAllowed(selectedModel, label);
    });
    const currentSelection = rowDoorSelections[label] || [];
    const allCompatibleSelected =
      compatibleDoors.length > 0 &&
      compatibleDoors.every(doorNumber => currentSelection.includes(doorNumber));
    const nextSelection = allCompatibleSelected ? [] : compatibleDoors;

    setRowDoorSelections(prev => ({ ...prev, [label]: nextSelection }));
    if (nextSelection.length > 0) {
      setSelectedDoor(nextSelection[0]);
      setRowActiveDoor(prev => ({ ...prev, [label]: nextSelection[0] }));
    }

    if (!allCompatibleSelected) {
      const sourceDoorNumber = rowActiveDoor[label] ?? selectedDoor;
      setDoorConfigs(prev => {
        const sourceDoor = prev.find(dc => dc.doorNumber === sourceDoorNumber);
        const sourceRow = sourceDoor?.rows[rowIndex];
        const sourceForce = sourceRow && sourceRow.force > 0 ? sourceRow.force : 5;
        const sourceCycle = sourceRow && sourceRow.cycle > 0 ? sourceRow.cycle : 1;

        return prev.map(dc => {
          if (!compatibleDoors.includes(dc.doorNumber)) return dc;
          const newRows = [...dc.rows];
          const currentRow = newRows[rowIndex];
          if (!currentRow) return dc;
          newRows[rowIndex] = {
            ...currentRow,
            force: sourceForce,
            cycle: sourceCycle,
          };
          return { ...dc, rows: newRows };
        });
      });
    }
  };

  const handlePocketZigZagOptionChange = (idx: number, option: 'verticalSpiral' | 'horizontalSpiral' | 'edgeCoverage', checked: boolean) => {
    if (tableName === 'A' && doorConfigs && setDoorConfigs) {
      const rowLabel = rows[idx]?.label;
      if (!rowLabel) return;
      const selectedForRow = rowDoorSelections[rowLabel] || [];
      if (!selectedForRow.length) {
        return;
      }
      const targetDoor = rowActiveDoor[rowLabel] ?? selectedDoor;
      if (!selectedForRow.includes(targetDoor)) {
        // If the active door is currently deselected for this row, do not
        // implicitly retarget and mutate another door's pattern.
        return;
      }

      setDoorConfigs(prev =>
        prev.map(dc => {
          if (!selectedForRow.includes(dc.doorNumber)) return dc;

          const newRows = [...dc.rows];
          const currentRow = newRows[idx];
          if (!currentRow) return dc;

          const sourceDoor = prev.find(d => d.doorNumber === targetDoor);
          const sourceRow = sourceDoor?.rows[idx];

          const nextRow = { ...currentRow };
          const rowHasNoIntensity = (nextRow.force ?? 0) <= 0 && (nextRow.cycle ?? 0) <= 0;
          const sourceHasIntensity = !!sourceRow && ((sourceRow.force ?? 0) > 0 || (sourceRow.cycle ?? 0) > 0);

          // Preserve visible force/cycle continuity when active door was deselected
          // and the fallback selected door has empty values.
          if (rowHasNoIntensity && sourceHasIntensity) {
            nextRow.force = sourceRow!.force;
            nextRow.cycle = sourceRow!.cycle;
          }

          (nextRow as any)[option] = checked;

          newRows[idx] = nextRow;
          return { ...dc, rows: newRows };
        })
      );
    } else {
      setRows((prev: RowConfig[]) => {
        const next = [...prev];
        const nextRow = { ...next[idx], [option]: checked };
        if (tableName !== 'A') {
          if (option === 'verticalSpiral' && checked) {
            nextRow.horizontalSpiral = false;
          }
          if (option === 'horizontalSpiral' && checked) {
            nextRow.verticalSpiral = false;
          }
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
      <Card className="gap-0 shadow-xl border border-slate-300 bg-white/95 backdrop-blur-sm">
      <CardHeader className="bg-gradient-to-r from-indigo-50 to-cyan-50 px-4 py-3">
        <CardTitle className="flex items-center justify-between">
          Table {tableName} Configuration
          <Badge variant={isActive ? 'default' : 'secondary'} className={isActive ? 'bg-green-500' : ''}>
            {isActive ? 'Active' : 'Inactive'}
          </Badge>
        </CardTitle>
      </CardHeader>
      <CardContent className="px-2 pt-1 pb-2 bg-white rounded-b-lg">
        <div>
          {tableName === 'A' && doorConfigs ? (
            <>
              {/* Door Selection Tabs */}
              <div className="bg-white rounded-md p-2 border border-gray-200 mb-2">
                <select
                  value={model}
                  onChange={(e) => handleModelChange(e.target.value)}
                  disabled={isOperating}
                  className="w-full px-3 py-2 border border-gray-300 rounded-md text-sm bg-white focus:ring-2 focus:ring-blue-500 focus:border-transparent disabled:bg-gray-100 disabled:cursor-not-allowed"
                >
                  <option value="">Select a Model</option>
                  <option value="modelA">Model A - Shaker</option>
                  <option value="modelC">Model B - Moulure Externe</option>
                  <option value="modelD">Model C - Moulure Interne</option>
                  <option value="modelE">Model D - Moulure Interne et Externe</option>
                  <option value="modelF">Model E - Flat</option>
                </select>
                {getModelPreviewDisplaySrcs('A', tableAPreviewModel, previewAttemptIndexA).length > 0 && (
                  <div className="mt-3 rounded-md border border-slate-200 bg-slate-50 p-2">
                    <div className="flex flex-wrap items-center justify-center gap-2">
                      {getModelPreviewDisplaySrcs('A', tableAPreviewModel, previewAttemptIndexA).map((src, index) => (
                        <img
                          key={`${src}-${index}`}
                          src={src}
                          alt={`Table A ${formatModelName(tableAPreviewModel)} preview ${index + 1}`}
                          onError={(event) => {
                            if (getCanonicalModelKey(tableAPreviewModel) === 'modelD') {
                              event.currentTarget.style.display = 'none';
                              return;
                            }
                            const maxIdx = getModelPreviewCandidates('A', tableAPreviewModel).length - 1;
                            setPreviewAttemptIndexA((prev) => (prev < maxIdx ? prev + 1 : prev));
                          }}
                          style={{ width: '11.25rem', height: '7.5rem' }}
                          className="object-contain rounded-md bg-white border border-slate-200"
                        />
                      ))}
                    </div>
                  </div>
                )}
                {tableAPreviewModel && getCanonicalModelKey(tableAPreviewModel) !== 'modelF' && (
                  <div className="mt-3 rounded-md border border-red-200 bg-red-50/50 p-3">
                    <div className="mb-2 text-xs font-semibold text-red-700">Frame size used by scan</div>
                    <div className="flex flex-wrap items-center gap-3">
                      <label className="flex items-center gap-2 text-xs text-slate-700 whitespace-nowrap">
                        <span>X Frame Size (mm)</span>
                        <input
                          type="number"
                          min="1"
                          max="200"
                          step="0.1"
                          value={tableAFrameSizeX}
                          disabled={isOperating}
                          onChange={(e) => {
                            setTableAFrameSizeX(e.target.value);
                            setTableAFrameSizeConfirmed(false);
                          }}
                          className="w-24 rounded-md border border-red-200 bg-white px-2 py-1 text-sm focus:border-red-400 focus:outline-none focus:ring-1 focus:ring-red-300 disabled:bg-gray-100"
                        />
                      </label>
                      <label className="flex items-center gap-2 text-xs text-slate-700 whitespace-nowrap">
                        <span>Y Frame Size (mm)</span>
                        <input
                          type="number"
                          min="1"
                          max="200"
                          step="0.1"
                          value={tableAFrameSizeY}
                          disabled={isOperating}
                          onChange={(e) => {
                            setTableAFrameSizeY(e.target.value);
                            setTableAFrameSizeConfirmed(false);
                          }}
                          className="w-24 rounded-md border border-red-200 bg-white px-2 py-1 text-sm focus:border-red-400 focus:outline-none focus:ring-1 focus:ring-red-300 disabled:bg-gray-100"
                        />
                      </label>
                      <Button
                        type="button"
                        variant="outline"
                        disabled={isOperating}
                        onClick={confirmTableAFrameSizes}
                        className="h-8 border-red-300 px-4 text-red-700 hover:bg-red-100"
                      >
                        Confirm Values
                      </Button>
                    </div>
                    <div className={`mt-2 text-xs ${tableAFrameSizeConfirmed ? 'text-green-700' : 'text-amber-700'}`}>
                      {tableAFrameSizeConfirmed
                        ? `Confirmed: X=${tableAFrameSizeX} mm, Y=${tableAFrameSizeY} mm.`
                        : 'Confirm these values before scanning. They override laser frame-size classification for Table A scan geometry.'}
                    </div>
                  </div>
                )}
                <div className="mt-2 text-xs">
                  {scanCompleted && scanConfigMismatch ? (
                    <span className="text-amber-700">
                      Saved scan on record{lastScannedAt ? ` (${lastScannedAt})` : ''}, but it was made for {formatScanSignatureSummary(lastScanSignature || '')}. Re-scan or select the scanned setup before starting.
                    </span>
                  ) : scanCompleted ? (
                    <span className="text-green-700">
                      Scan on record{lastScannedAt ? ` (${lastScannedAt})` : ''} and available for the selected model/door setup.
                    </span>
                  ) : (
                    <span className="text-amber-700">No scan on record yet.</span>
                  )}
                </div>
              </div>

              <div className="mb-2">
                <div className="space-y-1">
                  {currentDisplayRows.map(({ row, idx }) => (
                    <div key={row.label} className={`bg-white rounded-md p-2 border ${row.label === 'Pocket ZigZag' ? 'border-indigo-300 shadow-sm' : 'border-gray-200'}`}>
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
                            {detectedDoorNumbers?.length === 4 && (() => {
                              const compatibleDoors = detectedDoorNumbers.filter(doorNumber => {
                                const selectedModel =
                                  doorConfigs.find(d => d.doorNumber === doorNumber)?.model || model;
                                return !selectedModel || isTableAOperationAllowed(selectedModel, row.label);
                              });
                              const selectedDoors = rowDoorSelections[row.label] || [];
                              const allSelected =
                                compatibleDoors.length > 0 &&
                                compatibleDoors.every(doorNumber => selectedDoors.includes(doorNumber));
                              return (
                                <button
                                  type="button"
                                  onClick={(e) => {
                                    e.preventDefault();
                                    toggleAllScannedDoorsForRow(row.label);
                                  }}
                                  disabled={isOperating || compatibleDoors.length === 0}
                                  className={`min-w-[94px] px-3 py-1 text-xs font-bold rounded-md border transition-colors disabled:cursor-not-allowed disabled:opacity-50 ${
                                    allSelected
                                      ? 'text-white bg-emerald-600 border-emerald-600 hover:bg-emerald-700'
                                      : 'text-emerald-800 bg-emerald-50 border-emerald-500 hover:bg-emerald-100'
                                  }`}
                                >
                                  {allSelected ? 'Clear All 4' : 'All 4 Doors'}
                                </button>
                              );
                            })()}
                            {[1, 2, 3, 4].map((doorNum) => {
                              const doorConfig = doorConfigs.find(d => d.doorNumber === doorNum);
                              const hasModel = doorConfig?.model && doorConfig.model !== '';
                              const isSelected = (rowDoorSelections[row.label] || []).includes(doorNum);
                              const isUndetected = detectedDoorNumbers !== null && !detectedDoorNumbers.includes(doorNum);
                              const isModelIncompatible =
                                !!doorConfig?.model &&
                                !isTableAOperationAllowed(doorConfig.model, row.label);
                              const isLocked = isUndetected || isModelIncompatible;
                              const lockReason = isUndetected
                                ? `Door ${doorNum} was not detected in the latest scan`
                                : isModelIncompatible
                                ? `${formatModelName(doorConfig?.model || '')} does not support ${row.label}`
                                : undefined;
                              return (
                                <button
                                  key={doorNum}
                                  type="button"
                                  onClick={(e) => {
                                    e.preventDefault();
                                    toggleRowDoor(row.label, doorNum);
                                  }}
                                  disabled={isOperating || isLocked}
                                  title={lockReason}
                                  className={`min-w-[78px] px-3 py-1 text-xs font-semibold text-center transition-colors relative disabled:cursor-not-allowed disabled:opacity-100 rounded-md border ${
                                    isLocked
                                      ? 'text-gray-400 bg-gray-200 border-gray-300 cursor-not-allowed'
                                      : isSelected
                                      ? 'text-white bg-blue-600 border-blue-600 hover:bg-blue-700'
                                      : 'text-gray-900 bg-white border-gray-500 hover:bg-gray-50'
                                  }`}
                                  style={{
                                    opacity: 1,
                                    color: isLocked ? '#9ca3af' : isSelected ? '#ffffff' : '#111827',
                                    backgroundColor: isLocked ? '#e5e7eb' : isSelected ? '#2563eb' : '#ffffff',
                                    borderColor: isLocked ? '#d1d5db' : isSelected ? '#2563eb' : '#6b7280',
                                    fontWeight: 600,
                                  }}
                                >
                                  {isLocked ? 'Locked ' : ''}Door {doorNum}
                                  {hasModel && !isLocked && (
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
                                {Array.from({ length: 50 }, (_, i) => i + 1).map((n) => (
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
                        <div className="mt-4 pt-6 border-t border-indigo-100">
                          <div className="flex items-center justify-center gap-3">
                            <span className="text-sm text-gray-500 font-medium">Pattern:</span>
                            <div className="flex items-center gap-3">
                              <label className={`flex items-center gap-2 px-4 py-2 rounded-lg border-2 transition-colors ${
                                row.cycle > 1 ? 'cursor-not-allowed opacity-75' : 'cursor-pointer'
                              } ${
                                row.verticalSpiral || row.cycle > 1
                                  ? 'bg-blue-500 border-blue-500 text-white'
                                  : 'bg-white border-gray-200 text-gray-700 hover:border-blue-400'
                              }`}>
                                <input
                                  type="checkbox"
                                  checked={row.verticalSpiral || row.cycle > 1}
                                  onChange={(e) => handlePocketZigZagOptionChange(idx, 'verticalSpiral', e.target.checked)}
                                  disabled={isOperating || row.cycle > 1}
                                  className="sr-only"
                                />
                                <span className="text-sm font-medium">↕ Vertical</span>
                              </label>
                              <label className={`flex items-center gap-2 px-4 py-2 rounded-lg border-2 transition-colors ${
                                row.cycle > 1 ? 'cursor-not-allowed opacity-75' : 'cursor-pointer'
                              } ${
                                row.horizontalSpiral || row.cycle > 1
                                  ? 'bg-blue-500 border-blue-500 text-white'
                                  : 'bg-white border-gray-200 text-gray-700 hover:border-blue-400'
                              }`}>
                                <input
                                  type="checkbox"
                                  checked={row.horizontalSpiral || row.cycle > 1}
                                  onChange={(e) => handlePocketZigZagOptionChange(idx, 'horizontalSpiral', e.target.checked)}
                                  disabled={isOperating || row.cycle > 1}
                                  className="sr-only"
                                />
                                <span className="text-sm font-medium">↔ Horizontal</span>
                              </label>
                            </div>
                          </div>
                          {row.cycle > 1 && (
                            <p className="mt-2 text-center text-xs font-medium text-indigo-700">
                              More than 1 cycle selected: every cycle will run both Vertical and Horizontal patterns.
                            </p>
                          )}
                        </div>
                      )}
                    </div>
                  ))}
                </div>
              </div>
            </>
          ) : (
            <div className="mb-2">
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
              {getModelPreviewSrc('B', tableBPreviewModel, previewAttemptIndexB) && (
                <div className="mt-3 rounded-md border border-slate-200 bg-slate-50 p-2">
                  <img
                    src={getModelPreviewSrc('B', tableBPreviewModel, previewAttemptIndexB)}
                    alt={`Table B ${formatModelName(tableBPreviewModel)}`}
                    onError={() => {
                      const maxIdx = getModelPreviewCandidates('B', tableBPreviewModel).length - 1;
                      setPreviewAttemptIndexB((prev) => (prev < maxIdx ? prev + 1 : prev));
                    }}
                    style={{ width: '11.25rem', height: '7.5rem' }}
                    className="mx-auto object-contain rounded-md bg-white border border-slate-200"
                  />
                </div>
              )}

              <div className="mt-2 space-y-1">
                {tableBDisplayRows.map(({ row, idx }) => (
                  <div key={row.label} className={`bg-white rounded-md p-2 border ${row.label === 'Pocket ZigZag' ? 'border-indigo-300 shadow-sm' : 'border-gray-200'}`}>
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
                            {Array.from({ length: 50 }, (_, i) => i + 1).map((n) => (
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
                    disabled={isOperating || isScanning || (tableName === 'A' && homingRequired)}
                    className={`scan-button ${isScanning ? 'bg-green-600 hover:bg-green-700' : 'bg-green-500 hover:bg-purple-600'} text-white disabled:bg-green-600 disabled:text-white disabled:opacity-100 disabled:brightness-95 disabled:cursor-not-allowed`}
                  >
                    {isScanning ? 'Scanning...' : (tableName === 'A' && homingRequired ? 'Home First' : 'Scan')}
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
            {tableName === 'A' && homingRequired && (
              <div className="mt-2 rounded-md border border-amber-300 bg-amber-50 px-3 py-2 text-xs text-amber-900">
                Homing required before scan after app/server restart.
              </div>
            )}
          </div>
        </div>
      </CardContent>
      </Card>
    </>
  );
}
