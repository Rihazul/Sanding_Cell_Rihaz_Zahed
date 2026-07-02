 import React, { useEffect, useRef, useState } from 'react';
import { DashboardHeader } from './dashboard/DashboardHeader';
import { RobotControlPanel } from './dashboard/RobotControlPanel';
import { RobotStatusCard } from './dashboard/RobotStatusCard';
import { SettingsPanel } from './dashboard/SettingsPanel';
import { SlidingPanel } from './dashboard/SlidingPanel';
import { CompactTableConfig, type RowConfig, type DoorConfig } from './dashboard/CompactTableConfig';
import { Button } from './ui/button';
import { Settings } from 'lucide-react';
import { checkToolStatus, getHomingStatus, getModalData, getRobotStatus, getStopperState, getTableState } from '../services/api';

interface DashboardProps {
  onNavigateToAnalytics: () => void;
  activities: Array<{id: number, timestamp: string, message: string, type: 'info' | 'success' | 'warning' | 'error'}>;
  addActivity: (message: string, type?: 'info' | 'success' | 'warning' | 'error') => void;
}

export function Dashboard({ onNavigateToAnalytics, activities, addActivity }: DashboardProps) {
  const TOOL_PENDING_MIN_PICK_MS = 2500;
  const TOOL_PENDING_MIN_DROP_MS = 6000;
  type ToolPending = { state: 'picking' | 'dropping'; since: number };

  const [robotEnabled, setRobotEnabled] = useState(false);
  const [robotSpeed, setRobotSpeed] = useState([100]);
  const [inverseOverlapping, setInverseOverlapping] = useState([0]);
  const [sandingSpeed, setSandingSpeed] = useState([75]);
  const [spiralSpeed, setSpiralSpeed] = useState([150]); // mm/s
  const [spiralRadius, setSpiralRadius] = useState([12]); // mm
  const [spiralLinearSpeed] = useState([150]); // mm/s
  const [laserOn, setLaserOn] = useState(false);
  const [isHoming, setIsHoming] = useState(false);
  const [homingRequired, setHomingRequired] = useState(false);
  const [isOperating, setIsOperating] = useState(false);
  
  // Toggle states
  const [stopperAUp, setStopperAUp] = useState(false);
  const [stopperBUp, setStopperBUp] = useState(false);
  const [tableAOpen, setTableAOpen] = useState(false);
  const [tableBOpen, setTableBOpen] = useState(false);
  const [tableAPending, setTableAPending] = useState<'opening' | 'closing' | null>(null);
  const [tableBPending, setTableBPending] = useState<'opening' | 'closing' | null>(null);
  const [t1Picked, setT1Picked] = useState(false);
  const [t2Picked, setT2Picked] = useState(false);
  const [t3Picked, setT3Picked] = useState(false);
  const [t4Picked, setT4Picked] = useState(false);
  const [t1Pending, setT1Pending] = useState<ToolPending | null>(null);
  const [t2Pending, setT2Pending] = useState<ToolPending | null>(null);
  const [t3Pending, setT3Pending] = useState<ToolPending | null>(null);
  const [t4Pending, setT4Pending] = useState<ToolPending | null>(null);
  const hardwarePollInFlightRef = useRef(false);

  // Initialize basic settings from backend (if available)
  useEffect(() => {
    let cancelled = false;

    (async () => {
      try {
        const data = await getModalData();
        const backendRobotSpeed = data?.tableA?.UI?.robotSpeed;
        if (!cancelled && typeof backendRobotSpeed === 'number' && Number.isFinite(backendRobotSpeed)) {
          // backend stores 0.90, UI uses percentage slider
          setRobotSpeed([Math.round(backendRobotSpeed * 100)]);
        }
      } catch {
        // Non-blocking: dashboard should still work without modal data
      }

      try {
        const homing = await getHomingStatus();
        if (!cancelled && typeof homing?.required === 'boolean') {
          setHomingRequired(homing.required);
        }
      } catch {
        // Keep hidden by default if homing status cannot be read.
      }
    })();

    return () => {
      cancelled = true;
    };
  }, []);

  useEffect(() => {
    let cancelled = false;

    const refreshHardwareStatus = async () => {
      if (hardwarePollInFlightRef.current) return;
      hardwarePollInFlightRef.current = true;

      const results = await Promise.allSettled([
        getRobotStatus(),
        getTableState('tableAOpenClose'),
        getTableState('tableBOpenClose'),
        getStopperState('A'),
        getStopperState('B'),
        checkToolStatus(1),
        checkToolStatus(2),
        checkToolStatus(3),
        checkToolStatus(4),
      ]);
      hardwarePollInFlightRef.current = false;

      if (cancelled) return;

      const [
        robotResult,
        tableAResult,
        tableBResult,
        stopperAResult,
        stopperBResult,
        tool1Result,
        tool2Result,
        tool3Result,
        tool4Result,
      ] = results;

      if (robotResult.status === 'fulfilled') {
        const flags = robotResult.value?.flags;
        if (flags && typeof flags.enabled === 'boolean') {
          setRobotEnabled(flags.enabled);
        }
      }

      if (tableAResult.status === 'fulfilled') {
        const state = tableAResult.value?.state;
        if (state === 'Open' || state === 'Close') {
          // Table A backend state: Close = physical 45 degrees, Open = horizontal.
          // tableAOpen is retained as the existing UI "45 degrees active" flag.
          setTableAOpen(state === 'Close');
          if (state === 'Close') {
            setTableAPending(prev => (prev === 'opening' ? null : prev));
          }
          if (state === 'Open') {
            setTableAPending(prev => (prev === 'closing' ? null : prev));
          }
        }
      }

      if (tableBResult.status === 'fulfilled') {
        const state = tableBResult.value?.state;
        if (state === 'Open' || state === 'Close') {
          // Table B backend state: Close = physical 45 degrees, Open = horizontal.
          // tableBOpen is retained as the existing UI "45 degrees active" flag.
          setTableBOpen(state === 'Close');
          if (state === 'Close') {
            setTableBPending(prev => (prev === 'opening' ? null : prev));
          }
          if (state === 'Open') {
            setTableBPending(prev => (prev === 'closing' ? null : prev));
          }
        }
      }

      if (stopperAResult.status === 'fulfilled') {
        const state = stopperAResult.value?.state;
        if (state === 'Up' || state === 'Down') {
          setStopperAUp(state === 'Up');
        }
      }

      if (stopperBResult.status === 'fulfilled') {
        const state = stopperBResult.value?.state;
        if (state === 'Up' || state === 'Down') {
          setStopperBUp(state === 'Up');
        }
      }

      if (tool1Result.status === 'fulfilled') {
        const status = tool1Result.value?.status;
        if (status === 'OK') {
          const picked = !!tool1Result.value?.shouldBlink;
          setT1Picked(picked);
          setT1Pending(prev => {
            if (!prev) return prev;
            const done = (prev.state === 'picking' && picked) || (prev.state === 'dropping' && !picked);
            const minMs = prev.state === 'dropping' ? TOOL_PENDING_MIN_DROP_MS : TOOL_PENDING_MIN_PICK_MS;
            if (done && Date.now() - prev.since >= minMs) return null;
            return prev;
          });
        }
      }
      if (tool2Result.status === 'fulfilled') {
        const status = tool2Result.value?.status;
        if (status === 'OK') {
          const picked = !!tool2Result.value?.shouldBlink;
          setT2Picked(picked);
          setT2Pending(prev => {
            if (!prev) return prev;
            const done = (prev.state === 'picking' && picked) || (prev.state === 'dropping' && !picked);
            const minMs = prev.state === 'dropping' ? TOOL_PENDING_MIN_DROP_MS : TOOL_PENDING_MIN_PICK_MS;
            if (done && Date.now() - prev.since >= minMs) return null;
            return prev;
          });
        }
      }
      if (tool3Result.status === 'fulfilled') {
        const status = tool3Result.value?.status;
        if (status === 'OK') {
          const picked = !!tool3Result.value?.shouldBlink;
          setT3Picked(picked);
          setT3Pending(prev => {
            if (!prev) return prev;
            const done = (prev.state === 'picking' && picked) || (prev.state === 'dropping' && !picked);
            const minMs = prev.state === 'dropping' ? TOOL_PENDING_MIN_DROP_MS : TOOL_PENDING_MIN_PICK_MS;
            if (done && Date.now() - prev.since >= minMs) return null;
            return prev;
          });
        }
      }
      if (tool4Result.status === 'fulfilled') {
        const status = tool4Result.value?.status;
        if (status === 'OK') {
          const picked = !!tool4Result.value?.shouldBlink;
          setT4Picked(picked);
          setT4Pending(prev => {
            if (!prev) return prev;
            const done = (prev.state === 'picking' && picked) || (prev.state === 'dropping' && !picked);
            const minMs = prev.state === 'dropping' ? TOOL_PENDING_MIN_DROP_MS : TOOL_PENDING_MIN_PICK_MS;
            if (done && Date.now() - prev.since >= minMs) return null;
            return prev;
          });
        }
      }
    };

    refreshHardwareStatus();
    const intervalId = window.setInterval(refreshHardwareStatus, 1000);

    return () => {
      cancelled = true;
      window.clearInterval(intervalId);
    };
  }, []);

  // Sliding panel view state
  const [currentPanelView, setCurrentPanelView] = useState<'robot-control' | 'table-a' | 'table-b'>('robot-control');
  
  // Table configuration states
  const [tableAModel, setTableAModel] = useState('');
  const [tableBModel, setTableBModel] = useState('');

  // Determine active table based on robot enabled state and current view
  const activeTable = robotEnabled && currentPanelView === 'table-a' ? 'A' 
    : robotEnabled && currentPanelView === 'table-b' ? 'B' 
    : null;

  const defaultRows: RowConfig[] = [
    { label: 'Frame', selection: '', force: 0, cycle: 0 },
    { label: 'Pocket ZigZag', selection: '', force: 0, cycle: 0, verticalSpiral: false, horizontalSpiral: false, edgeCoverage: false },
    { label: '3D', selection: '', force: 0, cycle: 0 },
    { label: 'Edge Outside', selection: '', force: 0, cycle: 0 },
    { label: 'Side', selection: '', force: 0, cycle: 0 },
  ];

  const makeRowSet = () => defaultRows.map(r => ({ ...r }));
  const makeTableARowSet = () => defaultRows.map(r => ({ ...r, force: 5, cycle: 1 }));

  const [tableARows, setTableARows] = useState<RowConfig[]>(makeTableARowSet());
  const [tableBRows, setTableBRows] = useState<RowConfig[]>(makeRowSet());

  // Door configurations for Table A (each door can have a different model)
  const [doorConfigs, setDoorConfigs] = useState<Array<{doorNumber: number, model: string, rows: RowConfig[]}>>([
    { doorNumber: 1, model: '', rows: makeTableARowSet() },
    { doorNumber: 2, model: '', rows: makeTableARowSet() },
    { doorNumber: 3, model: '', rows: makeTableARowSet() },
    { doorNumber: 4, model: '', rows: makeTableARowSet() },
  ]);

  // Check if Frame or Pocket ZigZag are configured (force AND cycle > 0) to enable Spiral Settings
  const isSpiralSettingsEnabled = () => {
    // Check Table A rows
    const tableAActive = tableARows.some(row => 
      (row.label === 'Frame' || row.label === 'Pocket ZigZag') && row.force > 0 && row.cycle > 0
    );
    // Check Table B rows
    const tableBActive = tableBRows.some(row => 
      (row.label === 'Frame' || row.label === 'Pocket ZigZag') && row.force > 0 && row.cycle > 0
    );
    // Check door configs
    const doorActive = doorConfigs.some(door => 
      door.rows.some(row => 
        (row.label === 'Frame' || row.label === 'Pocket ZigZag') && row.force > 0 && row.cycle > 0
      )
    );
    return tableAActive || tableBActive || doorActive;
  };

  return (
    <div className="min-h-screen bg-gradient-to-br from-cyan-50 via-blue-100 to-indigo-100 pb-24">
      <DashboardHeader 
        onNavigateToAnalytics={onNavigateToAnalytics}
      />

      <main className="max-w-7xl mx-auto px-4 sm:px-6 lg:px-8 pt-4 pb-8">
        {homingRequired && (
          <div className="mb-6 rounded-xl border-2 border-amber-300 bg-white px-5 py-4 shadow-sm">
            <div className="flex items-start gap-3">
              <div className="mt-0.5 text-xl">⚠️</div>
              <div>
                <div className="text-sm font-semibold text-amber-900">Homing Required Before Operation</div>
                <div className="text-sm text-amber-800 mt-1">
                  Run Homing first to calibrate the 7th axis before opening tables or starting tasks.
                </div>
              </div>
            </div>
          </div>
        )}

        <div className="grid grid-cols-1 lg:grid-cols-3 gap-6">
          <div className="lg:col-span-2 space-y-6">
            {/* Sliding Panel with Robot Control and Table Configurations */}
            <SlidingPanel currentView={currentPanelView} onViewChange={setCurrentPanelView}>
              {/* Robot Control Panel */}
              <RobotControlPanel
                robotEnabled={robotEnabled}
                setRobotEnabled={setRobotEnabled}
                stopperAUp={stopperAUp}
                setStopperAUp={setStopperAUp}
                stopperBUp={stopperBUp}
                setStopperBUp={setStopperBUp}
                tableAOpen={tableAOpen}
                setTableAOpen={setTableAOpen}
                tableBOpen={tableBOpen}
                setTableBOpen={setTableBOpen}
                tableAPending={tableAPending}
                setTableAPending={setTableAPending}
                tableBPending={tableBPending}
                setTableBPending={setTableBPending}
                t1Picked={t1Picked}
                setT1Picked={setT1Picked}
                t2Picked={t2Picked}
                setT2Picked={setT2Picked}
                t3Picked={t3Picked}
                setT3Picked={setT3Picked}
                t4Picked={t4Picked}
                setT4Picked={setT4Picked}
                t1Pending={t1Pending}
                setT1Pending={setT1Pending}
                t2Pending={t2Pending}
                setT2Pending={setT2Pending}
                t3Pending={t3Pending}
                setT3Pending={setT3Pending}
                t4Pending={t4Pending}
                setT4Pending={setT4Pending}
                laserOn={laserOn}
                setLaserOn={setLaserOn}
                isOperating={isOperating}
                addActivity={addActivity}
              />
              
              {/* Table A Configuration */}
              <CompactTableConfig
                tableName="A"
                model={tableAModel}
                setModel={setTableAModel}
                rows={tableARows}
                setRows={setTableARows}
                isActive={activeTable === 'A'}
                isOperating={isOperating}
                setIsOperating={setIsOperating}
                addActivity={addActivity}
                robotSpeed={robotSpeed}
                sandingSpeed={sandingSpeed}
                inverseOverlapping={inverseOverlapping}
                spiralSettings={{
                  enabled: isSpiralSettingsEnabled(),
                  speedPercent: spiralSpeed[0],
                  radiusMm: spiralRadius[0],
                  linearSpeedMmS: spiralLinearSpeed[0],
                }}
                homingRequired={homingRequired}
                doorConfigs={doorConfigs}
                setDoorConfigs={setDoorConfigs}
              />
              
              {/* Table B Configuration */}
              <CompactTableConfig
                tableName="B"
                model={tableBModel}
                setModel={setTableBModel}
                rows={tableBRows}
                setRows={setTableBRows}
                isActive={activeTable === 'B'}
                isOperating={isOperating}
                setIsOperating={setIsOperating}
                addActivity={addActivity}
                robotSpeed={robotSpeed}
                sandingSpeed={sandingSpeed}
                inverseOverlapping={inverseOverlapping}
                spiralSettings={{
                  enabled: isSpiralSettingsEnabled(),
                  speedPercent: spiralSpeed[0],
                  radiusMm: spiralRadius[0],
                  linearSpeedMmS: spiralLinearSpeed[0],
                }}
                homingRequired={homingRequired}
              />
            </SlidingPanel>
            
          </div>

          <div className="space-y-6">
            <SettingsPanel
              robotSpeed={robotSpeed}
              setRobotSpeed={setRobotSpeed}
              inverseOverlapping={inverseOverlapping}
              setInverseOverlapping={setInverseOverlapping}
              sandingSpeed={sandingSpeed}
              setSandingSpeed={setSandingSpeed}
            />

            <RobotStatusCard
              isHoming={isHoming}
              setIsHoming={setIsHoming}
              activities={activities}
              addActivity={addActivity}
              isOperating={isOperating}
              robotEnabled={robotEnabled}
              onHomingCompleted={() => setHomingRequired(false)}
            />
          </div>
        </div>
      </main>

      <footer className="mt-4 border-t border-cyan-200/60 bg-gradient-to-r from-cyan-50/80 via-blue-50/70 to-indigo-50/80 backdrop-blur-sm">
        <div className="max-w-7xl mx-auto px-4 sm:px-6 lg:px-8 py-2.5 flex flex-col sm:flex-row items-center justify-between gap-2 text-xs text-slate-700">
          <div className="flex items-center gap-2">
            <span className="inline-block h-1.5 w-1.5 rounded-full bg-cyan-500" />
            <span className="font-semibold tracking-wide text-cyan-950">© {new Date().getFullYear()} TechnoAccord Inc</span>
          </div>
          <span className="rounded-full border border-indigo-300/90 bg-white/85 px-2.5 py-1 font-semibold text-indigo-800 shadow-sm">
            Version 1.0
          </span>
        </div>
      </footer>
    </div>
  );
}
