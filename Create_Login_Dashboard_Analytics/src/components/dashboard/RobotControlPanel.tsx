import React from 'react';
import { Card, CardContent, CardHeader, CardTitle } from '../ui/card';
import { Badge } from '../ui/badge';
import { ToggleButton } from './ToggleButton';
import { toolToggle, performAction, toggleTableState } from '../../services/api';

type TablePendingState = 'opening' | 'closing' | null;
type ToolPendingState = { state: 'picking' | 'dropping'; since: number } | null;
type ToolNumber = 1 | 2 | 3 | 4;

interface RobotControlPanelProps {
  robotEnabled: boolean;
  setRobotEnabled: (enabled: boolean) => void;
  stopperAUp: boolean;
  setStopperAUp: (up: boolean) => void;
  stopperBUp: boolean;
  setStopperBUp: (up: boolean) => void;
  tableAOpen: boolean;
  setTableAOpen: (open: boolean) => void;
  tableBOpen: boolean;
  setTableBOpen: (open: boolean) => void;
  tableAPending: TablePendingState;
  setTableAPending: React.Dispatch<React.SetStateAction<TablePendingState>>;
  tableBPending: TablePendingState;
  setTableBPending: React.Dispatch<React.SetStateAction<TablePendingState>>;
  t1Picked: boolean;
  setT1Picked: (picked: boolean) => void;
  t2Picked: boolean;
  setT2Picked: (picked: boolean) => void;
  t3Picked: boolean;
  setT3Picked: (picked: boolean) => void;
  t4Picked: boolean;
  setT4Picked: (picked: boolean) => void;
  t1Pending: ToolPendingState;
  setT1Pending: React.Dispatch<React.SetStateAction<ToolPendingState>>;
  t2Pending: ToolPendingState;
  setT2Pending: React.Dispatch<React.SetStateAction<ToolPendingState>>;
  t3Pending: ToolPendingState;
  setT3Pending: React.Dispatch<React.SetStateAction<ToolPendingState>>;
  t4Pending: ToolPendingState;
  setT4Pending: React.Dispatch<React.SetStateAction<ToolPendingState>>;
  laserOn: boolean;
  setLaserOn: (on: boolean) => void;
  isOperating: boolean;
  addActivity: (message: string, type?: 'info' | 'success' | 'warning' | 'error') => void;
}

export function RobotControlPanel({
  robotEnabled,
  setRobotEnabled,
  stopperAUp,
  setStopperAUp,
  stopperBUp,
  setStopperBUp,
  tableAOpen,
  setTableAOpen,
  tableBOpen,
  setTableBOpen,
  tableAPending,
  setTableAPending,
  tableBPending,
  setTableBPending,
  t1Picked,
  setT1Picked,
  t2Picked,
  setT2Picked,
  t3Picked,
  setT3Picked,
  t4Picked,
  setT4Picked,
  t1Pending,
  setT1Pending,
  t2Pending,
  setT2Pending,
  t3Pending,
  setT3Pending,
  t4Pending,
  setT4Pending,
  isOperating,
  laserOn,
  setLaserOn,
  addActivity,
}: RobotControlPanelProps) {
  const TOOL_PENDING_FALLBACK_PICK_MS = 2500;
  const TOOL_PENDING_FALLBACK_DROP_MS = 6000;
  const TABLE_PENDING_FALLBACK_MS = 12000;
  const getHeldTool = (currentTool: ToolNumber) => {
    if (currentTool !== 1 && t1Picked) return 1;
    if (currentTool !== 2 && t2Picked) return 2;
    if (currentTool !== 3 && t3Picked) return 3;
    if (currentTool !== 4 && t4Picked) return 4;
    return null;
  };

  const isToolPicked = (toolNumber: ToolNumber) => {
    if (toolNumber === 1) return t1Picked;
    if (toolNumber === 2) return t2Picked;
    if (toolNumber === 3) return t3Picked;
    return t4Picked;
  };

  const setToolPickedState = (toolNumber: ToolNumber, picked: boolean) => {
    if (toolNumber === 1) setT1Picked(picked);
    if (toolNumber === 2) setT2Picked(picked);
    if (toolNumber === 3) setT3Picked(picked);
    if (toolNumber === 4) setT4Picked(picked);
  };

  const setToolPendingState = (toolNumber: ToolNumber, pending: ToolPendingState) => {
    if (toolNumber === 1) setT1Pending(pending);
    if (toolNumber === 2) setT2Pending(pending);
    if (toolNumber === 3) setT3Pending(pending);
    if (toolNumber === 4) setT4Pending(pending);
  };

  const clearMatchingToolPending = (toolNumber: ToolNumber, state: 'picking' | 'dropping', since: number) => {
    const clearIfMatching = (prev: ToolPendingState) =>
      prev && prev.state === state && prev.since === since ? null : prev;

    if (toolNumber === 1) setT1Pending(clearIfMatching);
    if (toolNumber === 2) setT2Pending(clearIfMatching);
    if (toolNumber === 3) setT3Pending(clearIfMatching);
    if (toolNumber === 4) setT4Pending(clearIfMatching);
  };

  const handleToolToggle = async (toolNumber: ToolNumber) => {
    const wasPicked = isToolPicked(toolNumber);
    const willPick = !wasPicked;
    const heldTool = willPick ? getHeldTool(toolNumber) : null;
    const pendingSince = Date.now();
    const targetPendingState = willPick ? 'picking' : 'dropping';

    try {
      if (heldTool) {
        setToolPendingState(heldTool, { state: 'dropping', since: pendingSince });
        setToolPendingState(toolNumber, { state: 'picking', since: pendingSince });
        addActivity(`Switching Tool ${heldTool} -> Tool ${toolNumber}: dropping current tool first.`, 'info');
      } else {
        setToolPendingState(toolNumber, { state: targetPendingState, since: pendingSince });
        addActivity(`Tool ${toolNumber} ${willPick ? 'pick' : 'drop'} command sent.`, 'info');
      }

      await toolToggle(toolNumber, willPick ? 'pick' : 'keep');

      if (heldTool) {
        setToolPickedState(heldTool, false);
        setToolPickedState(toolNumber, true);
        addActivity(`Tool ${heldTool} dropped, Tool ${toolNumber} picked up`, 'success');
        window.setTimeout(
          () => clearMatchingToolPending(heldTool, 'dropping', pendingSince),
          TOOL_PENDING_FALLBACK_DROP_MS
        );
        window.setTimeout(
          () => clearMatchingToolPending(toolNumber, 'picking', pendingSince),
          TOOL_PENDING_FALLBACK_PICK_MS
        );
        return;
      }

      setToolPickedState(toolNumber, willPick);
      addActivity(`Tool ${toolNumber} ${willPick ? 'picked up' : 'dropped'}`, 'success');
      const fallbackMs = willPick ? TOOL_PENDING_FALLBACK_PICK_MS : TOOL_PENDING_FALLBACK_DROP_MS;
      window.setTimeout(
        () => clearMatchingToolPending(toolNumber, targetPendingState, pendingSince),
        fallbackMs
      );
    } catch (error) {
      if (heldTool) setToolPendingState(heldTool, null);
      setToolPendingState(toolNumber, null);
      addActivity(
        heldTool
          ? `Tool switch ${heldTool} -> ${toolNumber} failed: ${error}`
          : `Tool ${toolNumber} action failed: ${error}`,
        'error'
      );
    }
  };

  return (
    <Card className="shadow-lg border-0 ">
      <CardHeader className="bg-gradient-to-r from-red-50 to-pink-50">
        <CardTitle className="flex items-center justify-between">
          Robot Control
          <Badge variant={robotEnabled ? 'default' : 'secondary'} className={robotEnabled ? 'bg-green-500' : ''}>
            {robotEnabled ? 'Active' : 'Inactive'}
          </Badge>
        </CardTitle>
      </CardHeader>
      <CardContent className="pt-6">
        {/* Primary Controls */}
        <div className="mb-6">
          <ToggleButton
            label="Robot Power"
            isActive={robotEnabled}
            onToggle={() => {
              setRobotEnabled(!robotEnabled);
              try {
                if (!robotEnabled) {
                  performAction('enable');
                } else {
                  performAction('disable');
                }
                addActivity(
                  `Robot Power ${!robotEnabled ? 'ENABLED' : 'DISABLED'}`,
                  !robotEnabled ? 'success' : 'warning'
                );
              } catch (error) {
                addActivity(
                  `Robot Power ${!robotEnabled ? 'ENABLED' : 'DISABLED'} action failed \n [ Error Reason | ${error} ]`,
                  !robotEnabled ? 'success' : 'warning'
                );
              }
            }}
            activeLabel="DISABLE ROBOT"
            inactiveLabel="ENABLE ROBOT"
            disabled={isOperating}
            showCheckmarkPosition="left"
          />
        </div>

        {!robotEnabled && (
          <div className="mb-4 bg-yellow-50 border-l-4 border-yellow-400 p-3 rounded">
            <p className="text-sm text-yellow-800 font-medium">
              ⚠️ Robot Power must be enabled to control robot functions
            </p>
          </div>
        )}

        <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
          {/* Stopper Controls */}
          <div className="space-y-3 min-w-0">
            <h3 className="text-sm font-semibold text-gray-600 mb-2">Stoppers</h3>
            <ToggleButton label="Stopper A" isActive={stopperAUp} onToggle={async () => { 
              try {
                if (tableAOpen) {
                  addActivity('Set Table A to Horizontal before activating Stopper A.', 'warning');
                  return;
                }
                await performAction(!stopperAUp ? 'stopperUp' : 'stopperDown');
                setStopperAUp(!stopperAUp); 
                addActivity(`Stopper A moved ${!stopperAUp ? 'UP' : 'DOWN'}`, 'success'); 
              } catch (error) {
                addActivity(`Stopper A action failed: ${error}`, 'error');
              }
            }} activeLabel="PUT DOWN" inactiveLabel="PUT UP" disabled={isOperating || !robotEnabled} />
            <ToggleButton label="Stopper B" isActive={stopperBUp} onToggle={async () => { 
              try {
                if (tableBOpen) {
                  addActivity('Set Table B to Horizontal before activating Stopper B.', 'warning');
                  return;
                }
                await performAction(!stopperBUp ? 'stopperUpB' : 'stopperDownB');
                setStopperBUp(!stopperBUp); 
                addActivity(`Stopper B moved ${!stopperBUp ? 'UP' : 'DOWN'}`, 'success'); 
              } catch (error) {
                addActivity(`Stopper B action failed: ${error}`, 'error');
              }
            }} activeLabel="PUT DOWN" inactiveLabel="PUT UP" disabled={isOperating || !robotEnabled} />
          </div>

          {/* Table Controls */}
          <div className="space-y-3 min-w-0">
            <h3 className="text-sm font-semibold text-gray-600 mb-2">Tables</h3>
            <div className="w-full">
              <ToggleButton
                label="Table A"
                isActive={tableAOpen}
                isPending={!!tableAPending}
                allowPendingToggle
                pendingLabel={tableAPending === 'opening' ? 'TO 45°' : 'TO HORIZONTAL'}
                onToggle={async () => { 
                const willOpen = !tableAOpen;
                let pendingTimeout: number | undefined;
                try {
                  const pendingState = willOpen ? 'opening' : 'closing';
                  setTableAPending(pendingState);
                  setTableAOpen(willOpen);
                  pendingTimeout = window.setTimeout(
                    () =>
                      setTableAPending(prev => {
                        if (prev === pendingState) {
                          addActivity('Table A command was sent but the position was not confirmed.', 'warning');
                          return null;
                        }
                        return prev;
                      }),
                    TABLE_PENDING_FALLBACK_MS
                  );
                  const response = await toggleTableState('tableAOpenClose', willOpen ? 'Close' : 'Open');
                  if (response?.error || response?.newState === 'Busy') {
                    if (pendingTimeout) window.clearTimeout(pendingTimeout);
                    setTableAOpen(!willOpen);
                    setTableAPending(null);
                    addActivity(response?.error || 'Table A action blocked. Please try again.', 'warning');
                    return;
                  }
                  addActivity(`Table A moving ${willOpen ? 'to 45°' : 'to Horizontal'}`, willOpen ? 'info' : 'warning'); 
                } catch (error) {
                  if (pendingTimeout) window.clearTimeout(pendingTimeout);
                  setTableAOpen(!willOpen);
                  setTableAPending(null);
                  addActivity(`Table A action failed: ${error}`, 'error');
                }
              }}
                activeLabel="PUT HORIZONTAL"
                inactiveLabel="PUT 45°"
                disabled={isOperating || !robotEnabled}
              />
            </div>
            <div className="w-full">
              <ToggleButton
                label="Table B"
                isActive={tableBOpen}
                isPending={!!tableBPending}
                allowPendingToggle
                pendingLabel={tableBPending === 'opening' ? 'TO 45°' : 'TO HORIZONTAL'}
                onToggle={async () => { 
                const willOpen = !tableBOpen;
                let pendingTimeout: number | undefined;
                try {
                  const pendingState = willOpen ? 'opening' : 'closing';
                  setTableBPending(pendingState);
                  setTableBOpen(willOpen);
                  pendingTimeout = window.setTimeout(
                    () =>
                      setTableBPending(prev => {
                        if (prev === pendingState) {
                          addActivity('Table B command was sent but the position was not confirmed.', 'warning');
                          return null;
                        }
                        return prev;
                      }),
                    TABLE_PENDING_FALLBACK_MS
                  );
                  const response = await toggleTableState('tableBOpenClose', willOpen ? 'Close' : 'Open');
                  if (response?.error || response?.newState === 'Busy') {
                    if (pendingTimeout) window.clearTimeout(pendingTimeout);
                    setTableBOpen(!willOpen);
                    setTableBPending(null);
                    addActivity(response?.error || 'Table B action blocked. Please try again.', 'warning');
                    return;
                  }
                  addActivity(`Table B moving ${willOpen ? 'to 45°' : 'to Horizontal'}`, willOpen ? 'info' : 'warning');
                } catch (error) {
                  if (pendingTimeout) window.clearTimeout(pendingTimeout);
                  setTableBOpen(!willOpen);
                  setTableBPending(null);
                  addActivity(`Table B action failed: ${error}`, 'error');
                }
              }}
                activeLabel="PUT HORIZONTAL"
                inactiveLabel="PUT 45°"
                disabled={isOperating || !robotEnabled}
              />
            </div>
          </div>
        </div>

        {/* Pick & Drop Controls */}
        <div className="mt-6">
          <h3 className="text-sm font-semibold text-gray-600 mb-2">Tool Stations</h3>
          <div className="grid grid-cols-2 md:grid-cols-4 gap-3">
            <ToggleButton label="T1" isActive={t1Picked} isPending={!!t1Pending} pendingLabel={t1Pending?.state === 'picking' ? 'PICKING' : 'DROPPING'} onToggle={() => handleToolToggle(1)} activeLabel="DROP TOOL" inactiveLabel="PICK TOOL" disabled={isOperating || !robotEnabled} />
            <ToggleButton label="T2" isActive={t2Picked} isPending={!!t2Pending} pendingLabel={t2Pending?.state === 'picking' ? 'PICKING' : 'DROPPING'} onToggle={() => handleToolToggle(2)} activeLabel="DROP TOOL" inactiveLabel="PICK TOOL" disabled={isOperating || !robotEnabled} />
            <ToggleButton label="T3" isActive={t3Picked} isPending={!!t3Pending} pendingLabel={t3Pending?.state === 'picking' ? 'PICKING' : 'DROPPING'} onToggle={() => handleToolToggle(3)} activeLabel="DROP TOOL" inactiveLabel="PICK TOOL" disabled={isOperating || !robotEnabled} />
            <ToggleButton label="T4" isActive={t4Picked} isPending={!!t4Pending} pendingLabel={t4Pending?.state === 'picking' ? 'PICKING' : 'DROPPING'} onToggle={() => handleToolToggle(4)} activeLabel="DROP TOOL" inactiveLabel="PICK TOOL" disabled={isOperating || !robotEnabled} />
          </div>
        </div>

        {/* Laser Control */}
        <div className="mt-6">
          <h3 className="text-sm font-semibold text-gray-600 mb-2">Laser</h3>
          <ToggleButton label="Laser" isActive={laserOn} onToggle={async () => { 
            try {
              const nextOn = !laserOn;
              await performAction(nextOn ? 'laserOn' : 'laserOff');
              setLaserOn(nextOn); 
              addActivity(`Laser turned ${nextOn ? 'ON' : 'OFF'}`, nextOn ? 'warning' : 'info'); 
            } catch (error) {
              addActivity(`Laser action failed: ${error}`, 'error');
            }
          }} activeLabel="ON" inactiveLabel="OFF" disabled={isOperating || !robotEnabled} />
        </div>
      </CardContent>
    </Card>
  );
}



