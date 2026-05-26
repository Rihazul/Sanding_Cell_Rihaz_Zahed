import React from 'react';
import { Card, CardContent, CardHeader, CardTitle } from '../ui/card';
import { Badge } from '../ui/badge';
import { ToggleButton } from './ToggleButton';
import { toolToggle, performAction, toggleTableState } from '../../services/api';

interface RobotControlPanelProps {
  robotEnabled: boolean;
  setRobotEnabled: (enabled: boolean) => void;
  stopperAUp: boolean;
  setStopperAUp: (up: boolean) => void;
  stopperBUp: boolean;
  setStopperBUp: (up: boolean) => void;
  tableAOpen: boolean;
  tableBOpen: boolean;
  tableAPending: 'opening' | 'closing' | null;
  setTableAPending: (pending: 'opening' | 'closing' | null) => void;
  tableBPending: 'opening' | 'closing' | null;
  setTableBPending: (pending: 'opening' | 'closing' | null) => void;
  t1Picked: boolean;
  setT1Picked: (picked: boolean) => void;
  t2Picked: boolean;
  setT2Picked: (picked: boolean) => void;
  t3Picked: boolean;
  setT3Picked: (picked: boolean) => void;
  t4Picked: boolean;
  setT4Picked: (picked: boolean) => void;
  t1Pending: { state: 'picking' | 'dropping'; since: number } | null;
  setT1Pending: (pending: { state: 'picking' | 'dropping'; since: number } | null) => void;
  t2Pending: { state: 'picking' | 'dropping'; since: number } | null;
  setT2Pending: (pending: { state: 'picking' | 'dropping'; since: number } | null) => void;
  t3Pending: { state: 'picking' | 'dropping'; since: number } | null;
  setT3Pending: (pending: { state: 'picking' | 'dropping'; since: number } | null) => void;
  t4Pending: { state: 'picking' | 'dropping'; since: number } | null;
  setT4Pending: (pending: { state: 'picking' | 'dropping'; since: number } | null) => void;
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
  tableBOpen,
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
  const getHeldTool = (currentTool: number) => {
    if (currentTool !== 1 && t1Picked) return 1;
    if (currentTool !== 2 && t2Picked) return 2;
    if (currentTool !== 3 && t3Picked) return 3;
    if (currentTool !== 4 && t4Picked) return 4;
    return null;
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
            activeLabel="ENABLED"
            inactiveLabel="DISABLED"
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
                  addActivity('Close Table A before activating Stopper A.', 'warning');
                  return;
                }
                await performAction(!stopperAUp ? 'stopperUp' : 'stopperDown');
                setStopperAUp(!stopperAUp); 
                addActivity(`Stopper A moved ${!stopperAUp ? 'UP' : 'DOWN'}`, 'success'); 
              } catch (error) {
                addActivity(`Stopper A action failed: ${error}`, 'error');
              }
            }} activeLabel="UP" inactiveLabel="DOWN" disabled={isOperating || !robotEnabled} />
            <ToggleButton label="Stopper B" isActive={stopperBUp} onToggle={async () => { 
              try {
                if (tableBOpen) {
                  addActivity('Close Table B before activating Stopper B.', 'warning');
                  return;
                }
                await performAction(!stopperBUp ? 'stopperUpB' : 'stopperDownB');
                setStopperBUp(!stopperBUp); 
                addActivity(`Stopper B moved ${!stopperBUp ? 'UP' : 'DOWN'}`, 'success'); 
              } catch (error) {
                addActivity(`Stopper B action failed: ${error}`, 'error');
              }
            }} activeLabel="UP" inactiveLabel="DOWN" disabled={isOperating || !robotEnabled} />
          </div>

          {/* Table Controls */}
          <div className="space-y-3 min-w-0">
            <h3 className="text-sm font-semibold text-gray-600 mb-2">Tables</h3>
            <div className="w-full">
              <ToggleButton
                label="Table A"
                isActive={tableAOpen}
                isPending={!!tableAPending}
                pendingLabel={tableAPending === 'opening' ? 'OPENING' : 'CLOSING'}
                onToggle={async () => { 
                try {
                  const willOpen = !tableAOpen;
                  setTableAPending(willOpen ? 'opening' : 'closing');
                  const response = await toggleTableState('tableAOpenClose');
                  if (response?.error || response?.newState === 'Busy') {
                    setTableAPending(null);
                    addActivity(response?.error || 'Table A action blocked. Please try again.', 'warning');
                    return;
                  }
                  addActivity(`Table A ${willOpen ? 'OPENING' : 'CLOSING'}`, willOpen ? 'info' : 'warning'); 
                } catch (error) {
                  setTableAPending(null);
                  addActivity(`Table A action failed: ${error}`, 'error');
                }
              }}
                activeLabel="OPEN"
                inactiveLabel="CLOSED"
                disabled={isOperating || !robotEnabled}
              />
            </div>
            <div className="w-full">
              <ToggleButton
                label="Table B"
                isActive={tableBOpen}
                isPending={!!tableBPending}
                pendingLabel={tableBPending === 'opening' ? 'OPENING' : 'CLOSING'}
                onToggle={async () => { 
                try {
                  const willOpen = !tableBOpen;
                  setTableBPending(willOpen ? 'opening' : 'closing');
                  const response = await toggleTableState('tableBOpenClose');
                  if (response?.error || response?.newState === 'Busy') {
                    setTableBPending(null);
                    addActivity(response?.error || 'Table B action blocked. Please try again.', 'warning');
                    return;
                  }
                  addActivity(`Table B ${willOpen ? 'OPENING' : 'CLOSING'}`, willOpen ? 'info' : 'warning'); 
                } catch (error) {
                  setTableBPending(null);
                  addActivity(`Table B action failed: ${error}`, 'error');
                }
              }}
                activeLabel="OPEN"
                inactiveLabel="CLOSED"
                disabled={isOperating || !robotEnabled}
              />
            </div>
          </div>
        </div>

        {/* Pick & Drop Controls */}
        <div className="mt-6">
          <h3 className="text-sm font-semibold text-gray-600 mb-2">Tool Stations</h3>
          <div className="grid grid-cols-2 md:grid-cols-4 gap-3">
            <ToggleButton label="T1" isActive={t1Picked} isPending={!!t1Pending} pendingLabel={t1Pending?.state === 'picking' ? 'PICKING' : 'DROPPING'} onToggle={async () => { 
              try {
                const willPick = !t1Picked;
                if (willPick) {
                  const heldTool = getHeldTool(1);
                  if (heldTool) {
                    addActivity(`Drop Tool ${heldTool} first, then pick Tool 1.`, 'warning');
                    return;
                  }
                }
                setT1Pending({ state: willPick ? 'picking' : 'dropping', since: Date.now() });
                await toolToggle(1, !t1Picked ? 'pick' : 'keep');
                setT1Picked(!t1Picked); 
                addActivity(`Tool 1 ${!t1Picked ? 'picked up' : 'dropped'}`, 'success'); 
                const fallbackMs = willPick ? TOOL_PENDING_FALLBACK_PICK_MS : TOOL_PENDING_FALLBACK_DROP_MS;
                window.setTimeout(() => setT1Pending(prev => (prev ? null : prev)), fallbackMs);
              } catch (error) {
                setT1Pending(null);
                addActivity(`Tool 1 action failed: ${error}`, 'error');
              }
            }} activeLabel="PICKED" inactiveLabel="DROPPED" disabled={isOperating || !robotEnabled} />
            <ToggleButton label="T2" isActive={t2Picked} isPending={!!t2Pending} pendingLabel={t2Pending?.state === 'picking' ? 'PICKING' : 'DROPPING'} onToggle={async () => { 
              try {
                const willPick = !t2Picked;
                if (willPick) {
                  const heldTool = getHeldTool(2);
                  if (heldTool) {
                    addActivity(`Drop Tool ${heldTool} first, then pick Tool 2.`, 'warning');
                    return;
                  }
                }
                setT2Pending({ state: willPick ? 'picking' : 'dropping', since: Date.now() });
                await toolToggle(2, !t2Picked ? 'pick' : 'keep');
                setT2Picked(!t2Picked); 
                addActivity(`Tool 2 ${!t2Picked ? 'picked up' : 'dropped'}`, 'success'); 
                const fallbackMs = willPick ? TOOL_PENDING_FALLBACK_PICK_MS : TOOL_PENDING_FALLBACK_DROP_MS;
                window.setTimeout(() => setT2Pending(prev => (prev ? null : prev)), fallbackMs);
              } catch (error) {
                setT2Pending(null);
                addActivity(`Tool 2 action failed: ${error}`, 'error');
              }
            }} activeLabel="PICKED" inactiveLabel="DROPPED" disabled={isOperating || !robotEnabled} />
            <ToggleButton label="T3" isActive={t3Picked} isPending={!!t3Pending} pendingLabel={t3Pending?.state === 'picking' ? 'PICKING' : 'DROPPING'} onToggle={async () => { 
              try {
                const willPick = !t3Picked;
                if (willPick) {
                  const heldTool = getHeldTool(3);
                  if (heldTool) {
                    addActivity(`Drop Tool ${heldTool} first, then pick Tool 3.`, 'warning');
                    return;
                  }
                }
                setT3Pending({ state: willPick ? 'picking' : 'dropping', since: Date.now() });
                await toolToggle(3, !t3Picked ? 'pick' : 'keep');
                setT3Picked(!t3Picked); 
                addActivity(`Tool 3 ${!t3Picked ? 'picked up' : 'dropped'}`, 'success'); 
                const fallbackMs = willPick ? TOOL_PENDING_FALLBACK_PICK_MS : TOOL_PENDING_FALLBACK_DROP_MS;
                window.setTimeout(() => setT3Pending(prev => (prev ? null : prev)), fallbackMs);
              } catch (error) {
                setT3Pending(null);
                addActivity(`Tool 3 action failed: ${error}`, 'error');
              }
            }} activeLabel="PICKED" inactiveLabel="DROPPED" disabled={isOperating || !robotEnabled} />
            <ToggleButton label="T4" isActive={t4Picked} isPending={!!t4Pending} pendingLabel={t4Pending?.state === 'picking' ? 'PICKING' : 'DROPPING'} onToggle={async () => { 
              try {
                const willPick = !t4Picked;
                if (willPick) {
                  const heldTool = getHeldTool(4);
                  if (heldTool) {
                    addActivity(`Drop Tool ${heldTool} first, then pick Tool 4.`, 'warning');
                    return;
                  }
                }
                setT4Pending({ state: willPick ? 'picking' : 'dropping', since: Date.now() });
                await toolToggle(4, !t4Picked ? 'pick' : 'keep');
                setT4Picked(!t4Picked); 
                addActivity(`Tool 4 ${!t4Picked ? 'picked up' : 'dropped'}`, 'success'); 
                const fallbackMs = willPick ? TOOL_PENDING_FALLBACK_PICK_MS : TOOL_PENDING_FALLBACK_DROP_MS;
                window.setTimeout(() => setT4Pending(prev => (prev ? null : prev)), fallbackMs);
              } catch (error) {
                setT4Pending(null);
                addActivity(`Tool 4 action failed: ${error}`, 'error');
              }
            }} activeLabel="PICKED" inactiveLabel="DROPPED" disabled={isOperating || !robotEnabled} />
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
