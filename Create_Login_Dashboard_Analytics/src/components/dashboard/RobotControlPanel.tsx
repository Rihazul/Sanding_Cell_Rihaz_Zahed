<<<<<<< HEAD
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
  toolLifted: boolean;
  setToolLifted: (lifted: boolean) => void;
  tableAOpen: boolean;
  setTableAOpen: (open: boolean) => void;
  tableBOpen: boolean;
  setTableBOpen: (open: boolean) => void;
  t1Picked: boolean;
  setT1Picked: (picked: boolean) => void;
  t2Picked: boolean;
  setT2Picked: (picked: boolean) => void;
  t3Picked: boolean;
  setT3Picked: (picked: boolean) => void;
  t4Picked: boolean;
  setT4Picked: (picked: boolean) => void;
  laserOn: boolean;
  setLaserOn: (on: boolean) => void;
  isOperating: boolean;
  addActivity: (message: string, type?: 'info' | 'success' | 'warning' | 'error') => void;
}

const getErrorMessage = (error: unknown) => (error instanceof Error ? error.message : String(error));

export function RobotControlPanel({
  robotEnabled,
  setRobotEnabled,
  stopperAUp,
  setStopperAUp,
  stopperBUp,
  setStopperBUp,
  toolLifted,
  setToolLifted,
  tableAOpen,
  setTableAOpen,
  tableBOpen,
  setTableBOpen,
  t1Picked,
  setT1Picked,
  t2Picked,
  setT2Picked,
  t3Picked,
  setT3Picked,
  t4Picked,
  setT4Picked,
  isOperating,
  laserOn,
  setLaserOn,
  addActivity,
}: RobotControlPanelProps) {
  const handleRobotPowerToggle = async () => {
    const next = !robotEnabled;
    setRobotEnabled(next);
    try {
      await performAction(next ? 'enable' : 'disable');
      addActivity(`Robot Power ${next ? 'ENABLED' : 'DISABLED'}`, next ? 'success' : 'warning');
    } catch (error) {
      setRobotEnabled(!next);
      addActivity(
        `Robot Power ${next ? 'ENABLE' : 'DISABLE'} action failed [${getErrorMessage(error)}]`,
        'error'
      );
    }
  };

  const handleStopperAToggle = async () => {
    const next = !stopperAUp;
    setStopperAUp(next);
    try {
      await performAction(next ? 'stopperUp' : 'stopperDown');
      addActivity(`Stopper A moved ${next ? 'UP' : 'DOWN'}`, 'info');
    } catch (error) {
      setStopperAUp(!next);
      addActivity(`Stopper A action failed: ${getErrorMessage(error)}`, 'error');
    }
  };

  const handleStopperBToggle = async () => {
    const next = !stopperBUp;
    setStopperBUp(next);
    try {
      await performAction(next ? 'stopperUpB' : 'stopperDownB');
      addActivity(`Stopper B moved ${next ? 'UP' : 'DOWN'}`, 'info');
    } catch (error) {
      setStopperBUp(!next);
      addActivity(`Stopper B action failed: ${getErrorMessage(error)}`, 'error');
    }
  };

  const handleTableToggle = async (table: 'A' | 'B') => {
    const tableId = table === 'A' ? 'tableAOpenClose' : 'tableBOpenClose';
    try {
      const response = await toggleTableState(tableId);
      const newState = (response?.newState as string | undefined) || '';
      const isOpen = newState.toLowerCase() == "open";
      if (table === 'A') {
        setTableAOpen(!isOpen);
      } else {
        setTableBOpen(!isOpen);
      }
      addActivity(`Table ${table} ${isOpen ? 'OPENED' : 'CLOSED'}`, 'info');
    } catch (error) {
      addActivity(`Table ${table} toggle failed: ${getErrorMessage(error)}`, 'error');
    }
  };

  const handleToolPositionToggle = async () => {
    const next = !toolLifted;
    setToolLifted(next);
    try {
      await performAction(next ? 'toolLift' : 'toolDrop');
      addActivity(`Tool ${next ? 'LIFTED' : 'DROPPED'}`, 'info');
    } catch (error) {
      setToolLifted(!next);
      addActivity(`Tool position change failed: ${getErrorMessage(error)}`, 'error');
    }
  };

  const handleToolStationToggle = async (
    toolNumber: 1 | 2 | 3 | 4,
    picked: boolean,
    setter: (picked: boolean) => void
  ) => {
    const next = !picked;
    setter(next);
    try {
      if (toolNumber === 4) {
        throw new Error('No backend endpoint configured for Tool 4');
      }
      await toolToggle(toolNumber as 1 | 2 | 3, next ? 'pick' : 'keep');
      addActivity(`Tool ${toolNumber} ${next ? 'picked up' : 'dropped'}`, 'info');
    } catch (error) {
      setter(!next);
      addActivity(`Tool ${toolNumber} action failed: ${getErrorMessage(error)}`, 'error');
    }
  };

  const handleLaserToggle = async () => {
    const next = !laserOn;
    setLaserOn(next);
    addActivity(`Laser turned ${next ? 'ON' : 'OFF'}`, next ? 'warning' : 'info');
  };

  return (
    <Card className="shadow-lg border-0">
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
            onToggle={() => void handleRobotPowerToggle()}
            activeLabel="ENABLED"
            inactiveLabel="DISABLED"
            disabled={isOperating}
          />
        </div>

        {!robotEnabled && (
          <div className="mb-4 bg-yellow-50 border-l-4 border-yellow-400 p-3 rounded">
            <p className="text-sm text-yellow-800 font-medium">
              Warning: Robot Power must be enabled to control robot functions
            </p>
          </div>
        )}

        <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
          {/* Stopper Controls */}
          <div className="space-y-3 min-w-0">
            <h3 className="text-sm font-semibold text-gray-600 mb-2">Stoppers</h3>
            <ToggleButton
              label="Stopper A"
              isActive={stopperAUp}
              onToggle={() => void handleStopperAToggle()}
              activeLabel="UP"
              inactiveLabel="DOWN"
              disabled={isOperating || !robotEnabled}
            />
            <ToggleButton
              label="Stopper B"
              isActive={stopperBUp}
              onToggle={() => void handleStopperBToggle()}
              activeLabel="UP"
              inactiveLabel="DOWN"
              disabled={isOperating || !robotEnabled}
            />
          </div>

          {/* Table Controls */}
          <div className="space-y-3 min-w-0">
            <h3 className="text-sm font-semibold text-gray-600 mb-2">Tables</h3>
            <div className="w-full">
              <ToggleButton
                label="Table A"
                isActive={tableAOpen}
                onToggle={() => void handleTableToggle('A')}
                activeLabel="OPEN"
                inactiveLabel="CLOSED"
                disabled={isOperating || !robotEnabled}
              />
            </div>
            <div className="w-full">
              <ToggleButton
                label="Table B"
                isActive={tableBOpen}
                onToggle={() => void handleTableToggle('B')}
                activeLabel="OPEN"
                inactiveLabel="CLOSED"
                disabled={isOperating || !robotEnabled}
              />
            </div>
          </div>
        </div>

        {/* Tool Controls */}
        <div className="mt-6">
          <h3 className="text-sm font-semibold text-gray-600 mb-2">Tool</h3>
          <ToggleButton
            label="Tool Position"
            isActive={toolLifted}
            onToggle={() => void handleToolPositionToggle()}
            activeLabel="LIFTED"
            inactiveLabel="DROPPED"
            disabled={isOperating || !robotEnabled}
          />
        </div>

        {/* Pick & Drop Controls */}
        <div className="mt-6">
          <h3 className="text-sm font-semibold text-gray-600 mb-2">Tool Stations</h3>
          <div className="grid grid-cols-2 md:grid-cols-4 gap-3">
            <ToggleButton
              label="T1"
              isActive={t1Picked}
              onToggle={() => void handleToolStationToggle(1, t1Picked, setT1Picked)}
              activeLabel="PICKED"
              inactiveLabel="DROPPED"
              disabled={isOperating || !robotEnabled}
            />
            <ToggleButton
              label="T2"
              isActive={t2Picked}
              onToggle={() => void handleToolStationToggle(2, t2Picked, setT2Picked)}
              activeLabel="PICKED"
              inactiveLabel="DROPPED"
              disabled={isOperating || !robotEnabled}
            />
            <ToggleButton
              label="T3"
              isActive={t3Picked}
              onToggle={() => void handleToolStationToggle(3, t3Picked, setT3Picked)}
              activeLabel="PICKED"
              inactiveLabel="DROPPED"
              disabled={isOperating || !robotEnabled}
            />
            <ToggleButton
              label="T4"
              isActive={t4Picked}
              onToggle={() => void handleToolStationToggle(4, t4Picked, setT4Picked)}
              activeLabel="PICKED"
              inactiveLabel="DROPPED"
              disabled={isOperating || !robotEnabled}
            />
          </div>
        </div>

        {/* Laser Control */}
        <div className="mt-6">
          <h3 className="text-sm font-semibold text-gray-600 mb-2">Laser</h3>
          <ToggleButton
            label="Laser"
            isActive={laserOn}
            onToggle={() => void handleLaserToggle()}
            activeLabel="ON"
            inactiveLabel="OFF"
            disabled={isOperating || !robotEnabled}
          />
        </div>
      </CardContent>
    </Card>
  );
}
=======
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
  setTableAOpen: (open: boolean) => void;
  tableBOpen: boolean;
  setTableBOpen: (open: boolean) => void;
  t1Picked: boolean;
  setT1Picked: (picked: boolean) => void;
  t2Picked: boolean;
  setT2Picked: (picked: boolean) => void;
  t3Picked: boolean;
  setT3Picked: (picked: boolean) => void;
  t4Picked: boolean;
  setT4Picked: (picked: boolean) => void;
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
  t1Picked,
  setT1Picked,
  t2Picked,
  setT2Picked,
  t3Picked,
  setT3Picked,
  t4Picked,
  setT4Picked,
  isOperating,
  laserOn,
  setLaserOn,
  addActivity,
}: RobotControlPanelProps) {
  return (
    <Card className="shadow-lg border-0">
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
          <ToggleButton label="Robot Power" isActive={robotEnabled} onToggle={() => { 
            setRobotEnabled(!robotEnabled); 
            try {
              if (!robotEnabled) {
                performAction('enable');
              } else {
                performAction('disable');
              }
              addActivity(`Robot Power ${!robotEnabled ? 'ENABLED' : 'DISABLED'}`, !robotEnabled ? 'success' : 'warning'); 
            } 
            catch (error) {
              addActivity(`Robot Power ${!robotEnabled ? 'ENABLED' : 'DISABLED'} action failed \n [ Error Reason | ${error} ]`, !robotEnabled ? 'success' : 'warning'); 
            }
            
          }
          } activeLabel="ENABLED" inactiveLabel="DISABLED" disabled={isOperating} />
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
                await performAction(!stopperAUp ? 'stopperUp' : 'stopperDown');
                setStopperAUp(!stopperAUp); 
                addActivity(`Stopper A moved ${!stopperAUp ? 'UP' : 'DOWN'}`, 'success'); 
              } catch (error) {
                addActivity(`Stopper A action failed: ${error}`, 'error');
              }
            }} activeLabel="UP" inactiveLabel="DOWN" disabled={isOperating || !robotEnabled} />
            <ToggleButton label="Stopper B" isActive={stopperBUp} onToggle={async () => { 
              try {
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
              <ToggleButton label="Table A" isActive={tableAOpen} onToggle={async () => { 
                try {
                  await toggleTableState('tableAOpenClose');
                  setTableAOpen(!tableAOpen); 
                  addActivity(`Table A ${!tableAOpen ? 'OPENED' : 'CLOSED'}`, 'success'); 
                } catch (error) {
                  addActivity(`Table A action failed: ${error}`, 'error');
                }
              }} activeLabel="OPEN" inactiveLabel="CLOSED" disabled={isOperating || !robotEnabled} />
            </div>
            <div className="w-full">
              <ToggleButton label="Table B" isActive={tableBOpen} onToggle={async () => { 
                try {
                  await toggleTableState('tableBOpenClose');
                  setTableBOpen(!tableBOpen); 
                  addActivity(`Table B ${!tableBOpen ? 'OPENED' : 'CLOSED'}`, 'success'); 
                } catch (error) {
                  addActivity(`Table B action failed: ${error}`, 'error');
                }
              }} activeLabel="OPEN" inactiveLabel="CLOSED" disabled={isOperating || !robotEnabled} />
            </div>
          </div>
        </div>

        {/* Pick & Drop Controls */}
        <div className="mt-6">
          <h3 className="text-sm font-semibold text-gray-600 mb-2">Tool Stations</h3>
          <div className="grid grid-cols-2 md:grid-cols-4 gap-3">
            <ToggleButton label="T1" isActive={t1Picked} onToggle={async () => { 
              try {
                await toolToggle(1, !t1Picked ? 'pick' : 'keep');
                setT1Picked(!t1Picked); 
                addActivity(`Tool 1 ${!t1Picked ? 'picked up' : 'dropped'}`, 'success'); 
              } catch (error) {
                addActivity(`Tool 1 action failed: ${error}`, 'error');
              }
            }} activeLabel="PICKED" inactiveLabel="DROPPED" disabled={isOperating || !robotEnabled} />
            <ToggleButton label="T2" isActive={t2Picked} onToggle={async () => { 
              try {
                await toolToggle(2, !t2Picked ? 'pick' : 'keep');
                setT2Picked(!t2Picked); 
                addActivity(`Tool 2 ${!t2Picked ? 'picked up' : 'dropped'}`, 'success'); 
              } catch (error) {
                addActivity(`Tool 2 action failed: ${error}`, 'error');
              }
            }} activeLabel="PICKED" inactiveLabel="DROPPED" disabled={isOperating || !robotEnabled} />
            <ToggleButton label="T3" isActive={t3Picked} onToggle={async () => { 
              try {
                await toolToggle(3, !t3Picked ? 'pick' : 'keep');
                setT3Picked(!t3Picked); 
                addActivity(`Tool 3 ${!t3Picked ? 'picked up' : 'dropped'}`, 'success'); 
              } catch (error) {
                addActivity(`Tool 3 action failed: ${error}`, 'error');
              }
            }} activeLabel="PICKED" inactiveLabel="DROPPED" disabled={isOperating || !robotEnabled} />
            <ToggleButton label="T4" isActive={t4Picked} onToggle={async () => { 
              try {
                await toolToggle(4, !t4Picked ? 'pick' : 'keep');
                setT4Picked(!t4Picked); 
                addActivity(`Tool 4 ${!t4Picked ? 'picked up' : 'dropped'}`, 'success'); 
              } catch (error) {
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
              // Note: Add laser API endpoint when available
              setLaserOn(!laserOn); 
              addActivity(`Laser turned ${!laserOn ? 'ON' : 'OFF'}`, !laserOn ? 'warning' : 'info'); 
            } catch (error) {
              addActivity(`Laser action failed: ${error}`, 'error');
            }
          }} activeLabel="ON" inactiveLabel="OFF" disabled={isOperating || !robotEnabled} />
        </div>
      </CardContent>
    </Card>
  );
}
>>>>>>> feat/UI
