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
