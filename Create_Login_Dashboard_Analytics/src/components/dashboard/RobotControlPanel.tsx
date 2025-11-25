import React from 'react';
import { Card, CardContent, CardHeader, CardTitle } from '../ui/card';
import { Badge } from '../ui/badge';
import { ToggleButton } from './ToggleButton';

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
          <ToggleButton label="Robot Power" isActive={robotEnabled} onToggle={() => { setRobotEnabled(!robotEnabled); addActivity(`Robot Power ${!robotEnabled ? 'ENABLED' : 'DISABLED'}`, !robotEnabled ? 'success' : 'warning'); }} activeLabel="ENABLED" inactiveLabel="DISABLED" disabled={isOperating} />
        </div>

        <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
          {/* Stopper Controls */}
          <div className="space-y-3 min-w-0">
            <h3 className="text-sm font-semibold text-gray-600 mb-2">Stoppers</h3>
            <ToggleButton label="Stopper A" isActive={stopperAUp} onToggle={() => { setStopperAUp(!stopperAUp); addActivity(`Stopper A moved ${!stopperAUp ? 'UP' : 'DOWN'}`, 'info'); }} activeLabel="UP" inactiveLabel="DOWN" disabled={isOperating} />
            <ToggleButton label="Stopper B" isActive={stopperBUp} onToggle={() => { setStopperBUp(!stopperBUp); addActivity(`Stopper B moved ${!stopperBUp ? 'UP' : 'DOWN'}`, 'info'); }} activeLabel="UP" inactiveLabel="DOWN" disabled={isOperating} />
          </div>

          {/* Table Controls */}
          <div className="space-y-3 min-w-0">
            <h3 className="text-sm font-semibold text-gray-600 mb-2">Tables</h3>
            <div className="w-full">
              <ToggleButton label="Table A" isActive={tableAOpen} onToggle={() => { setTableAOpen(!tableAOpen); addActivity(`Table A ${!tableAOpen ? 'OPENED' : 'CLOSED'}`, 'info'); }} activeLabel="OPEN" inactiveLabel="CLOSED" disabled={isOperating} />
            </div>
            <div className="w-full">
              <ToggleButton label="Table B" isActive={tableBOpen} onToggle={() => { setTableBOpen(!tableBOpen); addActivity(`Table B ${!tableBOpen ? 'OPENED' : 'CLOSED'}`, 'info'); }} activeLabel="OPEN" inactiveLabel="CLOSED" disabled={isOperating} />
            </div>
          </div>
        </div>

        {/* Tool Controls */}
        <div className="mt-6">
          <h3 className="text-sm font-semibold text-gray-600 mb-2">Tool</h3>
          <ToggleButton label="Tool Position" isActive={toolLifted} onToggle={() => { setToolLifted(!toolLifted); addActivity(`Tool ${!toolLifted ? 'LIFTED' : 'DROPPED'}`, 'info'); }} activeLabel="LIFTED" inactiveLabel="DROPPED" disabled={isOperating} />
        </div>

        {/* Pick & Drop Controls */}
        <div className="mt-6">
          <h3 className="text-sm font-semibold text-gray-600 mb-2">Tool Stations</h3>
          <div className="grid grid-cols-2 md:grid-cols-4 gap-3">
            <ToggleButton label="T1" isActive={t1Picked} onToggle={() => { setT1Picked(!t1Picked); addActivity(`Tool 1 ${!t1Picked ? 'picked up' : 'dropped'}`, 'info'); }} activeLabel="PICKED" inactiveLabel="DROPPED" disabled={isOperating} />
            <ToggleButton label="T2" isActive={t2Picked} onToggle={() => { setT2Picked(!t2Picked); addActivity(`Tool 2 ${!t2Picked ? 'picked up' : 'dropped'}`, 'info'); }} activeLabel="PICKED" inactiveLabel="DROPPED" disabled={isOperating} />
            <ToggleButton label="T3" isActive={t3Picked} onToggle={() => { setT3Picked(!t3Picked); addActivity(`Tool 3 ${!t3Picked ? 'picked up' : 'dropped'}`, 'info'); }} activeLabel="PICKED" inactiveLabel="DROPPED" disabled={isOperating} />
            <ToggleButton label="T4" isActive={t4Picked} onToggle={() => { setT4Picked(!t4Picked); addActivity(`Tool 4 ${!t4Picked ? 'picked up' : 'dropped'}`, 'info'); }} activeLabel="PICKED" inactiveLabel="DROPPED" disabled={isOperating} />
          </div>
        </div>

        {/* Laser Control */}
        <div className="mt-6">
          <h3 className="text-sm font-semibold text-gray-600 mb-2">Laser</h3>
          <ToggleButton label="Laser" isActive={laserOn} onToggle={() => { setLaserOn(!laserOn); addActivity(`Laser turned ${!laserOn ? 'ON' : 'OFF'}`, !laserOn ? 'warning' : 'info'); }} activeLabel="ON" inactiveLabel="OFF" disabled={isOperating} />
        </div>
      </CardContent>
    </Card>
  );
}
