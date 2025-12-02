 import React, { useState } from 'react';
import { DashboardHeader } from './dashboard/DashboardHeader';
import { StatusBanner } from './dashboard/StatusBanner';
import { RobotControlPanel } from './dashboard/RobotControlPanel';
import { RobotStatusCard } from './dashboard/RobotStatusCard';
import { SettingsPanel } from './dashboard/SettingsPanel';
import { SlidingPanel } from './dashboard/SlidingPanel';
import { CompactTableConfig, type RowConfig, type DoorConfig } from './dashboard/CompactTableConfig';
import { Button } from './ui/button';
import { Settings } from 'lucide-react';

interface DashboardProps {
  onNavigateToAnalytics: () => void;
  activities: Array<{id: number, timestamp: string, message: string, type: 'info' | 'success' | 'warning' | 'error'}>;
  addActivity: (message: string, type?: 'info' | 'success' | 'warning' | 'error') => void;
}

export function Dashboard({ onNavigateToAnalytics, activities, addActivity }: DashboardProps) {
  const [robotEnabled, setRobotEnabled] = useState(false);
  const [robotSpeed, setRobotSpeed] = useState([100]);
  const [inverseOverlapping, setInverseOverlapping] = useState([50]);
  const [sandingSpeed, setSandingSpeed] = useState([75]);
  const [laserOn, setLaserOn] = useState(false);
  const [isHoming, setIsHoming] = useState(false);
  const [isOperating, setIsOperating] = useState(false);
  
  // Toggle states
  const [stopperAUp, setStopperAUp] = useState(false);
  const [stopperBUp, setStopperBUp] = useState(false);
  const [toolLifted, setToolLifted] = useState(false);
  const [tableAOpen, setTableAOpen] = useState(false);
  const [tableBOpen, setTableBOpen] = useState(false);
  const [t1Picked, setT1Picked] = useState(false);
  const [t2Picked, setT2Picked] = useState(false);
  const [t3Picked, setT3Picked] = useState(false);
  const [t4Picked, setT4Picked] = useState(false);

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
    { label: 'Pocket ZigZag', selection: '', force: 0, cycle: 0 },
    { label: '3D', selection: '', force: 0, cycle: 0 },
    { label: 'Edge Outside', selection: '', force: 0, cycle: 0 },
    { label: 'Side', selection: '', force: 0, cycle: 0 },
  ];

  const [tableARows, setTableARows] = useState<RowConfig[]>(defaultRows);
  const [tableBRows, setTableBRows] = useState<RowConfig[]>(defaultRows);

  // Door configurations for Table A (each door can have a different model)
  const [doorConfigs, setDoorConfigs] = useState<Array<{doorNumber: number, model: string, rows: RowConfig[]}>>([
    { doorNumber: 1, model: '', rows: [...defaultRows] },
    { doorNumber: 2, model: '', rows: [...defaultRows] },
    { doorNumber: 3, model: '', rows: [...defaultRows] },
    { doorNumber: 4, model: '', rows: [...defaultRows] },
  ]);

  return (
    <div className="min-h-screen bg-gradient-to-br from-cyan-50 via-blue-100 to-indigo-100 pb-24">
      <DashboardHeader 
        onNavigateToAnalytics={onNavigateToAnalytics}
      />

      <main className="max-w-7xl mx-auto px-4 sm:px-6 lg:px-8 py-8">
        <StatusBanner 
          robotEnabled={robotEnabled}
          robotSpeed={robotSpeed}
        />

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
                toolLifted={toolLifted}
                setToolLifted={setToolLifted}
                tableAOpen={tableAOpen}
                setTableAOpen={setTableAOpen}
                tableBOpen={tableBOpen}
                setTableBOpen={setTableBOpen}
                t1Picked={t1Picked}
                setT1Picked={setT1Picked}
                t2Picked={t2Picked}
                setT2Picked={setT2Picked}
                t3Picked={t3Picked}
                setT3Picked={setT3Picked}
                t4Picked={t4Picked}
                setT4Picked={setT4Picked}
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
            />
          </div>
        </div>
      </main>
    </div>
  );
}