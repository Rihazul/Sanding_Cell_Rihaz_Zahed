 import React, { useState } from 'react';
import { DashboardHeader } from './dashboard/DashboardHeader';
import { StatusBanner } from './dashboard/StatusBanner';
import { RobotControlPanel } from './dashboard/RobotControlPanel';
import { RobotStatusCard } from './dashboard/RobotStatusCard';
import { SettingsPanel } from './dashboard/SettingsPanel';
import { SlidingPanel } from './dashboard/SlidingPanel';
import { CompactTableConfig, type RowConfig } from './dashboard/CompactTableConfig';
import { Button } from './ui/button';
import { Settings } from 'lucide-react';

interface DashboardProps {
  onNavigateToAnalytics: () => void;
}

export function Dashboard({ onNavigateToAnalytics }: DashboardProps) {
  const [robotEnabled, setRobotEnabled] = useState(false);
  const [robotSpeed, setRobotSpeed] = useState([100]);
  const [inverseOverlapping, setInverseOverlapping] = useState([50]);
  const [laserOn, setLaserOn] = useState(false);
  const [isHoming, setIsHoming] = useState(false);
  
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
    { label: 'Frame', selection: '1', force: 4, cycle: 1 },
    { label: 'Pocket ZigZag', selection: '1', force: 5, cycle: 1 },
    { label: '3D', selection: '1', force: 5, cycle: 1 },
    { label: 'Edge Outside', selection: '1', force: 3, cycle: 1 },
    { label: 'Side', selection: '1', force: 3, cycle: 1 },
  ];

  const [tableARows, setTableARows] = useState<RowConfig[]>(defaultRows);
  const [tableBRows, setTableBRows] = useState<RowConfig[]>(
    defaultRows.map((r) => ({ ...r, selection: '0', force: 1, cycle: 1 }))
  );

  return (
    <div className="min-h-screen bg-gradient-to-br from-slate-50 to-slate-100 pb-24">
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
              />
              
              {/* Table A Configuration */}
              <CompactTableConfig
                tableName="A"
                model={tableAModel}
                setModel={setTableAModel}
                rows={tableARows}
                setRows={setTableARows}
                isActive={activeTable === 'A'}
              />
              
              {/* Table B Configuration */}
              <CompactTableConfig
                tableName="B"
                model={tableBModel}
                setModel={setTableBModel}
                rows={tableBRows}
                setRows={setTableBRows}
                isActive={activeTable === 'B'}
              />
            </SlidingPanel>
            
          </div>

          <div className="space-y-6">
            <SettingsPanel
              robotSpeed={robotSpeed}
              setRobotSpeed={setRobotSpeed}
              inverseOverlapping={inverseOverlapping}
              setInverseOverlapping={setInverseOverlapping}
            />

            <RobotStatusCard
              isHoming={isHoming}
              setIsHoming={setIsHoming}
            />
          </div>
        </div>
      </main>
    </div>
  );
}