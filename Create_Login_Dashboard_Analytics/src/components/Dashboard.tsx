 import React, { useState } from 'react';
import { DashboardHeader } from './dashboard/DashboardHeader';
import { StatusBanner } from './dashboard/StatusBanner';
import { RobotControlPanel } from './dashboard/RobotControlPanel';
import { RobotStatusCard } from './dashboard/RobotStatusCard';
import { RobotStatusPanel } from './dashboard/RobotStatusPanel';
import { SettingsPanel } from './dashboard/SettingsPanel';
import { SystemIndicators } from './dashboard/SystemIndicators';
import { QuickStats } from './dashboard/QuickStats';
import { Button } from './ui/button';
import { Settings } from 'lucide-react';

interface DashboardProps {
  onNavigateToAnalytics: () => void;
  onNavigateToTableConfig: () => void;
  onLogout: () => void;
}

export function Dashboard({ onNavigateToAnalytics, onNavigateToTableConfig, onLogout }: DashboardProps) {
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

  return (
    <div className="min-h-screen bg-gradient-to-br from-slate-50 to-slate-100 pb-24">
      <DashboardHeader 
        onNavigateToAnalytics={onNavigateToAnalytics}
        onNavigateToTableConfig={onNavigateToTableConfig}
        onLogout={onLogout}
      />

      <main className="max-w-7xl mx-auto px-4 sm:px-6 lg:px-8 py-8">
        <StatusBanner 
          robotEnabled={robotEnabled}
          robotSpeed={robotSpeed}
        />

        <div className="grid grid-cols-1 lg:grid-cols-3 gap-6">
          <div className="lg:col-span-2 space-y-6">
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
            
            <RobotStatusCard
              isHoming={isHoming}
              setIsHoming={setIsHoming}
            />
          </div>

          <div className="space-y-6">
            <SettingsPanel
              robotSpeed={robotSpeed}
              setRobotSpeed={setRobotSpeed}
              inverseOverlapping={inverseOverlapping}
              setInverseOverlapping={setInverseOverlapping}
            />

            <SystemIndicators robotEnabled={robotEnabled} />

            <QuickStats />
          </div>
        </div>
      </main>
    </div>
  );
}