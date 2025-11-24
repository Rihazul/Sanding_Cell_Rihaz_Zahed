import React, { useState } from 'react';
import { Button } from './ui/button';
import { Card, CardContent, CardHeader, CardTitle } from './ui/card';
import { Badge } from './ui/badge';
import { Slider } from './ui/slider';
import { BarChart3, LogOut, Power, Zap, Circle } from 'lucide-react';
import { motion } from 'motion/react';

interface DashboardProps {
  onNavigateToAnalytics: () => void;
  onLogout: () => void;
}

type RowConfig = {
  label: string;
  selection: string;
  force: number;
  cycle: number;
};

export function Dashboard({ onNavigateToAnalytics, onLogout }: DashboardProps) {
  const [robotEnabled, setRobotEnabled] = useState(false);
  const [robotSpeed, setRobotSpeed] = useState([100]);
  const [inverseOverlapping, setInverseOverlapping] = useState([50]);
  const [laserOn, setLaserOn] = useState(false);
  const [isHoming, setIsHoming] = useState(false);
  const [activeControls, setActiveControls] = useState<Set<string>>(new Set());
  
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
  
  // Table Configuration
  const [tableAModel, setTableAModel] = useState('');
  const [tableBModel, setTableBModel] = useState('');

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

  const handleControlClick = (control: string) => {
    setActiveControls((prev: Set<string>) => {
      const newSet = new Set(prev);
      if (newSet.has(control)) newSet.delete(control);
      else newSet.add(control);
      return newSet;
    });

    setTimeout(() => {
      setActiveControls((prev: Set<string>) => {
        const newSet = new Set(prev);
        newSet.delete(control);
        return newSet;
      });
    }, 300);
  };

  const ControlButton = ({
    label,
    color,
    onClick,
  }: {
    label: string;
    color: 'red' | 'green' | 'blue' | 'pink' | 'purple';
    onClick?: () => void;
  }) => {
    const isActive = activeControls.has(label);
    const colorClasses: Record<string, string> = {
      red: 'bg-red-500 hover:bg-red-600 active:bg-red-700',
      green: 'bg-emerald-500 hover:bg-emerald-600 active:bg-emerald-700',
      blue: 'bg-blue-500 hover:bg-blue-600 active:bg-blue-700',
      pink: 'bg-pink-500 hover:bg-pink-600 active:bg-pink-700',
      purple: 'bg-purple-500 hover:bg-purple-600 active:bg-purple-700',
    };

    return (
      <motion.button
        whileTap={{ scale: 0.95 }}
        animate={{ scale: isActive ? 0.95 : 1 }}
        onClick={() => {
          handleControlClick(label);
          onClick?.();
        }}
        className={`${colorClasses[color]} text-white px-4 py-2.5 rounded-lg transition-all duration-200 shadow-md hover:shadow-lg`}
      >
        {label}
      </motion.button>
    );
  };

  const ToggleButton = ({
    label,
    isActive,
    onToggle,
    activeLabel,
    inactiveLabel,
  }: {
    label: string;
    isActive: boolean;
    onToggle: () => void;
    activeLabel: string;
    inactiveLabel: string;
  }) => {
    return (
      <motion.button
        whileTap={{ scale: 0.95 }}
        onClick={onToggle}
        className={`relative px-6 py-4 rounded-xl transition-all duration-300 shadow-md hover:shadow-lg overflow-hidden ${
          isActive
            ? 'bg-gradient-to-r from-emerald-500 to-green-500 text-white'
            : 'bg-gradient-to-r from-gray-100 to-gray-200 text-gray-700'
        }`}
      >
        <div className="flex items-center justify-between">
          <span className="font-medium">{label}</span>
          <div className={`flex items-center gap-2 px-3 py-1 rounded-full text-sm ${isActive ? 'bg-white/20' : 'bg-gray-300'}`}>
            <motion.div animate={{ rotate: isActive ? 180 : 0 }} transition={{ duration: 0.3 }}>
              {isActive ? '✓' : '○'}
            </motion.div>
            <span className="text-xs">{isActive ? activeLabel : inactiveLabel}</span>
          </div>
        </div>
      </motion.button>
    );
  };

  return (
    <div className="min-h-screen bg-gradient-to-br from-slate-50 to-slate-100">
      {/* Header */}
      <header className="bg-white border-b border-gray-200 shadow-sm">
        <div className="max-w-7xl mx-auto px-4 sm:px-6 lg:px-8">
          <div className="flex justify-between items-center py-4">
            <div className="flex items-center gap-3">
              <div className="p-2 bg-gradient-to-br from-blue-500 to-purple-600 rounded-lg">
                <Zap className="size-5 text-white" />
              </div>
              <h1>Robot Control Dashboard</h1>
            </div>

            <div className="flex gap-3">
              <Button onClick={onNavigateToAnalytics} variant="outline">
                <BarChart3 className="size-4 mr-2" />
                Analytics
              </Button>
              <Button onClick={onLogout} variant="ghost">
                <LogOut className="size-4 mr-2" />
                Logout
              </Button>
            </div>
          </div>
        </div>
      </header>

      {/* Main Content */}
      <main className="max-w-7xl mx-auto px-4 sm:px-6 lg:px-8 py-8">
        {/* Status Banner */}
        <motion.div
          initial={{ opacity: 0, y: -20 }}
          animate={{ opacity: 1, y: 0 }}
          className="mb-6 p-4 bg-white rounded-xl shadow-md border-l-4 border-blue-500 flex items-center justify-between"
        >
          <div className="flex items-center gap-3">
            <Power className={`size-6 ${robotEnabled ? 'text-green-500' : 'text-gray-400'}`} />
            <div>
              <p>System Status</p>
              <p className="text-sm text-gray-600">Robot is {robotEnabled ? 'ONLINE' : 'OFFLINE'} • Speed: {robotSpeed[0]}%</p>
            </div>
          </div>

          <div className="flex gap-2">
            {[1, 2, 3, 4, 5].map((i) => (
              <motion.div
                key={i}
                animate={{
                  scale: robotEnabled ? [1, 1.2, 1] : 1,
                  opacity: robotEnabled ? [0.5, 1, 0.5] : 0.3,
                }}
                transition={{ duration: 1.5, repeat: robotEnabled ? Infinity : 0, delay: i * 0.2 }}
              >
                <Circle className="size-3 fill-red-500 text-red-500" />
              </motion.div>
            ))}
          </div>
        </motion.div>

        <div className="grid grid-cols-1 lg:grid-cols-3 gap-6">
          {/* Robot Control Panel */}
          <div className="lg:col-span-2 space-y-6">
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
                  <ToggleButton label="Robot Power" isActive={robotEnabled} onToggle={() => setRobotEnabled(!robotEnabled)} activeLabel="ENABLED" inactiveLabel="DISABLED" />
                </div>

                <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
                  {/* Stopper Controls */}
                  <div className="space-y-3">
                    <h3 className="text-sm font-semibold text-gray-600 mb-2">Stoppers</h3>
                    <ToggleButton label="Stopper A" isActive={stopperAUp} onToggle={() => setStopperAUp(!stopperAUp)} activeLabel="UP" inactiveLabel="DOWN" />
                    <ToggleButton label="Stopper B" isActive={stopperBUp} onToggle={() => setStopperBUp(!stopperBUp)} activeLabel="UP" inactiveLabel="DOWN" />
                  </div>

                  {/* Table Controls */}
                  <div className="space-y-3">
                    <h3 className="text-sm font-semibold text-gray-600 mb-2">Tables</h3>
                    <ToggleButton label="Table A" isActive={tableAOpen} onToggle={() => setTableAOpen(!tableAOpen)} activeLabel="OPEN" inactiveLabel="CLOSED" />
                    <ToggleButton label="Table B" isActive={tableBOpen} onToggle={() => setTableBOpen(!tableBOpen)} activeLabel="OPEN" inactiveLabel="CLOSED" />
                  </div>
                </div>

                {/* Tool Controls */}
                <div className="mt-6">
                  <h3 className="text-sm font-semibold text-gray-600 mb-2">Tool</h3>
                  <ToggleButton label="Tool Position" isActive={toolLifted} onToggle={() => setToolLifted(!toolLifted)} activeLabel="LIFTED" inactiveLabel="DROPPED" />
                </div>

                {/* Pick & Drop Controls */}
                <div className="mt-6">
                  <h3 className="text-sm font-semibold text-gray-600 mb-2">Tool Stations</h3>
                  <div className="grid grid-cols-2 md:grid-cols-4 gap-3">
                    <ToggleButton label="T1" isActive={t1Picked} onToggle={() => setT1Picked(!t1Picked)} activeLabel="PICKED" inactiveLabel="DROPPED" />
                    <ToggleButton label="T2" isActive={t2Picked} onToggle={() => setT2Picked(!t2Picked)} activeLabel="PICKED" inactiveLabel="DROPPED" />
                    <ToggleButton label="T3" isActive={t3Picked} onToggle={() => setT3Picked(!t3Picked)} activeLabel="PICKED" inactiveLabel="DROPPED" />
                    <ToggleButton label="T4" isActive={t4Picked} onToggle={() => setT4Picked(!t4Picked)} activeLabel="PICKED" inactiveLabel="DROPPED" />
                  </div>
                </div>

                {/* Laser Control */}
                <div className="mt-6">
                  <h3 className="text-sm font-semibold text-gray-600 mb-2">Laser</h3>
                  <ToggleButton label="Laser" isActive={laserOn} onToggle={() => setLaserOn(!laserOn)} activeLabel="ON" inactiveLabel="OFF" />
                </div>
              </CardContent>
            </Card>

            {/* Robot Status */}
            <Card className="shadow-lg border-0">
              <CardHeader className="bg-gradient-to-r from-slate-50 to-gray-50">
                <CardTitle>Robot Status</CardTitle>
              </CardHeader>
              <CardContent className="pt-6">
                <div className="grid grid-cols-2 gap-3">
                  <motion.button
                    whileTap={{ scale: 0.95 }}
                    onClick={() => {
                      setIsHoming(true);
                      setTimeout(() => setIsHoming(false), 2000);
                    }}
                    className={`${isHoming ? 'bg-gray-700' : 'bg-gray-600 hover:bg-gray-700'} text-white px-6 py-3 rounded-lg transition-all shadow-md hover:shadow-lg`}
                  >
                    {isHoming ? 'Homing...' : 'Homing'}
                  </motion.button>
                  <motion.button whileTap={{ scale: 0.95 }} className="bg-red-600 hover:bg-red-700 text-white px-6 py-3 rounded-lg transition-all shadow-md hover:shadow-lg">
                    Stop
                  </motion.button>
                </div>
              </CardContent>
            </Card>

            {/* Table Configurations */}
            <Card className="shadow-lg border-0">
              <CardHeader className="bg-gradient-to-r from-indigo-50 to-cyan-50">
                <CardTitle>Table Configurations</CardTitle>
              </CardHeader>
              <CardContent className="pt-6">
                <div className="grid grid-cols-1 md:grid-cols-2 gap-8">
                  {/* Table A Configuration */}
                  <div className="border-2 border-red-300 rounded-lg p-6 min-w-0">
                    <h3 className="text-center mb-4">Table A</h3>

                    <div className="bg-gray-50 rounded-lg p-4 mb-4">
                      <label className="text-sm text-gray-500 mb-2 flex items-center gap-1">
                        Models
                        <span className="text-gray-400">ⓘ</span>
                      </label>
                      <select value={tableAModel} onChange={(e) => setTableAModel(e.target.value)} className="w-full px-3 py-2 border border-gray-300 rounded-md text-sm bg-white">
                        <option value="">Select a Model</option>
                        <option value="model1">Model 1</option>
                        <option value="model2">Model 2</option>
                        <option value="model3">Model 3</option>
                      </select>

                      <div className="mt-4 space-y-3">
                        {tableARows.map((row: RowConfig, idx: number) => (
                          <div key={row.label} className="grid grid-cols-1 sm:grid-cols-12 items-center gap-3 w-full">
                            <div className="sm:col-span-3 text-gray-600 flex items-center gap-1">
                              {row.label}
                              <span className="text-gray-400">ⓘ</span>
                            </div>

                            <div className="sm:col-span-5 flex flex-wrap items-center gap-1">
                              {['1', '2', '3', '4', 'A'].map((opt) => (
                                <button
                                  key={opt}
                                  onClick={() => {
                                    setTableARows((prev: RowConfig[]) => {
                                      const next = [...prev];
                                      next[idx] = { ...next[idx], selection: opt };
                                      return next;
                                    });
                                  }}
                                  className={`px-2 py-1 rounded-md border text-xs ${row.selection === opt ? 'bg-red-500 text-white border-red-500' : 'bg-white text-gray-700 border-gray-200'}`}
                                >
                                  {opt}
                                </button>
                              ))}
                            </div>

                            <div className="sm:col-span-2 flex items-center gap-2 justify-end">
                              <label className="text-xs text-gray-500">Force</label>
                              <select
                                value={row.force}
                                onChange={(e: React.ChangeEvent<HTMLSelectElement>) => {
                                  const v = Number(e.target.value);
                                  setTableARows((prev: RowConfig[]) => {
                                    const next = [...prev];
                                    next[idx] = { ...next[idx], force: v };
                                    return next;
                                  });
                                }}
                                className="px-2 py-1 border rounded-md text-sm w-20"
                              >
                                {Array.from({ length: 25 }, (_, i) => i + 1).map((n) => (
                                  <option key={n} value={n}>
                                    {n}
                                  </option>
                                ))}
                              </select>
                            </div>

                            <div className="sm:col-span-2 flex items-center gap-2 justify-end">
                              <label className="text-xs text-gray-500">Cycle</label>
                              <select
                                value={row.cycle}
                                onChange={(e: React.ChangeEvent<HTMLSelectElement>) => {
                                  const v = Number(e.target.value);
                                  setTableARows((prev: RowConfig[]) => {
                                    const next = [...prev];
                                    next[idx] = { ...next[idx], cycle: v };
                                    return next;
                                  });
                                }}
                                className="px-2 py-1 border rounded-md text-sm w-20"
                              >
                                {Array.from({ length: 25 }, (_, i) => i + 1).map((n) => (
                                  <option key={n} value={n}>
                                    {n}
                                  </option>
                                ))}
                              </select>
                            </div>
                          </div>
                        ))}
                      </div>
                    </div>

                    <div className="grid grid-cols-2 gap-2">
                      <Button className="bg-pink-400 hover:bg-pink-500">Start Scan</Button>
                      <Button className="bg-blue-400 hover:bg-blue-500">Start Task</Button>
                    </div>
                  </div>

                  {/* Table B Configuration */}
                  <div className="border-2 border-red-300 rounded-lg p-6 min-w-0">
                    <h3 className="text-center mb-4">Table B</h3>

                    <div className="bg-gray-50 rounded-lg p-4 mb-4">
                      <label className="text-sm text-gray-500 mb-2 flex items-center gap-1">
                        Models
                        <span className="text-gray-400">ⓘ</span>
                      </label>
                      <select value={tableBModel} onChange={(e) => setTableBModel(e.target.value)} className="w-full px-3 py-2 border border-gray-300 rounded-md text-sm bg-white">
                        <option value="">Select a Model</option>
                        <option value="model1">Model 1</option>
                        <option value="model2">Model 2</option>
                        <option value="model3">Model 3</option>
                      </select>

                      <div className="mt-4 space-y-3">
                        {tableBRows.map((row: RowConfig, idx: number) => (
                          <div key={row.label} className="grid grid-cols-1 sm:grid-cols-12 items-center gap-3 w-full">
                            <div className="sm:col-span-3 text-gray-600 flex items-center gap-1">
                              {row.label}
                              <span className="text-gray-400">ⓘ</span>
                            </div>

                            <div className="sm:col-span-5 flex flex-wrap items-center gap-1">
                              {['1', '2', '3', '4', 'A'].map((opt) => (
                                <button
                                  key={opt}
                                  onClick={() => {
                                    setTableBRows((prev: RowConfig[]) => {
                                      const next = [...prev];
                                      next[idx] = { ...next[idx], selection: opt };
                                      return next;
                                    });
                                  }}
                                  className={`px-2 py-1 rounded-md border text-xs ${row.selection === opt ? 'bg-red-500 text-white border-red-500' : 'bg-white text-gray-700 border-gray-200'}`}
                                >
                                  {opt}
                                </button>
                              ))}
                            </div>

                            <div className="sm:col-span-2 flex items-center gap-2 justify-end">
                              <label className="text-xs text-gray-500">Force</label>
                              <select
                                value={row.force}
                                onChange={(e: React.ChangeEvent<HTMLSelectElement>) => {
                                  const v = Number(e.target.value);
                                  setTableBRows((prev: RowConfig[]) => {
                                    const next = [...prev];
                                    next[idx] = { ...next[idx], force: v };
                                    return next;
                                  });
                                }}
                                className="px-2 py-1 border rounded-md text-sm w-20"
                              >
                                {Array.from({ length: 25 }, (_, i) => i + 1).map((n) => (
                                  <option key={n} value={n}>
                                    {n}
                                  </option>
                                ))}
                              </select>
                            </div>

                            <div className="sm:col-span-2 flex items-center gap-2 justify-end">
                              <label className="text-xs text-gray-500">Cycle</label>
                              <select
                                value={row.cycle}
                                onChange={(e: React.ChangeEvent<HTMLSelectElement>) => {
                                  const v = Number(e.target.value);
                                  setTableBRows((prev: RowConfig[]) => {
                                    const next = [...prev];
                                    next[idx] = { ...next[idx], cycle: v };
                                    return next;
                                  });
                                }}
                                className="px-2 py-1 border rounded-md text-sm w-20"
                              >
                                {Array.from({ length: 25 }, (_, i) => i + 1).map((n) => (
                                  <option key={n} value={n}>
                                    {n}
                                  </option>
                                ))}
                              </select>
                            </div>
                          </div>
                        ))}
                      </div>
                    </div>

                    <Button className="w-full bg-pink-400 hover:bg-pink-500">Upload 3D File</Button>
                  </div>
                </div>
              </CardContent>
            </Card>
          </div>

          {/* Settings Panel */}
          <div className="space-y-6">
            <Card className="shadow-lg border-0">
              <CardHeader className="bg-gradient-to-r from-purple-50 to-blue-50">
                <CardTitle>Settings</CardTitle>
              </CardHeader>
              <CardContent className="pt-6 space-y-6">
                {/* Robot Speed */}
                <div>
                  <div className="flex justify-between mb-2">
                    <label className="text-sm">Robot Speed</label>
                    <span className="text-sm text-gray-600">{robotSpeed[0]}%</span>
                  </div>
                  <Slider value={robotSpeed} onValueChange={setRobotSpeed} min={0} max={100} step={1} className="[&_[role=slider]]:bg-blue-500" />
                </div>

                {/* Inverse Overlapping */}
                <div>
                  <div className="flex justify-between mb-2">
                    <label className="text-sm">Inverse Overlapping</label>
                    <span className="text-sm text-gray-600">{inverseOverlapping[0]}%</span>
                  </div>
                  <Slider value={inverseOverlapping} onValueChange={setInverseOverlapping} min={0} max={100} step={1} className="[&_[role=slider]]:bg-purple-500" />
                </div>
              </CardContent>
            </Card>

            {/* System Indicators */}
            <Card className="shadow-lg border-0">
              <CardHeader className="bg-gradient-to-r from-orange-50 to-yellow-50">
                <CardTitle>System Indicators</CardTitle>
              </CardHeader>
              <CardContent className="pt-6">
                <div className="flex justify-center gap-3">
                  {[1, 2, 3, 4, 5].map((i) => (
                    <motion.div
                      key={i}
                      animate={{
                        scale: robotEnabled ? [1, 1.3, 1] : 1,
                        backgroundColor: robotEnabled ? ['#ef4444', '#dc2626', '#ef4444'] : '#9ca3af',
                      }}
                      transition={{ duration: 2, repeat: robotEnabled ? Infinity : 0, delay: i * 0.3 }}
                      className="w-10 h-10 rounded-full shadow-lg"
                    />
                  ))}
                </div>
              </CardContent>
            </Card>

            {/* Quick Stats */}
            <Card className="shadow-lg border-0">
              <CardHeader className="bg-gradient-to-r from-green-50 to-emerald-50">
                <CardTitle>Quick Stats</CardTitle>
              </CardHeader>
              <CardContent className="pt-6 space-y-3">
                <div className="flex justify-between">
                  <span className="text-sm text-gray-600">Operations Today</span>
                  <span>247</span>
                </div>
                <div className="flex justify-between">
                  <span className="text-sm text-gray-600">Uptime</span>
                  <span className="text-green-600">99.8%</span>
                </div>
                <div className="flex justify-between">
                  <span className="text-sm text-gray-600">Efficiency</span>
                  <span className="text-blue-600">94.2%</span>
                </div>
              </CardContent>
            </Card>
          </div>
        </div>
      </main>
    </div>
  );
}