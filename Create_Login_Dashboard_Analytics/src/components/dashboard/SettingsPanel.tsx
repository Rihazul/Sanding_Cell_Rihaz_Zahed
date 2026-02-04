import React from 'react';
import { Card, CardContent, CardHeader, CardTitle } from '../ui/card';
import { Slider } from '../ui/slider';

interface SettingsPanelProps {
  robotSpeed: number[];
  setRobotSpeed: (speed: number[]) => void;
  inverseOverlapping: number[];
  setInverseOverlapping: (overlap: number[]) => void;
  sandingSpeed: number[];
  setSandingSpeed: (speed: number[]) => void;
  spiralSpeed: number[];
  setSpiralSpeed: (speed: number[]) => void;
  spiralRadius: number[];
  setSpiralRadius: (radius: number[]) => void;
  spiralSettingsEnabled: boolean;
}

export function SettingsPanel({
  robotSpeed,
  setRobotSpeed,
  inverseOverlapping,
  setInverseOverlapping,
  sandingSpeed,
  setSandingSpeed,
  spiralSpeed,
  setSpiralSpeed,
  spiralRadius,
  setSpiralRadius,
  spiralSettingsEnabled,
}: SettingsPanelProps) {
  return (
    <Card className="shadow-lg border-0">
      <CardHeader className="bg-gradient-to-r from-purple-50 to-blue-50">
        <CardTitle className="flex items-center gap-2">
          <span className="text-lg">⚙️</span>
          Settings
        </CardTitle>
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

        {/* Sanding Speed */}
        <div>
          <div className="flex justify-between mb-2">
            <label className="text-sm">Sanding Speed</label>
            <span className="text-sm text-gray-600">{sandingSpeed[0]}%</span>
          </div>
          <Slider value={sandingSpeed} onValueChange={setSandingSpeed} min={0} max={100} step={1} className="[&_[role=slider]]:bg-pink-500" />
        </div>

        {/* Spiral Settings Sub-section */}
        <div className={`pt-6 mt-2 border-t border-gray-200 ${!spiralSettingsEnabled ? 'opacity-50' : ''}`}>
          <h3 className="text-sm font-semibold text-indigo-600 mb-1 flex items-center gap-2">
            <span className="text-lg">🌀</span>
            Spiral Settings
          </h3>
          {!spiralSettingsEnabled && (
            <p className="text-xs text-gray-400 mb-4">Configure Frame or Pocket ZigZag to enable</p>
          )}
          {spiralSettingsEnabled && <div className="mb-4"></div>}
          
          {/* Spiral Speed */}
          <div className="mb-4">
            <div className="flex justify-between mb-2">
              <label className="text-sm">Spiral Speed</label>
              <span className="text-sm text-gray-600">{spiralSpeed[0]} mm/s</span>
            </div>
            <Slider value={spiralSpeed} onValueChange={setSpiralSpeed} min={100} max={300} step={5} disabled={!spiralSettingsEnabled} className="[&_[role=slider]]:bg-violet-500" />
          </div>

          {/* Radius Size */}
          <div className="mb-4">
            <div className="flex justify-between mb-2">
              <label className="text-sm">Radius Size</label>
              <span className="text-sm text-gray-600">{spiralRadius[0]} mm</span>
            </div>
            <Slider value={spiralRadius} onValueChange={setSpiralRadius} min={10} max={15} step={1} disabled={!spiralSettingsEnabled} className="[&_[role=slider]]:bg-indigo-500" />
          </div>
        </div>
      </CardContent>
    </Card>
  );
}
