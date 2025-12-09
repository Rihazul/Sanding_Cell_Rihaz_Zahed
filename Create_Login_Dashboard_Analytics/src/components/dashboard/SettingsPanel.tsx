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
}

export function SettingsPanel({
  robotSpeed,
  setRobotSpeed,
  inverseOverlapping,
  setInverseOverlapping,
  sandingSpeed,
  setSandingSpeed,
}: SettingsPanelProps) {
  return (
    <Card className="shadow-lg border-0">
      <CardHeader className="bg-gradient-to-r from-purple-50 to-blue-50">
        <CardTitle>Settings</CardTitle>
      </CardHeader>
      <CardContent className="pt-6 space-y-6">
        {/* Spiral Speed */}
        <div>
          <div className="flex justify-between mb-2">
            <label className="text-sm">Spiral Speed</label>
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
      </CardContent>
    </Card>
  );
}
