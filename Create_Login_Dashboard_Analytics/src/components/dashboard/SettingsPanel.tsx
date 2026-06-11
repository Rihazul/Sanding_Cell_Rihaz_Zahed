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
  const ROBOT_MAX_SPEED = 400;
  const ROBOT_MAX_ACCEL = 1950;
  const SANDING_MAX_SPEED = 275;
  const SANDING_MAX_ACCEL = 350;
  const POCKET_MAX_OVERLAP_MM = 100;
  const robotSpeedMmS = Math.round((robotSpeed[0] / 100) * ROBOT_MAX_SPEED);
  const sandingSpeedMmS = Math.round((sandingSpeed[0] / 100) * SANDING_MAX_SPEED);
  const overlapMm = Math.max(0, Math.min(POCKET_MAX_OVERLAP_MM, inverseOverlapping[0] ?? 0));

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
            <span className="text-sm text-gray-600">{robotSpeed[0]}% ({robotSpeedMmS} mm/s, {ROBOT_MAX_ACCEL} mm/s²)</span>
          </div>
          <Slider value={robotSpeed} onValueChange={setRobotSpeed} min={0} max={100} step={1} className="[&_[role=slider]]:bg-blue-500" />
        </div>

        {/* Inverse Overlapping */}
        <div>
          <div className="flex justify-between mb-2">
            <label className="text-sm">Pocket Overlap</label>
            <span className="text-sm text-gray-600">{overlapMm} mm {overlapMm === 0 ? '(no overlap)' : overlapMm >= 100 ? '(~3/4 tool overlap)' : ''}</span>
          </div>
          <Slider value={inverseOverlapping} onValueChange={setInverseOverlapping} min={0} max={100} step={1} className="[&_[role=slider]]:bg-purple-500" />
          <div className="mt-1 text-xs text-gray-500">0 mm = no overlap, 100 mm = ~3/4 tool overlap</div>
        </div>

        {/* Sanding Speed */}
        <div>
          <div className="flex justify-between mb-2">
            <label className="text-sm">Sanding Speed</label>
            <span className="text-sm text-gray-600">{sandingSpeed[0]}% ({sandingSpeedMmS} mm/s, {SANDING_MAX_ACCEL} mm/s²)</span>
          </div>
          <Slider value={sandingSpeed} onValueChange={setSandingSpeed} min={0} max={100} step={1} className="[&_[role=slider]]:bg-pink-500" />
        </div>

      </CardContent>
    </Card>
  );
}
