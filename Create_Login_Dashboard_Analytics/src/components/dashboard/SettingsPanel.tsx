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
  tableAPocketEdgeOffset: number[];
  setTableAPocketEdgeOffset: (offset: number[]) => void;
}

export function SettingsPanel({
  robotSpeed,
  setRobotSpeed,
  inverseOverlapping,
  setInverseOverlapping,
  sandingSpeed,
  setSandingSpeed,
  tableAPocketEdgeOffset,
  setTableAPocketEdgeOffset,
}: SettingsPanelProps) {
  const ROBOT_MAX_SPEED = 400;
  const ROBOT_MAX_ACCEL = 1950;
  const SANDING_MAX_SPEED = 275;
  const SANDING_MAX_ACCEL = 350;
  const POCKET_MAX_OVERLAP_MM = 100;
  const TABLE_A_POCKET_EDGE_OFFSET_MAX_MM = 50;
  const robotSpeedMmS = Math.round((robotSpeed[0] / 100) * ROBOT_MAX_SPEED);
  const sandingSpeedMmS = Math.round((sandingSpeed[0] / 100) * SANDING_MAX_SPEED);
  const overlapMm = Math.max(0, Math.min(POCKET_MAX_OVERLAP_MM, inverseOverlapping[0] ?? 0));
  const tableAPocketEdgeOffsetMm = Math.max(
    0,
    Math.min(TABLE_A_POCKET_EDGE_OFFSET_MAX_MM, tableAPocketEdgeOffset[0] ?? 4)
  );

  const renderSetting = (
    label: string,
    valueLabel: string,
    value: number[],
    onChange: (value: number[]) => void,
    sliderClass: string,
    options?: { min?: number; max?: number; step?: number; help?: string }
  ) => (
    <div className="border-b border-slate-200/70 px-1 pb-1.5 last:border-b-0 last:pb-0">
      <div className="mb-0.5 flex items-center justify-between gap-3">
        <label className="text-[11px] font-semibold text-slate-800">{label}</label>
        <span className="text-[11px] font-semibold text-slate-700 text-right whitespace-nowrap">{valueLabel}</span>
      </div>
      <Slider
        value={value}
        onValueChange={onChange}
        min={options?.min ?? 0}
        max={options?.max ?? 100}
        step={options?.step ?? 1}
        className={sliderClass}
      />
      {options?.help && (
        <div className="mt-0.5 text-[9px] leading-tight text-slate-500">{options.help}</div>
      )}
    </div>
  );

  return (
    <Card className="border border-slate-200 shadow-sm overflow-hidden">
      <CardHeader className="px-4 py-2 bg-gradient-to-r from-slate-50 to-blue-50 border-b border-slate-100">
        <CardTitle className="flex items-center gap-2">
          <span className="text-lg">⚙️</span>
          <span className="text-lg font-extrabold text-slate-900">Settings</span>
        </CardTitle>
      </CardHeader>
      <CardContent className="px-3 py-2 space-y-1.5 bg-white">
        {renderSetting(
          'Robot Speed',
          `${robotSpeed[0]}% (${robotSpeedMmS} mm/s, ${ROBOT_MAX_ACCEL} mm/s²)`,
          robotSpeed,
          setRobotSpeed,
          '[&_[role=slider]]:bg-blue-500'
        )}
        {renderSetting(
          'Table A Pocket Overlap',
          `${overlapMm} mm${overlapMm === 0 ? ' (no overlap)' : overlapMm >= 100 ? ' (~3/4 overlap)' : ''}`,
          inverseOverlapping,
          setInverseOverlapping,
          '[&_[role=slider]]:bg-purple-500',
          { help: '0 mm = no overlap, 100 mm = ~3/4 tool overlap' }
        )}
        {renderSetting(
          'Table A Pocket Edge Offset',
          `${tableAPocketEdgeOffsetMm} mm`,
          tableAPocketEdgeOffset,
          setTableAPocketEdgeOffset,
          '[&_[role=slider]]:bg-emerald-500',
          {
            max: TABLE_A_POCKET_EDGE_OFFSET_MAX_MM,
            help: 'Added to Tool 3 pocket-edge offset. Larger = farther from edge.',
          }
        )}
        {renderSetting(
          'Sanding Speed',
          `${sandingSpeed[0]}% (${sandingSpeedMmS} mm/s, ${SANDING_MAX_ACCEL} mm/s²)`,
          sandingSpeed,
          setSandingSpeed,
          '[&_[role=slider]]:bg-pink-500'
        )}
      </CardContent>
    </Card>
  );
}
