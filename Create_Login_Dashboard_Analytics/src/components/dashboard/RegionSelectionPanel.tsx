import React from 'react';
import { Badge } from '../ui/badge';
import { Button } from '../ui/button';

export type TableBCadRegionType =
  | 'pocket_bottom'
  | 'pocket_bevel_3d'
  | 'frame_top'
  | 'outer_edge'
  | 'ignore';

export type TableBCadRegionTool = 'Tool 2' | 'Tool 3' | 'Tool 4';

export type TableBCadPathType =
  | 'zigzag_fill'
  | 'contour_3d'
  | 'frame_non_overlap'
  | 'edge_follow';

export interface TableBCadRegion {
  id: string;
  regionType: TableBCadRegionType;
  meshNames: string[];
  tool: TableBCadRegionTool;
  force: number;
  cycle: number;
  pathType: TableBCadPathType;
}

interface RegionSelectionPanelProps {
  selectedMeshNames: string[];
  regions: TableBCadRegion[];
  isOperating: boolean;
  forceOptions: number[];
  cycleOptions: number[];
  mappingStatus: string;
  onAddRegion: (region: Omit<TableBCadRegion, 'id'>) => void;
  onSaveMapping: () => void;
  onLoadMapping: () => void;
}

const regionTypeOptions: TableBCadRegionType[] = [
  'pocket_bottom',
  'pocket_bevel_3d',
  'frame_top',
  'outer_edge',
  'ignore',
];

const toolOptions: TableBCadRegionTool[] = ['Tool 2', 'Tool 3', 'Tool 4'];

const pathTypeOptions: TableBCadPathType[] = [
  'zigzag_fill',
  'contour_3d',
  'frame_non_overlap',
  'edge_follow',
];

export function RegionSelectionPanel({
  selectedMeshNames,
  regions,
  isOperating,
  forceOptions,
  cycleOptions,
  mappingStatus,
  onAddRegion,
  onSaveMapping,
  onLoadMapping,
}: RegionSelectionPanelProps) {
  const [regionType, setRegionType] = React.useState<TableBCadRegionType>('pocket_bottom');
  const [tool, setTool] = React.useState<TableBCadRegionTool>('Tool 4');
  const [force, setForce] = React.useState(0);
  const [cycle, setCycle] = React.useState(0);
  const [pathType, setPathType] = React.useState<TableBCadPathType>('zigzag_fill');
  const canAddRegion = !isOperating;

  const handleAddRegion = () => {
    if (!canAddRegion) return;
    onAddRegion({
      regionType,
      meshNames: selectedMeshNames.length ? [...selectedMeshNames] : ['manual_test_surface'],
      tool,
      force,
      cycle,
      pathType,
    });
  };

  return (
    <div className="rounded-2xl border border-slate-200 bg-white p-3 shadow-sm">
      <div className="flex flex-wrap items-start justify-between gap-3">
        <div>
          <div className="text-sm font-semibold text-slate-900">Region Classification</div>
          <div className="text-xs text-slate-500">
            Classify the currently selected viewer surfaces before mapping is saved.
          </div>
        </div>
        <Badge variant="outline" className="border-slate-200 bg-slate-50 text-slate-700">
          {regions.length} region(s)
        </Badge>
      </div>

      <div className="mt-3 grid gap-3 sm:grid-cols-2">
        <label className="text-xs font-semibold uppercase tracking-[0.18em] text-slate-500">
          Region Type
          <select
            value={regionType}
            onChange={(event) => setRegionType(event.target.value as TableBCadRegionType)}
            disabled={isOperating}
            className="mt-2 w-full rounded-md border border-slate-300 bg-white px-2 py-2 text-sm font-normal normal-case tracking-normal text-slate-800"
          >
            {regionTypeOptions.map((value) => (
              <option key={value} value={value}>{value}</option>
            ))}
          </select>
        </label>

        <label className="text-xs font-semibold uppercase tracking-[0.18em] text-slate-500">
          Tool
          <select
            value={tool}
            onChange={(event) => setTool(event.target.value as TableBCadRegionTool)}
            disabled={isOperating}
            className="mt-2 w-full rounded-md border border-slate-300 bg-white px-2 py-2 text-sm font-normal normal-case tracking-normal text-slate-800"
          >
            {toolOptions.map((value) => (
              <option key={value} value={value}>{value}</option>
            ))}
          </select>
        </label>

        <label className="text-xs font-semibold uppercase tracking-[0.18em] text-slate-500">
          Force
          <select
            value={force}
            onChange={(event) => setForce(Number(event.target.value))}
            disabled={isOperating}
            className="mt-2 w-full rounded-md border border-slate-300 bg-white px-2 py-2 text-sm font-normal normal-case tracking-normal text-slate-800"
          >
            <option value={0}>-</option>
            {forceOptions.map((value) => (
              <option key={value} value={value}>{value}</option>
            ))}
          </select>
        </label>

        <label className="text-xs font-semibold uppercase tracking-[0.18em] text-slate-500">
          Cycle
          <select
            value={cycle}
            onChange={(event) => setCycle(Number(event.target.value))}
            disabled={isOperating}
            className="mt-2 w-full rounded-md border border-slate-300 bg-white px-2 py-2 text-sm font-normal normal-case tracking-normal text-slate-800"
          >
            <option value={0}>-</option>
            {cycleOptions.map((value) => (
              <option key={value} value={value}>{value}</option>
            ))}
          </select>
        </label>
      </div>

      <label className="mt-3 block text-xs font-semibold uppercase tracking-[0.18em] text-slate-500">
        Path Type
        <select
          value={pathType}
          onChange={(event) => setPathType(event.target.value as TableBCadPathType)}
          disabled={isOperating}
          className="mt-2 w-full rounded-md border border-slate-300 bg-white px-2 py-2 text-sm font-normal normal-case tracking-normal text-slate-800"
        >
          {pathTypeOptions.map((value) => (
            <option key={value} value={value}>{value}</option>
          ))}
        </select>
      </label>

      <div className="mt-3 rounded-xl border border-slate-200 bg-slate-50 p-3 text-xs text-slate-700">
        Selected meshes: {selectedMeshNames.length ? selectedMeshNames.join(', ') : 'none'}
      </div>

      <div className="mt-3 flex flex-wrap gap-2">
        <Button
          type="button"
          onClick={handleAddRegion}
          disabled={!canAddRegion}
          className="bg-slate-900 text-white hover:bg-slate-700 disabled:cursor-not-allowed"
        >
          Add Region
        </Button>
        <Button
          type="button"
          variant="outline"
          onClick={onSaveMapping}
          disabled={isOperating || regions.length === 0}
        >
          Save Mapping
        </Button>
        <Button
          type="button"
          variant="outline"
          onClick={onLoadMapping}
          disabled={isOperating}
        >
          Load Mapping
        </Button>
      </div>
      <div className="mt-3 rounded-xl border border-slate-200 bg-slate-50 p-3 text-xs text-slate-700">
        Mapping status: {mappingStatus}
      </div>

      <div className="mt-4 space-y-2">
        {regions.length > 0 ? (
          regions.map((region) => (
            <div key={region.id} className="rounded-xl border border-slate-200 bg-slate-50 p-3 text-xs text-slate-700">
              <div className="flex flex-wrap items-center justify-between gap-2">
                <div className="font-semibold text-slate-900">{region.id}</div>
                <Badge variant="outline" className="border-cyan-200 bg-cyan-50 text-cyan-800">
                  {region.regionType}
                </Badge>
              </div>
              <div className="mt-2 break-words">
                Meshes: {region.meshNames.join(', ')}
              </div>
              <div className="mt-2 grid gap-2 sm:grid-cols-2">
                <div>Tool: {region.tool}</div>
                <div>Path: {region.pathType}</div>
                <div>Force: {region.force || '-'}</div>
                <div>Cycle: {region.cycle || '-'}</div>
              </div>
            </div>
          ))
        ) : (
          <div className="rounded-xl border border-dashed border-slate-300 bg-slate-50 p-3 text-xs text-slate-600">
            No classified regions yet.
          </div>
        )}
      </div>
    </div>
  );
}


