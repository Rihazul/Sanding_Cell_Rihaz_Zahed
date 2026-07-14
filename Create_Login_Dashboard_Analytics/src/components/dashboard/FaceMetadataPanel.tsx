import React from 'react';
import { Badge } from '../ui/badge';

export type TableBFaceTypeHint =
  | 'frame_top'
  | 'pocket_bottom'
  | 'pocket_bevel_3d'
  | 'vertical_edge'
  | 'unknown';

export interface TableBFaceMetadataFace {
  face_id: string;
  normal: number[];
  center: number[];
  area: number;
  z_level: number;
  type_hint: TableBFaceTypeHint | string;
}

export interface TableBFaceMetadata {
  job_id: string;
  source_file?: string;
  created_at?: string;
  status?: string;
  extractor?: string;
  todo?: string;
  message?: string;
  faces: TableBFaceMetadataFace[];
}

interface FaceMetadataPanelProps {
  metadata: TableBFaceMetadata | null;
  metadataStatus: string;
}

const typeHintTones: Record<TableBFaceTypeHint, string> = {
  frame_top: 'border-emerald-200 bg-emerald-50 text-emerald-800',
  pocket_bottom: 'border-cyan-200 bg-cyan-50 text-cyan-800',
  pocket_bevel_3d: 'border-amber-200 bg-amber-50 text-amber-800',
  vertical_edge: 'border-slate-200 bg-slate-50 text-slate-700',
  unknown: 'border-rose-200 bg-rose-50 text-rose-800',
};

export function FaceMetadataPanel({ metadata, metadataStatus }: FaceMetadataPanelProps) {
  const faces = metadata?.faces || [];
  const counts = faces.reduce<Record<string, number>>((acc, face) => {
    acc[face.type_hint] = (acc[face.type_hint] || 0) + 1;
    return acc;
  }, {});

  return (
    <div className="rounded-2xl border border-slate-200 bg-white p-3 shadow-sm">
      <div className="flex flex-wrap items-start justify-between gap-3">
        <div>
          <div className="text-sm font-semibold text-slate-900">Face Metadata</div>
          <div className="text-xs text-slate-500">
            Read-only CAD face hints loaded from the converted job.
          </div>
        </div>
        <Badge variant="outline" className="border-slate-200 bg-slate-50 text-slate-700">
          {metadataStatus}
        </Badge>
      </div>

      {metadata?.todo && (
        <div className="mt-3 rounded-xl border border-amber-200 bg-amber-50 p-3 text-xs text-amber-900">
          {metadata.todo}
        </div>
      )}

      {metadata?.message && (
        <div className="mt-3 rounded-xl border border-slate-200 bg-slate-50 p-3 text-xs text-slate-700">
          {metadata.message}
        </div>
      )}

      <div className="mt-3 grid gap-2 sm:grid-cols-2">
        <div className="rounded-xl border border-slate-200 bg-slate-50 p-3 text-xs text-slate-700">
          Faces: <span className="font-semibold text-slate-900">{faces.length}</span>
        </div>
        <div className="rounded-xl border border-slate-200 bg-slate-50 p-3 text-xs text-slate-700">
          Extractor: <span className="font-semibold text-slate-900">{metadata?.extractor || metadata?.status || '-'}</span>
        </div>
      </div>

      {Object.keys(counts).length > 0 && (
        <div className="mt-3 flex flex-wrap gap-2">
          {Object.entries(counts).map(([typeHint, count]) => (
            <Badge key={typeHint} variant="outline" className={getTypeHintTone(typeHint)}>
              {typeHint}: {count}
            </Badge>
          ))}
        </div>
      )}

      <div className="mt-3 max-h-64 space-y-2 overflow-auto pr-1">
        {faces.length > 0 ? (
          faces.slice(0, 20).map((face) => (
            <div key={face.face_id} className="rounded-xl border border-slate-200 bg-slate-50 p-3 text-xs text-slate-700">
              <div className="flex flex-wrap items-center justify-between gap-2">
                <span className="font-semibold text-slate-900">{face.face_id}</span>
                <Badge variant="outline" className={getTypeHintTone(face.type_hint)}>
                  {face.type_hint}
                </Badge>
              </div>
              <div className="mt-2 grid gap-1 sm:grid-cols-2">
                <div>Area: {Number(face.area || 0).toFixed(2)}</div>
                <div>Z: {Number(face.z_level || 0).toFixed(2)}</div>
                <div className="break-words">Center: {(face.center || []).map((value) => Number(value).toFixed(2)).join(', ')}</div>
                <div className="break-words">Normal: {(face.normal || []).map((value) => Number(value).toFixed(2)).join(', ')}</div>
              </div>
            </div>
          ))
        ) : (
          <div className="rounded-xl border border-dashed border-slate-300 bg-slate-50 p-3 text-xs text-slate-600">
            No face records loaded yet.
          </div>
        )}
      </div>
    </div>
  );
}

