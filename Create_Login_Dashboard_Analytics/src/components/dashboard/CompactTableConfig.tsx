import React from 'react';
import { Card, CardContent, CardHeader, CardTitle } from '../ui/card';
import { Button } from '../ui/button';
import { Badge } from '../ui/badge';

export type RowConfig = {
  label: string;
  selection: string;
  force: number;
  cycle: number;
};

interface CompactTableConfigProps {
  tableName: 'A' | 'B';
  model: string;
  setModel: (model: string) => void;
  rows: RowConfig[];
  setRows: React.Dispatch<React.SetStateAction<RowConfig[]>>;
  isActive: boolean;
}

export function CompactTableConfig({
  tableName,
  model,
  setModel,
  rows,
  setRows,
  isActive,
}: CompactTableConfigProps) {
  console.log('CompactTableConfig rendering:', tableName, 'rows:', rows.length);
  
  return (
    <Card className="shadow-lg border-0">
      <CardHeader className="bg-gradient-to-r from-indigo-50 to-cyan-50">
        <CardTitle className="flex items-center justify-between">
          Table {tableName} Configuration
          <Badge variant={isActive ? 'default' : 'secondary'} className={isActive ? 'bg-green-500' : ''}>
            {isActive ? 'Active' : 'Inactive'}
          </Badge>
        </CardTitle>
      </CardHeader>
      <CardContent className="pt-6">
        <div className="border-2 border-indigo-300 rounded-lg p-5">
          <div className="bg-gray-50 rounded-lg p-4 mb-4">
            <label className="text-sm text-gray-500 mb-2 flex items-center gap-1">
              Models
              <span className="text-gray-400 text-xs">ⓘ</span>
            </label>
            <select
              value={model}
              onChange={(e) => setModel(e.target.value)}
              className="w-full px-3 py-2 border border-gray-300 rounded-md text-sm bg-white focus:ring-2 focus:ring-blue-500 focus:border-transparent"
            >
              <option value="">Select a Model</option>
              <option value="modelA">Model A</option>
              <option value="modelB">Model B</option>
              <option value="modelC">Model C</option>
              <option value="modelD">Model D</option>
              <option value="modelE">Model E</option>
            </select>

            <div className="mt-6 space-y-4">
              {rows.map((row: RowConfig, idx: number) => (
              <div key={row.label} className="bg-white rounded-md p-3 border border-gray-200">
                <div className="flex items-center justify-between gap-4">
                  <div className="text-sm font-medium text-gray-700 flex items-center gap-1 min-w-[120px]">
                    {row.label}
                    <span className="text-gray-400 text-xs">ⓘ</span>
                  </div>

                  {tableName === 'A' && (
                    <div className="flex items-center gap-2">
                      <span className="text-xs text-gray-500">Selection:</span>
                      {['1', '2', '3', '4', 'A'].map((opt) => (
                        <button
                          key={opt}
                          onClick={() => {
                            setRows((prev: RowConfig[]) => {
                              const next = [...prev];
                              const currentSelections = next[idx].selection.split(',').filter(s => s.trim());
                              let newSelections;

                              if (opt === 'A') {
                                newSelections = currentSelections.includes('A') ? [] : ['A'];
                              } else {
                                const filteredSelections = currentSelections.filter(s => s !== 'A');
                                if (filteredSelections.includes(opt)) {
                                  newSelections = filteredSelections.filter(s => s !== opt);
                                } else {
                                  newSelections = [...filteredSelections, opt];
                                }
                              }

                              next[idx] = { ...next[idx], selection: newSelections.join(',') };
                              return next;
                            });
                          }}
                          className={`px-3 py-1 rounded-md border text-xs font-medium transition-colors ${
                            row.selection.split(',').includes(opt)
                              ? 'bg-blue-500 text-white border-blue-500'
                              : 'bg-white text-gray-700 border-gray-300 hover:border-gray-400'
                          }`}
                        >
                          {opt}
                        </button>
                      ))}
                    </div>
                  )}

                  <div className="flex items-center gap-2">
                    <label className="text-xs text-gray-500 font-medium">Force:</label>
                    <select
                      value={row.force}
                      onChange={(e: React.ChangeEvent<HTMLSelectElement>) => {
                        const v = Number(e.target.value);
                        setRows((prev: RowConfig[]) => {
                          const next = [...prev];
                          next[idx] = { ...next[idx], force: v };
                          return next;
                        });
                      }}
                      className="px-2 py-1 border border-gray-300 rounded-md text-sm w-16 focus:ring-2 focus:ring-blue-500 focus:border-transparent"
                    >
                      {Array.from({ length: 25 }, (_, i) => i + 1).map((n) => (
                        <option key={n} value={n}>
                          {n}
                        </option>
                      ))}
                    </select>
                  </div>

                  <div className="flex items-center gap-2">
                    <label className="text-xs text-gray-500 font-medium">Cycle:</label>
                    <select
                      value={row.cycle}
                      onChange={(e: React.ChangeEvent<HTMLSelectElement>) => {
                        const v = Number(e.target.value);
                        setRows((prev: RowConfig[]) => {
                          const next = [...prev];
                          next[idx] = { ...next[idx], cycle: v };
                          return next;
                        });
                      }}
                      className="px-2 py-1 border border-gray-300 rounded-md text-sm w-16 focus:ring-2 focus:ring-blue-500 focus:border-transparent"
                    >
                      {Array.from({ length: 25 }, (_, i) => i + 1).map((n) => (
                        <option key={n} value={n}>
                          {n}
                        </option>
                      ))}
                    </select>
                  </div>
                </div>
              </div>
            ))}
            </div>
          </div>

          <div className={`grid gap-3 mt-4 ${tableName === 'A' ? 'grid-cols-2' : 'grid-cols-1'}`}>
            {tableName === 'A' ? (
              <>
                <Button className="bg-pink-500 hover:bg-pink-600 text-white">
                  Start Scan
                </Button>
                <Button className="bg-blue-500 hover:bg-blue-600 text-white">
                  Start Task
                </Button>
              </>
            ) : (
              <Button className="bg-pink-500 hover:bg-pink-600 text-white w-full">
                Upload 3D File
              </Button>
            )}
          </div>
        </div>
      </CardContent>
    </Card>
  );
}
