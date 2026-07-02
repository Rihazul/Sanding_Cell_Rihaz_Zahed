import React from 'react';
import { Card, CardContent, CardHeader, CardTitle } from '../ui/card';
import { Button } from '../ui/button';

export type RowConfig = {
  label: string;
  selection: string;
  force: number;
  cycle: number;
};

interface TableConfigurationProps {
  tableAModel: string;
  setTableAModel: (model: string) => void;
  tableBModel: string;
  setTableBModel: (model: string) => void;
  tableARows: RowConfig[];
  setTableARows: React.Dispatch<React.SetStateAction<RowConfig[]>>;
  tableBRows: RowConfig[];
  setTableBRows: React.Dispatch<React.SetStateAction<RowConfig[]>>;
  activeTable: 'A' | 'B' | null;
  setActiveTable: (table: 'A' | 'B' | null) => void;
}

export function TableConfiguration({
  tableAModel,
  setTableAModel,
  tableBModel,
  setTableBModel,
  tableARows,
  setTableARows,
  tableBRows,
  setTableBRows,
  activeTable,
  setActiveTable,
}: TableConfigurationProps) {
  const isTableADisabled = activeTable === 'B';
  const isTableBDisabled = activeTable === 'A';
  return (
    <Card className="shadow-lg border-0">
      <CardHeader className="bg-gradient-to-r from-indigo-50 to-cyan-50">
        <CardTitle>Table Configurations</CardTitle>
      </CardHeader>
      <CardContent className="pt-6">
        <div className="grid grid-cols-1 md:grid-cols-2 gap-8">
          {/* Table A Configuration */}
          <div className={`border-2 rounded-lg p-6 min-w-0 relative transition-all ${
            isTableADisabled ? 'border-gray-300 opacity-60' : 'border-red-300'
          }`}>
            {isTableADisabled && (
              <div className="absolute inset-0 bg-gray-200 bg-opacity-30 rounded-lg z-10 flex items-center justify-center">
                <Button 
                  onClick={() => setActiveTable('A')}
                  className="bg-blue-500 hover:bg-blue-600 text-white shadow-lg"
                  size="lg"
                >
                  Switch to Table A
                </Button>
              </div>
            )}
            <h3 className="text-center mb-4">Table A</h3>

            <div className="bg-gray-50 rounded-lg p-4 mb-4">
              <label className="text-sm text-gray-500 mb-2 flex items-center gap-1">
                Models
                <span className="text-gray-400">ⓘ</span>
              </label>
              <select 
                value={tableAModel} 
                onChange={(e) => {
                  setActiveTable('A');
                  setTableAModel(e.target.value);
                }} 
                disabled={isTableADisabled}
                className="w-full px-3 py-2 border border-gray-300 rounded-md text-sm bg-white disabled:opacity-50 disabled:cursor-not-allowed"
              >
                <option value="">Select a Model</option>
                <option value="modelA">Model A - Shaker</option>
                <option value="modelC">Model B - Moulure Externe</option>
                <option value="modelD">Model C - Moulure Interne</option>
                <option value="modelE">Model D - Moulure Interne et Externe</option>
                <option value="modelF">Model E - Flat</option>
              </select>

              <div className="mt-4 space-y-3">
                {tableARows.map((row: RowConfig, idx: number) => (
                  <div key={row.label} className="flex items-center justify-between gap-3 w-full">
                    <div className="text-gray-600 flex items-center gap-1 min-w-0 flex-1">
                      {row.label}
                      <span className="text-gray-400">ⓘ</span>
                    </div>

                    <div className="flex items-center gap-2">
                      {['1', '2', '3', '4', 'A'].map((opt) => (
                        <button
                          key={opt}
                          disabled={isTableADisabled}
                          onClick={() => {
                            setActiveTable('A');
                            setTableARows((prev: RowConfig[]) => {
                              const next = [...prev];
                              const currentSelections = next[idx].selection.split(',').filter(s => s.trim());
                              let newSelections;
                              
                              if (opt === 'A') {
                                // If 'A' is clicked, toggle all selections
                                newSelections = currentSelections.includes('A') ? [] : ['A'];
                              } else {
                                // Remove 'A' if individual number is selected
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
                          className={`px-2 py-1 rounded-md border text-xs disabled:opacity-50 disabled:cursor-not-allowed ${
                            row.selection.split(',').includes(opt) 
                              ? 'bg-red-500 text-white border-red-500' 
                              : 'bg-white text-gray-700 border-gray-200'
                          }`}
                        >
                          {opt}
                        </button>
                      ))}
                      
                      <label className="text-xs text-gray-500 ml-2">Force</label>
                      <select
                        value={row.force}
                        disabled={isTableADisabled}
                        onChange={(e: React.ChangeEvent<HTMLSelectElement>) => {
                          setActiveTable('A');
                          const v = Number(e.target.value);
                          setTableARows((prev: RowConfig[]) => {
                            const next = [...prev];
                            next[idx] = { ...next[idx], force: v };
                            return next;
                          });
                        }}
                        className="px-2 py-1 border rounded-md text-sm w-16"
                      >
                        {Array.from({ length: 50 }, (_, i) => i + 1).map((n) => (
                          <option key={n} value={n}>
                            {n}
                          </option>
                        ))}
                      </select>
                      
                      <label className="text-xs text-gray-500 ml-2">Cycle</label>
                      <select
                        value={row.cycle}
                        disabled={isTableADisabled}
                        onChange={(e: React.ChangeEvent<HTMLSelectElement>) => {
                          setActiveTable('A');
                          const v = Number(e.target.value);
                          setTableARows((prev: RowConfig[]) => {
                            const next = [...prev];
                            next[idx] = { ...next[idx], cycle: v };
                            return next;
                          });
                        }}
                        className="px-2 py-1 border rounded-md text-sm w-16 disabled:opacity-50 disabled:cursor-not-allowed"
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
              <Button disabled={isTableADisabled} className="bg-pink-400 hover:bg-pink-500">Start Scan</Button>
              <Button disabled={isTableADisabled} className="bg-blue-400 hover:bg-blue-500">Start Task</Button>
            </div>
          </div>

          {/* Table B Configuration */}
          <div className={`border-2 rounded-lg p-6 min-w-0 relative transition-all ${
            isTableBDisabled ? 'border-gray-300 opacity-60' : 'border-red-300'
          }`}>
            {isTableBDisabled && (
              <div className="absolute inset-0 bg-gray-200 bg-opacity-30 rounded-lg z-10 flex items-center justify-center">
                <Button 
                  onClick={() => setActiveTable('B')}
                  className="bg-blue-500 hover:bg-blue-600 text-white shadow-lg"
                  size="lg"
                >
                  Switch to Table B
                </Button>
              </div>
            )}
            <h3 className="text-center mb-4">Table B</h3>

            <div className="bg-gray-50 rounded-lg p-4 mb-4">
              <label className="text-sm text-gray-500 mb-2 flex items-center gap-1">
                Models
                <span className="text-gray-400">ⓘ</span>
              </label>
              <select 
                value={tableBModel} 
                onChange={(e) => {
                  setActiveTable('B');
                  setTableBModel(e.target.value);
                }} 
                disabled={isTableBDisabled}
                className="w-full px-3 py-2 border border-gray-300 rounded-md text-sm bg-white disabled:opacity-50 disabled:cursor-not-allowed"
              >
                <option value="">Select a Model</option>
                <option value="modelA">Model A</option>
                <option value="modelB">Model B</option>
                <option value="modelC">Model C</option>
                <option value="modelD">Model D</option>
                <option value="modelE">Model E</option>
                <option value="modelF">Model F</option>
              </select>

              <div className="mt-4 space-y-3">
                {tableBRows.map((row: RowConfig, idx: number) => (
                  <div key={row.label} className="flex items-center justify-between gap-3 w-full">
                    <div className="text-gray-600 flex items-center gap-1 min-w-0 flex-1">
                      {row.label}
                      <span className="text-gray-400">ⓘ</span>
                    </div>

                    <div className="flex items-center gap-2">
                      <label className="text-xs text-gray-500">Force</label>
                      <select
                        value={row.force}
                        disabled={isTableBDisabled}
                        onChange={(e: React.ChangeEvent<HTMLSelectElement>) => {
                          setActiveTable('B');
                          const v = Number(e.target.value);
                          setTableBRows((prev: RowConfig[]) => {
                            const next = [...prev];
                            next[idx] = { ...next[idx], force: v };
                            return next;
                          });
                        }}
                        className="px-2 py-1 border rounded-md text-sm w-16 disabled:opacity-50 disabled:cursor-not-allowed"
                      >
                        {Array.from({ length: 50 }, (_, i) => i + 1).map((n) => (
                          <option key={n} value={n}>
                            {n}
                          </option>
                        ))}
                      </select>
                      
                      <label className="text-xs text-gray-500 ml-2">Cycle</label>
                      <select
                        value={row.cycle}
                        disabled={isTableBDisabled}
                        onChange={(e: React.ChangeEvent<HTMLSelectElement>) => {
                          setActiveTable('B');
                          const v = Number(e.target.value);
                          setTableBRows((prev: RowConfig[]) => {
                            const next = [...prev];
                            next[idx] = { ...next[idx], cycle: v };
                            return next;
                          });
                        }}
                        className="px-2 py-1 border rounded-md text-sm w-16 disabled:opacity-50 disabled:cursor-not-allowed"
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

            <Button disabled={isTableBDisabled} className="w-full bg-pink-400 hover:bg-pink-500">Upload 3D File</Button>
          </div>
        </div>
      </CardContent>
    </Card>
  );
}
