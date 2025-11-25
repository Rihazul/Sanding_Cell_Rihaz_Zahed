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
  isOperating: boolean;
  setIsOperating: (operating: boolean) => void;
  addActivity: (message: string, type?: 'info' | 'success' | 'warning' | 'error') => void;
}

export function CompactTableConfig({
  tableName,
  model,
  setModel,
  rows,
  setRows,
  isActive,
  isOperating,
  setIsOperating,
  addActivity,
}: CompactTableConfigProps) {
  console.log('CompactTableConfig rendering:', tableName, 'rows:', rows.length, 'addActivity:', !!addActivity);
  
  React.useEffect(() => {
    console.log(`Table ${tableName}: addActivity prop changed:`, !!addActivity);
  }, [addActivity, tableName]);
  
  const handleStartScan = () => {
    console.log('Start Scan clicked for Table', tableName);
    setIsOperating(true);
    addActivity(`Table ${tableName}: Starting scan operation...`, 'info');
    
    // Simulate scan operation
    setTimeout(() => {
      console.log('Scan completed for Table', tableName);
      addActivity(`Table ${tableName}: Scan completed successfully`, 'success');
      setIsOperating(false);
    }, 5000); // 5 second operation
  };
  
  const handleStartTask = () => {
    console.log('Start Task clicked for Table', tableName);
    
    // Check if model is selected
    if (!model || model === '') {
      addActivity(`Table ${tableName}: Cannot start task - Please select a model first`, 'error');
      return;
    }
    
    setIsOperating(true);
    const modelName = model === 'modelA' ? 'Model A' : 
                     model === 'modelB' ? 'Model B' : 
                     model === 'modelC' ? 'Model C' : 
                     model === 'modelD' ? 'Model D' : 
                     model === 'modelE' ? 'Model E' : model;
    addActivity(`Table ${tableName}: Starting task with ${modelName}`, 'info');
    
    // Simulate task operation
    setTimeout(() => {
      console.log('Task completed for Table', tableName);
      addActivity(`Table ${tableName}: Task completed successfully with ${modelName}`, 'success');
      setIsOperating(false);
    }, 8000); // 8 second operation
  };
  
  const handleUpload3DFile = () => {
    console.log('Upload 3D File clicked for Table', tableName);
    setIsOperating(true);
    addActivity(`Table ${tableName}: Uploading 3D file...`, 'info');
    
    // Simulate upload operation
    setTimeout(() => {
      console.log('Upload completed for Table', tableName);
      addActivity(`Table ${tableName}: 3D file uploaded successfully`, 'success');
      setIsOperating(false);
    }, 6000); // 6 second operation
  };
  
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
              disabled={isOperating}
              className="w-full px-3 py-2 border border-gray-300 rounded-md text-sm bg-white focus:ring-2 focus:ring-blue-500 focus:border-transparent disabled:bg-gray-100 disabled:cursor-not-allowed"
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
                <div className="grid items-center gap-4" style={{ gridTemplateColumns: tableName === 'A' ? '140px 1fr auto auto' : '140px 1fr auto auto' }}>
                  <div className="text-sm font-medium text-gray-700 flex items-center gap-1">
                    {row.label}
                    <span className="text-gray-400 text-xs">ⓘ</span>
                  </div>

                  {tableName === 'A' ? (
                    <div className="flex items-center gap-2">
                      <span className="text-xs text-gray-500 whitespace-nowrap">Selection:</span>
                      <div className="flex gap-1">
                        {['1', '2', '3', '4', 'A'].map((opt) => (
                            <button
                              key={opt}
                              disabled={isOperating}
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
                              className={`px-3 py-1 rounded-md border text-xs font-medium transition-colors disabled:opacity-50 disabled:cursor-not-allowed ${
                                row.selection.split(',').includes(opt)
                                  ? 'bg-blue-500 text-white border-blue-500'
                                  : 'bg-white text-gray-700 border-gray-300 hover:border-gray-400'
                              }`}
                          >
                            {opt}
                          </button>
                        ))}
                      </div>
                    </div>
                  ) : (
                    <div></div>
                  )}

                  <div className="flex items-center gap-2 justify-end">
                    <label className="text-xs text-gray-500 font-medium whitespace-nowrap">Force:</label>
                    <select
                      value={row.force}
                      disabled={isOperating}
                      onChange={(e: React.ChangeEvent<HTMLSelectElement>) => {
                        const v = Number(e.target.value);
                        setRows((prev: RowConfig[]) => {
                          const next = [...prev];
                          next[idx] = { ...next[idx], force: v };
                          return next;
                        });
                      }}
                      className="px-2 py-1 border border-gray-300 rounded-md text-sm w-16 focus:ring-2 focus:ring-blue-500 focus:border-transparent disabled:bg-gray-100 disabled:cursor-not-allowed"
                    >
                      <option value={0}>-</option>
                      {Array.from({ length: 25 }, (_, i) => i + 1).map((n) => (
                        <option key={n} value={n}>
                          {n}
                        </option>
                      ))}
                    </select>
                  </div>

                  <div className="flex items-center gap-2 justify-end">
                    <label className="text-xs text-gray-500 font-medium whitespace-nowrap">Cycle:</label>
                    <select
                      value={row.cycle}
                      disabled={isOperating}
                      onChange={(e: React.ChangeEvent<HTMLSelectElement>) => {
                        const v = Number(e.target.value);
                        setRows((prev: RowConfig[]) => {
                          const next = [...prev];
                          next[idx] = { ...next[idx], cycle: v };
                          return next;
                        });
                      }}
                      className="px-2 py-1 border border-gray-300 rounded-md text-sm w-16 focus:ring-2 focus:ring-blue-500 focus:border-transparent disabled:bg-gray-100 disabled:cursor-not-allowed"
                    >
                      <option value={0}>-</option>
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
                <Button 
                  onClick={handleStartScan} 
                  disabled={isOperating}
                  className="bg-pink-500 hover:bg-pink-600 text-white disabled:opacity-50 disabled:cursor-not-allowed"
                >
                  {isOperating ? 'Operating...' : 'Start Scan'}
                </Button>
                <Button 
                  onClick={handleStartTask} 
                  disabled={isOperating}
                  className="bg-blue-500 hover:bg-blue-600 text-white disabled:opacity-50 disabled:cursor-not-allowed"
                >
                  {isOperating ? 'Operating...' : 'Start Task'}
                </Button>
              </>
            ) : (
              <Button 
                onClick={handleUpload3DFile} 
                disabled={isOperating}
                className="bg-pink-500 hover:bg-pink-600 text-white w-full disabled:opacity-50 disabled:cursor-not-allowed"
              >
                {isOperating ? 'Operating...' : 'Upload 3D File'}
              </Button>
            )}
          </div>
        </div>
      </CardContent>
    </Card>
  );
}
