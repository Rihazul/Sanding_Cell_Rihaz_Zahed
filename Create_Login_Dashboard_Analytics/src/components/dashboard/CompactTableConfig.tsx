import React from 'react';
import { Card, CardContent, CardHeader, CardTitle } from '../ui/card';
import { Button } from '../ui/button';
import { Badge } from '../ui/badge';
import { startTableAProcess, startTableBProcess, performAction } from '../../services/api';

export type RowConfig = {
  label: string;
  selection: string;
  force: number;
  cycle: number;
};

export type DoorConfig = {
  doorNumber: number;
  model: string;
  rows: RowConfig[];
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
  robotSpeed: number[];
  sandingSpeed: number[];
  inverseOverlapping: number[];
  doorConfigs?: DoorConfig[];
  setDoorConfigs?: React.Dispatch<React.SetStateAction<DoorConfig[]>>;
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
  robotSpeed,
  sandingSpeed,
  inverseOverlapping,
  doorConfigs,
  setDoorConfigs,
}: CompactTableConfigProps) {
  console.log('CompactTableConfig rendering:', tableName, 'rows:', rows.length, 'addActivity:', !!addActivity);
  
  const [selectedDoor, setSelectedDoor] = React.useState<number>(1);
  const [scanCompleted, setScanCompleted] = React.useState<boolean>(false);
  
  React.useEffect(() => {
    console.log(`Table ${tableName}: addActivity prop changed:`, !!addActivity);
  }, [addActivity, tableName]);
  
  const handleStartScan = async () => {
    console.log('Start Scan clicked for Table', tableName);
    setIsOperating(true);
    addActivity(`Table ${tableName}: Starting scan operation...`, 'info');
    
    try {
      // Simulate scan operation for testing (3 seconds)
      await new Promise(resolve => setTimeout(resolve, 3000));
      // await performAction('scan');
      addActivity(`Table ${tableName}: Scan completed successfully`, 'success');
      setScanCompleted(true);
    } catch (error) {
      addActivity(`Table ${tableName}: Scan failed - ${error}`, 'error');
    } finally {
      setIsOperating(false);
    }
  };
  
  const handleStartTask = async () => {
    console.log('Start Task clicked for Table', tableName);
    
    setIsOperating(true);
    
    // For Table A, process all doors (configured or not)
    if (tableName === 'A' && doorConfigs) {
      const configuredDoors = doorConfigs.filter(d => d.model && d.model !== '');
      const totalDoors = doorConfigs.length;
      
      addActivity(`Table ${tableName}: Starting task for all doors (${configuredDoors.length} configured, ${totalDoors - configuredDoors.length} unconfigured)...`, 'info');
      
      try {
        // Process all doors
        for (const doorConfig of doorConfigs) {
          if (doorConfig.model && doorConfig.model !== '') {
            const modelName = doorConfig.model === 'modelA' ? 'Model A' : 
                             doorConfig.model === 'modelB' ? 'Model B' : 
                             doorConfig.model === 'modelC' ? 'Model C' : 
                             doorConfig.model === 'modelD' ? 'Model D' : 
                             doorConfig.model === 'modelE' ? 'Model E' : doorConfig.model;
            
            addActivity(`Table ${tableName}: Processing Door ${doorConfig.doorNumber} with ${modelName}...`, 'info');
            
            // Simulate task operation for testing (3 seconds per configured door)
            await new Promise(resolve => setTimeout(resolve, 3000));
            
            // // Build payload from door's rows
            // const taskData: any = {
            //   door: doorConfig.doorNumber,
            //   model: doorConfig.model,
            //   frame: { cycle: doorConfig.rows[0].cycle, force: doorConfig.rows[0].force },
            //   pocketzigzag: { cycle: doorConfig.rows[1].cycle, force: doorConfig.rows[1].force },
            //   '3D': { cycle: doorConfig.rows[2].cycle, force: doorConfig.rows[2].force },
            //   edgeOutside: { cycle: doorConfig.rows[3].cycle, force: doorConfig.rows[3].force },
            //   side: { cycle: doorConfig.rows[4].cycle, force: doorConfig.rows[4].force },
            // };
            // await startTableAProcess(taskData as any);
            
            addActivity(`Table ${tableName}: Door ${doorConfig.doorNumber} completed with ${modelName}`, 'success');
          } else {
            addActivity(`Table ${tableName}: Skipping Door ${doorConfig.doorNumber} (not configured)`, 'info');
          }
        }
        
        addActivity(`Table ${tableName}: Task completed successfully`, 'success');
        // Reset scan so it can be done again
        setScanCompleted(false);
      } catch (error) {
        addActivity(`Table ${tableName}: Task failed - ${error}`, 'error');
      } finally {
        setIsOperating(false);
      }
    } else {
      // Table B logic (single model)
      const modelName = model === 'modelA' ? 'Model A' : 
                       model === 'modelB' ? 'Model B' : 
                       model === 'modelC' ? 'Model C' : 
                       model === 'modelD' ? 'Model D' : 
                       model === 'modelE' ? 'Model E' : model;
      
      addActivity(`Table ${tableName}: Starting task with ${modelName}`, 'info');
      
      try {
        // Simulate task operation for testing (5 seconds)
        await new Promise(resolve => setTimeout(resolve, 5000));
        
        // // Build payload from rows
        // const taskData: any = {
        //   model,
        //   frame: { cycle: rows[0].cycle, force: rows[0].force },
        //   pocketzigzag: { cycle: rows[1].cycle, force: rows[1].force },
        //   '3D': { cycle: rows[2].cycle, force: rows[2].force },
        //   edgeOutside: { cycle: rows[3].cycle, force: rows[3].force },
        //   side: { cycle: rows[4].cycle, force: rows[4].force },
        // };
        // const tableBData = {
        //   ...taskData,
        //   robotSpeed: (robotSpeed[0] / 100).toFixed(2),
        //   sandingSpeed: (sandingSpeed[0] / 100).toFixed(2),
        //   inverseOverlapping: inverseOverlapping[0],
        // };
        // await startTableBProcess(tableBData as any);
        
        addActivity(`Table ${tableName}: Task completed successfully with ${modelName}`, 'success');
        // Reset scan so it can be done again
        setScanCompleted(false);
      } catch (error) {
        addActivity(`Table ${tableName}: Task failed - ${error}`, 'error');
      } finally {
        setIsOperating(false);
      }
    }
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
  
  // Get current door configuration
  const currentDoorConfig = doorConfigs?.find(d => d.doorNumber === selectedDoor);
  const currentModel = tableName === 'A' && doorConfigs ? (currentDoorConfig?.model || '') : model;
  const currentRows = tableName === 'A' && doorConfigs ? (currentDoorConfig?.rows || rows) : rows;

  const handleModelChange = (newModel: string) => {
    if (tableName === 'A' && doorConfigs && setDoorConfigs) {
      setDoorConfigs(prev => {
        const updated = [...prev];
        const doorIndex = updated.findIndex(d => d.doorNumber === selectedDoor);
        if (doorIndex >= 0) {
          updated[doorIndex] = { ...updated[doorIndex], model: newModel };
        }
        return updated;
      });
    } else {
      setModel(newModel);
    }
  };

  const handleRowChange = (idx: number, field: 'selection' | 'force' | 'cycle', value: any) => {
    if (tableName === 'A' && doorConfigs && setDoorConfigs) {
      setDoorConfigs(prev => {
        const updated = [...prev];
        const doorIndex = updated.findIndex(d => d.doorNumber === selectedDoor);
        if (doorIndex >= 0) {
          const newRows = [...updated[doorIndex].rows];
          newRows[idx] = { ...newRows[idx], [field]: value };
          updated[doorIndex] = { ...updated[doorIndex], rows: newRows };
        }
        return updated;
      });
    } else {
      setRows((prev: RowConfig[]) => {
        const next = [...prev];
        next[idx] = { ...next[idx], [field]: value };
        return next;
      });
    }
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
          {tableName === 'A' && doorConfigs ? (
            <>
              {/* Door Selection Tabs */}
              <div className="mb-4">
                <div className="flex gap-3 border-b-2 border-gray-300">
                  {[1, 2, 3, 4].map((doorNum) => {
                    const doorConfig = doorConfigs.find(d => d.doorNumber === doorNum);
                    const hasModel = doorConfig?.model && doorConfig.model !== '';
                    return (
                      <button
                        key={doorNum}
                        type="button"
                        onClick={(e) => {
                          e.preventDefault();
                          setSelectedDoor(doorNum);
                        }}
                        disabled={isOperating}
                        className={`px-6 py-3 text-base font-semibold transition-colors relative disabled:opacity-50 disabled:cursor-not-allowed rounded-t-lg ${
                          selectedDoor === doorNum
                            ? 'text-blue-600 bg-blue-50 border-b-3 border-blue-600'
                            : 'text-gray-600 hover:text-gray-800 hover:bg-gray-50'
                        }`}
                      >
                        Door {doorNum}
                        {hasModel && (
                          <span className="absolute top-2 right-2 w-2.5 h-2.5 bg-green-500 rounded-full ring-2 ring-white"></span>
                        )}
                      </button>
                    );
                  })}
                </div>
              </div>

              <div className="bg-gray-50 rounded-lg p-4 mb-4">
                <label className="text-sm text-gray-500 mb-2 flex items-center gap-1">
                  Model for Door {selectedDoor}
                  <span className="text-gray-400 text-xs">ⓘ</span>
                </label>
                <select
                  value={currentModel}
                  onChange={(e) => handleModelChange(e.target.value)}
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

                <div className="mt-6 space-y-3">
                  {currentRows.map((row: RowConfig, idx: number) => (
                    <div key={row.label} className="bg-white rounded-md p-3 border border-gray-200">
                      <div className="flex items-center justify-between gap-4">
                        <div className="text-sm font-medium text-gray-700 flex items-center gap-1">
                          {row.label}
                          <span className="text-gray-400 text-xs">ⓘ</span>
                        </div>

                        <div className="flex items-center gap-3">
                          <div className="flex items-center gap-1.5">
                            <label className="text-xs text-gray-500 font-medium whitespace-nowrap">Force:</label>
                            <select
                              value={row.force}
                              disabled={isOperating}
                              onChange={(e: React.ChangeEvent<HTMLSelectElement>) => {
                                handleRowChange(idx, 'force', Number(e.target.value));
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

                          <div className="flex items-center gap-1.5">
                            <label className="text-xs text-gray-500 font-medium whitespace-nowrap">Cycle:</label>
                            <select
                              value={row.cycle}
                              disabled={isOperating}
                              onChange={(e: React.ChangeEvent<HTMLSelectElement>) => {
                                handleRowChange(idx, 'cycle', Number(e.target.value));
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
                    </div>
                  ))}
                </div>
              </div>
            </>
          ) : (
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

              <div className="mt-6 space-y-3">
                {rows.map((row: RowConfig, idx: number) => (
                  <div key={row.label} className="bg-white rounded-md p-3 border border-gray-200">
                    <div className="flex items-center justify-between gap-4">
                      <div className="text-sm font-medium text-gray-700 flex items-center gap-1">
                        {row.label}
                        <span className="text-gray-400 text-xs">ⓘ</span>
                      </div>

                      <div className="flex items-center gap-3">
                        <div className="flex items-center gap-1.5">
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

                        <div className="flex items-center gap-1.5">
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
                  </div>
                ))}
              </div>
            </div>
          )}

          <div className={`grid gap-3 mt-4 ${tableName === 'A' ? 'grid-cols-2' : 'grid-cols-1'}`}>
            {tableName === 'A' ? (
              <>
                <Button 
                  onClick={handleStartScan} 
                  disabled={isOperating || scanCompleted}
                  className="bg-pink-500 hover:bg-pink-600 text-white disabled:opacity-50 disabled:cursor-not-allowed"
                >
                  {scanCompleted ? 'Scan Completed' : isOperating ? 'Operating...' : 'Start Scan'}
                </Button>
                <Button 
                  onClick={handleStartTask} 
                  disabled={isOperating || !scanCompleted}
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
