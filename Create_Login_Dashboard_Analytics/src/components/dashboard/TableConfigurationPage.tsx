import React, { useState } from 'react';
import { Button } from '../ui/button';
import { ArrowLeft } from 'lucide-react';
import { TableConfiguration, type RowConfig } from './TableConfiguration';

interface TableConfigurationPageProps {
  onBack: () => void;
}

export function TableConfigurationPage({ onBack }: TableConfigurationPageProps) {
  const [tableAModel, setTableAModel] = useState('');
  const [tableBModel, setTableBModel] = useState('');
  const [activeTable, setActiveTable] = useState<'A' | 'B' | null>(null);

  const defaultRows: RowConfig[] = [
    { label: 'Frame', selection: '1', force: 4, cycle: 1 },
    { label: 'Pocket ZigZag', selection: '1', force: 5, cycle: 1 },
    { label: '3D', selection: '1', force: 5, cycle: 1 },
    { label: 'Edge Outside', selection: '1', force: 3, cycle: 1 },
    { label: 'Side', selection: '1', force: 3, cycle: 1 },
  ];

  const [tableARows, setTableARows] = useState<RowConfig[]>(defaultRows);
  const [tableBRows, setTableBRows] = useState<RowConfig[]>(
    defaultRows.map((r) => ({ ...r, selection: '0', force: 1, cycle: 1 }))
  );

  return (
    <div className="min-h-screen bg-gradient-to-br from-slate-50 to-slate-100 pb-24">
      {/* Header */}
      <header className="bg-white border-b border-gray-200 shadow-sm">
        <div className="max-w-7xl mx-auto px-4 sm:px-6 lg:px-8">
          <div className="flex justify-between items-center py-4">
            <div className="flex items-center gap-3">
              <Button onClick={onBack} variant="ghost" size="sm">
                <ArrowLeft className="size-4 mr-2" />
                Back to Dashboard
              </Button>
              <h1 className="text-xl font-semibold">Table Configurations</h1>
            </div>
          </div>
        </div>
      </header>

      {/* Main Content */}
      <main className="max-w-7xl mx-auto px-4 sm:px-6 lg:px-8 py-8">
        <TableConfiguration
          tableAModel={tableAModel}
          setTableAModel={setTableAModel}
          tableBModel={tableBModel}
          setTableBModel={setTableBModel}
          tableARows={tableARows}
          setTableARows={setTableARows}
          tableBRows={tableBRows}
          setTableBRows={setTableBRows}
          activeTable={activeTable}
          setActiveTable={setActiveTable}
        />
      </main>
    </div>
  );
}
