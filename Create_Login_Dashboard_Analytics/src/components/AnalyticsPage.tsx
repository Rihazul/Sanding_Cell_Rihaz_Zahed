import { useState, useEffect } from 'react';
import { Button } from './ui/button';
import { Card, CardContent } from './ui/card';
import { LayoutDashboard, Calendar, Clock, ChevronRight, Activity } from 'lucide-react';

interface LogEntry {
  id: number;
  timestamp: string;
  message: string;
  type: 'info' | 'success' | 'warning' | 'error';
}

interface AnalyticsPageProps {
  onNavigateToDashboard: () => void;
  liveActivities: LogEntry[];
}

interface DailyLog {
  date: string;
  displayDate: string;
  entries: LogEntry[];
  isLive?: boolean;
}

export function AnalyticsPage({ onNavigateToDashboard, liveActivities }: AnalyticsPageProps) {
  const [selectedLog, setSelectedLog] = useState<DailyLog | null>(null);

  // Prevent body scroll when modal is open
  useEffect(() => {
    if (selectedLog) {
      document.body.style.overflow = 'hidden';
    } else {
      document.body.style.overflow = '';
    }
    return () => {
      document.body.style.overflow = '';
    };
  }, [selectedLog]);

  // Get today's date formatted
  const today = new Date();
  const todayFormatted = today.toLocaleDateString('en-US', { 
    year: 'numeric', 
    month: 'long', 
    day: 'numeric' 
  });
  const todayKey = today.toISOString().split('T')[0];

  // Create live session log from current activities
  const liveSessionLog: DailyLog = {
    date: todayKey,
    displayDate: `${todayFormatted} (Live Session)`,
    entries: liveActivities,
    isLive: true,
  };

  // Sample historical logs data - in real app, this would come from backend/storage
  const historicalLogs: DailyLog[] = [
    liveSessionLog,
    {
      date: '2025-12-02',
      displayDate: 'December 2, 2025',
      entries: [
        { id: 1, timestamp: '09:15:23', message: 'Robot system initialized', type: 'success' },
        { id: 2, timestamp: '09:16:45', message: 'Robot Power ENABLED', type: 'success' },
        { id: 3, timestamp: '09:18:12', message: 'Homing sequence initiated...', type: 'warning' },
        { id: 4, timestamp: '09:18:45', message: 'Homing completed successfully', type: 'success' },
        { id: 5, timestamp: '09:25:30', message: 'Table A: Starting task with Model A', type: 'info' },
        { id: 6, timestamp: '09:33:45', message: 'Table A: Task completed successfully with Model A', type: 'success' },
        { id: 7, timestamp: '10:15:00', message: 'Tool 1 picked up', type: 'info' },
        { id: 8, timestamp: '10:45:22', message: 'Laser turned ON', type: 'warning' },
        { id: 9, timestamp: '11:30:15', message: 'Table B: Starting task with Model C', type: 'info' },
        { id: 10, timestamp: '11:38:45', message: 'Table B: Task completed successfully with Model C', type: 'success' },
      ]
    },
    {
      date: '2025-12-01',
      displayDate: 'December 1, 2025',
      entries: [
        { id: 1, timestamp: '08:30:00', message: 'Robot system initialized', type: 'success' },
        { id: 2, timestamp: '08:31:15', message: 'Robot Power ENABLED', type: 'success' },
        { id: 3, timestamp: '08:35:00', message: 'Stopper A moved UP', type: 'info' },
        { id: 4, timestamp: '08:40:22', message: 'Table A OPENED', type: 'info' },
        { id: 5, timestamp: '09:00:00', message: 'Table A: Starting scan operation...', type: 'info' },
        { id: 6, timestamp: '09:05:30', message: 'Table A: Scan completed successfully', type: 'success' },
        { id: 7, timestamp: '09:10:45', message: 'Table A: Starting task with Model B', type: 'info' },
        { id: 8, timestamp: '09:18:30', message: 'Table A: Task completed successfully with Model B', type: 'success' },
        { id: 9, timestamp: '14:22:10', message: 'Emergency stop activated!', type: 'error' },
        { id: 10, timestamp: '14:25:00', message: 'Robot Power DISABLED', type: 'warning' },
        { id: 11, timestamp: '14:30:00', message: 'Robot Power ENABLED', type: 'success' },
        { id: 12, timestamp: '14:32:00', message: 'Homing sequence initiated...', type: 'warning' },
        { id: 13, timestamp: '14:32:30', message: 'Homing completed successfully', type: 'success' },
      ]
    },
    {
      date: '2025-11-30',
      displayDate: 'November 30, 2025',
      entries: [
        { id: 1, timestamp: '10:00:00', message: 'Robot system initialized', type: 'success' },
        { id: 2, timestamp: '10:01:30', message: 'Robot Power ENABLED', type: 'success' },
        { id: 3, timestamp: '10:05:00', message: 'Homing sequence initiated...', type: 'warning' },
        { id: 4, timestamp: '10:05:30', message: 'Homing completed successfully', type: 'success' },
        { id: 5, timestamp: '10:15:00', message: 'Tool 2 picked up', type: 'info' },
        { id: 6, timestamp: '10:20:00', message: 'Tool 3 picked up', type: 'info' },
        { id: 7, timestamp: '11:00:00', message: 'Table B: Starting task with Model D', type: 'info' },
        { id: 8, timestamp: '11:08:30', message: 'Table B: Task completed successfully with Model D', type: 'success' },
      ]
    },
    {
      date: '2025-11-29',
      displayDate: 'November 29, 2025',
      entries: [
        { id: 1, timestamp: '09:00:00', message: 'Robot system initialized', type: 'success' },
        { id: 2, timestamp: '09:02:00', message: 'Robot Power ENABLED', type: 'success' },
        { id: 3, timestamp: '09:10:00', message: 'Table A: Cannot start task - Please select a model first', type: 'error' },
        { id: 4, timestamp: '09:12:00', message: 'Table A: Starting task with Model E', type: 'info' },
        { id: 5, timestamp: '09:20:30', message: 'Table A: Task completed successfully with Model E', type: 'success' },
        { id: 6, timestamp: '12:00:00', message: 'Laser turned ON', type: 'warning' },
        { id: 7, timestamp: '12:30:00', message: 'Laser turned OFF', type: 'info' },
      ]
    },
    {
      date: '2025-11-28',
      displayDate: 'November 28, 2025',
      entries: [
        { id: 1, timestamp: '08:00:00', message: 'Robot system initialized', type: 'success' },
        { id: 2, timestamp: '08:05:00', message: 'Robot Power ENABLED', type: 'success' },
        { id: 3, timestamp: '08:10:00', message: 'Stopper A moved UP', type: 'info' },
        { id: 4, timestamp: '08:12:00', message: 'Stopper B moved UP', type: 'info' },
        { id: 5, timestamp: '08:30:00', message: 'Table A: Starting scan operation...', type: 'info' },
        { id: 6, timestamp: '08:35:00', message: 'Table A: Scan completed successfully', type: 'success' },
      ]
    },
  ];

  const getEntryTypeColor = (type: LogEntry['type']) => {
    switch (type) {
      case 'success': return 'text-green-600';
      case 'warning': return 'text-yellow-600';
      case 'error': return 'text-red-600';
      default: return 'text-blue-600';
    }
  };

  const getEntryTypeBadge = (type: LogEntry['type']) => {
    const colors = {
      success: 'bg-green-100 text-green-800',
      warning: 'bg-yellow-100 text-yellow-800',
      error: 'bg-red-100 text-red-800',
      info: 'bg-blue-100 text-blue-800',
    };
    return colors[type];
  };

  const getLogSummary = (entries: LogEntry[]) => {
    const errors = entries.filter(e => e.type === 'error').length;
    const warnings = entries.filter(e => e.type === 'warning').length;
    const success = entries.filter(e => e.type === 'success').length;
    return { errors, warnings, success, total: entries.length };
  };

  return (
    <div className="min-h-screen bg-gradient-to-br from-blue-100 via-purple-100 to-pink-100 pb-24">
      {/* Header */}
      <header className="bg-white border-b border-gray-200 shadow-sm">
        <div className="max-w-7xl mx-auto px-4 sm:px-6 lg:px-8">
          <div className="flex justify-between items-center py-4">
            <h1 className="text-2xl font-semibold text-gray-800">Activity History</h1>
            <div className="flex gap-3">
              <Button onClick={onNavigateToDashboard} variant="outline">
                <LayoutDashboard className="size-4 mr-2" />
                Dashboard
              </Button>
            </div>
          </div>
        </div>
      </header>

      {/* Main Content */}
      <main className="max-w-7xl mx-auto px-4 sm:px-6 lg:px-8 py-8">
        <div className="mb-8">
          <h2 className="text-xl font-semibold text-gray-800">Log History</h2>
          <p className="text-gray-600">Browse past activity logs to track platform operations</p>
        </div>

        {/* Logs List */}
        <div className="space-y-4">
          {historicalLogs.map((log) => {
            const summary = getLogSummary(log.entries);
            return (
              <Card 
                key={log.date + (log.isLive ? '-live' : '')} 
                className={`cursor-pointer hover:shadow-lg transition-all duration-200 border-0 ${log.isLive ? 'ring-2 ring-green-400' : ''}`}
                onClick={() => setSelectedLog(log)}
              >
                <CardContent className="p-6">
                  <div className="flex items-center justify-between">
                    <div className="flex items-center gap-4">
                      <div className={`${log.isLive ? 'bg-green-100' : 'bg-blue-100'} p-3 rounded-full`}>
                        {log.isLive ? (
                          <Activity className="size-6 text-green-600" />
                        ) : (
                          <Calendar className="size-6 text-blue-600" />
                        )}
                      </div>
                      <div>
                        <div className="flex items-center gap-2">
                          <h3 className="text-lg font-semibold text-gray-800">{log.displayDate}</h3>
                          {log.isLive && (
                            <span className="px-2 py-0.5 bg-green-500 text-white text-xs font-medium rounded-full animate-pulse">
                              LIVE
                            </span>
                          )}
                        </div>
                        <div className="flex items-center gap-2 mt-1 text-sm text-gray-500">
                          <Clock className="size-4" />
                          <span>{summary.total} activities recorded</span>
                        </div>
                      </div>
                    </div>
                    
                    <div className="flex items-center gap-4">
                      {/* Summary badges */}
                      <div className="flex gap-2">
                        {summary.errors > 0 && (
                          <span className="px-2 py-1 bg-red-100 text-red-700 text-xs font-medium rounded-full">
                            {summary.errors} errors
                          </span>
                        )}
                        {summary.warnings > 0 && (
                          <span className="px-2 py-1 bg-yellow-100 text-yellow-700 text-xs font-medium rounded-full">
                            {summary.warnings} warnings
                          </span>
                        )}
                        <span className="px-2 py-1 bg-green-100 text-green-700 text-xs font-medium rounded-full">
                          {summary.success} success
                        </span>
                      </div>
                      <ChevronRight className="size-5 text-gray-400" />
                    </div>
                  </div>
                </CardContent>
              </Card>
            );
          })}
        </div>
      </main>

      {/* Log Detail Modal - using same styling as Activity Log popup */}
      {selectedLog && (
        <div 
          className="activity-log-backdrop"
          onClick={() => setSelectedLog(null)}
        >
          <div 
            className="activity-log-modal"
            onClick={(e) => e.stopPropagation()}
          >
            {/* Modal Header */}
            <div className="activity-log-modal-header">
              <h2>Activity Log - {selectedLog.displayDate}</h2>
              <button
                type="button"
                className="activity-log-close-btn"
                onClick={() => setSelectedLog(null)}
              >
                ✕
              </button>
            </div>

            {/* Modal Body - Scrollable */}
            <div className="activity-log-modal-body">
              {selectedLog.entries.map((entry) => (
                <div key={entry.id} className="activity-log-line">
                  <span className="text-gray-500 font-mono">[{entry.timestamp}]</span>{' '}
                  <span className={getEntryTypeColor(entry.type)}>{entry.message}</span>
                </div>
              ))}
            </div>

            {/* Modal Footer */}
            <div className="mt-4 pt-4 border-t border-gray-200 flex justify-between items-center">
              <div className="flex gap-3">
                <span className={`px-3 py-1 rounded-full text-xs font-medium ${getEntryTypeBadge('success')}`}>
                  {selectedLog.entries.filter(e => e.type === 'success').length} Success
                </span>
                <span className={`px-3 py-1 rounded-full text-xs font-medium ${getEntryTypeBadge('warning')}`}>
                  {selectedLog.entries.filter(e => e.type === 'warning').length} Warnings
                </span>
                <span className={`px-3 py-1 rounded-full text-xs font-medium ${getEntryTypeBadge('error')}`}>
                  {selectedLog.entries.filter(e => e.type === 'error').length} Errors
                </span>
                <span className={`px-3 py-1 rounded-full text-xs font-medium ${getEntryTypeBadge('info')}`}>
                  {selectedLog.entries.filter(e => e.type === 'info').length} Info
                </span>
              </div>
              <Button onClick={() => setSelectedLog(null)} variant="outline">
                Close
              </Button>
            </div>
          </div>
        </div>
      )}
    </div>
  );
}
