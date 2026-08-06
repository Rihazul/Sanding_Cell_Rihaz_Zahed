import { useState, useEffect } from 'react';
import { Button } from './ui/button';
import { Card, CardContent } from './ui/card';
import { LayoutDashboard, Calendar, Clock, ChevronRight, Activity } from 'lucide-react';
import { getLogsHistory, getLogsForDate, type HistoricalLogDay } from '../services/api';

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
  /** From summary mode: counts for the list row, without the entry bodies. */
  entryCount?: number;
  counts?: { total: number; success: number; warning: number; error: number; info: number };
}

export function AnalyticsPage({ onNavigateToDashboard, liveActivities }: AnalyticsPageProps) {
  const [selectedLog, setSelectedLog] = useState<DailyLog | null>(null);
  const [backendLogs, setBackendLogs] = useState<HistoricalLogDay[]>([]);
  const [historyError, setHistoryError] = useState<string | null>(null);
  const [loadingEntries, setLoadingEntries] = useState(false);

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

  useEffect(() => {
    let active = true;

    const fetchHistory = async () => {
      try {
        const res = await getLogsHistory(21, 60000, true);
        if (!active) return;
        setBackendLogs(Array.isArray(res?.logs) ? res.logs : []);
        setHistoryError(null);
      } catch (err) {
        if (!active) return;
        setBackendLogs([]);
        setHistoryError(err instanceof Error ? err.message : 'Failed to load log history');
      }
    };

    fetchHistory();
    const timer = window.setInterval(fetchHistory, 5000);

    return () => {
      active = false;
      window.clearInterval(timer);
    };
  }, []);

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

  const historicalLogs: DailyLog[] = [liveSessionLog, ...backendLogs.filter((d) => d.date !== todayKey).map((day) => ({
    date: day.date,
    displayDate: day.displayDate,
    entries: day.entries,
    entryCount: day.entryCount,
    counts: day.counts,
  }))];

  // The list rows come from summary mode, so entries[] is empty and the counts arrive
  // precomputed. Fall back to counting entries for the live session, which is local.
  const openLog = async (log: DailyLog) => {
    if (log.isLive || (log.entries && log.entries.length > 0)) {
      setSelectedLog(log);
      return;
    }
    setSelectedLog({ ...log, entries: [] });
    setLoadingEntries(true);
    try {
      const res = await getLogsForDate(log.date);
      const day = Array.isArray(res?.logs)
        ? res.logs.find((d: HistoricalLogDay) => d.date === log.date)
        : undefined;
      setSelectedLog({ ...log, entries: (day?.entries ?? []) as LogEntry[] });
    } catch (err) {
      setHistoryError(err instanceof Error ? err.message : 'Failed to load entries for that day');
      setSelectedLog({ ...log, entries: [] });
    } finally {
      setLoadingEntries(false);
    }
  };

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

  // Prefer the counts the backend already computed (summary mode ships no entry
  // bodies); only fall back to counting locally for the live session.
  const getLogSummary = (log: DailyLog) => {
    if (log.counts) {
      return {
        errors: log.counts.error,
        warnings: log.counts.warning,
        success: log.counts.success,
        total: log.counts.total,
      };
    }
    const entries = log.entries || [];
    return {
      errors: entries.filter(e => e.type === 'error').length,
      warnings: entries.filter(e => e.type === 'warning').length,
      success: entries.filter(e => e.type === 'success').length,
      total: log.entryCount ?? entries.length,
    };
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
          {historyError && (
            <p className="text-sm text-red-600 mt-2">History source unavailable: {historyError}</p>
          )}
        </div>

        {/* Logs List */}
        <div className="space-y-4">
          {historicalLogs.map((log) => {
            const summary = getLogSummary(log);
            return (
              <Card 
                key={log.date + (log.isLive ? '-live' : '')} 
                className={`cursor-pointer hover:shadow-lg transition-all duration-200 border-0 ${log.isLive ? 'ring-2 ring-green-400' : ''}`}
                onClick={() => { void openLog(log); }}
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
              {loadingEntries && selectedLog.entries.length === 0 && (
                <div className="activity-log-line text-gray-500">Loading entries…</div>
              )}
              {!loadingEntries && selectedLog.entries.length === 0 && (
                <div className="activity-log-line text-gray-500">No entries for this day.</div>
              )}
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
