import { useState, useCallback, useRef, useEffect } from 'react';
import { Dashboard } from './components/Dashboard';
import { AnalyticsPage } from './components/AnalyticsPage';
import { RobotStatusPanel } from './components/dashboard/RobotStatusPanel';
import { AlertTriangle, CheckCircle2, XCircle } from 'lucide-react';

type Page = 'dashboard' | 'analytics';

export interface ActivityEntry {
  id: number;
  timestamp: string;
  message: string;
  type: 'info' | 'success' | 'warning' | 'error';
}

export default function App() {
  const [currentPage, setCurrentPage] = useState<Page>('dashboard');
  const [isHoming, setIsHoming] = useState(false);
  const [actionPopup, setActionPopup] = useState<{ message: string; type: 'success' | 'warning' | 'error' } | null>(null);
  const popupTimerRef = useRef<ReturnType<typeof setTimeout> | null>(null);
  
  // Shared activities state - lifted from Dashboard
  const [activities, setActivities] = useState<ActivityEntry[]>([
    { id: 1, timestamp: new Date().toLocaleTimeString(), message: 'Robot system initialized', type: 'success' },
    { id: 2, timestamp: new Date().toLocaleTimeString(), message: 'Waiting for commands...', type: 'info' },
  ]);

  // Activity log function shared between components
  const addActivity = useCallback((message: string, type: 'info' | 'success' | 'warning' | 'error' = 'info') => {
    console.log('App addActivity called:', message, type);
    const newActivity: ActivityEntry = {
      id: Date.now(),
      timestamp: new Date().toLocaleTimeString(),
      message,
      type,
    };
    setActivities((prev) => [...prev, newActivity]);

    if (type === 'success' || type === 'warning' || type === 'error') {
      setActionPopup({ message, type });
      if (popupTimerRef.current) {
        clearTimeout(popupTimerRef.current);
      }
      popupTimerRef.current = setTimeout(() => {
        setActionPopup(null);
        popupTimerRef.current = null;
      }, 1200);
    }
  }, []);

  useEffect(() => {
    return () => {
      if (popupTimerRef.current) {
        clearTimeout(popupTimerRef.current);
      }
    };
  }, []);

  return (
    <div className="min-h-screen bg-gray-50">
      {currentPage === 'dashboard' && (
        <Dashboard 
          onNavigateToAnalytics={() => setCurrentPage('analytics')}
          activities={activities}
          addActivity={addActivity}
        />
      )}
      {currentPage === 'analytics' && (
        <AnalyticsPage 
          onNavigateToDashboard={() => setCurrentPage('dashboard')}
          liveActivities={activities}
        />
      )}

      {actionPopup && (
        <div className="pointer-events-none fixed inset-0 z-[90] flex items-center justify-center px-4">
          <div
            className={`w-full max-w-xl rounded-2xl px-6 py-6 shadow-2xl ${
              actionPopup.type === 'success'
                ? 'border border-emerald-300 bg-gradient-to-r from-emerald-50 via-green-50 to-emerald-100'
                : actionPopup.type === 'warning'
                  ? 'border border-amber-300 bg-gradient-to-r from-amber-50 via-yellow-50 to-amber-100'
                  : 'border border-red-300 bg-gradient-to-r from-red-50 via-rose-50 to-red-100'
            }`}
          >
            <div className="flex items-center gap-4">
              {actionPopup.type === 'success' ? (
                <CheckCircle2 className="size-10 shrink-0 text-emerald-600" />
              ) : actionPopup.type === 'warning' ? (
                <AlertTriangle className="size-10 shrink-0 text-amber-600" />
              ) : (
                <XCircle className="size-10 shrink-0 text-red-600" />
              )}
              <div>
                <div
                  className={`text-lg font-semibold ${
                    actionPopup.type === 'success'
                      ? 'text-emerald-900'
                      : actionPopup.type === 'warning'
                        ? 'text-amber-900'
                        : 'text-red-900'
                  }`}
                >
                  {actionPopup.type === 'success'
                    ? 'Action Completed'
                    : actionPopup.type === 'warning'
                      ? 'Warning'
                      : 'Error'}
                </div>
                <div
                  className={`text-sm ${
                    actionPopup.type === 'success'
                      ? 'text-emerald-800'
                      : actionPopup.type === 'warning'
                        ? 'text-amber-800'
                        : 'text-red-800'
                  }`}
                >
                  {actionPopup.message}
                </div>
              </div>
            </div>
          </div>
        </div>
      )}
      
      {/* Global Robot Status Bar */}
      <RobotStatusPanel
        isHoming={isHoming}
        setIsHoming={setIsHoming}
      />
    </div>
  );
}
