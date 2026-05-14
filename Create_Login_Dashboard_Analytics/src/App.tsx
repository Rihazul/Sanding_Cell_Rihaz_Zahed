import { useState, useCallback, useEffect } from 'react';
import { Dashboard } from './components/Dashboard';
import { AnalyticsPage } from './components/AnalyticsPage';
import { RobotStatusPanel } from './components/dashboard/RobotStatusPanel';

type Page = 'dashboard' | 'analytics';

export interface ActivityEntry {
  id: number;
  timestamp: string;
  message: string;
  type: 'info' | 'success' | 'warning' | 'error';
}

const APP_ZOOM_KEY = 'appZoomPercent';
const MIN_ZOOM = 80;
const MAX_ZOOM = 150;
const ZOOM_STEP = 10;

export default function App() {
  const [currentPage, setCurrentPage] = useState<Page>('dashboard');
  const [isHoming, setIsHoming] = useState(false);
  
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
      const swalIcon = type === 'success' ? 'success' : type === 'warning' ? 'warning' : 'error';
      const swalTitle = type === 'success' ? 'Action Completed' : type === 'warning' ? 'Warning' : 'Error';
      const swal = (window as any).Swal;
      if (swal?.fire) {
        swal.fire({
          title: swalTitle,
          text: message,
          icon: swalIcon,
          timer: 2000,
          showConfirmButton: false
        });
      }
    }
  }, []);

  useEffect(() => {
    const applyZoom = (zoomPercent: number) => {
      const clamped = Math.max(MIN_ZOOM, Math.min(MAX_ZOOM, zoomPercent));
      document.documentElement.style.zoom = `${clamped}%`;
      try {
        localStorage.setItem(APP_ZOOM_KEY, String(clamped));
      } catch {
        // Ignore storage errors in restricted environments.
      }
    };

    const getCurrentZoom = () => {
      try {
        const stored = Number(localStorage.getItem(APP_ZOOM_KEY));
        if (!Number.isNaN(stored) && stored > 0) {
          return stored;
        }
      } catch {
        // Ignore storage errors in restricted environments.
      }
      return 100;
    };

    applyZoom(getCurrentZoom());

    const changeZoom = (delta: number) => {
      applyZoom(getCurrentZoom() + delta);
    };

    const resetZoom = () => {
      applyZoom(100);
    };

    const onKeyDown = (event: KeyboardEvent) => {
      const ctrlOrCmd = event.ctrlKey || event.metaKey;
      if (!ctrlOrCmd) {
        return;
      }

      if (event.key === '+' || event.key === '=' || event.code === 'NumpadAdd') {
        event.preventDefault();
        changeZoom(ZOOM_STEP);
        return;
      }

      if (event.key === '-' || event.key === '_' || event.code === 'NumpadSubtract') {
        event.preventDefault();
        changeZoom(-ZOOM_STEP);
        return;
      }

      if (event.key === '0' || event.code === 'Numpad0') {
        event.preventDefault();
        resetZoom();
      }
    };

    const onWheel = (event: WheelEvent) => {
      const ctrlOrCmd = event.ctrlKey || event.metaKey;
      if (!ctrlOrCmd) {
        return;
      }
      event.preventDefault();
      if (event.deltaY < 0) {
        changeZoom(ZOOM_STEP);
      } else if (event.deltaY > 0) {
        changeZoom(-ZOOM_STEP);
      }
    };

    window.addEventListener('keydown', onKeyDown, { passive: false });
    window.addEventListener('wheel', onWheel, { passive: false });

    return () => {
      window.removeEventListener('keydown', onKeyDown);
      window.removeEventListener('wheel', onWheel);
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
      
      {/* Global Robot Status Bar */}
      <RobotStatusPanel
        isHoming={isHoming}
        setIsHoming={setIsHoming}
      />
    </div>
  );
}
