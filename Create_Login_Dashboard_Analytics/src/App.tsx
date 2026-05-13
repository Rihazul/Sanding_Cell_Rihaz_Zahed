import { useState, useCallback } from 'react';
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
