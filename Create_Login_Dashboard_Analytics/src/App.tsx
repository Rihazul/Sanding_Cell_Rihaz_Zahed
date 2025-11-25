import { useState } from 'react';
import { Dashboard } from './components/Dashboard';
import { AnalyticsPage } from './components/AnalyticsPage';
import { RobotStatusPanel } from './components/dashboard/RobotStatusPanel';

type Page = 'dashboard' | 'analytics';

export default function App() {
  const [currentPage, setCurrentPage] = useState<Page>('dashboard');
  const [isHoming, setIsHoming] = useState(false);

  return (
    <div className="min-h-screen bg-gray-50">
      {currentPage === 'dashboard' && (
        <Dashboard 
          onNavigateToAnalytics={() => setCurrentPage('analytics')}
        />
      )}
      {currentPage === 'analytics' && (
        <AnalyticsPage 
          onNavigateToDashboard={() => setCurrentPage('dashboard')}
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
