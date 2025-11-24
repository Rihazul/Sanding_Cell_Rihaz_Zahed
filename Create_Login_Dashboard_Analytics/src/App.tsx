import { useState } from 'react';
import { LoginPage } from './components/LoginPage';
import { Dashboard } from './components/Dashboard';
import { AnalyticsPage } from './components/AnalyticsPage';
import { TableConfigurationPage } from './components/dashboard/TableConfigurationPage';
import { RobotStatusPanel } from './components/dashboard/RobotStatusPanel';

type Page = 'login' | 'dashboard' | 'analytics' | 'tableConfig';

export default function App() {
  const [isAuthenticated, setIsAuthenticated] = useState(false);
  const [currentPage, setCurrentPage] = useState<Page>('login');
  const [isHoming, setIsHoming] = useState(false);

  const handleLogin = () => {
    setIsAuthenticated(true);
    setCurrentPage('dashboard');
  };

  const handleLogout = () => {
    setIsAuthenticated(false);
    setCurrentPage('login');
  };

  if (!isAuthenticated) {
    return <LoginPage onLogin={handleLogin} />;
  }

  return (
    <div className="min-h-screen bg-gray-50">
      {currentPage === 'dashboard' && (
        <Dashboard 
          onNavigateToAnalytics={() => setCurrentPage('analytics')}
          onNavigateToTableConfig={() => setCurrentPage('tableConfig')}
          onLogout={handleLogout}
        />
      )}
      {currentPage === 'analytics' && (
        <AnalyticsPage 
          onNavigateToDashboard={() => setCurrentPage('dashboard')}
          onLogout={handleLogout}
        />
      )}
      {currentPage === 'tableConfig' && (
        <TableConfigurationPage 
          onBack={() => setCurrentPage('dashboard')}
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
