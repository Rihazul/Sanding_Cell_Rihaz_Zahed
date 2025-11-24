import React from 'react';
import { Button } from '../ui/button';
import { BarChart3, LogOut, Zap, Settings } from 'lucide-react';

interface DashboardHeaderProps {
  onNavigateToAnalytics: () => void;
  onNavigateToTableConfig: () => void;
  onLogout: () => void;
}

export function DashboardHeader({ onNavigateToAnalytics, onNavigateToTableConfig, onLogout }: DashboardHeaderProps) {
  return (
    <header className="bg-white border-b border-gray-200 shadow-sm">
      <div className="max-w-7xl mx-auto px-4 sm:px-6 lg:px-8">
        <div className="flex justify-between items-center py-4">
          <div className="flex items-center gap-3">
            <div className="p-2 bg-gradient-to-br from-blue-500 to-purple-600 rounded-lg">
              <Zap className="size-5 text-white" />
            </div>
            <h1>Robot Control Dashboard</h1>
          </div>

          <div className="flex gap-3">
            <Button onClick={onNavigateToTableConfig} variant="outline">
              <Settings className="size-4 mr-2" />
              Table Config
            </Button>
            <Button onClick={onNavigateToAnalytics} variant="outline">
              <BarChart3 className="size-4 mr-2" />
              Analytics
            </Button>
            <Button onClick={onLogout} variant="ghost">
              <LogOut className="size-4 mr-2" />
              Logout
            </Button>
          </div>
        </div>
      </div>
    </header>
  );
}
