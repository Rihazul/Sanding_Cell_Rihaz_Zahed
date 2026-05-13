import React from 'react';
import { Button } from '../ui/button';
import { BarChart3 } from 'lucide-react';
import logo from '../../assets/logo.png';

interface DashboardHeaderProps {
  onNavigateToAnalytics: () => void;
}

export function DashboardHeader({ onNavigateToAnalytics }: DashboardHeaderProps) {
  return (
    <header className="bg-white border-b border-gray-200 shadow-sm">
      <div className="max-w-7xl mx-auto px-4 sm:px-6 lg:px-8">
        <div className="relative flex items-center justify-between py-4 whitespace-nowrap">
          <div className="z-10 flex shrink-0 items-center gap-3">
            <img src={logo} alt="Sanding Cell Logo" className="h-10 w-auto object-contain" />
          </div>

          <h1
            className="absolute whitespace-nowrap text-center text-3xl font-black tracking-wide leading-none"
            style={{ left: '50%', transform: 'translateX(-50%)', margin: 0 }}
          >
              Sanding Cell
          </h1>

          <div className="z-10 flex shrink-0 gap-3 whitespace-nowrap">
            <Button onClick={onNavigateToAnalytics} variant="outline" className="shrink-0 whitespace-nowrap">
              <BarChart3 className="size-4 mr-2" />
              Analytics
            </Button>
          </div>
        </div>
      </div>
    </header>
  );
}
