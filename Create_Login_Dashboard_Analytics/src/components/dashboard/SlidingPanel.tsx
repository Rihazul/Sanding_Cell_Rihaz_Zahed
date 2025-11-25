import React from 'react';
import { Button } from '../ui/button';
import { ChevronLeft, ChevronRight } from 'lucide-react';

type PanelView = 'robot-control' | 'table-a' | 'table-b';

interface SlidingPanelProps {
  currentView: PanelView;
  onViewChange: (view: PanelView) => void;
  children: React.ReactNode;
}

export function SlidingPanel({ currentView, onViewChange, children }: SlidingPanelProps) {
  const views: { id: PanelView; label: string }[] = [
    { id: 'robot-control', label: 'Robot Control' },
    { id: 'table-a', label: 'Table A Config' },
    { id: 'table-b', label: 'Table B Config' },
  ];

  const childrenArray = React.Children.toArray(children);
  const currentIndex = views.findIndex(v => v.id === currentView);
  const canGoPrevious = currentIndex > 0;
  const canGoNext = currentIndex < views.length - 1;

  const handlePrevious = () => {
    if (canGoPrevious) {
      onViewChange(views[currentIndex - 1].id);
    }
  };

  const handleNext = () => {
    if (canGoNext) {
      onViewChange(views[currentIndex + 1].id);
    }
  };

  return (
    <div className="relative">
      {/* Navigation Header */}
      <div className="flex items-center justify-between mb-4 bg-white rounded-lg shadow-sm p-3">
        <Button
          onClick={handlePrevious}
          disabled={!canGoPrevious}
          variant="outline"
          size="sm"
          className="disabled:opacity-30"
        >
          <ChevronLeft className="size-4 mr-1" />
          Previous
        </Button>

        <div className="flex items-center gap-2">
          {views.map((view, idx) => (
            <React.Fragment key={view.id}>
              <button
                onClick={() => onViewChange(view.id)}
                className={`px-4 py-2 rounded-md text-sm font-medium transition-all ${
                  currentView === view.id
                    ? 'bg-blue-500 text-white shadow-md'
                    : 'bg-gray-100 text-gray-600 hover:bg-gray-200'
                }`}
              >
                {view.label}
              </button>
              {idx < views.length - 1 && (
                <ChevronRight className="size-4 text-gray-400" />
              )}
            </React.Fragment>
          ))}
        </div>

        <Button
          onClick={handleNext}
          disabled={!canGoNext}
          variant="outline"
          size="sm"
          className="disabled:opacity-30"
        >
          Next
          <ChevronRight className="size-4 ml-1" />
        </Button>
      </div>

      {/* Content Area with Slide Animation */}
      <div className="overflow-hidden relative">
        <div
          className="transition-transform duration-500 ease-in-out flex"
          style={{
            transform: `translateX(-${(currentIndex * 100) / childrenArray.length}%)`,
            width: `${childrenArray.length * 100}%`,
          }}
        >
          {childrenArray.map((child, idx) => (
            <div key={idx} style={{ width: `${100 / childrenArray.length}%`, flexShrink: 0 }}>
              <div className="px-2">
                {child}
              </div>
            </div>
          ))}
        </div>
      </div>
    </div>
  );
}
