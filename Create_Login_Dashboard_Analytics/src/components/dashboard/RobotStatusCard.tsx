import React, { useState, useEffect, useRef } from 'react';
import { motion } from 'motion/react';
import { Card, CardContent, CardHeader, CardTitle } from '../ui/card';
import ActivityLog from './ActivityLog';
import { performAction } from '../../services/api';

interface RobotStatusCardProps {
  isHoming: boolean;
  setIsHoming: (homing: boolean) => void;
  activities: ActivityMessage[];
  addActivity: (message: string, type?: 'info' | 'success' | 'warning' | 'error') => void;
  isOperating: boolean;
  robotEnabled: boolean;
}

interface ActivityMessage {
  id: number;
  timestamp: string;
  message: string;
  type: 'info' | 'success' | 'warning' | 'error';
}

export function RobotStatusCard({ isHoming, setIsHoming, activities, addActivity, isOperating, robotEnabled }: RobotStatusCardProps) {
  const messagesEndRef = useRef<HTMLDivElement>(null);
  const [isLogExpanded, setIsLogExpanded] = useState(false);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [activities]);

  const handleHoming = async (robotEnabled: boolean) => {
    if (!robotEnabled) {
      addActivity('Cannot perform homing - Robot power is disabled', 'error');
      return;
    }
    setIsHoming(true);
    addActivity('Homing sequence initiated...', 'warning');
    
    try {
      await performAction('homing');
      addActivity('Homing completed successfully', 'success');
    } catch (error) {
      addActivity(`Homing failed - ${error}`, 'error');
    } finally {
      setIsHoming(false);
    }
  };

  const handleStop = (robotEnabled: boolean) => {
    if (!robotEnabled) {
      addActivity('Cannot activate emergency stop - Robot power is disabled', 'error');
      return;
    }
    addActivity('Emergency stop activated!', 'error');
  };

  const getMessageColor = (type: ActivityMessage['type']) => {
    switch (type) {
      case 'success': return 'text-green-600';
      case 'warning': return 'text-yellow-600';
      case 'error': return 'text-red-600';
      default: return 'text-gray-600';
    }
  };

  return (
    <Card className="shadow-lg border-0">
      <CardHeader className="bg-gradient-to-r from-slate-50 to-gray-50">
        <CardTitle>Robot Status</CardTitle>
      </CardHeader>
      <CardContent className="pt-6 space-y-4">
        {/* Control Buttons */}
        <div className="grid grid-cols-2 gap-3">
          <motion.button
            whileTap={isOperating ? {} : { scale: 0.95 }}
            onClick={isOperating ? undefined : () => handleHoming(robotEnabled)}
            disabled={isOperating}
            className={`${isHoming ? 'bg-green-500 hover:bg-green-600' : 'bg-gray-600 hover:bg-gray-700'} text-white px-6 py-3 rounded-lg transition-all shadow-md hover:shadow-lg ${isOperating ? 'opacity-50 cursor-not-allowed' : ''}`}
          >
            {isHoming ? 'Homing...' : 'Homing'}
          </motion.button>
          <motion.button 
            whileTap={{ scale: 0.95 }} 
            onClick={() => handleStop(robotEnabled)}
            className="bg-red-600 hover:bg-red-700 text-white px-6 py-3 rounded-lg transition-all shadow-md hover:shadow-lg"
          >
            Stop
          </motion.button>
        </div>

        {/* Activity Display Panel */}
        <div className="flex flex-col min-h-0">
          <div className="flex items-center justify-between mb-2 flex-shrink-0">
            <h3 className="text-sm font-semibold text-gray-600">Activity Log</h3>
            <button
              type="button"
              onClick={() => setIsLogExpanded(true)}
              className="text-xs bg-blue-500 hover:bg-blue-600 text-white px-3 py-1 rounded-full transition-colors"
            >
              Expand
            </button>
          </div>
          <div 
            className="activity-log-scroll bg-gray-900 border-2 border-gray-700 rounded-lg p-4 shadow-inner flex-shrink-0"
            style={{
              height: '256px',
              maxHeight: '256px',
              minHeight: '256px',
              overflowY: 'scroll',
              overflowX: 'hidden',
              scrollbarWidth: 'thin',
              scrollbarColor: '#4B5563 #1F2937',
              display: 'flex',
              flexDirection: 'column',
              gap: '0.5rem'
            }}
          >
            {activities.map((activity) => (
              <motion.div
                key={activity.id}
                initial={{ opacity: 0, x: -20 }}
                animate={{ opacity: 1, x: 0 }}
                className="text-sm font-mono flex-shrink-0"
              >
                <span className="text-gray-400">[{activity.timestamp}]</span>{' '}
                <span className={getMessageColor(activity.type)}>{activity.message}</span>
              </motion.div>
            ))}
            <div ref={messagesEndRef} />
          </div>
        </div>
      </CardContent>

      {/* Activity Log Modal */}
      {isLogExpanded && (
        <ActivityLog
          entries={activities.map((activity) => `[${activity.timestamp}] ${activity.message}`)}
          onClose={() => setIsLogExpanded(false)}
        />
      )}
    </Card>
  );
}
