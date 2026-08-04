import React, { useState, useEffect, useRef } from 'react';
import { motion } from 'motion/react';
import { Card, CardContent, CardHeader, CardTitle } from '../ui/card';
import ActivityLog from './ActivityLog';
import { getProcessStatus, performAction } from '../../services/api';

interface RobotStatusCardProps {
  isHoming: boolean;
  setIsHoming: (homing: boolean) => void;
  activities: ActivityMessage[];
  addActivity: (message: string, type?: 'info' | 'success' | 'warning' | 'error') => void;
  isOperating: boolean;
  robotEnabled: boolean;
  onHomingCompleted?: () => void;
}

interface ActivityMessage {
  id: number;
  timestamp: string;
  message: string;
  type: 'info' | 'success' | 'warning' | 'error';
}

export function RobotStatusCard({ isHoming, setIsHoming, activities, addActivity, isOperating, robotEnabled, onHomingCompleted }: RobotStatusCardProps) {
  const messagesEndRef = useRef<HTMLDivElement>(null);
  const scrollContainerRef = useRef<HTMLDivElement>(null);
  const [isLogExpanded, setIsLogExpanded] = useState(false);
  // Homing rule: NOT allowed while a task is actively running. It becomes allowed again only
  // after the operator triggers a Stop (so they can recover), or when nothing is running.
  // `stopTriggered` re-enables Homing during the window between pressing Stop and isOperating
  // actually flipping to false. It resets whenever a fresh task starts.
  const [stopTriggered, setStopTriggered] = useState(false);
  const wasOperatingRef = useRef(isOperating);

  useEffect(() => {
    // A new task starting (idle -> operating) clears any prior stop, re-arming the safety gate.
    if (isOperating && !wasOperatingRef.current) {
      setStopTriggered(false);
    }
    wasOperatingRef.current = isOperating;
  }, [isOperating]);

  // Homing is blocked only while a task runs AND no stop has been triggered.
  const homingBlocked = isOperating && !stopTriggered;

  const scrollToBottom = () => {
    if (scrollContainerRef.current) {
      scrollContainerRef.current.scrollTop = scrollContainerRef.current.scrollHeight;
    }
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
      const start = Date.now();
      const timeoutMs = 10 * 60 * 1000;
      const pollIntervalMs = 1000;
      while (true) {
        const status = await getProcessStatus();
        if (status?.status === 'completed') {
          if (status?.homingRequired !== false) {
            throw new Error(`Homing ended without confirmation (${status?.homingReason || 'not_homed'})`);
          }
          addActivity('Homing completed successfully', 'success');
          onHomingCompleted?.();
          break;
        }
        if (status?.status === 'failed') {
          throw new Error(`Homing process failed (${status?.homingReason || 'not_homed'})`);
        }
        if (Date.now() - start > timeoutMs) {
          throw new Error('Homing timed out');
        }
        await new Promise(resolve => setTimeout(resolve, pollIntervalMs));
      }
    } catch (error) {
      addActivity(`Homing failed - ${error}`, 'error');
    } finally {
      setIsHoming(false);
    }
  };

  const handleStop = async () => {
    if (!robotEnabled) {
      addActivity('Cannot stop - Robot power is disabled', 'error');
      return;
    }
    addActivity('Sending stop to robot...', 'warning');
    try {
      const response = await performAction('stop');
      if (response?.error) {
        addActivity(response.error, 'error');
        return;
      }
      // A stop was triggered — allow Homing so the operator can recover, even while the task
      // thread is still unwinding (isOperating may stay true briefly).
      setStopTriggered(true);
      addActivity('Emergency stop activated!', 'error');
    } catch (error) {
      addActivity(`Failed to send stop: ${error}`, 'error');
    }
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
        <CardTitle className="flex items-center gap-2">
          <span className="text-lg">🤖</span>
          Robot Status
        </CardTitle>
      </CardHeader>
      <CardContent className="pt-6 space-y-4">
        {/* Control Buttons */}
        <div className="grid grid-cols-2 gap-3">
          <motion.button
            type="button"
            whileTap={homingBlocked ? {} : { scale: 0.95 }}
            onClick={homingBlocked ? undefined : (e) => { e.preventDefault(); setStopTriggered(false); handleHoming(robotEnabled); }}
            disabled={homingBlocked}
            title={homingBlocked ? 'Homing is disabled while a task is running. Press Stop first.' : undefined}
            className={`${isHoming ? 'bg-green-500 hover:bg-green-600' : 'bg-gray-600 hover:bg-gray-700'} text-white px-6 py-3 rounded-lg transition-all shadow-md hover:shadow-lg ${homingBlocked ? 'brightness-95 cursor-not-allowed' : ''}`}
          >
            {isHoming ? 'Homing...' : 'Homing'}
          </motion.button>
          <motion.button 
            type="button"
            whileTap={robotEnabled ? { scale: 0.95 } : {}}
            onClick={robotEnabled ? (e) => { e.preventDefault(); handleStop(); } : undefined}
            disabled={!robotEnabled}
            className={`bg-red-600 hover:bg-red-700 text-white px-6 py-3 rounded-lg transition-all shadow-md hover:shadow-lg ${!robotEnabled ? 'opacity-60 cursor-not-allowed' : ''}`}
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
            ref={scrollContainerRef}
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
