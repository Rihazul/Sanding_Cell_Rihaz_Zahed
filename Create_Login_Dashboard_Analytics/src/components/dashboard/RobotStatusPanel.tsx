import React, { useState } from 'react';
import { motion, AnimatePresence } from 'motion/react';
import { ChevronUp } from 'lucide-react';

interface RobotStatusPanelProps {
  isHoming: boolean;
  setIsHoming: (homing: boolean) => void;
}

export function RobotStatusPanel({ isHoming, setIsHoming }: RobotStatusPanelProps) {
  const [showIOStatus, setShowIOStatus] = useState(false);
  const [isPinned, setIsPinned] = useState(false);
  
  // Mock I/O signals - randomly assign green (true) or red (false) for demo
  // In real application, these would come from actual robot state
  const controlInputs = Array.from({ length: 8 }, (_, i) => Math.random() > 0.5);
  const digitalInputs = Array.from({ length: 8 }, (_, i) => Math.random() > 0.5);
  const controlOutputs = Array.from({ length: 8 }, (_, i) => Math.random() > 0.5);
  const digitalOutputs = Array.from({ length: 8 }, (_, i) => Math.random() > 0.5);

  const handleMouseEnter = () => {
    if (!isPinned) setShowIOStatus(true);
  };

  const handleMouseLeave = () => {
    if (!isPinned) setShowIOStatus(false);
  };

  return (
    <div className="fixed bottom-0 left-0 right-0 z-50">
      {/* I/O Status Popup */}
      <AnimatePresence>
        {(showIOStatus || isPinned) && (
          <motion.div
            initial={{ opacity: 0, y: 20 }}
            animate={{ opacity: 1, y: 0 }}
            exit={{ opacity: 0, y: 20 }}
            transition={{ duration: 0.3 }}
            onMouseEnter={handleMouseEnter}
            onMouseLeave={handleMouseLeave}
            className="bg-white border-t-2 border-gray-300 shadow-2xl px-8 py-4 mx-4 rounded-t-lg"
          >
            <div className="grid grid-cols-2 gap-4">
              {/* INPUTS Column */}
              <div className="space-y-3">
                {/* Control Inputs */}
                <div>
                  <div className="text-xs font-semibold text-gray-600 mb-2 text-center">
                    CONTROL INPUTS <span className="text-gray-400 font-normal">(8 Signals)</span>
                  </div>
                  <div className="flex gap-2 justify-center">
                    {controlInputs.map((isActive, idx) => (
                      <div key={`control-input-${idx}`} className="flex flex-col items-center gap-1">
                        <span className="text-[10px] font-semibold text-gray-600">CI{idx}</span>
                        <motion.div
                          whileHover={{ scale: 1.2, rotate: 5 }}
                          whileTap={{ scale: 0.9 }}
                          className={`w-10 h-10 ${
                            isActive ? 'bg-green-500' : 'bg-red-500'
                          } shadow-lg flex items-center justify-center text-white text-xs font-bold cursor-pointer transition-all`}
                        >
                          {idx}
                        </motion.div>
                      </div>
                    ))}
                  </div>
                </div>

                {/* Digital Inputs */}
                <div>
                  <div className="text-xs font-semibold text-gray-600 mb-2 text-center">
                    DIGITAL INPUTS <span className="text-gray-400 font-normal">(8 Signals)</span>
                  </div>
                  <div className="flex gap-2 justify-center">
                    {digitalInputs.map((isActive, idx) => (
                      <div key={`digital-input-${idx}`} className="flex flex-col items-center gap-1">
                        <span className="text-[10px] font-semibold text-gray-600">DI{idx}</span>
                        <motion.div
                          whileHover={{ scale: 1.2, rotate: 5 }}
                          whileTap={{ scale: 0.9 }}
                          className={`w-10 h-10 ${
                            isActive ? 'bg-green-500' : 'bg-red-500'
                          } shadow-lg flex items-center justify-center text-white text-xs font-bold cursor-pointer transition-all`}
                        >
                          {idx}
                        </motion.div>
                      </div>
                    ))}
                  </div>
                </div>
              </div>

              {/* OUTPUTS Column */}
              <div className="space-y-3">
                {/* Control Outputs */}
                <div>
                  <div className="text-xs font-semibold text-gray-600 mb-2 text-center">
                    CONTROL OUTPUTS <span className="text-gray-400 font-normal">(8 Signals)</span>
                  </div>
                  <div className="flex gap-2 justify-center">
                    {controlOutputs.map((isActive, idx) => (
                      <div key={`control-output-${idx}`} className="flex flex-col items-center gap-1">
                        <span className="text-[10px] font-semibold text-gray-600">CO{idx}</span>
                        <motion.div
                          whileHover={{ scale: 1.2, rotate: -5 }}
                          whileTap={{ scale: 0.9 }}
                          className={`w-10 h-10 ${
                            isActive ? 'bg-green-500' : 'bg-red-500'
                          } shadow-lg flex items-center justify-center text-white text-xs font-bold cursor-pointer transition-all`}
                        >
                          {idx}
                        </motion.div>
                      </div>
                    ))}
                  </div>
                </div>

                {/* Digital Outputs */}
                <div>
                  <div className="text-xs font-semibold text-gray-600 mb-2 text-center">
                    DIGITAL OUTPUTS <span className="text-gray-400 font-normal">(8 Signals)</span>
                  </div>
                  <div className="flex gap-2 justify-center">
                    {digitalOutputs.map((isActive, idx) => (
                      <div key={`digital-output-${idx}`} className="flex flex-col items-center gap-1">
                        <span className="text-[10px] font-semibold text-gray-600">DO{idx}</span>
                        <motion.div
                          whileHover={{ scale: 1.2, rotate: -5 }}
                          whileTap={{ scale: 0.9 }}
                          className={`w-10 h-10 ${
                            isActive ? 'bg-green-500' : 'bg-red-500'
                          } shadow-lg flex items-center justify-center text-white text-xs font-bold cursor-pointer transition-all`}
                        >
                          {idx}
                        </motion.div>
                      </div>
                    ))}
                  </div>
                </div>
              </div>
            </div>
          </motion.div>
        )}
      </AnimatePresence>

      {/* I/O Status Bottom Bar */}
      <div 
        className="bg-gradient-to-r from-slate-800 to-gray-800 shadow-2xl border-t-2 border-gray-600"
        onMouseEnter={handleMouseEnter}
        onMouseLeave={handleMouseLeave}
      >
        <div className="px-6 py-3 flex items-center justify-center">
          <div className="flex items-center gap-4">
            <h3 className="text-white font-semibold text-base">I/O Signal Status</h3>
            
            {/* Animated Arrow - Click to Pin */}
            <motion.div
            onClick={() => setIsPinned(!isPinned)}
            whileHover={{ y: -3 }}
            animate={!isPinned ? { y: [0, -5, 0] } : {}}
            transition={{ 
              y: { repeat: Infinity, duration: 1.5, ease: "easeInOut" }
            }}
            className={`cursor-pointer p-2 rounded-full hover:bg-blue-600 transition-colors ${
              isPinned ? 'bg-green-500' : 'bg-blue-500'
            }`}
          >
            <ChevronUp className="w-5 h-5 text-white" />
          </motion.div>
          </div>
        </div>
      </div>
    </div>
  );
}
