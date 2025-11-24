import React from 'react';
import { motion } from 'motion/react';
import { Power, Circle } from 'lucide-react';

interface StatusBannerProps {
  robotEnabled: boolean;
  robotSpeed: number[];
}

export function StatusBanner({ robotEnabled, robotSpeed }: StatusBannerProps) {
  return (
    <motion.div
      initial={{ opacity: 0, y: -20 }}
      animate={{ opacity: 1, y: 0 }}
      className="mb-6 p-4 bg-white rounded-xl shadow-md border-l-4 border-blue-500 flex items-center justify-between"
    >
      <div className="flex items-center gap-3">
        <Power className={`size-6 ${robotEnabled ? 'text-green-500' : 'text-gray-400'}`} />
        <div>
          <p>System Status</p>
          <p className="text-sm text-gray-600">Robot is {robotEnabled ? 'ONLINE' : 'OFFLINE'} • Speed: {robotSpeed[0]}%</p>
        </div>
      </div>

      <div className="flex gap-2">
        {[1, 2, 3, 4, 5].map((i) => (
          <motion.div
            key={i}
            animate={{
              scale: robotEnabled ? [1, 1.2, 1] : 1,
              opacity: robotEnabled ? [0.5, 1, 0.5] : 0.3,
            }}
            transition={{ duration: 1.5, repeat: robotEnabled ? Infinity : 0, delay: i * 0.2 }}
          >
            <Circle className="size-3 fill-red-500 text-red-500" />
          </motion.div>
        ))}
      </div>
    </motion.div>
  );
}
