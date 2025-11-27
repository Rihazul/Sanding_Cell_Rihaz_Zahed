import React from 'react';
import { motion } from 'motion/react';

interface ToggleButtonProps {
  label: string;
  isActive: boolean;
  onToggle: () => void;
  activeLabel: string;
  inactiveLabel: string;
  disabled?: boolean;
}

export function ToggleButton({
  label,
  isActive,
  onToggle,
  activeLabel,
  inactiveLabel,
  disabled = false,
}: ToggleButtonProps) {
  const handleClick = (e: React.MouseEvent<HTMLButtonElement>) => {
    e.preventDefault();
    if (!disabled) {
      onToggle();
    }
  };

  return (
    <motion.button
      type="button"
      whileTap={disabled ? {} : { scale: 0.95 }}
      onClick={handleClick}
      disabled={disabled}
      className={`w-full relative px-6 py-4 rounded-xl transition-all duration-300 shadow-md hover:shadow-lg overflow-hidden ${
        disabled
          ? 'bg-gradient-to-r from-gray-300 to-gray-400 text-gray-500 cursor-not-allowed opacity-50'
          : isActive
          ? 'bg-gradient-to-r from-emerald-500 to-green-500 text-white'
          : 'bg-gradient-to-r from-gray-100 to-gray-200 text-gray-700'
      }`}
    >
      <div className="flex items-center justify-between">
        <span className="font-medium">{label}</span>
        <div className={`flex items-center gap-2 px-3 py-1 rounded-full text-sm ${isActive ? 'bg-white/20' : 'bg-gray-300'}`}>
          <motion.div animate={{ rotate: isActive ? 180 : 0 }} transition={{ duration: 0.3 }}>
            {isActive ? '✓' : '○'}
          </motion.div>
          <span className="text-xs min-w-[80px] inline-block text-center">{isActive ? activeLabel : inactiveLabel}</span>
        </div>
      </div>
    </motion.button>
  );
}
