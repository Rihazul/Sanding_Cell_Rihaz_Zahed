import React from 'react';
import { motion } from 'motion/react';

interface ToggleButtonProps {
  label: string;
  isActive: boolean;
  onToggle: () => void;
  activeLabel: string;
  inactiveLabel: string;
  isPending?: boolean;
  pendingLabel?: string;
  disabled?: boolean;
  showCheckmarkPosition?: 'left' | 'right';
}

export function ToggleButton({
  label,
  isActive,
  onToggle,
  activeLabel,
  inactiveLabel,
  isPending = false,
  pendingLabel,
  disabled = false,
  showCheckmarkPosition = 'left',
}: ToggleButtonProps) {
  const handleClick = (e: React.MouseEvent<HTMLButtonElement>) => {
    e.preventDefault();
    if (!disabled && !isPending) {
      onToggle();
    }
  };

  return (
    <motion.button
      type="button"
      whileTap={disabled || isPending ? {} : { scale: 0.95 }}
      onClick={handleClick}
      disabled={disabled}
      aria-busy={isPending}
      className={`w-full relative px-6 py-4 rounded-xl transition-all duration-300 shadow-md hover:shadow-lg overflow-hidden ${
        isPending
          ? 'toggle-pending'
          : disabled
          ? 'bg-gradient-to-r from-slate-400 to-slate-500 text-white cursor-not-allowed'
          : isActive
          ? 'bg-gradient-to-r from-emerald-500 to-green-500 text-white'
          : 'bg-gradient-to-r from-gray-100 to-gray-200 text-gray-700'
      }`}
    >
      <div className="flex items-center justify-between">
        <span className="font-medium">{label}</span>
        <div
          className={`flex items-center gap-2 px-3 py-1 rounded-full text-sm ${
            isPending ? 'toggle-pending-badge' : isActive ? 'bg-white/20' : 'bg-gray-300'
          }`}
        >
          {showCheckmarkPosition === 'left' && (
            <span className="font-bold text-lg leading-none">{isActive ? '✓' : '✗'}</span>
          )}
          <span className="text-xs min-w-[110px] inline-block text-center">
            {isPending ? (pendingLabel || activeLabel) : isActive ? activeLabel : inactiveLabel}
          </span>
          {showCheckmarkPosition === 'right' && (
            <span className="font-bold text-lg leading-none">{isActive ? '✓' : '✗'}</span>
          )}
        </div>
      </div>
    </motion.button>
  );
}
