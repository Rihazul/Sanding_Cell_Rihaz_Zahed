import React from 'react';

interface RobotStatusPanelProps {
  isHoming: boolean;
  setIsHoming: (homing: boolean) => void;
}

export function RobotStatusPanel({ isHoming, setIsHoming }: RobotStatusPanelProps) {
  void isHoming;
  void setIsHoming;
  return null;
}
