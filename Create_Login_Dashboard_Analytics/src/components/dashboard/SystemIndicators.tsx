import React from 'react';
import { motion } from 'motion/react';
import { Card, CardContent, CardHeader, CardTitle } from '../ui/card';

interface SystemIndicatorsProps {
  robotEnabled: boolean;
}

export function SystemIndicators({ robotEnabled }: SystemIndicatorsProps) {
  return (
    <Card className="shadow-lg border-0">
      <CardHeader className="bg-gradient-to-r from-orange-50 to-yellow-50">
        <CardTitle>System Indicators</CardTitle>
      </CardHeader>
      <CardContent className="pt-6">
        <div className="flex justify-center gap-3">
          {[1, 2, 3, 4, 5].map((i) => (
            <motion.div
              key={i}
              animate={{
                scale: robotEnabled ? [1, 1.3, 1] : 1,
                backgroundColor: robotEnabled ? ['#ef4444', '#dc2626', '#ef4444'] : '#9ca3af',
              }}
              transition={{ duration: 2, repeat: robotEnabled ? Infinity : 0, delay: i * 0.3 }}
              className="w-10 h-10 rounded-full shadow-lg"
            />
          ))}
        </div>
      </CardContent>
    </Card>
  );
}
