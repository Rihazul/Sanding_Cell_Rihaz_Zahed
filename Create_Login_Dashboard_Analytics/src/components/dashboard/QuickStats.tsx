import React from 'react';
import { Card, CardContent, CardHeader, CardTitle } from '../ui/card';

export function QuickStats() {
  return (
    <Card className="shadow-lg border-0">
      <CardHeader className="bg-gradient-to-r from-green-50 to-emerald-50">
        <CardTitle>Quick Stats</CardTitle>
      </CardHeader>
      <CardContent className="pt-6 space-y-3">
        <div className="flex justify-between">
          <span className="text-sm text-gray-600">Operations Today</span>
          <span>247</span>
        </div>
        <div className="flex justify-between">
          <span className="text-sm text-gray-600">Uptime</span>
          <span className="text-green-600">99.8%</span>
        </div>
        <div className="flex justify-between">
          <span className="text-sm text-gray-600">Efficiency</span>
          <span className="text-blue-600">94.2%</span>
        </div>
      </CardContent>
    </Card>
  );
}
