import { Button } from './ui/button';
import { Card, CardContent, CardHeader, CardTitle } from './ui/card';
import { LayoutDashboard, LogOut } from 'lucide-react';
import { LineChart, Line, BarChart, Bar, XAxis, YAxis, CartesianGrid, Tooltip, ResponsiveContainer, Legend } from 'recharts';

interface AnalyticsPageProps {
  onNavigateToDashboard: () => void;
}

export function AnalyticsPage({ onNavigateToDashboard }: AnalyticsPageProps) {
  const revenueData = [
    { month: 'Jan', revenue: 4000, users: 2400 },
    { month: 'Feb', revenue: 3000, users: 1398 },
    { month: 'Mar', revenue: 5000, users: 3800 },
    { month: 'Apr', revenue: 4500, users: 3908 },
    { month: 'May', revenue: 6000, users: 4800 },
    { month: 'Jun', revenue: 7500, users: 5300 },
  ];

  const categoryData = [
    { category: 'Electronics', value: 4000 },
    { category: 'Clothing', value: 3000 },
    { category: 'Food', value: 2000 },
    { category: 'Books', value: 2780 },
    { category: 'Home', value: 1890 },
  ];

  return (
    <div className="min-h-screen bg-gray-50 pb-24">
      {/* Header */}
      <header className="bg-white border-b border-gray-200">
        <div className="max-w-7xl mx-auto px-4 sm:px-6 lg:px-8">
          <div className="flex justify-between items-center py-4">
            <h1>Analytics</h1>
            <div className="flex gap-3">
              <Button onClick={onNavigateToDashboard} variant="outline">
                <LayoutDashboard className="size-4 mr-2" />
                Dashboard
              </Button>
            </div>
          </div>
        </div>
      </header>

      {/* Main Content */}
      <main className="max-w-7xl mx-auto px-4 sm:px-6 lg:px-8 py-8">
        <div className="mb-8">
          <h2>Performance Analytics</h2>
          <p className="text-gray-600">Detailed insights into your business performance</p>
        </div>

        {/* Charts Grid */}
        <div className="space-y-6">
          {/* Revenue & Users Chart */}
          <Card>
            <CardHeader>
              <CardTitle>Revenue & User Growth</CardTitle>
            </CardHeader>
            <CardContent>
              <ResponsiveContainer width="100%" height={300}>
                <LineChart data={revenueData}>
                  <CartesianGrid strokeDasharray="3 3" />
                  <XAxis dataKey="month" />
                  <YAxis />
                  <Tooltip />
                  <Legend />
                  <Line 
                    type="monotone" 
                    dataKey="revenue" 
                    stroke="#8b5cf6" 
                    strokeWidth={2}
                    name="Revenue ($)"
                  />
                  <Line 
                    type="monotone" 
                    dataKey="users" 
                    stroke="#3b82f6" 
                    strokeWidth={2}
                    name="Users"
                  />
                </LineChart>
              </ResponsiveContainer>
            </CardContent>
          </Card>

          {/* Category Performance Chart */}
          <Card>
            <CardHeader>
              <CardTitle>Sales by Category</CardTitle>
            </CardHeader>
            <CardContent>
              <ResponsiveContainer width="100%" height={300}>
                <BarChart data={categoryData}>
                  <CartesianGrid strokeDasharray="3 3" />
                  <XAxis dataKey="category" />
                  <YAxis />
                  <Tooltip />
                  <Legend />
                  <Bar 
                    dataKey="value" 
                    fill="#10b981" 
                    name="Sales ($)"
                  />
                </BarChart>
              </ResponsiveContainer>
            </CardContent>
          </Card>

          {/* Key Metrics */}
          <div className="grid grid-cols-1 md:grid-cols-3 gap-6">
            <Card>
              <CardHeader>
                <CardTitle>Avg. Session Duration</CardTitle>
              </CardHeader>
              <CardContent>
                <div className="text-3xl">5m 32s</div>
                <p className="text-sm text-gray-600 mt-2">+12% from last week</p>
              </CardContent>
            </Card>
            <Card>
              <CardHeader>
                <CardTitle>Bounce Rate</CardTitle>
              </CardHeader>
              <CardContent>
                <div className="text-3xl">32.4%</div>
                <p className="text-sm text-gray-600 mt-2">-5% from last week</p>
              </CardContent>
            </Card>
            <Card>
              <CardHeader>
                <CardTitle>Pages per Session</CardTitle>
              </CardHeader>
              <CardContent>
                <div className="text-3xl">4.2</div>
                <p className="text-sm text-gray-600 mt-2">+8% from last week</p>
              </CardContent>
            </Card>
          </div>
        </div>
      </main>
    </div>
  );
}
