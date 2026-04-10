import { useState, useEffect } from 'react';

interface AlertEntry {
  id: number;
  message: string;
  timestamp: string;
  isNew: boolean;
}

export const useAlerts = () => {
  const [alerts, setAlerts] = useState<AlertEntry[]>([]);
  const [connected, setConnected] = useState(false);
  const [battery, setBattery] = useState(100);

  const fetchRobotData = async () => {
    try {
      const response = await fetch('http://192.168.0.24:5000/api/robot/status');
      const data = await response.json();
      setConnected(true);
      // setBattery(data.battery);
      if (data.new_alert) {
        setAlerts(prev => [{
          id: Date.now(),
          message: data.new_alert,
          timestamp: new Date().toLocaleTimeString(),
          isNew: true
        }, ...prev]);
      }
    } catch (error) {
      setConnected(false);
    }
  };

  useEffect(() => {
    const timer = setInterval(fetchRobotData, 5000);
    return () => clearInterval(timer);
  }, []);

  const clearNew = (id: number) => {
    setAlerts((prev) => {
      return prev.map((item) => {
        if (item.id === id) {
          return { ...item, isNew: false };
        }
        return item;
      });
    });
  };

  return { alerts, connected, battery, clearNew };
};