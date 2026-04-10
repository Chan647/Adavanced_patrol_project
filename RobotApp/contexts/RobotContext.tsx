/* RobotContext.tsx */

import React, { createContext, useContext, useState, useEffect, useRef } from 'react';
import axios from 'axios';

// 타입 정의
export type RobotStatus = 'idle' | 'charging' | 'running' | 'stopped' | 'manual';
export type RobotMode = 'auto' | 'manual' | 'stopped';

interface RobotPosition {
  x: number;
  y: number;
  theta?: number; // 서버의 yaw 값에 해당
  label?: string;
  type?: 'start' | 'waypoint';
  section?: 'A' | 'B';
}

interface LogEntry {
  id: string;
  situation: string;
  position: string;
  time: string;
  hasImage: boolean;
  imageUrl: string;
}

interface RobotContextType {
  status: RobotStatus;
  rawDriveStatus: string; 
  mode: RobotMode;
  batteryLevel: number;
  isCharging: boolean;
  connectionStatus: string;
  position: RobotPosition;
  location: string;
  logs: LogEntry[];
  autoMode: 'once' | 'loop';
  routePoints: RobotPosition[];
  plannedPath: RobotPosition[];
  API_BASE_URL: string; 

  isFire: boolean;
  isPerson: boolean;
  isIntruder: boolean;
  isTheft: boolean;
  isFireDetectionOn: boolean;
  isPersonDetectionOn: boolean;

  setMode: (mode: RobotMode) => void;
  setAutoMode: (mode: 'once' | 'loop') => void;
  setRoutePoints: (points: RobotPosition[]) => void;
  startRobot: (params?: any) => Promise<void>; 
  stopRobot: () => Promise<void>; 
  startCharging: () => Promise<void>; 
  stopCharging: () => Promise<void>; 
  refreshLogs: () => Promise<LogEntry[]>;
  API_URL: (path: string) => string; 
}

const RobotContext = createContext<RobotContextType | undefined>(undefined);

const API_BASE_URL = 'http://192.168.0.24:5000'; 

export function RobotProvider({ children }: { children: React.ReactNode }) {
  const [battery, setBattery] = useState(0);
  const [driveStatus, setDriveStatus] = useState('IDLE');
  const [isCharging, setIsCharging] = useState(false);
  const [connection, setConnection] = useState('None');
  const [mode, setMode] = useState<RobotMode>('stopped');
  const [position, setPosition] = useState<RobotPosition>({ x: 0, y: 0, theta: 0 });
  const [autoMode, setAutoMode] = useState<'once' | 'loop'>('once');
  const [routePoints, setRoutePoints] = useState<RobotPosition[]>([]);
  const [plannedPath, setPlannedPath] = useState<RobotPosition[]>([]);
  const [logs, setLogs] = useState<LogEntry[]>([]);

  const [isFire, setIsFire] = useState(false);
  const [isPerson, setIsPerson] = useState(false);
  const [isIntruder, setIsIntruder] = useState(false);
  const [isTheft, setIsTheft] = useState(false);
  const [isFireDetectionOn, setIsFireDetectionOn] = useState(true); 
  const [isPersonDetectionOn, setIsPersonDetectionOn] = useState(true); 

  const isMounted = useRef(true);

  const API_URL = (path: string) => `${API_BASE_URL}${path}`;

  const fetchRobotData = async () => {
    try {
      const response = await axios.get(API_URL(`/api/robot/status?t=${Date.now()}`));
      const data = response.data;

      if (isMounted.current) {
        setBattery(data.battery || 0);
        setDriveStatus(data.drive_status || 'IDLE');
        setConnection(data.connection || 'Online');
        setIsCharging(data.isCharging || false);
        setPlannedPath(data.planned_path || []);

        if (data.x !== undefined && data.y !== undefined) {
          setPosition({ x: data.x, y: data.y, theta: data.theta || 0 });
        }
      }
    } catch (err) {
      if (isMounted.current) setConnection('Offline');
    }
  };

  useEffect(() => {
    isMounted.current = true;
    fetchRobotData();
    const interval = setInterval(fetchRobotData, 1000); 
    return () => {
      isMounted.current = false;
      clearInterval(interval);
    };
  }, []);

  const getMappedStatus = (): RobotStatus => {
    if (connection === 'Offline') return 'stopped';
    if (isCharging) return 'charging';
    if (['RUNNING', 'PATROLLING'].some(s => driveStatus.toUpperCase().includes(s))) return 'running';
    return 'idle';
  };

  /**
   * 🚀 핵심 수정 구간: startRobot
   * 서버 C++ 코드의 if (cmd == "once" || cmd == "loop") 조건문에 맞춥니다.
   */
  const startRobot = async (params: any = {}) => {
    try {
      // 1. 서버에 보낼 웨이포인트 데이터 가공
      const formattedWaypoints = routePoints.map(p => ({
        type: p.type || 'waypoint',
        section: p.section || 'A',
        label: p.label || 'POINT',
        x: Number(p.x.toFixed(2)),
        y: Number(p.y.toFixed(2)),
        yaw: Number((p.theta || 0).toFixed(3))
      }));

      // 2. 서버 명령 전송
      // 서버 C++ 코드(132라인)가 인식할 수 있도록 command를 autoMode('once' 또는 'loop')로 보냅니다.
      await axios.post(API_URL('/api/robot/command'), { 
        command: autoMode, // 'WAYPOINT_NAVIGATION' 대신 'once' 또는 'loop' 전달
        waypoints: formattedWaypoints,
        ...params 
      });
      
      setMode('auto');
    } catch (err) {
      console.error("로봇 주행 시작 실패:", err);
    }
  };

  const stopRobot = async () => {
    try {
      await axios.post(API_URL('/api/robot/command'), { command: 'stop' });
      setMode('stopped');
    } catch (err) {
      console.error("로봇 정지 실패:", err);
    }
  };

  const startCharging = async () => {
    try {
      // 서버 코드의 charging 로직(165라인)에 맞춰 'charging' 명령 전송
      await axios.post(API_URL('/api/robot/command'), { command: 'charging' });
    } catch (err) {
      console.error("충전 이동 실패:", err);
    }
  };

  const stopCharging = async () => {
    try {
      await axios.post(API_URL('/api/robot/command'), { command: 'stop' });
      setMode('stopped');
    } catch (err) {
      console.error("충전 모드 해제 실패:", err);
    }
  };

  const refreshLogs = async () => {
    try {
      const res = await axios.get(API_URL('/api/logs'));
      setLogs(res.data);
      return res.data;
    } catch (err) {
      console.error("로그 갱신 실패:", err);
      return [];
    }
  };

  return (
    <RobotContext.Provider
      value={{
        status: getMappedStatus(),
        rawDriveStatus: driveStatus,
        mode,
        batteryLevel: battery,
        isCharging,
        connectionStatus: connection,
        position,
        logs,
        autoMode,
        routePoints,
        location: "실시간 위치 추적 중",
        plannedPath,
        API_BASE_URL,
        API_URL,
        isFire,
        isPerson,
        isIntruder,
        isTheft,
        isFireDetectionOn,
        isPersonDetectionOn,
        setMode,
        setAutoMode,
        setRoutePoints,
        startRobot,
        stopRobot,
        startCharging,
        stopCharging,
        refreshLogs,
      }}
    >
      {children}
    </RobotContext.Provider>
  );
}

export function useRobot() {
  const context = useContext(RobotContext);
  if (!context) throw new Error('useRobot must be used within a RobotProvider');
  return context;
}