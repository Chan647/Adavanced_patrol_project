/* StatusBar.tsx */

import React, { useState, useEffect } from 'react';
import { View, Text, StyleSheet, Platform } from 'react-native';
import { Battery, BatteryCharging, Clock, Activity, MapPin, ShieldCheck } from 'lucide-react-native';
import { useRobot } from '../contexts/RobotContext';

export function StatusBar() {
  const {
    rawDriveStatus,
    status,
    batteryLevel,
    isCharging,
    connectionStatus,
    location: robotLocation,
    isFire,
    isPerson,
    isIntruder,
    isTheft,
    isFireDetectionOn,
    isPersonDetectionOn,
  } = useRobot();

  const [time, setTime] = useState(new Date());

  const displayBattery = Math.min(batteryLevel || 0, 100);
  const isOffline = connectionStatus === 'Offline' || connectionStatus === 'None';

  useEffect(() => {
    const timer = setInterval(() => {
      setTime(new Date());
    }, 1000);
    return () => clearInterval(timer);
  }, []);

  // 시간 포맷팅 (React Native 환경 호환)
  const formatTime = (date: Date) => {
    const hours = date.getHours().toString().padStart(2, '0');
    const minutes = date.getMinutes().toString().padStart(2, '0');
    const seconds = date.getSeconds().toString().padStart(2, '0');
    return `${hours}:${minutes}:${seconds}`;
  };

  const getDetectionStatusText = () => {
    if (isTheft) return "🚨 도난!!";
    if (isFire) return "🔥 화재!";
    if (isIntruder) return "🚨 침입!";
    if (isPerson) return "👤 사람";

    if (!isFireDetectionOn && !isPersonDetectionOn) return "OFF";
    const fireStr = isFireDetectionOn ? "화재" : "";
    const personStr = isPersonDetectionOn ? "사람" : "";
    return [personStr, fireStr].filter(Boolean).join('/');
  };

  const getStatusText = () => {
    if (isOffline) return '연결 안됨';
    if (status === 'running') return rawDriveStatus || '주행 중';
    switch (status) {
      case 'idle': return '대기';
      case 'charging': return '충전 중';
      case 'stopped': return '정지';
      case 'manual': return '수동';
      default: return '알 수 없음';
    }
  };

  const getStatusColor = () => {
    if (isOffline) return '#ef4444'; // red-500
    if (isCharging || status === 'charging') return '#22c55e'; // green-500
    switch (status) {
      case 'running': return '#22c55e';
      case 'stopped': return '#ef4444';
      case 'manual': return '#a855f7'; // purple-500
      default: return '#64748b'; // slate-500
    }
  };

  const getBatteryColor = () => {
    if (isOffline) return '#ef4444';
    if (isCharging || displayBattery > 50) return '#4ade80'; // green-400
    if (displayBattery > 20) return '#facc15'; // yellow-400
    return '#ef4444';
  };

  return (
    <View style={styles.container}>
      {/* 상단: 주요 상태 요약 */}
      <View style={styles.topRow}>
        {/* 로봇 주행 상태 */}
        <View style={styles.statusBadge}>
          <Activity size={14} color="white" />
          <View style={[styles.indicator, { backgroundColor: getStatusColor() }]} />
          <Text style={[styles.statusText, { color: isOffline ? '#ef4444' : 'white' }]}>
            {getStatusText()}
          </Text>
        </View>

        {/* 현재 시간 */}
        <View style={styles.timeBadge}>
          <Clock size={14} color="#94a3b8" />
          <Text style={styles.timeText}>{formatTime(time)}</Text>
        </View>
      </View>

      {/* 하단: 상세 데이터 (그리드) */}
      <View style={styles.bottomRow}>
        <View style={styles.infoBox}>
          {isCharging && !isOffline ? (
            <BatteryCharging size={18} color={getBatteryColor()} />
          ) : (
            <Battery size={18} color={getBatteryColor()} />
          )}
          <Text style={[styles.infoValue, { color: getBatteryColor() }]}>
            {isOffline ? '--' : `${displayBattery}%`}
          </Text>
        </View>

        <View style={styles.infoBox}>
          <MapPin size={18} color="white" />
          <Text style={[styles.infoValue, { color: isOffline ? 'white' : '#60a5fa' }]}>
            {isOffline ? '--' : robotLocation}
          </Text>
        </View>

        <View style={styles.infoBox}>
          <ShieldCheck size={18} color="white" />
          <Text style={[styles.infoValue, { color: (isTheft || isFire || isIntruder) ? '#ef4444' : '#10b981' }]}>
            {getDetectionStatusText()}
          </Text>
        </View>
      </View>
    </View>
  );
}

const styles = StyleSheet.create({
  container: {
    backgroundColor: '#1e293b', // slate-800
    paddingTop: Platform.OS === 'ios' ? 10 : 15,
    paddingBottom: 15,
    paddingHorizontal: 16,
    borderBottomWidth: 1,
    borderBottomColor: 'rgba(255,255,255,0.1)',
  },
  topRow: {
    flexDirection: 'row',
    justifyContent: 'space-between',
    alignItems: 'center',
    marginBottom: 12,
  },
  statusBadge: {
    flexDirection: 'row',
    alignItems: 'center',
    backgroundColor: 'rgba(255,255,255,0.1)',
    paddingHorizontal: 10,
    paddingVertical: 5,
    borderRadius: 20,
  },
  indicator: {
    width: 6,
    height: 6,
    borderRadius: 3,
    marginHorizontal: 6,
  },
  statusText: {
    fontSize: 13,
    fontWeight: '700',
  },
  timeBadge: {
    flexDirection: 'row',
    alignItems: 'center',
    gap: 5,
  },
  timeText: {
    color: '#94a3b8',
    fontSize: 12,
    fontWeight: '500',
    fontFamily: Platform.OS === 'ios' ? 'Courier' : 'monospace',
  },
  bottomRow: {
    flexDirection: 'row',
    justifyContent: 'space-between',
    gap: 8,
  },
  infoBox: {
    flex: 1,
    flexDirection: 'row',
    alignItems: 'center',
    justifyContent: 'center',
    backgroundColor: 'rgba(255,255,255,0.05)',
    paddingVertical: 8,
    borderRadius: 12,
    gap: 6,
  },
  infoValue: {
    fontSize: 14,
    fontWeight: 'bold',
  },
});