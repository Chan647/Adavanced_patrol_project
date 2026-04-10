/* ManualScreen.tsx - 전체 코드 */

import React, { useState, useEffect, useRef } from 'react';
import {
  View,
  Text,
  TouchableOpacity,
  StyleSheet,
  Dimensions,
} from 'react-native';
import { useNavigation } from '@react-navigation/native';
import axios from 'axios';
import { Home, ArrowUp, ArrowDown, ArrowLeft, ArrowRight, StopCircle, Camera } from 'lucide-react-native';
import { useRobot } from '../contexts/RobotContext';
import { SafeAreaView } from 'react-native-safe-area-context';
import { WebView } from 'react-native-webview';

const { width } = Dimensions.get('window');

export function ManualScreen() {
  const navigation = useNavigation<any>();
  const { setMode } = useRobot();
  const [activeKey, setActiveKey] = useState<string | null>(null);

  // 🚀 타이머(Interval)를 관리하기 위한 Ref (화면 리렌더링과 관계없이 값 유지)
  const timerRef = useRef<NodeJS.Timeout | null>(null);

  // 로봇 서버로 명령을 전송하는 함수
  const sendCommand = async (action: string) => {
    try {
      await axios.post('http://192.168.0.24:5000/api/robot/manual', { action });
    } catch (err) {
      console.error("명령 전송 실패:", err);
    }
  };

  // 🚀 버튼을 누르기 시작했을 때 (onPressIn)
  const handlePressIn = (direction: string, action: string) => {
    setActiveKey(direction);

    // 1. 혹시 실행 중인 기존 타이머가 있다면 먼저 제거 (중복 방지)
    if (timerRef.current) {
      clearInterval(timerRef.current);
    }

    // 2. 즉시 한 번 명령을 전송
    sendCommand(action);

    // 3. 200ms(0.2초)마다 반복적으로 명령 전송
    // 로봇이 '계속 눌려있음'을 인지하게 하여 패킷 유실 시에도 주행 유지
    timerRef.current = setInterval(() => {
      sendCommand(action);
    }, 200);
  };

  // 🚀 버튼에서 손을 뗐을 때 (onPressOut)
  const handlePressOut = () => {
    setActiveKey(null);

    // 1. 반복 전송 중인 타이머를 즉시 중단
    if (timerRef.current) {
      clearInterval(timerRef.current);
      timerRef.current = null;
    }

    // 2. 즉시 정지(STOP) 명령 전송
    sendCommand('STOP');
  };

  // 메인 화면으로 돌아가기
  const handleExit = async () => {
    // 혹시 모를 안전을 위해 타이머 정리 및 정지 명령
    if (timerRef.current) clearInterval(timerRef.current);
    await sendCommand('STOP');
    setMode('stopped');
    navigation.navigate('MainScreen');
  };

  // 컴포넌트 마운트/언마운트 시 처리
  useEffect(() => {
    setMode('manual');
    return () => {
      // 화면을 나갈 때 반드시 타이머를 죽이고 로봇을 멈춤
      if (timerRef.current) clearInterval(timerRef.current);
      sendCommand('STOP');
      setMode('stopped');
    };
  }, []);

  // 방향 버튼 컴포넌트 (내부 재사용)
  const DirectionButton = ({ direction, action, Icon }: any) => (
    <TouchableOpacity
      activeOpacity={1}
      onPressIn={() => handlePressIn(direction, action)}
      onPressOut={handlePressOut}
      style={[
        styles.dirButton,
        activeKey === direction ? styles.dirButtonActive : styles.dirButtonInactive
      ]}
    >
      <Icon size={40} color="white" strokeWidth={3} />
      <Text style={styles.dirText}>{direction.toUpperCase()}</Text>
    </TouchableOpacity>
  );

  return (
    <SafeAreaView style={styles.container}>
      <View style={styles.content}>
        
        {/* 카메라 스트리밍 섹션 */}
        <View style={styles.cameraCard}>
          <View style={styles.cardHeader}>
            <View style={styles.headerTitleRow}>
              <View style={styles.iconBadge}>
                <Camera size={18} color="white" />
              </View>
              <Text style={styles.cardTitle}>수동 조작 모드</Text>
            </View>
            <View style={styles.manualBadge}>
              <Text style={styles.manualBadgeText}>MANUAL</Text>
            </View>
          </View>

          <View style={styles.cameraWrapper}>
            <WebView
              source={{ uri: 'http://192.168.0.24:8080/stream?topic=/detection/annotated_image&type=mjpeg' }}
              style={styles.cameraStream}
              scrollEnabled={false}
              containerStyle={{ borderRadius: 16 }}
            />
          </View>
        </View>

        {/* 조작 패널 */}
        <View style={styles.controlPanel}>
          <View style={styles.row}>
            <View style={styles.empty} />
            <DirectionButton direction="up" action="FORWARD" Icon={ArrowUp} />
            <View style={styles.empty} />
          </View>

          <View style={styles.row}>
            <DirectionButton direction="left" action="LEFT" Icon={ArrowLeft} />
            
            {/* STOP 버튼 (누르고 있을 때만 빨간색 피드백) */}
            <TouchableOpacity 
              style={[styles.stopCircle, activeKey === 'stop' && styles.stopCircleActive]}
              onPressIn={() => handlePressIn('stop', 'STOP')}
              onPressOut={handlePressOut}
              activeOpacity={1}
            >
              <StopCircle size={32} color={activeKey === 'stop' ? '#ef4444' : '#64748b'} />
              <Text style={styles.stopText}>STOP</Text>
            </TouchableOpacity>

            <DirectionButton direction="right" action="RIGHT" Icon={ArrowRight} />
          </View>

          <View style={styles.row}>
            <View style={styles.empty} />
            <DirectionButton direction="down" action="BACKWARD" Icon={ArrowDown} />
            <View style={styles.empty} />
          </View>
        </View>

        <TouchableOpacity style={styles.exitButton} onPress={handleExit}>
          <Home size={24} color="white" />
          <Text style={styles.exitButtonText}>메인 화면 이동</Text>
        </TouchableOpacity>

      </View>
    </SafeAreaView>
  );
}

const styles = StyleSheet.create({
  container: { flex: 1, backgroundColor: '#f8fafc' },
  content: { flex: 1, padding: 20, justifyContent: 'space-between' },
  cameraCard: {
    backgroundColor: 'white',
    borderRadius: 24,
    padding: 15,
    elevation: 5,
  },
  cardHeader: { flexDirection: 'row', justifyContent: 'space-between', alignItems: 'center', marginBottom: 12 },
  headerTitleRow: { flexDirection: 'row', alignItems: 'center' },
  iconBadge: { backgroundColor: '#f97316', padding: 8, borderRadius: 10, marginRight: 8 },
  cardTitle: { fontSize: 18, fontWeight: 'bold', color: '#1e293b' },
  manualBadge: { backgroundColor: '#f97316', paddingHorizontal: 12, paddingVertical: 4, borderRadius: 8 },
  manualBadgeText: { color: 'white', fontSize: 12, fontWeight: 'bold' },
  cameraWrapper: { width: '100%', aspectRatio: 16 / 9, backgroundColor: '#0f172a', borderRadius: 16, overflow: 'hidden' },
  cameraStream: { flex: 1 },

  controlPanel: { alignItems: 'center', marginVertical: 30 },
  row: { flexDirection: 'row', gap: 20, marginBottom: 20 },
  empty: { width: 90, height: 90 },
  dirButton: {
    width: 90,
    height: 90,
    borderRadius: 25,
    justifyContent: 'center',
    alignItems: 'center',
    elevation: 6,
  },
  dirButtonInactive: { backgroundColor: '#3b82f6' },
  dirButtonActive: { backgroundColor: '#1d4ed8', transform: [{ scale: 0.92 }] },
  dirText: { color: 'rgba(255,255,255,0.6)', fontSize: 10, fontWeight: 'bold', marginTop: 4 },
  
  stopCircle: {
    width: 90,
    height: 90,
    borderRadius: 25,
    backgroundColor: '#e2e8f0',
    justifyContent: 'center',
    alignItems: 'center',
    borderWidth: 2,
    borderColor: '#cbd5e1',
  },
  stopCircleActive: { backgroundColor: '#fee2e2', borderColor: '#fca5a5' },
  stopText: { fontSize: 10, fontWeight: 'bold', color: '#64748b', marginTop: 4 },

  exitButton: {
    backgroundColor: '#334155',
    flexDirection: 'row',
    justifyContent: 'center',
    alignItems: 'center',
    paddingVertical: 18,
    borderRadius: 20,
    gap: 10,
  },
  exitButtonText: { color: 'white', fontSize: 18, fontWeight: 'bold' },
});