/* ChargingScreen.tsx */

import React, { useEffect, useRef } from 'react';
import {
  View,
  Text,
  TouchableOpacity,
  StyleSheet,
  Animated,
  Dimensions,
  Platform,
} from 'react-native';
import { useNavigation } from '@react-navigation/native';
import { Battery, CheckCircle } from 'lucide-react-native'; // 앱용 라이브러리
import { useRobot } from '../contexts/RobotContext';
import { SafeAreaView } from 'react-native-safe-area-context';

const { width } = Dimensions.get('window');

export default function ChargingScreen() {
  const navigation = useNavigation<any>();
  const { batteryLevel, stopCharging } = useRobot();
  
  // 애니메이션 값 (핑 효과 및 배터리 바)
  const pingAnim = useRef(new Animated.Value(1)).current;

  useEffect(() => {
    if (batteryLevel < 100) {
      Animated.loop(
        Animated.sequence([
          Animated.timing(pingAnim, {
            toValue: 1.5,
            duration: 1500,
            useNativeDriver: true,
          }),
          Animated.timing(pingAnim, {
            toValue: 1,
            duration: 0,
            useNativeDriver: true,
          }),
        ])
      ).start();
    }
  }, [batteryLevel]);

  const handleComplete = () => {
    stopCharging();
    navigation.navigate('MainScreen'); // 앱 내 메인화면 경로명 확인 필요
  };

  const getBatteryColor = () => {
    if (batteryLevel > 80) return '#10b981'; // emerald-500
    if (batteryLevel > 50) return '#f59e0b'; // orange-500
    return '#ef4444'; // red-500
  };

  return (
    <SafeAreaView style={styles.container}>
      <View style={styles.card}>
        <View style={styles.iconContainer}>
          {/* 핑 애니메이션 효과 (충전 중일 때만) */}
          {batteryLevel < 100 && (
            <Animated.View
              style={[
                styles.pingCircle,
                {
                  transform: [{ scale: pingAnim }],
                  opacity: pingAnim.interpolate({
                    inputRange: [1, 1.5],
                    outputRange: [0.3, 0],
                  }),
                },
              ]}
            />
          )}
          
          <View style={[styles.batteryCircle, { backgroundColor: getBatteryColor() }]}>
            <Battery size={80} color="white" strokeWidth={2} />
          </View>
        </View>

        <Text style={styles.statusText}>
          {batteryLevel === 100 ? '충전 완료' : '충전 중...'}
        </Text>

        <View style={styles.progressSection}>
          <Text style={[styles.percentageText, { color: getBatteryColor() }]}>
            {batteryLevel}%
          </Text>
          
          {/* 배터리 프로그레스 바 */}
          <View style={styles.progressBarBg}>
            <View
              style={[
                styles.progressBarFill,
                { width: `${batteryLevel}%`, backgroundColor: getBatteryColor() },
              ]}
            />
          </View>
        </View>

        {batteryLevel === 100 && (
          <View style={styles.completeMsg}>
            <CheckCircle size={24} color="#16a34a" />
            <Text style={styles.completeText}>배터리가 완전히 충전되었습니다</Text>
          </View>
        )}

        <TouchableOpacity
          activeOpacity={0.8}
          onPress={handleComplete}
          style={styles.button}
        >
          <Text style={styles.buttonText}>완료</Text>
        </TouchableOpacity>
      </View>
    </SafeAreaView>
  );
}

const styles = StyleSheet.create({
  container: {
    flex: 1,
    backgroundColor: '#0f172a', // slate-950 느낌
    alignItems: 'center',
    justifyContent: 'center',
    padding: 20,
  },
  card: {
    width: '100%',
    maxWidth: 500,
    backgroundColor: 'rgba(255, 255, 255, 0.95)',
    borderRadius: 30,
    padding: 40,
    alignItems: 'center',
    ...Platform.select({
      ios: { shadowColor: '#000', shadowOffset: { width: 0, height: 10 }, shadowOpacity: 0.3, shadowRadius: 20 },
      android: { elevation: 10 },
    }),
  },
  iconContainer: {
    position: 'relative',
    marginBottom: 30,
    alignItems: 'center',
    justifyContent: 'center',
  },
  batteryCircle: {
    width: 150,
    height: 150,
    borderRadius: 75,
    alignItems: 'center',
    justifyContent: 'center',
    zIndex: 2,
  },
  pingCircle: {
    position: 'absolute',
    width: 150,
    height: 150,
    borderRadius: 75,
    borderWidth: 4,
    borderColor: 'rgba(0,0,0,0.1)',
  },
  statusText: {
    fontSize: 32,
    fontWeight: 'bold',
    color: '#1e293b',
    marginBottom: 20,
  },
  progressSection: {
    width: '100%',
    alignItems: 'center',
    marginBottom: 30,
  },
  percentageText: {
    fontSize: 60,
    fontWeight: 'bold',
    marginBottom: 15,
  },
  progressBarBg: {
    width: '100%',
    height: 24,
    backgroundColor: '#e2e8f0',
    borderRadius: 12,
    overflow: 'hidden',
  },
  progressBarFill: {
    height: '100%',
    borderRadius: 12,
  },
  completeMsg: {
    flexDirection: 'row',
    alignItems: 'center',
    marginBottom: 20,
  },
  completeText: {
    marginLeft: 8,
    fontSize: 16,
    fontWeight: '600',
    color: '#16a34a',
  },
  button: {
    width: '100%',
    backgroundColor: '#2563eb',
    paddingVertical: 18,
    borderRadius: 20,
    alignItems: 'center',
    shadowColor: '#2563eb',
    shadowOffset: { width: 0, height: 4 },
    shadowOpacity: 0.3,
    shadowRadius: 8,
    elevation: 5,
  },
  buttonText: {
    color: 'white',
    fontSize: 20,
    fontWeight: 'bold',
  },
});