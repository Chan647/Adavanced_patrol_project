/* WaitScreen.tsx */

import React from 'react';
import {
  View,
  Text,
  TouchableOpacity,
  StyleSheet,
  Dimensions,
} from 'react-native';
import { useNavigation } from '@react-navigation/native';
import { Home, Battery, Sparkles } from 'lucide-react-native';
import { useRobot } from '../contexts/RobotContext';
import { SafeAreaView } from 'react-native-safe-area-context';

const { width } = Dimensions.get('window');

export function WaitScreen() {
  const navigation = useNavigation<any>();
  const { startCharging } = useRobot();

  const handleCharge = () => {
    startCharging();
    navigation.navigate('ChargingScreen');
  };

  const handleGoMain = () => {
    navigation.navigate('MainScreen');
  };

  return (
    <SafeAreaView style={styles.container}>
      <View style={styles.content}>
        
        {/* 중앙 카드 섹션 */}
        <View style={styles.card}>
          <View style={styles.iconContainer}>
            {/* 메인 아이콘 박스 */}
            <View style={styles.homeIconBox}>
              <Home size={64} color="white" strokeWidth={2} />
            </View>
            
            {/* 반짝이 효과 아이콘 */}
            <View style={styles.sparkleBadge}>
              <Sparkles size={40} color="#facc15" />
            </View>
          </View>

          <Text style={styles.title}>대기 중</Text>
          <Text style={styles.subtitle}>
            로봇이 대기 상태입니다.{"\n"}작업을 시작하세요.
          </Text>

          {/* 버튼 영역 */}
          <View style={styles.buttonRow}>
            <TouchableOpacity 
              activeOpacity={0.8}
              onPress={handleGoMain}
              style={[styles.button, styles.mainButton]}
            >
              <Home size={24} color="white" />
              <Text style={styles.buttonText}>메인 이동</Text>
            </TouchableOpacity>

            <TouchableOpacity 
              activeOpacity={0.8}
              onPress={handleCharge}
              style={[styles.button, styles.chargeButton]}
            >
              <Battery size={24} color="white" />
              <Text style={styles.buttonText}>충전</Text>
            </TouchableOpacity>
          </View>
        </View>

      </View>
    </SafeAreaView>
  );
}

const styles = StyleSheet.create({
  container: {
    flex: 1,
    backgroundColor: '#f8fafc', // slate-50
  },
  content: {
    flex: 1,
    alignItems: 'center',
    justifyContent: 'center',
    padding: 24,
  },
  card: {
    width: '100%',
    backgroundColor: 'white',
    borderRadius: 40,
    paddingVertical: 60,
    paddingHorizontal: 20,
    alignItems: 'center',
    // iOS 그림자
    shadowColor: '#000',
    shadowOffset: { width: 0, height: 20 },
    shadowOpacity: 0.1,
    shadowRadius: 30,
    // Android 그림자
    elevation: 10,
    borderWidth: 1,
    borderColor: 'rgba(255,255,255,0.5)',
  },
  iconContainer: {
    position: 'relative',
    marginBottom: 30,
  },
  homeIconBox: {
    width: 120,
    height: 120,
    backgroundColor: '#3b82f6', // blue-500
    borderRadius: 35,
    alignItems: 'center',
    justifyContent: 'center',
    shadowColor: '#3b82f6',
    shadowOpacity: 0.4,
    shadowRadius: 15,
    elevation: 8,
  },
  sparkleBadge: {
    position: 'absolute',
    top: -15,
    right: -15,
  },
  title: {
    fontSize: 48,
    fontWeight: 'bold',
    color: '#1e293b', // slate-800
    marginBottom: 12,
  },
  subtitle: {
    fontSize: 18,
    color: '#64748b', // slate-600
    textAlign: 'center',
    lineHeight: 26,
    marginBottom: 40,
  },
  buttonRow: {
    flexDirection: 'row',
    gap: 12,
  },
  button: {
    flexDirection: 'row',
    alignItems: 'center',
    justifyContent: 'center',
    paddingVertical: 18,
    paddingHorizontal: 24,
    borderRadius: 20,
    gap: 8,
    minWidth: 140,
  },
  mainButton: {
    backgroundColor: '#4f46e5', // indigo-600
    shadowColor: '#4f46e5',
    shadowOpacity: 0.3,
    shadowRadius: 10,
    elevation: 5,
  },
  chargeButton: {
    backgroundColor: '#10b981', // emerald-500
    shadowColor: '#10b981',
    shadowOpacity: 0.3,
    shadowRadius: 10,
    elevation: 5,
  },
  buttonText: {
    color: 'white',
    fontSize: 16,
    fontWeight: 'bold',
  },
});