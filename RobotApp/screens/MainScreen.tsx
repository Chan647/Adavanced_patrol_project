/* MainScreen.tsx */

import React, { useState } from 'react';
import {
  View, Text, StyleSheet, TouchableOpacity, Image, Switch, Dimensions, Alert
} from 'react-native';
import { SafeAreaView } from 'react-native-safe-area-context';
import { useNavigation } from '@react-navigation/native';
import { WebView } from 'react-native-webview';
import { Picker } from '@react-native-picker/picker';
import { 
  Activity, MapPin, Navigation, Zap, Gamepad2, 
  Square, Play, History, Camera, Home
} from 'lucide-react-native';

import { useRobot } from '../contexts/RobotContext'; 
import { StatusBar } from './StatusBar'; 
import { LogHistoryModal } from './LogModal'; 
import { RouteSettingModal } from './RouteSettingModal'; 

const { width: SCREEN_WIDTH } = Dimensions.get('window');
const BUTTON_WIDTH = (SCREEN_WIDTH - 70) / 6; 

// 포인트 데이터 인터페이스 정의
interface Point {
  id: string;
  type: 'start' | 'waypoint';
  section: string;
  x: number;
  y: number;
  yaw: number;
}

export default function MainScreen() {
  const navigation = useNavigation<any>();
  const { API_URL, startRobot, stopRobot, autoMode, setAutoMode, startCharging } = useRobot();
  
  const [controlMode, setControlMode] = useState<'auto' | 'manual'>('auto');
  const [isVoiceOn, setIsVoiceOn] = useState(false);
  const [isFireOn, setIsFireOn] = useState(false);
  const [isPersonOn, setIsPersonOn] = useState(false);

  // [수정] 좌표 리스트를 MainScreen에서 관리하여 모달과 공유합니다.
  const [points, setPoints] = useState<Point[]>([
    { id: 'start', type: 'start', section: 'A', x: 0, y: 0, yaw: 0 }
  ]);
  const [selectedPath, setSelectedPath] = useState('start');

  const [isLogVisible, setIsLogVisible] = useState(false); 
  const [isRouteVisible, setIsRouteVisible] = useState(false); 

  // [추가] 지점의 라벨(A-1, B-1 등)을 계산하는 로직
  const getPointLabel = (currentIndex: number) => {
    const currentPoint = points[currentIndex];
    if (currentPoint.type === 'start') return 'START';

    const sectionType = currentPoint.section || 'A';
    let count = 0;
    for (let i = 0; i <= currentIndex; i++) {
      if (points[i].type === 'waypoint' && points[i].section === sectionType) {
        count++;
      }
    }
    return `${sectionType}-${count}`;
  };

  return (
    <SafeAreaView style={styles.container}>
      <StatusBar />

      {/* 모니터링 섹션 */}
      <View style={styles.topSection}>
        <View style={styles.card}>
          <View style={styles.cardHeader}>
            <View style={styles.iconCircle}><Activity size={10} color="white" /></View>
            <Text style={styles.cardLabel}>실시간 카메라</Text>
          </View>
          <View style={styles.mediaBox}>
            <WebView 
              source={{ uri: `http://192.168.0.24:8080/stream?topic=/detection/annotated_image&type=mjpeg` }} 
              style={styles.fullImage} 
              scrollEnabled={false}
            />
          </View>
        </View>

        <View style={styles.card}>
          <View style={styles.cardHeader}>
            <View style={[styles.iconCircle, { backgroundColor: '#00c853' }]}><MapPin size={10} color="white" /></View>
            <Text style={styles.cardLabel}>실시간 지도</Text>
          </View>
          <View style={styles.mediaBox}>
            <Image 
              source={{ uri: API_URL(`/map/current_map.jpg`) }} 
              style={styles.fullImage} 
              resizeMode="contain" 
            />
          </View>
        </View>
      </View>

      {/* 하단 제어 패널 */}
      <View style={styles.controlPanel}>
        <View style={styles.panelHeaderRow}>
          <Text style={styles.panelTitle}>제어 패널</Text>
          <View style={styles.topActionsContainer}>
            <View style={styles.sectionPickerWrapper}>
              <Navigation size={10} color="#2962ff" />
              <Text style={styles.miniLabel}>구역 이동:</Text>
              <View style={styles.dynamicPickerContainer}>
                {/* [수정] pathList 대신 points 배열을 직접 map하여 Picker.Item 생성 */}
                <Picker
                  selectedValue={selectedPath}
                  onValueChange={(val) => setSelectedPath(val)}
                  style={styles.dynamicPicker}
                  dropdownIconColor="#2962ff"
                  mode="dropdown"
                >
                  {points.map((p, index) => (
                    <Picker.Item 
                      key={p.id} 
                      label={getPointLabel(index)} 
                      value={p.id} 
                      style={{ fontSize: 11 }} 
                    />
                  ))}
                </Picker>
              </View>
              <TouchableOpacity style={styles.goBtn}><Text style={styles.goText}>GO</Text></TouchableOpacity>
            </View>

            <View style={styles.togglesGroup}>
              <View style={styles.toggleItem}>
                <Text style={styles.miniText}>음성</Text>
                <Switch value={isVoiceOn} onValueChange={setIsVoiceOn} style={styles.miniSwitch} />
              </View>
              <View style={styles.toggleItem}>
                <Text style={styles.miniText}>화재</Text>
                <Switch value={isFireOn} onValueChange={setIsFireOn} style={styles.miniSwitch} />
              </View>
              <View style={styles.toggleItem}>
                <Text style={styles.miniText}>사람</Text>
                <Switch value={isPersonOn} onValueChange={setIsPersonOn} style={styles.miniSwitch} />
              </View>
            </View>
          </View>
        </View>

        {/* 제어 버튼 그리드 */}
        <View style={styles.mainButtonGrid}>
          <View style={styles.combinedAutoBtn}>
            <TouchableOpacity 
              style={[styles.autoTopPart, controlMode === 'auto' ? styles.activeBlue : styles.inactiveGray]} 
              onPress={() => setControlMode('auto')}
            >
              <Zap size={14} color={controlMode === 'auto' ? "white" : "#64748b"} />
              <Text style={[styles.btnLabel, controlMode === 'auto' && styles.whiteText]}>자동</Text>
            </TouchableOpacity>
            <View style={[styles.autoBottomPart, controlMode === 'auto' ? styles.borderBlue : styles.borderGray]}>
              <Picker selectedValue={autoMode} onValueChange={(v) => setAutoMode(v)} style={styles.integratedPicker}>
                <Picker.Item label="1회" value="once" style={{fontSize: 14}} />
                <Picker.Item label="무한반복" value="loop" style={{fontSize: 14}} />
              </Picker>
            </View>
          </View>

          <TouchableOpacity style={[styles.squareBtn, styles.inactiveGray]} onPress={() => navigation.navigate('ManualScreen')}>
            <Gamepad2 size={18} color="#64748b" />
            <Text style={styles.btnLabel}>수동</Text>
          </TouchableOpacity>

          <TouchableOpacity style={[styles.squareBtn, styles.stopBtn]} onPress={() => stopRobot()}>
            <Square size={14} color="white" fill="white" />
            <Text style={styles.whiteBtnLabel}>정지</Text>
          </TouchableOpacity>

          <TouchableOpacity style={[styles.squareBtn, styles.startBtn]} onPress={() => startRobot()}>
            <Play size={14} color="white" fill="white" />
            <Text style={styles.whiteBtnLabel}>시작</Text>
          </TouchableOpacity>

          <TouchableOpacity style={[styles.squareBtn, styles.logBtn]} onPress={() => setIsLogVisible(true)}>
            <History size={16} color="white" />
            <Text style={styles.whiteBtnLabel}>로그</Text>
          </TouchableOpacity>

          <TouchableOpacity style={[styles.squareBtn, styles.panoBtn]} onPress={() => navigation.navigate('PanoramaScreen')}>
            <Camera size={16} color="white" />
            <Text style={styles.whiteBtnLabel}>사진</Text>
          </TouchableOpacity>
        </View>

        {/* 하단 버튼 바 */}
        <View style={styles.footerRow}>
          <TouchableOpacity 
            style={styles.wideBlueBtn} 
            onPress={() => setIsRouteVisible(true)}
          >
            <Navigation size={18} color="white" style={{marginRight: 6}} />
            <Text style={styles.footerBtnText}>경로 지정</Text>
          </TouchableOpacity>

          <TouchableOpacity 
            style={styles.wideDarkBtn}
            onPress={async () => {
              try {
                await startCharging();
                navigation.navigate('WaitScreen');
              } catch (err) {
                Alert.alert("에러", "대기 이동 명령을 전송하지 못했습니다.");
              }
            }}
          >
            <Home size={18} color="white" style={{marginRight: 6}} />
            <Text style={styles.footerBtnText}>대기 이동</Text>
          </TouchableOpacity>
        </View>
      </View>

      {/* 시스템 로그 모달 */}
      <LogHistoryModal 
        visible={isLogVisible} 
        onClose={() => setIsLogVisible(false)} 
      />

      {/* 경로 설정 모달 - [수정] points와 setPoints를 props로 전달합니다. */}
      <RouteSettingModal 
        visible={isRouteVisible} 
        onClose={() => setIsRouteVisible(false)}
        points={points}
        setPoints={setPoints}
      />

    </SafeAreaView>
  );
}

const styles = StyleSheet.create({
  container: { flex: 1, backgroundColor: '#f0f3f6' },
  topSection: { flexDirection: 'row', justifyContent: 'space-between', height: '36%', padding: 10 },
  card: { width: '48.5%', backgroundColor: 'white', borderRadius: 12, padding: 8, elevation: 2 },
  cardHeader: { flexDirection: 'row', alignItems: 'center', marginBottom: 5 },
  iconCircle: { width: 16, height: 16, borderRadius: 8, backgroundColor: '#2962ff', justifyContent: 'center', alignItems: 'center', marginRight: 4 },
  cardLabel: { fontWeight: '700', fontSize: 10, color: '#333' },
  mediaBox: { flex: 1, backgroundColor: '#f8f9fb', borderRadius: 8, overflow: 'hidden' },
  fullImage: { width: '100%', height: '100%' },

  controlPanel: { backgroundColor: 'white', marginHorizontal: 10, marginBottom: 10, borderRadius: 20, padding: 12, flex: 1, elevation: 4 },
  panelHeaderRow: { marginBottom: 12 },
  panelTitle: { fontSize: 14, fontWeight: '800', marginBottom: 8 },
  topActionsContainer: { flexDirection: 'row', justifyContent: 'space-between', alignItems: 'center' },
  
  sectionPickerWrapper: { flexDirection: 'row', alignItems: 'center', backgroundColor: '#f1f4f8', borderRadius: 10, paddingHorizontal: 8, height: 32, marginRight: 8 },
  dynamicPickerContainer: { width: 85, justifyContent: 'center', overflow: 'hidden' },
  dynamicPicker: { width: 120, height: 32, marginLeft: -5 },
  miniLabel: { fontSize: 9, color: '#2962ff', fontWeight: 'bold', marginLeft: 4, marginRight: 2 },
  goBtn: { backgroundColor: '#4d61ff', paddingHorizontal: 8, height: 20, borderRadius: 10, justifyContent: 'center' },
  goText: { color: 'white', fontSize: 9, fontWeight: 'bold' },

  togglesGroup: { flexDirection: 'row', gap: 4 },
  toggleItem: { flexDirection: 'row', alignItems: 'center', backgroundColor: '#f8fafc', paddingHorizontal: 4, borderRadius: 8 },
  miniText: { fontSize: 8, color: '#64748b' },
  miniSwitch: { transform: [{ scaleX: 0.45 }, { scaleY: 0.45 }], marginLeft: -6, marginRight: -4 },

  mainButtonGrid: { flexDirection: 'row', justifyContent: 'space-between', alignItems: 'center', marginBottom: 15 },
  combinedAutoBtn: { width: BUTTON_WIDTH, height: 70 },
  autoTopPart: { flex: 1.4, borderTopLeftRadius: 12, borderTopRightRadius: 12, justifyContent: 'center', alignItems: 'center' },
  autoBottomPart: { flex: 1, borderBottomLeftRadius: 12, borderBottomRightRadius: 12, borderWidth: 1, borderTopWidth: 0, backgroundColor: 'white', justifyContent: 'center' },
  borderBlue: { borderColor: '#2962ff' },
  borderGray: { borderColor: '#cbd5e1' },
  inactiveGray: { backgroundColor: '#f1f4f8' },
  integratedPicker: { width: '135%', height: 30, marginLeft: -8 },

  squareBtn: { width: BUTTON_WIDTH, height: 70, borderRadius: 12, justifyContent: 'center', alignItems: 'center' },
  activeBlue: { backgroundColor: '#2962ff' },
  stopBtn: { backgroundColor: '#ff1744' },
  startBtn: { backgroundColor: '#00c853' },
  logBtn: { backgroundColor: '#af52de' },
  panoBtn: { backgroundColor: '#ff6d00' },
  btnLabel: { fontSize: 9, fontWeight: 'bold', color: '#64748b', marginTop: 2 },
  whiteBtnLabel: { fontSize: 9, fontWeight: 'bold', color: 'white', marginTop: 2 },
  whiteText: { color: 'white' },

  footerRow: { flexDirection: 'row', gap: 8 },
  wideBlueBtn: { flex: 1, height: 50, backgroundColor: '#4d61ff', borderRadius: 14, flexDirection: 'row', justifyContent: 'center', alignItems: 'center' },
  wideDarkBtn: { flex: 1, height: 50, backgroundColor: '#37474f', borderRadius: 14, flexDirection: 'row', justifyContent: 'center', alignItems: 'center' },
  footerBtnText: { color: 'white', fontWeight: 'bold', fontSize: 14 },
});