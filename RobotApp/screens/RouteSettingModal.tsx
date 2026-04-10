/* RouteSettingModal.tsx */

import React, { useState, useEffect } from 'react';
import {
  View, Text, Modal, TouchableOpacity, Image, StyleSheet, 
  Dimensions, Alert, ScrollView
} from 'react-native';
import { SafeAreaView } from 'react-native-safe-area-context';
import { X, MapPin, Home, Navigation, Settings2 } from 'lucide-react-native';
import Svg, { Line } from 'react-native-svg';
import axios from 'axios';
import { useRobot } from '../contexts/RobotContext';
// 1. 순서 편집 모달 임포트
import { OrderEditModal } from './OrderEditModal'; 

const { width: SCREEN_WIDTH, height: SCREEN_HEIGHT } = Dimensions.get('window');
const MAP_WIDTH = SCREEN_WIDTH - 40; 
const MAP_HEIGHT = SCREEN_HEIGHT * 0.4; 

// [수정] 부모로부터 전달받을 Props 타입 정의
interface RouteSettingModalProps {
  visible: boolean;
  onClose: () => void;
  points: any[]; // 추가된 부분
  setPoints: React.Dispatch<React.SetStateAction<any[]>>; // 추가된 부분
}

export function RouteSettingModal({ 
  visible, 
  onClose, 
  points,    // [수정] Props로 받음
  setPoints  // [수정] Props로 받음
}: RouteSettingModalProps) {
  const { setRoutePoints, API_BASE_URL } = useRobot();
  
  // [삭제] const [points, setPoints] = useState<any[]>([]); -> 부모에서 관리하므로 삭제
  
  const [mapMeta, setMapMeta] = useState<any>(null);
  const [mapTick, setMapTick] = useState(Date.now());
  const [showOrderModal, setShowOrderModal] = useState(false); // 순서 편집 모달 상태
  
  const [activeMenuId, setActiveMenuId] = useState<string | null>(null);

  useEffect(() => {
    if (visible) {
      const initData = async () => {
        try {
          const metaRes = await axios.get(`${API_BASE_URL}/api/map/meta`);
          if (metaRes.data && metaRes.data.width > 0) {
            setMapMeta(metaRes.data);
          }
          setMapTick(Date.now());
        } catch (e) {
          console.error("메타데이터 로드 실패", e);
        }
      };
      initData();
    } else {
      setMapMeta(null);
      setActiveMenuId(null);
    }
  }, [visible]);

  const handleComplete = async () => {
    if (!mapMeta || mapMeta.width === 0) {
      Alert.alert("알림", "지도 정보를 불러오는 중입니다.");
      return;
    }

    const realCoordinates = points.map((p, idx) => {
      const label = p.type === 'start' ? 'START' : `W-${idx}`;
      const original_yPx = (1 - (p.x / 100)) * mapMeta.width;
      const original_xPx = (1 - (p.y / 100)) * mapMeta.height;
      const realX = mapMeta.origin_x + (original_xPx * mapMeta.resolution);
      const realY = mapMeta.origin_y + (original_yPx * mapMeta.resolution);

      return {
        type: p.type,
        section: p.section || 'A',
        label: label,
        x: parseFloat(realX.toFixed(2)),
        y: parseFloat(realY.toFixed(2)),
        yaw: parseFloat((p.yaw || 0).toFixed(3))
      };
    });

    try {
      await axios.post(`${API_BASE_URL}/api/robot/command`, {
        command: 'WAYPOINT_NAVIGATION',
        waypoints: realCoordinates
      });
      setRoutePoints(realCoordinates);
      Alert.alert("성공", "경로가 전송되었습니다.");
      onClose();
    } catch (err) {
      Alert.alert("오류", "전송에 실패했습니다.");
    }
  };

  const onMapPress = (evt: any) => {
    if (activeMenuId) {
      setActiveMenuId(null);
      return;
    }
    const { locationX, locationY } = evt.nativeEvent;
    const xPct = (locationX / MAP_WIDTH) * 100;
    const yPct = (locationY / MAP_HEIGHT) * 100;

    const newPoint = {
      id: Date.now().toString(),
      x: xPct,
      y: yPct,
      yaw: 0,
      section: 'A',
      type: points.length === 0 ? 'start' : 'waypoint'
    };
    setPoints([...points, newPoint]);
  };

  const changePointType = (id: string, newType: 'start' | 'waypoint') => {
    setPoints(points.map(p => {
      if (p.id === id) return { ...p, type: newType };
      if (newType === 'start' && p.type === 'start') return { ...p, type: 'waypoint' };
      return p;
    }));
  };

  const removePoint = (id: string) => {
    setPoints(points.filter(pt => pt.id !== id));
  };

  return (
    <Modal visible={visible} animationType="slide" transparent={true}>
      <View style={styles.overlay}>
        <SafeAreaView style={styles.container}>
          <View style={styles.header}>
            <View style={styles.headerLeft}>
              <Navigation size={24} color="#4f46e5" />
              <Text style={styles.headerTitle}>경로 지정</Text>
            </View>
            <TouchableOpacity onPress={onClose}><X size={28} color="#64748b" /></TouchableOpacity>
          </View>

          <ScrollView contentContainerStyle={{ padding: 20 }}>
            <Text style={styles.guideText}>지도를 터치하여 추가하고, 길게 눌러 관리하세요.</Text>
            
            <TouchableOpacity activeOpacity={1} onPress={onMapPress} style={styles.mapWrapper}>
              <Image 
                source={{ uri: `${API_BASE_URL}/map/current_map.jpg?t=${mapTick}` }}
                style={styles.mapImage}
                resizeMode="stretch"
              />

              <Svg style={StyleSheet.absoluteFill}>
                {points.map((p, i) => {
                  if (i === points.length - 1) return null;
                  const next = points[i + 1];
                  return (
                    <Line 
                      key={`line-${p.id}`}
                      x1={`${p.x}%`} y1={`${p.y}%`}
                      x2={`${next.x}%`} y2={`${next.y}%`}
                      stroke="#6366f1" strokeWidth="2" strokeDasharray="5,5"
                    />
                  );
                })}
              </Svg>

              {points.map((p) => {
                const isMenuVisible = activeMenuId === p.id;
                return (
                  <View key={p.id} style={[styles.markerContainer, { left: `${p.x}%`, top: `${p.y}%`, zIndex: isMenuVisible ? 1000 : 100 }]}>
                    {isMenuVisible && (
                      <View style={styles.optionMenu}>
                        <TouchableOpacity style={styles.menuItem} onPress={() => { changePointType(p.id, 'start'); setActiveMenuId(null); }}>
                          <Text style={styles.menuText}>시작점으로 설정</Text>
                        </TouchableOpacity>
                        <TouchableOpacity style={styles.menuItem} onPress={() => { changePointType(p.id, 'waypoint'); setActiveMenuId(null); }}>
                          <Text style={styles.menuText}>경유지로 변경</Text>
                        </TouchableOpacity>
                        <TouchableOpacity style={[styles.menuItem, styles.deleteItem]} onPress={() => { removePoint(p.id); setActiveMenuId(null); }}>
                          <Text style={styles.deleteText}>삭제</Text>
                        </TouchableOpacity>
                      </View>
                    )}
                    <TouchableOpacity onLongPress={() => setActiveMenuId(p.id)} delayLongPress={500} activeOpacity={0.7} style={[styles.marker, p.type === 'start' ? styles.markerStart : (p.section === 'A' ? styles.markerA : styles.markerB)]}>
                      {p.type === 'start' ? <Home size={12} color="white" /> : <MapPin size={12} color="white" />}
                    </TouchableOpacity>
                    <View style={[styles.arrow, { transform: [{ rotate: `${-(p.yaw * 180 / Math.PI)}deg` }] }]} />
                  </View>
                );
              })}
            </TouchableOpacity>

            <View style={styles.legend}>
              <View style={styles.legendItem}><View style={[styles.dot, { backgroundColor: '#10b981' }]} /><Text style={styles.legendText}>시작점</Text></View>
              <View style={styles.legendItem}><View style={[styles.dot, { backgroundColor: '#3b82f6' }]} /><Text style={styles.legendText}>A구역</Text></View>
              <View style={styles.legendItem}><View style={[styles.dot, { backgroundColor: '#a855f7' }]} /><Text style={styles.legendText}>B구역</Text></View>
            </View>

            {/* 2. 편집 모달 열기 버튼 */}
            <TouchableOpacity style={styles.orderBtn} onPress={() => setShowOrderModal(true)}>
              <Settings2 size={18} color="#4f46e5" />
              <Text style={styles.orderBtnText}>주행 순서 및 구역 편집</Text>
            </TouchableOpacity>
          </ScrollView>

          <View style={styles.footer}>
            <TouchableOpacity style={styles.completeBtn} onPress={handleComplete}>
              <Text style={styles.completeBtnText}>경로 전송 및 설정 완료</Text>
            </TouchableOpacity>
          </View>

          {/* 3. 편집 모달 컴포넌트 호출 */}
          <OrderEditModal 
            visible={showOrderModal}
            onClose={() => setShowOrderModal(false)}
            points={points}
            setPoints={setPoints}
          />

        </SafeAreaView>
      </View>
    </Modal>
  );
}

const styles = StyleSheet.create({
  overlay: { flex: 1, backgroundColor: 'rgba(0,0,0,0.5)' },
  container: { flex: 1, backgroundColor: 'white', borderTopLeftRadius: 30, borderTopRightRadius: 30, marginTop: 50 },
  header: { flexDirection: 'row', justifyContent: 'space-between', alignItems: 'center', padding: 20, borderBottomWidth: 1, borderBottomColor: '#f1f5f9' },
  headerLeft: { flexDirection: 'row', alignItems: 'center', gap: 10 },
  headerTitle: { fontSize: 20, fontWeight: 'bold', color: '#1e293b' },
  guideText: { fontSize: 13, color: '#64748b', marginBottom: 15, textAlign: 'center' },
  mapWrapper: { width: MAP_WIDTH, height: MAP_HEIGHT, backgroundColor: '#f8fafc', borderRadius: 15, overflow: 'visible', elevation: 5, shadowColor: '#000', shadowOffset: { width: 0, height: 2 }, shadowOpacity: 0.2, alignSelf: 'center' },
  mapImage: { width: '100%', height: '100%', borderRadius: 15 },
  markerContainer: { position: 'absolute', width: 30, height: 30, marginLeft: -15, marginTop: -15, alignItems: 'center', justifyContent: 'center' },
  marker: { width: 24, height: 24, borderRadius: 12, alignItems: 'center', justifyContent: 'center', borderWidth: 2, borderColor: 'white', elevation: 3 },
  markerStart: { backgroundColor: '#10b981' },
  markerA: { backgroundColor: '#3b82f6' },
  markerB: { backgroundColor: '#a855f7' },
  arrow: { position: 'absolute', top: -10, width: 2, height: 10, backgroundColor: '#ef4444' },
  optionMenu: { position: 'absolute', bottom: 35, width: 120, backgroundColor: '#1e293b', borderRadius: 12, padding: 4, zIndex: 2000 },
  menuItem: { padding: 10, borderBottomWidth: 0.5, borderBottomColor: '#334155' },
  menuText: { color: 'white', fontSize: 12, fontWeight: '600', textAlign: 'center' },
  deleteItem: { borderBottomWidth: 0 },
  deleteText: { color: '#fb7185', fontSize: 12, fontWeight: 'bold', textAlign: 'center' },
  legend: { flexDirection: 'row', justifyContent: 'center', gap: 20, marginTop: 20 },
  legendItem: { flexDirection: 'row', alignItems: 'center', gap: 6 },
  dot: { width: 10, height: 10, borderRadius: 5 },
  legendText: { fontSize: 12, fontWeight: 'bold', color: '#475569' },
  orderBtn: { flexDirection: 'row', alignItems: 'center', justifyContent: 'center', gap: 8, marginTop: 20, padding: 15, borderRadius: 12, backgroundColor: '#f5f7ff', borderWidth: 1, borderColor: '#e0e7ff' },
  orderBtnText: { color: '#4f46e5', fontWeight: 'bold' },
  footer: { padding: 20, borderTopWidth: 1, borderTopColor: '#f1f5f9' },
  completeBtn: { backgroundColor: '#4f46e5', padding: 16, borderRadius: 15, alignItems: 'center' },
  completeBtnText: { color: 'white', fontSize: 16, fontWeight: 'bold' }
});