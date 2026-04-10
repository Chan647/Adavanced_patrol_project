import React from 'react';
import {
  View, Text, Modal, TouchableOpacity, StyleSheet, 
  ScrollView, Platform
} from 'react-native';
import { X, ArrowUp, ArrowDown, ChevronDown } from 'lucide-react-native';
import { Picker } from '@react-native-picker/picker';

interface Point {
  id: string;
  type: 'start' | 'waypoint';
  section: string;
  x: number;
  y: number;
  yaw: number;
}

interface OrderEditModalProps {
  visible: boolean;
  onClose: () => void;
  points: Point[];
  setPoints: (points: Point[]) => void;
}

export function OrderEditModal({ visible, onClose, points, setPoints }: OrderEditModalProps) {
  
  // 지점의 인덱스와 구역 정보를 바탕으로 A-1, B-1 등의 넘버링을 계산하는 함수
  const getPointLabel = (currentIndex: number) => {
    const currentPoint = points[currentIndex];
    if (currentPoint.type === 'start') return 'START';

    // 현재 지점과 같은 섹션을 가진 이전 지점들의 개수를 세어 순번 계산
    const sectionType = currentPoint.section || 'A';
    let count = 0;
    for (let i = 0; i <= currentIndex; i++) {
      if (points[i].type === 'waypoint' && points[i].section === sectionType) {
        count++;
      }
    }
    
    return `${sectionType}-${count}`;
  };

  const moveStep = (index: number, direction: 'up' | 'down') => {
    if (direction === 'up' && index === 0) return;
    if (direction === 'down' && index === points.length - 1) return;

    const newPoints = [...points];
    const targetIndex = direction === 'up' ? index - 1 : index + 1;
    [newPoints[index], newPoints[targetIndex]] = [newPoints[targetIndex], newPoints[index]];
    setPoints(newPoints);
  };

  const updateSection = (id: string, newSection: string) => {
    const updatedPoints = points.map(p => 
      p.id === id ? { ...p, section: newSection } : p
    );
    setPoints(updatedPoints);
  };

  return (
    <Modal visible={visible} animationType="fade" transparent={true}>
      <View style={styles.overlay}>
        <View style={styles.modalContainer}>
          <View style={styles.header}>
            <View>
              <Text style={styles.title}>주행 순서 & 구역 편집</Text>
              <Text style={styles.subtitle}>화살표를 눌러 주행 순서를 섞을 수 있습니다.</Text>
            </View>
            <TouchableOpacity onPress={onClose}>
              <X size={24} color="#64748b" />
            </TouchableOpacity>
          </View>

          <ScrollView style={styles.listContainer} showsVerticalScrollIndicator={false}>
            {points.length === 0 ? (
              <Text style={styles.emptyText}>추가된 경로가 없습니다.</Text>
            ) : (
              points.map((item, index) => {
                const label = getPointLabel(index);
                const isSectionB = item.section === 'B';

                return (
                  <View key={item.id} style={styles.itemCard}>
                    <View style={styles.itemLeft}>
                      {/* 라벨: START(Green), Section A(Blue), Section B(Purple) */}
                      <Text style={[
                        styles.itemLabel, 
                        item.type === 'start' 
                          ? styles.startText 
                          : (isSectionB ? styles.sectionBText : styles.sectionAText)
                      ]}>
                        {label}
                      </Text>
                      
                      {/* Waypoint인 경우에만 구역 선택 드롭다운 표시 */}
                      {item.type !== 'start' && (
                        <View style={styles.pickerWrapper}>
                          <Picker
                            selectedValue={item.section || 'A'}
                            onValueChange={(itemValue) => updateSection(item.id, itemValue)}
                            style={styles.picker}
                            dropdownIconColor="#64748b"
                            mode="dropdown"
                          >
                            <Picker.Item label="Section A" value="A" />
                            <Picker.Item label="Section B" value="B" />
                          </Picker>
                          <View pointerEvents="none" style={styles.pickerIcon}>
                            <ChevronDown size={14} color="#64748b" />
                          </View>
                        </View>
                      )}
                    </View>

                    {/* 순서 조절 버튼 */}
                    <View style={styles.controls}>
                      <TouchableOpacity 
                        onPress={() => moveStep(index, 'up')} 
                        disabled={index === 0} 
                        style={[styles.arrowBtn, index === 0 && styles.disabledBtn]}
                      >
                        <ArrowUp size={18} color={index === 0 ? "#cbd5e1" : "#64748b"} />
                      </TouchableOpacity>
                      <TouchableOpacity 
                        onPress={() => moveStep(index, 'down')} 
                        disabled={index === points.length - 1} 
                        style={[styles.arrowBtn, index === points.length - 1 && styles.disabledBtn]}
                      >
                        <ArrowDown size={18} color={index === points.length - 1 ? "#cbd5e1" : "#64748b"} />
                      </TouchableOpacity>
                    </View>
                  </View>
                );
              })
            )}
          </ScrollView>
          
          <TouchableOpacity style={styles.closeBtn} onPress={onClose}>
            <Text style={styles.closeBtnText}>경로 전송 및 설정 완료</Text>
          </TouchableOpacity>
        </View>
      </View>
    </Modal>
  );
}

const styles = StyleSheet.create({
  overlay: { flex: 1, backgroundColor: 'rgba(0,0,0,0.4)', justifyContent: 'center', alignItems: 'center' },
  modalContainer: { width: '92%', maxHeight: '80%', backgroundColor: 'white', borderRadius: 24, padding: 20, elevation: 10 },
  header: { flexDirection: 'row', justifyContent: 'space-between', marginBottom: 20 },
  title: { fontSize: 18, fontWeight: 'bold', color: '#1e293b' },
  subtitle: { fontSize: 12, color: '#94a3b8', marginTop: 4 },
  listContainer: { marginBottom: 15 },
  emptyText: { textAlign: 'center', color: '#94a3b8', marginVertical: 30, fontSize: 14 },
  itemCard: { 
    flexDirection: 'row', 
    justifyContent: 'space-between', 
    alignItems: 'center', 
    backgroundColor: '#f8fafc', 
    borderRadius: 12, 
    padding: 12, 
    marginBottom: 8, 
    borderWidth: 1, 
    borderColor: '#e2e8f0' 
  },
  itemLeft: { flexDirection: 'row', alignItems: 'center', gap: 10 },
  itemLabel: { fontSize: 14, fontWeight: 'bold', width: 60 },
  startText: { color: '#10b981' }, // Green
  sectionAText: { color: '#3b82f6' }, // Blue
  sectionBText: { color: '#a855f7' }, // Purple (웹 화면의 보라색 느낌)
  
  pickerWrapper: { 
    flexDirection: 'row', 
    alignItems: 'center', 
    backgroundColor: 'white', 
    borderRadius: 8, 
    borderWidth: 1, 
    borderColor: '#e2e8f0', 
    width: 130, 
    height: 38, 
    overflow: 'hidden' 
  },
  picker: { 
    width: Platform.OS === 'android' ? '140%' : '100%', 
    height: 40, 
    marginLeft: Platform.OS === 'android' ? -10 : 0,
    color: '#334155'
  },
  pickerIcon: { position: 'absolute', right: 8, zIndex: 1 },
  
  controls: { flexDirection: 'row', gap: 6 },
  arrowBtn: { padding: 6, backgroundColor: 'white', borderRadius: 8, borderWidth: 1, borderColor: '#e2e8f0' },
  disabledBtn: { backgroundColor: '#f1f5f9', borderColor: '#f1f5f9' },
  closeBtn: { backgroundColor: '#4f46e5', padding: 16, borderRadius: 15, alignItems: 'center', marginTop: 10 },
  closeBtnText: { color: 'white', fontWeight: 'bold', fontSize: 15 }
});