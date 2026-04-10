/* LogModal.tsx */

import React, { useState, useEffect } from 'react'; // 🚀 useEffect 추가
import {
  View,
  Text,
  Modal,
  TouchableOpacity,
  FlatList,
  StyleSheet,
  Alert,
  Dimensions,
} from 'react-native';
import { SafeAreaView } from 'react-native-safe-area-context';
import { X, Eye, History, Trash2, CheckSquare, Square, MapPin, Flame, User } from 'lucide-react-native';
import axios from 'axios';
import { useRobot } from '../contexts/RobotContext';
import { ImageModal } from './ImageModal';

const { height: SCREEN_HEIGHT } = Dimensions.get('window');

interface LogHistoryModalProps {
  visible: boolean;
  onClose: () => void;
}

// 컴포넌트 이름을 MainScreen에서 사용하는 이름과 통일합니다.
export function LogHistoryModal({ visible, onClose }: LogHistoryModalProps) {
  const { logs, refreshLogs, API_BASE_URL } = useRobot();
  const [selectedIds, setSelectedIds] = useState<string[]>([]);
  const [selectedImage, setSelectedImage] = useState<string | null>(null);

  const isAdmin = true;

  // 🚀 핵심 수정 사항: 모달이 활성화될 때마다 서버에서 로그를 새로고침합니다.
  useEffect(() => {
    if (visible) {
      refreshLogs();
    }
  }, [visible]);

  const toggleSelectAll = () => {
    if (selectedIds.length === logs.length && logs.length > 0) {
      setSelectedIds([]);
    } else {
      setSelectedIds(logs.map(log => String(log.id)));
    }
  };

  const toggleSelectOne = (id: string) => {
    const stringId = String(id);
    setSelectedIds(prev =>
      prev.includes(stringId) ? prev.filter(item => item !== stringId) : [...prev, stringId]
    );
  };

  const handleDeleteSelected = async () => {
    if (selectedIds.length === 0) return;
    Alert.alert("삭제 확인", `선택한 ${selectedIds.length}개의 로그를 영구 삭제하시겠습니까?`, [
      { text: "취소", style: "cancel" },
      {
        text: "삭제",
        style: "destructive",
        onPress: async () => {
          try {
            await Promise.all(
              selectedIds.map(id => axios.delete(`${API_BASE_URL}/api/logs/${id}`))
            );
            setSelectedIds([]);
            await refreshLogs();
            Alert.alert("완료", "삭제되었습니다.");
          } catch (err) {
            Alert.alert("오류", "삭제 중 문제가 발생했습니다.");
          }
        }
      }
    ]);
  };

  const renderLogItem = ({ item }: { item: any }) => {
    // situation 필드가 없을 경우를 대비해 기본값 처리
    const situation = item.situation || "";
    const isSpecial = situation.includes('화재') || situation.includes('침입자');
    const isPano = situation.includes('파노라마');
    const isSelected = selectedIds.includes(String(item.id));

    return (
      <View style={[
        styles.logCard,
        isSelected && styles.selectedCard,
        isSpecial && styles.fireHighlight
      ]}>
        {/* 왼쪽: 체크박스 */}
        <TouchableOpacity onPress={() => toggleSelectOne(item.id)} style={styles.checkArea}>
          {isSelected ? <CheckSquare size={22} color="#4f46e5" /> : <Square size={22} color="#cbd5e1" />}
        </TouchableOpacity>

        {/* 중앙: 로그 정보 */}
        <View style={styles.logInfo}>
          <Text style={styles.timeText}>{item.time}</Text>
          <View style={styles.contentRow}>
            {/* 카테고리 배지 */}
            <View style={[
              styles.badge,
              isSpecial ? styles.badgeRed : isPano ? styles.badgeOrange : styles.badgeGray
            ]}>
              {isSpecial ? <Flame size={10} color="white" /> : <User size={10} color="#64748b" />}
              <Text style={[styles.badgeText, isSpecial && { color: 'white' }]}>
                {situation}
              </Text>
            </View>
            
            {/* 위치 표시 */}
            <View style={styles.locationContainer}>
              <MapPin size={10} color="#94a3b8" />
              <Text style={styles.positionText}>{item.position === 'S' ? 'START' : item.position}</Text>
            </View>
          </View>
        </View>

        {/* 오른쪽: 이미지 보기 버튼 */}
        {item.imageUrl ? (
          <TouchableOpacity 
            style={[styles.viewBtn, isSpecial ? styles.viewBtnRed : styles.viewBtnDark]}
            onPress={() => setSelectedImage(item.imageUrl.startsWith('http') ? item.imageUrl : `${API_BASE_URL}${item.imageUrl}`)}
          >
            <Eye size={14} color="white" style={{ marginRight: 4 }} />
            <Text style={styles.viewBtnText}>이미지 보기</Text>
          </TouchableOpacity>
        ) : (
          <View style={{ width: 85 }} /> 
        )}
      </View>
    );
  };

  return (
    <Modal visible={visible} animationType="slide" transparent={true} onRequestClose={onClose}>
      <View style={styles.overlay}>
        <SafeAreaView style={styles.container}>
          {/* 헤더 섹션 */}
          <View style={styles.header}>
            <View style={styles.headerLeft}>
              <View style={styles.iconBox}><History size={22} color="white" /></View>
              <View>
                <Text style={styles.headerTitle}>시스템 로그 내역</Text>
                <Text style={styles.headerSub}>로봇의 모든 주행 기록 및 특이사항을 확인합니다.</Text>
              </View>
            </View>
            <TouchableOpacity onPress={onClose} style={styles.closeBtn}>
              <X size={24} color="#64748b" />
            </TouchableOpacity>
          </View>

          {/* 툴바: 전체선택 및 삭제 */}
          <View style={styles.toolBar}>
            <TouchableOpacity onPress={toggleSelectAll} style={styles.selectAllBtn}>
              <Text style={styles.toolText}>
                {selectedIds.length === logs.length && logs.length > 0 ? "전체 해제" : "전체 선택"}
              </Text>
            </TouchableOpacity>
            {isAdmin && selectedIds.length > 0 && (
              <TouchableOpacity onPress={handleDeleteSelected} style={styles.deleteBtn}>
                <Trash2 size={14} color="white" />
                <Text style={styles.deleteBtnText}>{selectedIds.length}개 삭제</Text>
              </TouchableOpacity>
            )}
          </View>

          {/* 리스트 헤더 라벨 */}
          <View style={styles.listHeader}>
            <Text style={styles.headerLabel}>기록 시간 / 카테고리</Text>
            <Text style={styles.headerLabel}>동작</Text>
          </View>

          <FlatList
            data={logs}
            keyExtractor={(item) => String(item.id)}
            renderItem={renderLogItem}
            contentContainerStyle={styles.listContent}
            ListEmptyComponent={
              <View style={styles.emptyContainer}>
                <Text style={styles.emptyText}>저장된 로그가 없습니다.</Text>
                <TouchableOpacity onPress={() => refreshLogs()} style={styles.retryBtn}>
                  <Text style={styles.retryText}>새로고침</Text>
                </TouchableOpacity>
              </View>
            }
          />

          <View style={styles.footer}>
            <TouchableOpacity onPress={onClose} style={styles.doneBtn}>
              <Text style={styles.doneBtnText}>닫기</Text>
            </TouchableOpacity>
          </View>
        </SafeAreaView>
      </View>

      {selectedImage && (
        <ImageModal visible={!!selectedImage} imageUrl={selectedImage} onClose={() => setSelectedImage(null)} />
      )}
    </Modal>
  );
}

const styles = StyleSheet.create({
  overlay: { flex: 1, backgroundColor: 'rgba(0,0,0,0.6)', justifyContent: 'flex-end' },
  container: { 
    width: '100%', 
    height: SCREEN_HEIGHT * 0.88, 
    backgroundColor: 'white', 
    borderTopLeftRadius: 30, 
    borderTopRightRadius: 30 
  },
  header: { 
    flexDirection: 'row', 
    justifyContent: 'space-between', 
    padding: 20, 
    borderBottomWidth: 1, 
    borderBottomColor: '#f1f5f9' 
  },
  headerLeft: { flexDirection: 'row', alignItems: 'center' },
  iconBox: { backgroundColor: '#7c3aed', padding: 10, borderRadius: 12, marginRight: 12 },
  headerTitle: { fontSize: 18, fontWeight: '800', color: '#1e293b' },
  headerSub: { fontSize: 11, color: '#64748b', marginTop: 2 },
  closeBtn: { padding: 4 },
  
  toolBar: { 
    flexDirection: 'row', 
    justifyContent: 'space-between', 
    paddingHorizontal: 20, 
    paddingVertical: 12, 
    alignItems: 'center',
    backgroundColor: '#f8fafc' 
  },
  toolText: { color: '#4f46e5', fontWeight: 'bold', fontSize: 13 },
  deleteBtn: { 
    backgroundColor: '#ef4444', 
    flexDirection: 'row', 
    paddingHorizontal: 12, 
    paddingVertical: 6, 
    borderRadius: 8, 
    alignItems: 'center' 
  },
  deleteBtnText: { color: 'white', fontWeight: 'bold', marginLeft: 4, fontSize: 11 },

  listHeader: { 
    flexDirection: 'row', 
    justifyContent: 'space-between', 
    paddingHorizontal: 25, 
    marginTop: 15,
    marginBottom: 5
  },
  headerLabel: { fontSize: 11, color: '#94a3b8', fontWeight: 'bold' },

  listContent: { paddingHorizontal: 20, paddingBottom: 30 },
  
  logCard: { 
    flexDirection: 'row', 
    alignItems: 'center', 
    backgroundColor: 'white', 
    padding: 14, 
    borderRadius: 16, 
    marginBottom: 10, 
    borderWidth: 1, 
    borderColor: '#f1f5f9',
    elevation: 1
  },
  selectedCard: { backgroundColor: '#f5f7ff', borderColor: '#c7d2fe' },
  fireHighlight: { backgroundColor: '#fff1f2', borderColor: '#fecaca' },
  
  checkArea: { marginRight: 12 },
  logInfo: { flex: 1 },
  timeText: { fontSize: 11, color: '#64748b', marginBottom: 6 },
  contentRow: { flexDirection: 'row', alignItems: 'center' },
  
  badge: { 
    flexDirection: 'row', 
    alignItems: 'center', 
    paddingHorizontal: 8, 
    paddingVertical: 4, 
    borderRadius: 6, 
    marginRight: 10 
  },
  badgeRed: { backgroundColor: '#ef4444' },
  badgeOrange: { backgroundColor: '#fff7ed', borderWidth: 1, borderColor: '#fb923c' },
  badgeGray: { backgroundColor: '#f1f5f9' },
  badgeText: { fontSize: 10, fontWeight: 'bold', color: '#64748b', marginLeft: 4 },
  
  locationContainer: { flexDirection: 'row', alignItems: 'center' },
  positionText: { fontSize: 11, fontWeight: '700', color: '#475569', marginLeft: 2, fontStyle: 'italic' },
  
  viewBtn: { 
    flexDirection: 'row', 
    alignItems: 'center', 
    paddingHorizontal: 10, 
    paddingVertical: 8, 
    borderRadius: 8,
    width: 85,
    justifyContent: 'center'
  },
  viewBtnDark: { backgroundColor: '#1e293b' },
  viewBtnRed: { backgroundColor: '#ef4444' },
  viewBtnText: { color: 'white', fontSize: 10, fontWeight: 'bold' },

  emptyContainer: { alignItems: 'center', marginTop: 50 },
  emptyText: { color: '#94a3b8', fontSize: 14, marginBottom: 10 },
  retryBtn: { padding: 10, backgroundColor: '#f1f5f9', borderRadius: 8 },
  retryText: { color: '#4f46e5', fontWeight: 'bold' },

  footer: { padding: 20, borderTopWidth: 1, borderTopColor: '#f1f5f9' },
  doneBtn: { backgroundColor: '#0f172a', padding: 16, borderRadius: 16, alignItems: 'center' },
  doneBtnText: { color: 'white', fontSize: 16, fontWeight: 'bold' },
  selectAllBtn: { paddingVertical: 5 }
});