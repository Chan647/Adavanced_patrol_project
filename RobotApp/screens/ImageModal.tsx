/* ImageModal.tsx */

import React from 'react';
import { 
  View, 
  Text, 
  Modal, 
  Image, 
  StyleSheet, 
  TouchableOpacity, 
  Dimensions, 
  Platform 
} from 'react-native';
import { X, Image as ImageIcon } from 'lucide-react-native';

interface ImageModalProps {
  visible: boolean;
  imageUrl: string;
  onClose: () => void;
}

// 기기 화면의 크기를 가져옵니다.
const { width: SCREEN_WIDTH, height: SCREEN_HEIGHT } = Dimensions.get('window');

export function ImageModal({ visible, imageUrl, onClose }: ImageModalProps) {
  if (!imageUrl) return null;

  return (
    <Modal 
      visible={visible} 
      transparent 
      animationType="fade" 
      onRequestClose={onClose}
    >
      <View style={styles.overlay}>
        <View style={styles.contentCard}>
          {/* 1. 상단 헤더 영역 */}
          <View style={styles.header}>
            <View style={styles.titleContainer}>
              <View style={styles.iconWrapper}>
                <ImageIcon size={16} color="#4d61ff" />
              </View>
              <Text style={styles.headerTitle}>이미지 상세 보기</Text>
            </View>
            <TouchableOpacity onPress={onClose} style={styles.closeBtn}>
              <X size={22} color="#64748b" />
            </TouchableOpacity>
          </View>

          {/* 2. 이미지 표시 영역 (핵심!) */}
          <View style={styles.imageContainer}>
            <Image 
              source={{ uri: imageUrl }} 
              style={styles.fullImage}
              // ✅ 중요: 이미지가 잘리지 않고 전체가 영역 안에 들어오게 합니다.
              resizeMode="contain" 
            />
          </View>

          {/* 3. 하단 닫기 버튼 */}
          <View style={styles.footer}>
            <TouchableOpacity onPress={onClose} style={styles.bottomBtn}>
              <Text style={styles.bottomBtnText}>닫기</Text>
            </TouchableOpacity>
          </View>
        </View>
      </View>
    </Modal>
  );
}

const styles = StyleSheet.create({
  overlay: {
    flex: 1,
    backgroundColor: 'rgba(0, 0, 0, 0.75)', // 배경을 어둡게 하여 이미지 집중도 향상
    justifyContent: 'center',
    alignItems: 'center',
    padding: 16,
  },
  contentCard: {
    width: '100%',
    maxHeight: SCREEN_HEIGHT * 0.8, // 화면의 80%를 넘지 않도록 설정
    backgroundColor: 'white',
    borderRadius: 24,
    overflow: 'hidden',
    ...Platform.select({
      ios: { shadowColor: '#000', shadowOffset: { width: 0, height: 10 }, shadowOpacity: 0.3, shadowRadius: 20 },
      android: { elevation: 15 },
    }),
  },
  header: {
    flexDirection: 'row',
    justifyContent: 'space-between',
    alignItems: 'center',
    paddingHorizontal: 20,
    paddingVertical: 16,
    borderBottomWidth: 1,
    borderBottomColor: '#f1f5f9',
  },
  titleContainer: {
    flexDirection: 'row',
    alignItems: 'center',
    gap: 8,
  },
  iconWrapper: {
    backgroundColor: '#eef2ff',
    padding: 6,
    borderRadius: 8,
  },
  headerTitle: {
    fontSize: 16,
    fontWeight: '700',
    color: '#1e293b',
  },
  closeBtn: {
    padding: 4,
  },
  imageContainer: {
    width: '100%',
    height: SCREEN_HEIGHT * 0.5, // 이미지 영역 높이를 충분히 확보
    backgroundColor: '#000', // 이미지 비율이 달라도 여백을 검은색으로 처리해 깔끔함 유지
    justifyContent: 'center',
    alignItems: 'center',
  },
  fullImage: {
    width: '100%',
    height: '100%',
  },
  footer: {
    padding: 16,
    backgroundColor: '#f8fafc',
  },
  bottomBtn: {
    backgroundColor: '#1e293b',
    paddingVertical: 14,
    borderRadius: 14,
    alignItems: 'center',
  },
  bottomBtnText: {
    color: 'white',
    fontSize: 16,
    fontWeight: 'bold',
  },
});