import React, { useState, useCallback, useEffect, useRef } from 'react';
import {
  View,
  Text,
  StyleSheet,
  TouchableOpacity,
  Image,
  ActivityIndicator,
  ScrollView,
  RefreshControl,
  Alert,
} from 'react-native';
import { useNavigation } from '@react-navigation/native';
import { SafeAreaView } from 'react-native-safe-area-context';
import { ChevronLeft, Camera, RefreshCw, Calendar, Image as ImageIcon } from 'lucide-react-native';
import { useRobot } from '../contexts/RobotContext';
import { StatusBar } from './StatusBar';
import axios from 'axios';

export default function PanoramaScreen() {
  const navigation = useNavigation<any>();
  const { refreshLogs, API_BASE_URL, connectionStatus } = useRobot();

  const [imageUri, setImageUri] = useState<string | null>(null);
  const [loading, setLoading] = useState(false);
  const [capturing, setCapturing] = useState(false);
  const [refreshing, setRefreshing] = useState(false);
  const [uploadedAt, setUploadedAt] = useState<string | null>(null);
  const [error, setError] = useState<string | null>(null);
  
  // 웹 로직처럼 마지막으로 확인한 이미지 ID를 저장하여 중복 체크 방지
  const [lastId, setLastId] = useState<string | null>(null);
  const pollingInterval = useRef<any>(null);

  // 최신 파노라마 데이터 가져오기 함수
  const fetchLatestPanorama = async () => {
    setError(null);
    try {
      const logs = await refreshLogs();
      // "파노라마" 단어가 포함된 가장 최신 로그 검색
      const panoLog = logs.find((l: any) => l.situation.includes('파노라마'));

      if (panoLog && panoLog.imageUrl) {
        const fullImageUrl = panoLog.imageUrl.startsWith('http')
          ? panoLog.imageUrl
          : `${API_BASE_URL}${panoLog.imageUrl}`;

        setImageUri(`${fullImageUrl}?t=${Date.now()}`);
        setUploadedAt(panoLog.time);
        setLastId(String(panoLog.id));
        return panoLog;
      } else {
        setError('촬영된 파노라마 이미지가 없습니다.');
        setImageUri(null);
        return null;
      }
    } catch (e) {
      setError('서버 연결 실패');
      setImageUri(null);
      return null;
    }
  };

  // 촬영 명령 요청 및 폴링 로직 (웹 코드 스타일 이식)
  const handleCaptureRequest = async () => {
    if (connectionStatus === 'Offline') {
      Alert.alert('연결 오류', '로봇이 오프라인 상태입니다.');
      return;
    }

    setCapturing(true); // 촬영 대기 화면 활성화
    
    try {
      // 1. 서버에 촬영 명령 전송
      const response = await axios.post(`${API_BASE_URL}/api/robot/command`, {
        command: 'PANORAMA'
      });

      if (response.status === 200) {
        // 2. 명령 성공 시, 새로운 ID가 생성될 때까지 폴링 시작 (2초 간격)
        let attempts = 0;
        const maxAttempts = 15; // 최대 30초 대기

        pollingInterval.current = setInterval(async () => {
          attempts++;
          const logs = await refreshLogs();
          const latest = logs[0]; // 가장 최신 로그

          // 새로운 파노라마 로그가 생성되었는지 확인
          if (latest && 
              latest.situation.includes("파노라마") && 
              String(latest.id) !== lastId) {
            
            clearInterval(pollingInterval.current);
            await fetchLatestPanorama(); // 최종 데이터 갱신
            setCapturing(false);
            Alert.alert('성공', '새 파노라마 이미지를 불러왔습니다.');
          }

          if (attempts >= maxAttempts) {
            clearInterval(pollingInterval.current);
            setCapturing(false);
            Alert.alert('시간 초과', '로봇으로부터 응답이 없거나 촬영이 지연되고 있습니다.');
          }
        }, 2000);
      }
    } catch (e) {
      setCapturing(false);
      Alert.alert('에러', '서버 통신 중 오류가 발생했습니다.');
    }
  };

  // 컴포넌트 마운트 시 데이터 로드 및 언마운트 시 인터벌 정리
  useEffect(() => {
    fetchLatestPanorama();
    return () => {
      if (pollingInterval.current) clearInterval(pollingInterval.current);
    };
  }, []);

  return (
    <SafeAreaView style={styles.safeArea}>
      <StatusBar />
      
      <View style={styles.navBar}>
        <TouchableOpacity style={styles.backButton} onPress={() => navigation.goBack()}>
          <ChevronLeft size={28} color="#007bff" strokeWidth={3} />
          <Text style={styles.backButtonText}>메인</Text>
        </TouchableOpacity>
        <Text style={styles.navTitle}>파노라마 뷰어</Text>
        <View style={{ width: 80 }} /> 
      </View>

      <ScrollView
        style={styles.container}
        refreshControl={
          <RefreshControl 
            refreshing={refreshing} 
            onRefresh={async () => {
              setRefreshing(true);
              await fetchLatestPanorama();
              setRefreshing(false);
            }} 
            tintColor="#fff" 
          />
        }
      >
        <View style={styles.contentHeader}>
          <ImageIcon size={20} color="#aaa" />
          <Text style={styles.contentHeaderText}>로봇 파노라마 제어 시스템</Text>
        </View>

        <View style={styles.buttonGroup}>
          <TouchableOpacity
            style={[styles.captureButton, capturing && styles.buttonDisabled]}
            onPress={handleCaptureRequest}
            disabled={capturing}
          >
            {capturing ? (
              <View style={styles.buttonInner}>
                <ActivityIndicator color="white" />
                <Text style={styles.buttonText}>촬영 대기 중...</Text>
              </View>
            ) : (
              <View style={styles.buttonInner}>
                <Camera size={20} color="white" />
                <Text style={styles.buttonText}>새 파노라마 촬영</Text>
              </View>
            )}
          </TouchableOpacity>

          <TouchableOpacity
            style={[styles.loadButton, loading && styles.buttonDisabled]}
            onPress={async () => {
              setLoading(true);
              await fetchLatestPanorama();
              setLoading(false);
            }}
          >
            <View style={styles.buttonInner}>
              <RefreshCw size={20} color="white" />
              <Text style={styles.buttonText}>이미지 새로고침</Text>
            </View>
          </TouchableOpacity>
        </View>

        {imageUri ? (
          <View style={styles.imageCard}>
            <View style={styles.imageInfo}>
              <Calendar size={14} color="#007bff" />
              <Text style={styles.imageTime}>{uploadedAt}</Text>
            </View>
            <Image
              source={{ uri: imageUri }}
              style={styles.mainImage}
              resizeMode="contain"
            />
          </View>
        ) : (
          <View style={styles.emptyState}>
            {capturing ? (
              <ActivityIndicator size="large" color="#007bff" style={{ marginBottom: 20 }} />
            ) : null}
            <Text style={styles.emptyText}>
              {capturing ? '로봇이 이미지를 생성하고 있습니다...' : error || '데이터를 불러오는 중...'}
            </Text>
          </View>
        )}
      </ScrollView>
    </SafeAreaView>
  );
}

const styles = StyleSheet.create({
  safeArea: { flex: 1, backgroundColor: '#000' },
  navBar: {
    height: 60,
    flexDirection: 'row',
    alignItems: 'center',
    justifyContent: 'space-between',
    paddingHorizontal: 10,
    backgroundColor: '#1a1a1a',
    borderBottomWidth: 1,
    borderBottomColor: '#333',
  },
  backButton: {
    flexDirection: 'row',
    alignItems: 'center',
    width: 80,
  },
  backButtonText: {
    color: '#007bff',
    fontSize: 17,
    fontWeight: 'bold',
    marginLeft: -4,
  },
  navTitle: {
    color: '#fff',
    fontSize: 18,
    fontWeight: 'bold',
  },
  container: { flex: 1, padding: 15 },
  contentHeader: { flexDirection: 'row', alignItems: 'center', gap: 8, marginBottom: 20, paddingLeft: 5 },
  contentHeaderText: { color: '#aaa', fontSize: 13, fontWeight: '500' },
  buttonGroup: { gap: 10, marginBottom: 25 },
  buttonInner: { flexDirection: 'row', alignItems: 'center', gap: 10 },
  captureButton: { backgroundColor: '#28a745', padding: 16, borderRadius: 12, justifyContent: 'center', alignItems: 'center' },
  loadButton: { backgroundColor: '#333', padding: 16, borderRadius: 12, justifyContent: 'center', alignItems: 'center' },
  buttonDisabled: { opacity: 0.5 },
  buttonText: { color: 'white', fontSize: 16, fontWeight: 'bold' },
  imageCard: { backgroundColor: '#1a1a1a', borderRadius: 20, overflow: 'hidden', borderWidth: 1, borderColor: '#333' },
  imageInfo: { flexDirection: 'row', alignItems: 'center', gap: 6, padding: 15, backgroundColor: '#222' },
  imageTime: { color: '#007bff', fontSize: 14, fontWeight: '600' },
  mainImage: { width: '100%', height: 350, backgroundColor: '#000' },
  emptyState: { marginTop: 100, alignItems: 'center' },
  emptyText: { color: '#666', fontSize: 15, textAlign: 'center' }
});