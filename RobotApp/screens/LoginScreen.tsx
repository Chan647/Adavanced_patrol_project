/* LoginScreen.tsx */

import React, { useState } from 'react';
import { 
  View, 
  Text, 
  TextInput, 
  TouchableOpacity, 
  StyleSheet, 
  Alert, 
  KeyboardAvoidingView, 
  Platform,
  ScrollView 
} from 'react-native';
import { SafeAreaView } from 'react-native-safe-area-context';
import { useNavigation } from '@react-navigation/native';
import axios from 'axios';
import { useAuth } from '../contexts/AuthContext';

// ✅ 2-PC 환경: 서버 PC(Ubuntu)의 IP를 직접 입력합니다.
// adb reverse는 사용하지 않습니다.
const API_BASE_URL = 'http://192.168.0.24:5000';

export default function LoginScreen() {
  const [username, setUsername] = useState('');
  const [password, setPassword] = useState('');
  const [isLoggingIn, setIsLoggingIn] = useState(false);
  
  const navigation = useNavigation<any>();
  const { login: setAuthUser } = useAuth();

  const handleLogin = async () => {
    if (!username || !password) {
      Alert.alert('알림', '사원증 번호와 비밀번호를 입력해주세요.');
      return;
    }

    setIsLoggingIn(true);
    
    try {
      // ✅ 디버깅용 로그: 입력 데이터 확인
      console.log("로그인 시도:", { userId: username.trim(), password });

      const response = await axios.post(`${API_BASE_URL}/api/login`, {
        // 서버 DB 구조에 맞춰 trim() 처리
        userId: username.trim(), 
        password: password
      }, {
        timeout: 5000 // 5초 동안 응답 없으면 타임아웃
      });

      console.log("서버 응답 성공:", response.data);

      // ✅ 서버의 응답 형식(result: "success")에 맞춘 조건문
      if (response.status === 200 && response.data.result === "success") {
        setAuthUser(response.data.user);
        navigation.replace('WaitScreen'); 
      } else {
        Alert.alert('로그인 실패', response.data.error || '아이디 또는 비밀번호를 확인하세요.');
      }
    } catch (err: any) {
      console.log("통신 에러 상세:", err);

      let errorMessage = '서버에 연결할 수 없습니다.';
      
      if (err.response) {
        // 서버는 응답했으나 에러 발생 (401, 500 등)
        errorMessage = err.response.data?.error || `서버 오류 (${err.response.status})`;
      } else if (err.request) {
        // 요청은 보냈으나 응답이 없음 (방화벽 또는 네트워크 문제)
        errorMessage = '서버 응답이 없습니다. Ubuntu PC의 IP 주소와 방화벽 설정을 확인하세요.';
      }

      Alert.alert('연결 에러', errorMessage);
    } finally {
      setIsLoggingIn(false);
    }
  };

  return (
    <SafeAreaView style={styles.container}>
      <KeyboardAvoidingView 
        behavior={Platform.OS === 'ios' ? 'padding' : 'height'}
        style={{ flex: 1 }}
      >
        <ScrollView contentContainerStyle={styles.scrollContent}>
          <View style={styles.header}>
            <View style={styles.logoBox}>
              <Text style={{ fontSize: 40 }}>🤖</Text>
            </View>
            <Text style={styles.title}>로봇 제어 시스템</Text>
            <Text style={styles.subtitle}>사원증 번호로 로그인하세요</Text>
          </View>

          <View style={styles.form}>
            <View style={styles.inputGroup}>
              <Text style={styles.label}>사원증 번호 (ID)</Text>
              <TextInput
                style={styles.input}
                placeholder="ID를 입력하세요"
                value={username}
                onChangeText={setUsername}
                autoCapitalize="none"
              />
            </View>

            <View style={styles.inputGroup}>
              <Text style={styles.label}>비밀번호</Text>
              <TextInput
                style={styles.input}
                placeholder="비밀번호를 입력하세요"
                value={password}
                onChangeText={setPassword}
                autoCapitalize="none"
                secureTextEntry
              />
            </View>

            <TouchableOpacity 
              style={[styles.loginButton, isLoggingIn && { opacity: 0.7 }]}
              onPress={handleLogin}
              disabled={isLoggingIn}
            >
              <Text style={styles.loginButtonText}>
                {isLoggingIn ? '연결 중...' : '로그인'}
              </Text>
            </TouchableOpacity>

            <TouchableOpacity 
              style={styles.signupButton}
              onPress={() => navigation.navigate('SignupScreen')}
            >
              <Text style={styles.signupButtonText}>회원가입</Text>
            </TouchableOpacity>
          </View>
        </ScrollView>
      </KeyboardAvoidingView>
    </SafeAreaView>
  );
}

const styles = StyleSheet.create({
  container: { flex: 1, backgroundColor: '#0f172a' },
  scrollContent: { flexGrow: 1, justifyContent: 'center', padding: 24 },
  header: { alignItems: 'center', marginBottom: 40 },
  logoBox: {
    width: 80,
    height: 80,
    backgroundColor: '#4d61ff',
    borderRadius: 20,
    justifyContent: 'center',
    alignItems: 'center',
    marginBottom: 16,
    elevation: 10,
  },
  title: { fontSize: 28, fontWeight: 'bold', color: 'white', marginBottom: 8 },
  subtitle: { fontSize: 16, color: '#94a3b8' },
  form: { backgroundColor: 'white', borderRadius: 24, padding: 24, elevation: 5 },
  inputGroup: { marginBottom: 20 },
  label: { fontSize: 14, fontWeight: 'bold', color: '#334155', marginBottom: 8 },
  input: {
    borderWidth: 1,
    borderColor: '#e2e8f0',
    borderRadius: 12,
    padding: 16,
    fontSize: 16,
    backgroundColor: '#f8fafc',
    color: '#334155',
  },
  loginButton: {
    backgroundColor: '#4d61ff',
    paddingVertical: 18,
    borderRadius: 12,
    alignItems: 'center',
    marginTop: 10,
  },
  loginButtonText: { color: 'white', fontSize: 18, fontWeight: 'bold' },
  signupButton: {
    marginTop: 15,
    paddingVertical: 15,
    alignItems: 'center',
  },
  signupButtonText: { color: '#64748b', fontSize: 16 },
});