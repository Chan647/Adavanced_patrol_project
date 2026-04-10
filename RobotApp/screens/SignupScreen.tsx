/* SignupScreen.tsx */

import React, { useState } from 'react';
import {
  View,
  Text,
  TextInput,
  TouchableOpacity,
  StyleSheet,
  KeyboardAvoidingView,
  Platform,
  ScrollView,
  Alert,
} from 'react-native';
import { useNavigation } from '@react-navigation/native';
import axios from 'axios';
import { Lock, CheckCircle, Bot, CreditCard, ChevronLeft } from 'lucide-react-native';
import { SafeAreaView } from 'react-native-safe-area-context';

// ✅ 서버 PC(Ubuntu)의 IP 주소를 변수로 관리 (LoginScreen과 동일하게 설정)
const API_BASE_URL = 'http://192.168.0.24:5000';

export default function SignupScreen() {
  const [employeeId, setEmployeeId] = useState('');
  const [password, setPassword] = useState('');
  const [confirmPassword, setConfirmPassword] = useState('');
  const [error, setError] = useState('');
  const [success, setSuccess] = useState(false);

  const navigation = useNavigation<any>();

  const handleSignup = async () => {
    setError('');
    
    // 1. 프론트엔드 유효성 검사
    if (!employeeId.trim()) {
      setError('사원증 번호를 입력해주세요.');
      return;
    }

    if (password !== confirmPassword) {
      setError('비밀번호가 일치하지 않습니다.');
      return;
    }

    if (password.length < 4) {
      setError('비밀번호는 최소 4자 이상이어야 합니다.');
      return;
    }

    try {
      // ✅ API_BASE_URL 사용 및 데이터 전송
      const response = await axios.post(`${API_BASE_URL}/api/signup`, {
        employeeId: employeeId.trim(), 
        password: password,
      }, {
        timeout: 5000 // 5초 타임아웃 추가
      });

      if (response.status === 201 || (response.data && response.data.result === "success")) {
        setSuccess(true);
        
        // ✅ 2초 후 로그인 화면으로 자동 이동
        // [주의] App.tsx에 등록된 로그인 화면의 name이 'Login'인지 'LoginScreen'인지 확인하세요.
        // 대부분 'Login'으로 등록되어 있을 확률이 높습니다.
        setTimeout(() => {
          navigation.navigate('Login'); // 'LoginScreen' 대신 'Login'으로 시도 (App.tsx 확인 필요)
        }, 2000);
      }
    } catch (err: any) {
      console.log("회원가입 에러:", err);
      const errorMessage = err.response?.data?.error || '서버에 연결할 수 없거나 회원가입 중 오류가 발생했습니다.';
      setError(errorMessage);
    }
  };

  // --- [A] 회원가입 성공 화면 (가입 완료 후 2초간 표시)
  if (success) {
    return (
      <View style={styles.successContainer}>
        <View style={styles.successCard}>
          <View style={styles.successBadge}>
            <CheckCircle size={60} color="white" strokeWidth={2} />
          </View>
          <Text style={styles.successTitle}>인증 및 가입 완료!</Text>
          <Text style={styles.successSub}>로그인 화면으로 이동합니다...</Text>
        </View>
      </View>
    );
  }

  // --- [B] 가입 입력 폼 화면
  return (
    <SafeAreaView style={styles.container}>
      <View style={styles.bgDecoration1} />
      <View style={styles.bgDecoration2} />

      <KeyboardAvoidingView
        behavior={Platform.OS === 'ios' ? 'padding' : 'height'}
        style={{ flex: 1 }}
      >
        <ScrollView contentContainerStyle={styles.scrollContent}>
          <View style={styles.card}>
            <View style={styles.header}>
              <View style={styles.logoIcon}>
                <Bot size={48} color="white" strokeWidth={2} />
              </View>
              <Text style={styles.title}>시스템 등록</Text>
              <Text style={styles.subtitle}>사원증 번호로 계정을 생성하세요</Text>
            </View>

            <View style={styles.form}>
              <View style={styles.inputGroup}>
                <Text style={styles.label}>사원증 번호 (ID)</Text>
                <View style={styles.inputWrapper}>
                  <CreditCard size={20} color="#94a3b8" style={styles.inputIcon} />
                  <TextInput
                    style={styles.input}
                    placeholder="사원증 번호를 입력하세요"
                    value={employeeId}
                    onChangeText={setEmployeeId}
                    autoCapitalize="none"
                  />
                </View>
              </View>

              <View style={styles.inputGroup}>
                <Text style={styles.label}>비밀번호 설정</Text>
                <View style={styles.inputWrapper}>
                  <Lock size={20} color="#94a3b8" style={styles.inputIcon} />
                  <TextInput
                    style={styles.input}
                    placeholder="비밀번호를 입력하세요"
                    value={password}
                    onChangeText={setPassword}
                    secureTextEntry
                  />
                </View>
              </View>

              <View style={styles.inputGroup}>
                <Text style={styles.label}>비밀번호 확인</Text>
                <View style={styles.inputWrapper}>
                  <Lock size={20} color="#94a3b8" style={styles.inputIcon} />
                  <TextInput
                    style={styles.input}
                    placeholder="비밀번호를 다시 입력하세요"
                    value={confirmPassword}
                    onChangeText={setConfirmPassword}
                    secureTextEntry
                  />
                </View>
              </View>

              {error ? (
                <View style={styles.errorBox}>
                  <Text style={styles.errorText}>{error}</Text>
                </View>
              ) : null}

              <TouchableOpacity
                style={styles.signupButton}
                onPress={handleSignup}
                activeOpacity={0.8}
              >
                <Text style={styles.signupButtonText}>인증 및 가입 완료</Text>
              </TouchableOpacity>

              <TouchableOpacity
                style={styles.backButton}
                onPress={() => navigation.navigate('Login')} // [수정] 네비게이션 이름 확인
              >
                <ChevronLeft size={20} color="#64748b" />
                <Text style={styles.backButtonText}>로그인으로 돌아가기</Text>
              </TouchableOpacity>
            </View>
          </View>
        </ScrollView>
      </KeyboardAvoidingView>
    </SafeAreaView>
  );
}

const styles = StyleSheet.create({
  container: { flex: 1, backgroundColor: '#0f172a' },
  bgDecoration1: {
    position: 'absolute', top: -50, left: -50, width: 250, height: 250,
    borderRadius: 125, backgroundColor: 'rgba(147, 51, 234, 0.1)',
  },
  bgDecoration2: {
    position: 'absolute', bottom: -50, right: -50, width: 250, height: 250,
    borderRadius: 125, backgroundColor: 'rgba(219, 39, 119, 0.1)',
  },
  scrollContent: { flexGrow: 1, justifyContent: 'center', padding: 20 },
  card: {
    backgroundColor: 'rgba(255, 255, 255, 0.95)', borderRadius: 32,
    padding: 25, elevation: 8,
  },
  header: { alignItems: 'center', marginBottom: 25 },
  logoIcon: {
    width: 72, height: 72, backgroundColor: '#9333ea', borderRadius: 20,
    justifyContent: 'center', alignItems: 'center', marginBottom: 12,
  },
  title: { fontSize: 28, fontWeight: 'bold', color: '#1e293b', marginBottom: 6 },
  subtitle: { fontSize: 14, color: '#64748b' },
  form: { width: '100%' },
  inputGroup: { marginBottom: 16 },
  label: { fontSize: 14, fontWeight: '600', color: '#334155', marginBottom: 6 },
  inputWrapper: {
    flexDirection: 'row', alignItems: 'center', borderWidth: 2,
    borderColor: '#e2e8f0', borderRadius: 16, backgroundColor: '#f8fafc',
    paddingHorizontal: 15,
  },
  inputIcon: { marginRight: 10 },
  input: { flex: 1, paddingVertical: 14, fontSize: 16, color: '#1e293b' },
  errorBox: {
    backgroundColor: '#fef2f2', borderColor: '#fecaca', borderWidth: 1,
    padding: 12, borderRadius: 12, marginBottom: 16,
  },
  errorText: { color: '#b91c1c', fontSize: 14, fontWeight: '500', textAlign: 'center' },
  signupButton: {
    backgroundColor: '#9333ea', paddingVertical: 18, borderRadius: 16,
    alignItems: 'center', marginBottom: 12, elevation: 4,
  },
  signupButtonText: { color: 'white', fontSize: 18, fontWeight: 'bold' },
  backButton: {
    flexDirection: 'row', justifyContent: 'center', alignItems: 'center',
    paddingVertical: 12, gap: 4,
  },
  backButtonText: { color: '#64748b', fontSize: 14, fontWeight: '600' },
  successContainer: {
    flex: 1, backgroundColor: '#0f172a', justifyContent: 'center',
    alignItems: 'center', padding: 30,
  },
  successCard: {
    width: '100%', backgroundColor: 'white', borderRadius: 32,
    padding: 40, alignItems: 'center', elevation: 10,
  },
  successBadge: {
    width: 90, height: 90, backgroundColor: '#10b981', borderRadius: 45,
    justifyContent: 'center', alignItems: 'center', marginBottom: 20,
  },
  successTitle: { fontSize: 24, fontWeight: 'bold', color: '#059669', marginBottom: 10 },
  successSub: { fontSize: 16, color: '#64748b' },
});