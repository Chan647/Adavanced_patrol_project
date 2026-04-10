/* AuthContext.tsx */

import React, { createContext, useContext, useState, useEffect } from 'react';
import AsyncStorage from '@react-native-async-storage/async-storage';

// 1. 서버 응답 데이터 인터페이스
interface User {
  userId: string;
  employeeId: string;
}

interface AuthContextType {
  user: User | null;
  isLoading: boolean; // 로컬 저장소에서 데이터를 읽어오는 중인지 확인하는 상태 추가
  login: (userData: User) => Promise<void>; 
  logout: () => Promise<void>;
}

const AuthContext = createContext<AuthContextType | undefined>(undefined);

export function AuthProvider({ children }: { children: React.ReactNode }) {
  const [user, setUser] = useState<User | null>(null);
  const [isLoading, setIsLoading] = useState(true);

  // 2. 앱 실행 시 AsyncStorage에서 기존 로그인 정보 불러오기
  useEffect(() => {
    const loadStorageData = async () => {
      try {
        const savedUser = await AsyncStorage.getItem('currentUser');
        if (savedUser) {
          setUser(JSON.parse(savedUser));
        }
      } catch (e) {
        console.error('Failed to load user from storage', e);
      } finally {
        setIsLoading(false);
      }
    };

    loadStorageData();
  }, []);

  // 3. Flask 서버 로그인 성공 후 호출될 함수 (AsyncStorage 저장)
  const login = async (userData: User) => {
    try {
      setUser(userData);
      await AsyncStorage.setItem('currentUser', JSON.stringify(userData));
    } catch (e) {
      console.error('Failed to save user', e);
    }
  };

  // 4. 로그아웃 함수
  const logout = async () => {
    try {
      setUser(null);
      await AsyncStorage.removeItem('currentUser');
    } catch (e) {
      console.error('Failed to remove user', e);
    }
  };

  return (
    <AuthContext.Provider value={{ user, isLoading, login, logout }}>
      {children}
    </AuthContext.Provider>
  );
}

export function useAuth() {
  const context = useContext(AuthContext);
  if (context === undefined) {
    throw new Error('useAuth must be used within an AuthProvider');
  }
  return context;
}