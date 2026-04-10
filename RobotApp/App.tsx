/* App.tsx */

import React from 'react';
import { NavigationContainer } from '@react-navigation/native';
import { createNativeStackNavigator } from '@react-navigation/native-stack';
import { SafeAreaProvider } from 'react-native-safe-area-context';
import { LogBox, ActivityIndicator, View } from 'react-native'; 

// ✅ Contexts
import { AuthProvider, useAuth } from './contexts/AuthContext'; 
import { RobotProvider } from './contexts/RobotContext'; 

// ✅ Screens
import LoginScreen from './screens/LoginScreen';
import MainScreen from './screens/MainScreen';
import { WaitScreen } from './screens/WaitScreen';
import PanoramaScreen from './screens/PanoramaScreen';
import SignupScreen from './screens/SignupScreen';
import ChargingScreen from './screens/ChargingScreen';
import { ManualScreen } from './screens/ManualScreen';

// 💡 참고: 모달 컴포넌트들은 여기서 import 할 필요가 없습니다. 
// 각 모달을 실제로 사용하는 화면(예: MainScreen) 내부에서 import 하세요.

// 경고 메시지 무시
LogBox.ignoreAllLogs(); 

const Stack = createNativeStackNavigator();

/**
 * 로그인 여부 및 로딩 상태에 따라 화면을 분기하는 컴포넌트
 */
function AppNavigator() {
  const { user, isLoading } = useAuth();

  if (isLoading) {
    return (
      <View style={{ flex: 1, justifyContent: 'center', alignItems: 'center', backgroundColor: '#0f172a' }}>
        <ActivityIndicator size="large" color="#4d61ff" />
      </View>
    );
  }

  return (
    <Stack.Navigator screenOptions={{ headerShown: false }}>
      {/* ✅ 회원가입과 로그인은 어떤 상태에서든 접근 가능하도록 밖으로 뺍니다 */}
      <Stack.Screen name="LoginScreen" component={LoginScreen} />
      <Stack.Screen name="SignupScreen" component={SignupScreen} />

      {user ? (
        // ✅ 로그인 성공 시에만 접근 가능한 화면들
        <>
          <Stack.Screen name="WaitScreen" component={WaitScreen} />
          <Stack.Screen name="MainScreen" component={MainScreen} />
          <Stack.Screen name="PanoramaScreen" component={PanoramaScreen} />
          <Stack.Screen name="ChargingScreen" component={ChargingScreen} />
          <Stack.Screen name="ManualScreen" component={ManualScreen} />
        </>
      ) : null}
    </Stack.Navigator>
  );
}

export default function App() {
  return (
    <SafeAreaProvider>
      <AuthProvider>
        <RobotProvider>
          <NavigationContainer>
            <AppNavigator />
          </NavigationContainer>
        </RobotProvider>
      </AuthProvider>
    </SafeAreaProvider>
  );
}