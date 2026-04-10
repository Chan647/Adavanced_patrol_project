import { useColorScheme } from './use-color-scheme';
// 6번 줄의 경로가 맞다면 그대로 두시고, 만약 파일이 없으면 아래처럼 직접 정의합니다.
import { Colors } from '../../constants/theme'; 

export function useThemeColor(
  props: { light?: string; dark?: string },
  colorName: keyof typeof Colors.light & keyof typeof Colors.dark
) {
  // theme 타입을 'light' | 'dark'로 강제 지정하여 오류를 방지합니다.
  const theme = (useColorScheme() ?? 'light') as 'light' | 'dark';
  const colorFromProps = props[theme];

  if (colorFromProps) {
    return colorFromProps;
  } else {
    // Colors[theme] 부분의 인덱스 에러를 방지하기 위해 타입을 명시합니다.
    return Colors[theme][colorName];
  }
}