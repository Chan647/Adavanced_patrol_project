import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import requests
import cv2
import time
import threading 
import numpy as np # [추가] 이미지 강제 병합 처리를 위해 필수
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, BatteryState
from geometry_msgs.msg import Twist
# [추가] 오도메트리 메시지 타입 임포트
from nav_msgs.msg import Odometry

class CaptureTestNode(Node):
    def __init__(self):
        super().__init__('capture_test_node')
        
        # Flask 서버 설정
        self.base_url = "http://192.168.0.5:5000/api"
        self.logs_url = f"{self.base_url}/logs"
        self.update_url = f"{self.base_url}/robot/update"
        self.command_url = f"{self.base_url}/robot/command"

        self.bridge = CvBridge()
        self.current_battery = 0.0
        self.current_mode = "대기"
        self.is_charging = False
        self.is_moving = False
        self.is_panorama_running = False 
        
        # 현재 좌표 저장 변수 초기화
        self.current_x = 0.0
        self.current_y = 0.0
        
        self.last_cmd_time = time.time()
        self.last_image_msg = None 
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # 구독 및 발행
        self.create_subscription(Image, '/image_raw', self.image_callback, qos_profile)
        self.create_subscription(BatteryState, '/battery_state', self.battery_callback, qos_profile)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        # [추가] 실시간 위치(Odometry) 구독 설정
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # 타이머
        self.status_timer = self.create_timer(1.0, self.status_update_timer)
        self.command_timer = self.create_timer(0.1, self.check_command_timer)

        self.get_logger().info(">>> 터틀봇 파노라마 Stitching 노드 시작")

    # [추가] 오도메트리 데이터를 변수에 업데이트하는 콜백
    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

    def battery_callback(self, msg):
        self.current_battery = round(msg.percentage, 1)
        
        is_plugged_in = msg.power_supply_status in [0, 1, 4]
        is_high_voltage = msg.voltage > 12.1  # 전압 수치가 12.1 초과일때 충전중 표시 
        
        self.get_logger().info(f"Battery Voltage: {msg.voltage:.2f}V | Charging: {'YES' if is_plugged_in and is_high_voltage else 'NO'}")
        self.is_charging = is_plugged_in and is_high_voltage

    def image_callback(self, msg):
        self.last_image_msg = msg

    def cmd_vel_callback(self, msg):
        if self.is_panorama_running: return 
        if abs(msg.linear.x) > 0.001 or abs(msg.angular.z) > 0.001:
            self.last_cmd_time = time.time()
            self.is_moving = True
        else:
            self.is_moving = False

    def status_update_timer(self):
        if self.is_panorama_running:
            self.current_mode = "파노라마 촬영"
        elif time.time() - self.last_cmd_time < 1.0:
            pass 
        elif self.is_charging:
            self.current_mode = "충전 중"
        else:
            self.current_mode = "대기"

        try:
            # [수정] 좌표(x, y)를 페이로드에 포함하여 전송
            status_payload = {
                "battery": float(self.current_battery),
                "drive_status": self.current_mode,
                "connection": "Stable",
                "power_status": 1 if self.is_charging else 0,
                "x": float(self.current_x),
                "y": float(self.current_y)
            }
            requests.post(self.update_url, json=status_payload, timeout=0.5)
        except:
            pass

    def check_command_timer(self):
        try:
            response = requests.get(self.command_url, timeout=0.2)
            if response.status_code == 200:
                data = response.json()
                command = data.get('command')
                if command:
                    self.execute_command(command)
        except:
            pass

    def execute_command(self, command):
        twist = Twist()
        if command.startswith('MANUAL_'):
            self.current_mode = "수동"
            self.last_cmd_time = time.time()
            if command == 'MANUAL_FORWARD': twist.linear.x = 0.15
            elif command == 'MANUAL_BACKWARD': twist.linear.x = -0.15
            elif command == 'MANUAL_LEFT': twist.angular.z = 0.5
            elif command == 'MANUAL_RIGHT': twist.angular.z = -0.5
            self.cmd_vel_pub.publish(twist)
            
        # 웹에서 'PANORAMA' 명령을 보냄
        elif command == 'PANORAMA' and not self.is_panorama_running:
            threading.Thread(target=self.handle_panorama).start()
            
        elif command == 'MANUAL_STOP':
            self.current_mode = "정지"
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)

    def handle_panorama(self):
        """[수정됨] 병합 실패 시 강제 연결 기능이 포함된 파노라마 로직"""
        self.is_panorama_running = True
        self.get_logger().info("📸 파노라마 시퀀스 시작 ")
        
        captured_images = []
        try:
            twist = Twist()
            # 1. 3개 지점에서 촬영 (약 120~150도 범위)
            for i in range(8):
                # 정지 및 대기 (흔들림 방지)
                twist.angular.z = 0.0
                self.cmd_vel_pub.publish(twist)
                time.sleep(1.0) 
                
                if self.last_image_msg:
                    cv_img = self.bridge.imgmsg_to_cv2(self.last_image_msg, "bgr8")
                    captured_images.append(cv_img)
                    self.get_logger().info(f"📸 {i+1}번 사진 캡처 완료")
                
                # 회전 
                if i < 7:
                    twist.angular.z = 0.2 
                    self.cmd_vel_pub.publish(twist)
                    time.sleep(1.0) 

            # 2. 이미지 합성 (Stitching)
            self.get_logger().info("🎨 이미지 합성 시작...")
            stitcher = cv2.Stitcher_create()
            status, pano = stitcher.stitch(captured_images)

            final_image = None
            if status == cv2.Stitcher_OK:
                self.get_logger().info("✅ 파노라마 병합 성공!")
                final_image = pano
            else:
                # 🔥 병합 실패 시 (Error 1 등) 단순 가로 연결로 대응 (무조건 결과물 생성)
                self.get_logger().warn(f"⚠️ 병합 실패(에러코드:{status}). 강제 연결 모드 작동.")
                # 모든 이미지 높이를 통일한 뒤 가로로 붙임
                h_min = min(img.shape[0] for img in captured_images)
                resized_imgs = [cv2.resize(img, (int(img.shape[1] * h_min / img.shape[0]), h_min)) for img in captured_images]
                final_image = cv2.hconcat(resized_imgs)

            # 3. 서버 전송
            if final_image is not None:
                _, buffer = cv2.imencode('.jpg', final_image)
                files = {'image': ('panorama.jpg', buffer.tobytes(), 'image/jpeg')}
                data = {
                    'situation': '파노라마',
                    'position': f'Section {round(self.current_x, 2)}'
                }
                requests.post(self.logs_url, files=files, data=data, timeout=10.0)
                self.get_logger().info("📡 파노라마 전송 완료")

        except Exception as e:
            self.get_logger().error(f"파노라마 프로세스 에러: {e}")
        finally:
            self.is_panorama_running = False
            self.current_mode = "대기"

def main(args=None):
    rclpy.init(args=args)
    node = CaptureTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()