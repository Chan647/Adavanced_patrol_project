import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import requests
import cv2
import time
import threading 
import numpy as np 
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, BatteryState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid

class CaptureTestNode(Node):
    def __init__(self):
        super().__init__('capture_test_node')
        
        # Flask 서버 설정
        self.base_url = "http://192.168.0.5:5000/api"
        self.logs_url = f"{self.base_url}/logs"
        self.update_url = f"{self.base_url}/robot/update"
        self.command_url = f"{self.base_url}/robot/command"

        try:
            requests.post(self.command_url, json={"command": "NONE"}, timeout=1.0)
        except:
            pass

        # [추가] 지도 업로드 URL 및 스레드 락 설정
        self.map_url = f"{self.base_url}/map/upload"
        self.latest_map_msg = None
        self.map_lock = threading.Lock()

        self.saved_waypoints = []

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
        self.last_battery = 0.0
        self.drop_start_time = None

        self.max_battery_seen = 0.0

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # 구독 및 발행
        self.create_subscription(Image, '/image_raw', self.image_callback, qos_profile)
        self.create_subscription(BatteryState, '/battery_state', self.battery_callback, qos_profile)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        # [추가] 실시간 지도(OccupancyGrid) 구독 설정
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # 타이머
        self.status_timer = self.create_timer(1.0, self.status_update_timer)
        self.command_timer = self.create_timer(0.1, self.check_command_timer)

        # [추가] 지도 처리 및 전송을 위한 백그라운드 스레드 시작
        self.map_thread = threading.Thread(target=self.map_processing_loop, daemon=True)
        self.map_thread.start()

        self.get_logger().info(">>> 터틀봇 제어 및 맵 스트리밍 노드 시작")

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

    def battery_callback(self, msg):
        # 💡 배터리 값이 0.x 단위로 들어오면 100을 곱해 %로 맞춰줌 (기존 로직 유지)
        raw_pct = msg.percentage
        if 0.0 < raw_pct <= 1.0:
            raw_pct *= 100.0
            
        current_pct = round(raw_pct, 1)
        current_time = self.get_clock().now()

        # 사용자님 원본 배터리 로직 100% 그대로 복구
        if self.max_battery_seen == 0.0:
            self.max_battery_seen = current_pct
            self.current_battery = current_pct
            return

        diff = current_pct - self.current_battery
        if diff >= 3.0:
            self.is_charging = True
            self.max_battery_seen = current_pct
            self.drop_start_time = None

        if self.is_charging:
            if current_pct > self.max_battery_seen:
                self.max_battery_seen = current_pct

            if current_pct <= (self.max_battery_seen - 2.0):
                if self.drop_start_time is None:
                    self.drop_start_time = current_time 
                else:
                    elapsed = (current_time - self.drop_start_time).nanoseconds / 1e9
                    if elapsed >= 0.5: 
                        self.is_charging = False
                        self.drop_start_time = None
                        self.get_logger().info(f"🔌 충전 해제 확정: {self.max_battery_seen}% -> {current_pct}%")
            else:
                self.drop_start_time = None

        self.current_battery = current_pct
        self.get_logger().info(f"Batt: {current_pct}% | Max: {self.max_battery_seen}% | Charging: {self.is_charging}")

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
                    self.execute_command(command,data)
        except:
            pass

    def execute_command(self, command):
        twist = Twist()
        
        if command == 'WAYPOINT_NAVIGATION':
            if data and 'waypoints' in data:
                self.saved_waypoints = data['waypoints']
                self.get_logger().info(f"📍 경로 저장됨: 총 {len(self.saved_waypoints)}개 지점")
                for i, wp in enumerate(self.saved_waypoints):
                    self.get_logger().info(f"  - [{i}] {wp['type']}: x={wp['x']}, y={wp['y']}")
            
            # 명령 중복 실행 방지를 위해 서버 비우기
            try: requests.post(self.command_url, json={"command": "NONE"}, timeout=0.5)
            except: pass

        # 쓸데없는 터미널 도배 로그 삭제 완료
        elif command.startswith('MANUAL_'):
            self.current_mode = "수동"
            self.last_cmd_time = time.time()
            if command == 'MANUAL_FORWARD': twist.linear.x = 0.15
            elif command == 'MANUAL_BACKWARD': twist.linear.x = -0.15
            elif command == 'MANUAL_LEFT': twist.angular.z = 0.5
            elif command == 'MANUAL_RIGHT': twist.angular.z = -0.5
            self.cmd_vel_pub.publish(twist)
            
        elif command == 'PANORAMA' and not self.is_panorama_running:
            try:
                requests.post(self.command_url, json={"command": "NONE"}, timeout=0.5)
            except:
                pass
            threading.Thread(target=self.handle_panorama).start()
            
        elif command == 'MANUAL_STOP':
            self.current_mode = "정지"
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)

    def handle_panorama(self):
        self.is_panorama_running = True
        self.get_logger().info("📸 파노라마 시퀀스 시작")
        
        captured_images = []
        try:
            twist = Twist()
            for i in range(8):
                twist.angular.z = 0.0
                self.cmd_vel_pub.publish(twist)
                time.sleep(1.0) 
                
                if self.last_image_msg:
                    cv_img = self.bridge.imgmsg_to_cv2(self.last_image_msg, "bgr8")
                    captured_images.append(cv_img)
                    self.get_logger().info(f"📸 {i+1}번 사진 캡처 완료")
                
                if i < 7:
                    twist.angular.z = 0.2 
                    self.cmd_vel_pub.publish(twist)
                    time.sleep(1.0) 

            self.get_logger().info("🎨 이미지 합성 시작...")
            stitcher = cv2.Stitcher_create()
            status, pano = stitcher.stitch(captured_images)

            final_image = None
            if status == cv2.Stitcher_OK:
                self.get_logger().info("✅ 파노라마 병합 성공!")

                gray = cv2.cvtColor(pano, cv2.COLOR_BGR2GRAY)
                _, thresh = cv2.threshold(gray, 1, 255, cv2.THRESH_BINARY)
                contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                if contours:
                    c = max(contours, key=cv2.contourArea)
                    x, y, w, h = cv2.boundingRect(c)
                    cropped = pano[y:y+h, x:x+w]
                    
                    # 가장자리 곡선을 없애기 위해 안쪽으로 조금 더 파내기 (상하 10%, 좌우 5%)
                    ch, cw = cropped.shape[:2]
                    crop_y = int(ch * 0.10)
                    crop_x = int(cw * 0.05)
                    final_image = cropped[crop_y:ch-crop_y, crop_x:cw-crop_x]
                else:
                    final_image = pano
            else:
                self.get_logger().warn(f"⚠️ 병합 실패(에러코드:{status}). 강제 연결 모드 작동.")
                h_min = min(img.shape[0] for img in captured_images)
                resized_imgs = [cv2.resize(img, (int(img.shape[1] * h_min / img.shape[0]), h_min)) for img in captured_images]
                final_image = cv2.hconcat(resized_imgs)

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

    # ------------------------------------------------------------------
    # [추가] 지도 관련 메서드 영역 (완벽 독립 스레드)
    # ------------------------------------------------------------------
    def map_callback(self, msg):
        with self.map_lock:
            self.latest_map_msg = msg

    def map_processing_loop(self):
        import rclpy
        while rclpy.ok():
            msg = None
            with self.map_lock:
                if self.latest_map_msg is not None:
                    msg = self.latest_map_msg
                    self.latest_map_msg = None

            if msg is not None:
                try:
                    width = msg.info.width
                    height = msg.info.height
                    resolution = msg.info.resolution
                    origin_x = msg.info.origin.position.x
                    origin_y = msg.info.origin.position.y

                    data = np.array(msg.data, dtype=np.int8).reshape((height, width))
                    img = np.full((height, width), 255, dtype=np.uint8)
                    
                    img[data == -1] = 200  
                    img[data >= 50] = 0    
                    img = cv2.flip(img, 0)
                    img = cv2.rotate(img, cv2.ROTATE_90_COUNTERCLOCKWISE)

                    ret, buffer = cv2.imencode('.jpg', img)
                    if ret:
                        files = {'map_image': ('current_map.jpg', buffer.tobytes(), 'image/jpeg')}
                        meta_data = {
                            'resolution': resolution,
                            'origin_x': origin_x,
                            'origin_y': origin_y,
                            'width': height,
                            'height': width
                        }
                        requests.post(self.map_url, files=files, data=meta_data, timeout=2.0)
                except Exception as e:
                    pass
            
            time.sleep(1.0)

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