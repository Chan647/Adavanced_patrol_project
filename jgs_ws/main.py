# main.py

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import requests
import cv2
import time
import threading 
import numpy as np 
import concurrent.futures
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, BatteryState, CompressedImage, Imu
from geometry_msgs.msg import Twist,Pose, PoseArray
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from std_msgs.msg import Bool, String 
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import yaml
import os
import math

class CaptureTestNode(Node):
    def __init__(self):
        super().__init__('capture_test_node')
        
        # Flask 서버 설정
        self.base_url = "http://192.168.0.24:5000/api"
        self.logs_url = f"{self.base_url}/logs"
        self.update_url = f"{self.base_url}/robot/update"
        self.command_url = f"{self.base_url}/robot/command"
        self.map_url = f"{self.base_url}/map/upload"
        self.thread_pool = concurrent.futures.ThreadPoolExecutor(max_workers=5)

        try:
            requests.post(self.command_url, json={"command": "NONE"}, timeout=1.0)
        except:
            pass

        self.latest_map_msg = None
        self.map_lock = threading.Lock()
        
        self.saved_waypoints = [] 

        yaml_path = '/home/cho/lch_ws/src/pt_pkg/config/waypoints.yaml'
        if os.path.exists(yaml_path):
            try:
                with open(yaml_path, 'r') as f:
                    yaml_data = yaml.safe_load(f)
                    if yaml_data and 'waypoints' in yaml_data:
                        for wp in yaml_data['waypoints']:
                            self.saved_waypoints.append({
                                'x': float(wp['x']), 
                                'y': float(wp['y']), 
                                'label': wp.get('label', '미지정')
                            })
                self.get_logger().info(f"기존 경로 {len(self.saved_waypoints)}개 불러오기 성공")
            except Exception as e:
                self.get_logger().error(f"기존 경로 불러오기 실패 : {e}")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.bridge = CvBridge()
        self.current_battery = 0.0
        self.current_mode = "대기"
        self.is_charging = False
        self.is_moving = False
        self.is_panorama_running = False 
        
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        
        self.last_cmd_time = time.time()
        self.last_image_msg = None 
        self.last_command = None;
        self.last_battery = 0.0
        self.drop_start_time = None
        self.max_battery_seen = 0.0
        self.min_battery_seen = 0.0

        # 객체감지 / 화재감지 
        self.last_cpp_node_heartbeat = 0.0  
        self.fire_enable_pub = self.create_publisher(Bool, '/toggle_fire_detection', 10)
        self.person_enable_pub = self.create_publisher(Bool, '/toggle_person_detection', 10)
        self.is_fire_on = False
        self.is_person_on = False
        self.cpp_node_heartbeat_sub = self.create_subscription(Bool, '/cpp_node_heartbeat', self.cpp_node_callback, 10)

        # 음성 인식
        self.last_voice_node_heartbeat = 0.0  
        self.voice_node_enable_pub = self.create_publisher(Bool, '/toggle_voice_node', 10) 
        self.voice_node_heartbeat_sub = self.create_subscription(Bool, '/voice_node_heartbeat', self.voice_node_callback, 10)

        self.cpp_nav_command_pub = self.create_publisher(String, '/cpp_nav_command', 10)
        self.cpp_nav_waypoints_pub = self.create_publisher(PoseArray, '/cpp_nav_waypoints', 10)

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        img_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, 
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        ) 
        
        map_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.planned_path = []
        self.create_subscription(Path, '/planned_path', self.path_callback,10)
        self.create_subscription(CompressedImage, '/image_raw/compressed', self.image_callback, img_qos_profile)
        self.create_subscription(BatteryState, '/battery_state', self.battery_callback, qos_profile)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, map_qos_profile)
        self.create_subscription(Bool, '/person_detected', self.person_detect_callback, 10)
        self.create_subscription(Bool, '/fire_detected', self.fire_detect_callback, 10)
        self.create_subscription(Bool, '/intruder_detected', self.intruder_detect_callback, 10)

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.create_timer(0.2, self.update_robot_pose_from_tf)
        self.status_timer = self.create_timer(1.0, self.status_update_timer)
        self.command_timer = self.create_timer(0.1, self.check_command_timer)

        self.map_thread = threading.Thread(target=self.map_processing_loop, daemon=True)
        self.map_thread.start()

        self.target_twist = Twist()
        self.is_manual_enabled = False
        self.manual_timer = self.create_timer(0.1,self.publish_manual_cmd)
        self.get_logger().info(">>> 터틀봇 제어 노드 시작")

        # 객체 감지 
        self.last_log_time = {'사람 감지': 0, '화재 감지': 0, '침입자 감지': 0}
        self.LOG_COOLDOWN = 10.0

        self.is_fire_detected = False
        self.is_person_detected = False
        self.is_intruder_detected = False
        self.is_theft_detected = False

        # 충전 구역 이동 
        self.charging_pub = self.create_publisher(String, '/go_to_charging_zone', 10)

        # 도난 감지
        self.create_subscription(Bool, '/theft_detected', self.theft_detect_callback, 10)

    ############ threading 추가 #######
    def send_post_request_async(self, url, **kwargs):
        """HTTP POST 요청을 스레드 풀에서 비동기로 실행하여 메인 스레드 블로킹 방지"""
        def task():
            try:
                requests.post(url, **kwargs)
            except Exception as e:
                self.get_logger().error(f"비동기 통신 에러 ({url}): {e}")
        self.thread_pool.submit(task)


    ##################### 사람 or 화재 감지 및 로그 전송 (10sec) ##################
    def person_detect_callback(self, msg):
        self.is_person_detected = msg.data

        if not getattr(self, 'is_person_on', False):
            return
            
        current_time = time.time()

        if msg.data and (current_time - self.last_log_time['사람 감지'] >= self.LOG_COOLDOWN):
            self.last_log_time['사람 감지'] = current_time
            self.get_logger().warn("👤 사람 감지! 사진 촬영을 시작합니다.")
            self.capture_event_image("사람 감지")

    # 화재 감지 시 호출되는 함수
    def fire_detect_callback(self, msg):
        self.is_fire_detected = msg.data

        if not getattr(self, 'is_fire_on', False):
            return
            
        current_time = time.time()

        if msg.data and (current_time - self.last_log_time['화재 감지'] >= self.LOG_COOLDOWN):
            self.last_log_time['화재 감지'] = current_time
            self.get_logger().error("🔥 화재 감지! 긴급 사진 촬영을 시작합니다.")
            self.capture_event_image("🔥화재 감지")

    def intruder_detect_callback(self, msg):
        self.is_intruder_detected = msg.data 
        if msg.data and (time.time() - self.last_log_time.get('침입자 감지') > self.LOG_COOLDOWN):
            self.last_log_time['침입자 감지'] = time.time()
            self.get_logger().error("🚨 침입자 감지! 즉시 사진 촬영!")
            self.capture_event_image("🚨침입자 감지")

    def theft_detect_callback(self, msg):
        self.is_theft_detected = msg.data 
        
        # 쿨타임 X -> True 들어오면 즉시 촬영 
        if msg.data:
            self.get_logger().error("🚨 도난 감지! 즉시 사진 촬영 및 로그 저장!")
            self.capture_event_image("🚨도난 감지")


    # 공용 촬영 및 서버 전송 함수
    def capture_event_image(self, situation):
        if self.last_image_msg is None:
            self.get_logger().error("❌ 카메라 영상이 없어 사진을 찍을 수 없습니다.")
            return

        try:
            # 1. 현재 카메라 화면 캡처 및 변환
            np_arr = np.frombuffer(self.last_image_msg.data, np.uint8)
            cv_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            if cv_img is None:
                self.get_logger().error("❌ 이미지 디코딩 실패. DB 저장을 재시도합니다.")
                return

            _, buffer = cv2.imencode('.jpg', cv_img)


            # 2. 현재 위치 라벨 매칭 (파노라마 로직과 동일)
            position_label = "미지정 위치"
            if self.saved_waypoints:
                distances = [math.hypot(self.current_x - float(wp['x']), self.current_y - float(wp['y'])) 
                             for wp in self.saved_waypoints]
                closest_idx = int(np.argmin(distances))
                
                # 가장 가까운 지점과의 거리가 2.0m 이내일 때만 해당 라벨 사용
                if distances[closest_idx] < 2.0:
                    position_label = self.saved_waypoints[closest_idx]['label']
                else:
                    position_label = f"이동 중 (오차 {distances[closest_idx]:.1f}m)"

            # 3. Flask 서버(/api/logs)로 사진과 데이터 전송
            files = {'image': ('event.jpg', buffer.tobytes(), 'image/jpeg')}
            data = {'situation': situation, 'position': position_label}
            
            # 비동기식 
            self.send_post_request_async(self.logs_url, files=files, data=data, timeout=5.0)
            self.get_logger().info(f" 로그 전송 : {situation}")

        except Exception as e:
            self.get_logger().error(f"⚠️ 이벤트 로그 생성 중 에러 발생: {e}")




    def path_callback(self,msg):
        if not msg.poses:
            self.planned_path = []
            return
            
        path_data = []
        total_points = len(msg.poses)
        
        # 경로가 아무리 길어도 무조건 최대 40개의 포인트로 압축하여 통신 과부하 방지
        step = max(1, total_points // 40) 
        
        for i in range(0, total_points, step):
            pose = msg.poses[i].pose.position
            path_data.append({'x': float(pose.x), 'y': float(pose.y)})
            
        # 마지막 목적지는 도착 확인을 위해 정확하게 꽂아줌
        last_pose = msg.poses[-1].pose.position
        path_data.append({'x': float(last_pose.x), 'y': float(last_pose.y)})
        
        self.planned_path = path_data

    def cpp_node_callback(self, msg):
        if msg.data:
            self.last_cpp_node_heartbeat = time.time()

    def voice_node_callback(self,msg):
        if msg.data:
            self.last_voice_node_heartbeat = time.time()

    def update_robot_pose_from_tf(self):
        try:
            now = rclpy.time.Time()
            can_transform = self.tf_buffer.can_transform('map', 'base_footprint', now, rclpy.duration.Duration(seconds=0.1))
            if can_transform:
                trans = self.tf_buffer.lookup_transform('map', 'base_footprint', now)
                self.current_x = trans.transform.translation.x
                self.current_y = trans.transform.translation.y

                q = trans.transform.rotation
                siny_cosp = 2 * (q.w * q.z + q.x * q.y)
                cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
                self.current_theta = math.atan2(siny_cosp, cosy_cosp) # 라디안 값

        except Exception:
            pass
    
    def odom_callback(self, msg):
        pass

    def battery_callback(self, msg):
        raw_pct = msg.percentage
        if 0.0 < raw_pct <= 1.0: raw_pct *= 100.0
        current_pct = round(raw_pct, 1)
        current_time = self.get_clock().now()

        if self.max_battery_seen == 0.0:
            self.max_battery_seen = current_pct
            self.current_battery = current_pct
            return

        if self.is_moving or (time.time() - self.last_cmd_time < 5.0):
            self.is_charging = False
            self.min_battery_seen = current_pct
            self.current_battery = current_pct
            return

        if current_pct < self.min_battery_seen:
            self.min_battery_seen = current_pct

        diff = current_pct - self.current_battery
        if diff >= 4.0:
            if not self.is_charging: self.get_logger().info(f"🔌 충전 시작 감지: {current_pct}%")
            self.is_charging = True
            self.max_battery_seen = current_pct
            self.drop_start_time = None

        if self.is_charging:
            if current_pct > self.max_battery_seen: self.max_battery_seen = current_pct
            if current_pct <= (self.max_battery_seen - 2.0):
                if self.drop_start_time is None: self.drop_start_time = current_time 
                else:
                    elapsed = (current_time - self.drop_start_time).nanoseconds / 1e9
                    if elapsed >= 2.5: 
                        self.is_charging = False
                        self.drop_start_time = None
                        self.get_logger().info(f"🔋 충전 해제: {current_pct}%")
            else: self.drop_start_time = None
        self.current_battery = current_pct

    def image_callback(self, msg):
        self.last_image_msg = msg

    def cmd_vel_callback(self, msg):
        if self.is_panorama_running: return 
        if abs(msg.linear.x) > 0.001 or abs(msg.angular.z) > 0.001:
            self.last_cmd_time = time.time()
            self.is_moving = True
        else: self.is_moving = False

    def status_update_timer(self):
        if self.is_panorama_running: 
            self.current_mode = "파노라마 촬영"
        elif time.time() - self.last_cmd_time < 1.0: 
            pass 
        elif self.is_charging and not self.is_moving: 
            self.current_mode = "충전 중"
        else: 
            self.current_mode = "대기"
    
        is_cpp_alive = (time.time() - self.last_cpp_node_heartbeat < 2.0)
        is_voice_alive = (time.time() - self.last_voice_node_heartbeat < 2.0)


        try:
            status_payload = {
                "battery": float(self.current_battery),
                "drive_status": self.current_mode,
                "connection": "Stable",
                "power_status": 1 if self.is_charging else 0,
                "x": float(self.current_x),
                "y": float(self.current_y),
                "theta": float(getattr(self, 'current_theta', 0.0)),
                "isDetectionRunning": is_cpp_alive,
                "isFireDetectionOn": self.is_fire_on,
                "isPersonDetectionOn": self.is_person_on,
                "isVoiceRunning" : is_voice_alive,
                "planned_path": self.planned_path,
                "isFire": self.is_fire_detected,       
                "isPerson": self.is_person_detected,   
                "isIntruder": self.is_intruder_detected,
                "isTheft": getattr(self, 'is_theft_detected', False)
            }
            ## 비동기식##
            self.send_post_request_async(self.update_url, json=status_payload, timeout=0.5)
        except:
            pass

    def check_command_timer(self):
        try:
            response = requests.get(self.command_url, timeout=0.1)
            if response.status_code == 200:
                data = response.json()
                command = data.get('command')

                if command == self.last_command:
                    return

                if command and command != "NONE":
                    self.execute_command(command, data)
                    self.last_command = command 

                elif command == "NONE":
                    self.last_command = "NONE"

        except Exception as e:
            pass

    def publish_manual_cmd(self):
        if hasattr(self, 'is_manual_active') and self.is_manual_active:
            self.cmd_vel_pub.publish(self.target_twist)

    def execute_command(self, command, data=None):
        print(f"웹 : [{command}] ")
        
        # 객체 감지 스위치 ON/OFF
        if command == 'FIRE_ON':
            self.fire_enable_pub.publish(Bool(data=True))
            self.is_fire_on = True
            self.get_logger().info("🔥 화재 감지 시스템 활성화")

        elif command == 'FIRE_OFF':
            self.fire_enable_pub.publish(Bool(data=False))
            self.is_fire_on = False
            self.get_logger().info("🔥 화재 감지 시스템 비활성화")

        elif command == 'PERSON_ON':
            self.person_enable_pub.publish(Bool(data=True))
            self.is_person_on = True
            self.get_logger().info("👤 사람 감지 시스템 활성화")

        elif command == 'PERSON_OFF':
            self.person_enable_pub.publish(Bool(data=False))
            self.is_person_on = False
            self.get_logger().info("👤 사람 감지 시스템 비활성화")

        elif command == 'VOICE_ON':
            self.voice_node_enable_pub.publish(Bool(data=True))
            self.get_logger().info("🎙️ 음성 인식 시스템 활성화")

        elif command == 'VOICE_OFF':
            self.voice_node_enable_pub.publish(Bool(data=False))
            self.get_logger().info("🔇 음성 인식 시스템 비활성화")


       # 자율주행 시작 (once / loop)
        elif command in ['once', 'loop']:
            self.is_manual_active = False
            nav_msg = String()
            nav_msg.data = command 
            self.cpp_nav_command_pub.publish(nav_msg)
            self.current_mode = "주행 중"
            self.get_logger().info(f"🟢 주행명령 전송: {nav_msg.data}")

        # 자율주행 정지 (stop)
        elif command == 'stop':
            nav_msg = String()
            nav_msg.data = "stop"
            self.cpp_nav_command_pub.publish(nav_msg)
            self.current_mode = "정지"
            self.get_logger().info(f"🛑 정지명령 전송: {nav_msg.data}")

        # 경로 수신 및 YAML 파일 직접 저장
        elif command == 'WAYPOINT_NAVIGATION':
            if data and 'waypoints' in data:
                raw_points = data['waypoints']
                self.saved_waypoints = []
                
                yaml_data = {'waypoints': []}

                self.get_logger().info(f"\n📍 [웹 경로 수신] 총 {len(raw_points)}개 지점 수신")                
                
                for wp in raw_points:
                    label = wp.get('label', '미지정')

                    yaw_val = float(wp.get('yaw', 0.0))
                    
                    self.saved_waypoints.append({'x': float(wp['x']), 'y': float(wp['y']), 'label': label})
                    
                    yaml_data['waypoints'].append({
                        'x': float(wp['x']),
                        'y': float(wp['y']),
                        'yaw': yaw_val, 
                        'label': label
                    })

                    # 로그에 x, y, yaw 값 추가 
                    self.get_logger().info(f"  👉 {label} 등록: X={wp['x']:.2f}, Y={wp['y']:.2f}, Yaw={yaw_val:.2f}")
                
                yaml_path = '/home/cho/lch_ws/src/pt_pkg/config/waypoints.yaml'
                try:
                    os.makedirs(os.path.dirname(yaml_path), exist_ok=True)
                    with open(yaml_path, 'w') as f:
                        yaml.dump(yaml_data, f, default_flow_style=False, allow_unicode=True)
                    self.get_logger().info(f"✅ YAML 파일 저장 완료: {yaml_path}")
                except Exception as e:
                    self.get_logger().error(f"❌ YAML 저장 실패: {e}")
                
                self.get_logger().info("-" * 40)
                
            try: requests.post(self.command_url, json={"command": "NONE"}, timeout=0.5)
            except: pass

        elif command.startswith('MANUAL_'):
            self.last_cmd_time = time.time() 

            if not getattr(self, 'is_manual_active', False):
                nav_msg = String()
                nav_msg.data = "manual"
                self.cpp_nav_command_pub.publish(nav_msg)
                self.get_logger().info("🕹️ 수동 조작 모드 진입 → C++ 노드 cmd_vel 해제")

            if command == 'MANUAL_FORWARD':
                self.target_twist.linear.x = 0.15
                self.target_twist.angular.z = 0.0
                self.is_manual_active = True
            elif command == 'MANUAL_BACKWARD':
                self.target_twist.linear.x = -0.15
                self.target_twist.angular.z = 0.0
                self.is_manual_active = True
            elif command == 'MANUAL_LEFT':
                self.target_twist.linear.x = 0.0
                self.target_twist.angular.z = 0.5
                self.is_manual_active = True
            elif command == 'MANUAL_RIGHT':
                self.target_twist.linear.x = 0.0
                self.target_twist.angular.z = -0.5
                self.is_manual_active = True
            elif command == 'MANUAL_STOP':
                self.target_twist.linear.x = 0.0
                self.target_twist.angular.z = 0.0
                self.cmd_vel_pub.publish(self.target_twist) 
                self.is_manual_active = False
            
        elif command == 'PANORAMA' and not self.is_panorama_running:
            try: requests.post(self.command_url, json={"command": "NONE"}, timeout=0.5)
            except: pass
            threading.Thread(target=self.handle_panorama).start()

        # 구역 선택 이동 
        elif command == 'GO_TO_WAYPOINT':
            target_label = data.get('target') if data else None
            
            if not target_label:
                self.get_logger().error("❌ 목적지 이름이 비어있습니다.")
                return

            target_wp = next((wp for wp in self.saved_waypoints if wp['label'] == target_label), None)
            
            if target_wp:
                self.is_manual_active = False
                nav_msg = String()
                nav_msg.data = target_label  
                self.cpp_nav_command_pub.publish(nav_msg)
                
                self.current_mode = f"이동 중 ({target_label})"
                self.get_logger().info(f"🚀 목적지 이동 승인: '{target_label}'")
            else:
                self.get_logger().error(f"❌ '{target_label}' 은(는) 등록되지 않은 경로입니다!")

            try: requests.post(self.command_url, json={"command": "NONE"}, timeout=0.5)
            except: pass

        elif command == 'go_to_charging_zone':
            self.is_manual_active = False
            msg = String()
            msg.data = "charging"  
            self.cpp_nav_command_pub.publish(msg) 
            
            self.current_mode = "이동 중 (충전 구역)"
            self.get_logger().info("🔋 충전 구역(시작 위치)으로 복귀 명령 전송 완료!")

        # 충전 종료 후 대기 모드 전환 
        elif command == 'set_idle':
            self.is_manual_active = False
            msg = String()
            msg.data = "stop"  
            self.cpp_nav_command_pub.publish(msg)
            
            self.current_mode = "대기"
            self.get_logger().info("🛑 대기 모드 전환 (충전 종료)")

    # 파노라마 사진 촬영 -> 왼쪽으로 약92 도 회전이후 오른쪽으로 회전하며 쵤영 
    def handle_panorama(self):
        self.is_panorama_running = True
        self.get_logger().info("📸 파노라마 시퀀스 시작 (회전 보강)")
        
        captured_images = []
        try:
            twist = Twist()
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.5)

            twist.angular.z = -0.4
            for _ in range(30): # 0.4 rad/s * 4.0초 = 1.6 rad (약 92도)
                self.cmd_vel_pub.publish(twist)
                time.sleep(0.1)

            for i in range(8):
                twist.angular.z = 0.0
                self.cmd_vel_pub.publish(twist)
                time.sleep(1.0) 
                
                if self.last_image_msg is not None:
                    np_arr = np.frombuffer(self.last_image_msg.data, np.uint8)
                    cv_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                    
                    if cv_img is not None:
                        captured_images.append(cv_img)
                        self.get_logger().info(f"📸 {i+1}번 사진 캡처 완료")
                
                if i < 7:
                    twist.angular.z = 0.3 
                    for _ in range(10):
                        self.cmd_vel_pub.publish(twist)
                        time.sleep(0.1)

            if not captured_images:
                self.get_logger().error("❌ 카메라 데이터가 없어 파노라마를 종료합니다.")
                return

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
                    ch, cw = cropped.shape[:2]
                    final_image = cropped[int(ch * 0.10):ch-int(ch * 0.10), int(cw * 0.05):cw-int(cw * 0.05)]
                else:
                    final_image = pano
            else:
                self.get_logger().warn(f"⚠️ 병합 실패(에러코드:{status}). 강제 연결 모드 작동.")
                h_min = min(img.shape[0] for img in captured_images)
                resized_imgs = [cv2.resize(img, (int(img.shape[1] * h_min / img.shape[0]), h_min)) for img in captured_images]
                final_image = cv2.hconcat(resized_imgs)

            if final_image is not None:
                position_label = "미지정 위치"
                if self.saved_waypoints:
                    self.get_logger().info("\n🔍 [위치 매칭] 현재 TF 좌표와 경로 좌표 직접 비교")
                    distances = []

                    for wp in self.saved_waypoints:
                        target_x = float(wp['x'])
                        target_y = float(wp['y'])
                        
                        dist = np.hypot(self.current_x - target_x, self.current_y - target_y)
                        distances.append(dist)
                        
                        self.get_logger().info(f"   👉 {wp['label']} | 로봇:({self.current_x:.2f}, {self.current_y:.2f}) vs 웹:({target_x:.2f}, {target_y:.2f}) -> 거리: {dist:.2f}m")

                    if distances:
                        closest_idx = int(np.argmin(distances))
                        min_dist = distances[closest_idx]
                        matched_label = self.saved_waypoints[closest_idx]['label']
                        
                        if min_dist < 2.0: 
                            position_label = matched_label
                        else:
                            position_label = f"미지정 (오차 {min_dist:.1f}m)"

                _, buffer = cv2.imencode('.jpg', final_image)
                files = {'image': ('panorama.jpg', buffer.tobytes(), 'image/jpeg')}
                data = {'situation': '파노라마', 'position': position_label}
                requests.post(self.logs_url, files=files, data=data, timeout=10.0)
                self.get_logger().info(f"📡 DB 전송 완료 (위치 기록: {position_label})")

        except Exception as e: self.get_logger().error(f"에러: {e}")
        finally:
            self.is_panorama_running = False; self.current_mode = "대기"

    def map_callback(self, msg):
        with self.map_lock:
            self.latest_map_msg = msg

    def map_processing_loop(self):
        while rclpy.ok():
            msg = None
            with self.map_lock:
                if self.latest_map_msg is not None:
                    msg = self.latest_map_msg
                    self.latest_map_msg = None

            if msg is not None:
                try:
                    width, height = msg.info.width, msg.info.height
                    data = np.array(msg.data, dtype=np.int8).reshape((height, width))
                    img = np.full((height, width), 255, dtype=np.uint8)
                    img[data == -1] = 200  
                    img[data >= 50] = 0    
                    img = cv2.flip(img, 0)
                    img = cv2.rotate(img, cv2.ROTATE_90_COUNTERCLOCKWISE)

                    ret, buffer = cv2.imencode('.jpg', img)
                    if ret:
                        meta_data = {
                            'resolution': msg.info.resolution,
                            'origin_x': msg.info.origin.position.x,
                            'origin_y': msg.info.origin.position.y,
                            'width': height, 'height': width
                        }
                        ## 비동기식 ##
                        self.send_post_request_async(
                            self.map_url,files={'map_image': ('current_map.jpg', buffer.tobytes())},data=meta_data,timeout=2.0)
                except: pass
            time.sleep(1.0)

def main(args=None):
    rclpy.init(args=args)
    node = CaptureTestNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()