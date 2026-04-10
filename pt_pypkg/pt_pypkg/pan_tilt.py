import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Vector3
import serial
import threading


class PanTiltNode(Node):
    def __init__(self):
        super().__init__('pan_tilt_node')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('kp_pan',  30.0)
        self.declare_parameter('kp_tilt', 20.0)
        self.declare_parameter('lpf_alpha', 0.3)

        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baud').get_parameter_value().integer_value

        try:
            self.ser = serial.Serial(port, baud, timeout=1)
            self.get_logger().info(f'아두이노 연결 성공: {port}')
        except Exception as e:
            self.get_logger().error(f'아두이노 연결 실패: {e}')
            raise

        self.pan_angle  = 90.0
        self.tilt_angle = 30.0

        self.target_pan  = 90.0
        self.target_tilt = 90.0

        self.person_visible = False
        self.bbox_x = 0.0
        self.bbox_y = 0.0

        self.sub_person = self.create_subscription(
            Bool, '/person_detected',
            self.person_callback, 10)

        self.sub_bbox = self.create_subscription(
            Vector3, '/person_bbox',
            self.bbox_callback, 10)

        self.timer = self.create_timer(0.05, self.control_loop)

        self.recv_thread = threading.Thread(
            target=self.serial_recv_loop, daemon=True)
        self.recv_thread.start()

        self.send_command(90, 90)
        self.get_logger().info('PanTilt 노드 시작 완료')

    def person_callback(self, msg: Bool):
        self.person_visible = msg.data
        if not msg.data:
            self.target_pan  = 90.0
            self.target_tilt = 90.0
            self.get_logger().info('사람 사라짐 → 정면 복귀')

    def bbox_callback(self, msg: Vector3):
        if not self.person_visible:
            return

        kp_pan  = self.get_parameter('kp_pan').get_parameter_value().double_value
        kp_tilt = self.get_parameter('kp_tilt').get_parameter_value().double_value

        self.bbox_x = msg.x
        self.bbox_y = msg.y
        self.target_pan  = self.pan_angle  + kp_pan  * self.bbox_x
        self.target_tilt = self.tilt_angle + kp_tilt * self.bbox_y

        self.target_pan  = max(0.0,   min(180.0, self.target_pan))
        self.target_tilt = max(0.0,  min(150.0, self.target_tilt))

    def control_loop(self):
        alpha = self.get_parameter('lpf_alpha').get_parameter_value().double_value

        self.pan_angle  = alpha * self.target_pan  + (1.0 - alpha) * self.pan_angle
        self.tilt_angle = alpha * self.target_tilt + (1.0 - alpha) * self.tilt_angle

        self.send_command(int(self.pan_angle), int(self.tilt_angle))

    def send_command(self, pan: int, tilt: int):
        cmd = f'P{pan}T{tilt}\n'
        try:
            self.ser.write(cmd.encode())
        except Exception as e:
            self.get_logger().warn(f'시리얼 전송 오류: {e}')

    def serial_recv_loop(self):
        while rclpy.ok():
            try:
                line = self.ser.readline().decode().strip()
                if line:
                    self.get_logger().debug(f'아두이노 응답: {line}')
            except Exception as e:
                self.get_logger().warn(f'시리얼 수신 오류: {e}')

    def destroy_node(self):
        self.send_command(90, 90)
        self.get_logger().info('정면 복귀 후 종료')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PanTiltNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
