import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from math import atan2, sqrt, sin, pi
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import LaserScan
import numpy as np
from rclpy.qos import qos_profile_sensor_data

class PurePursuit(Node):
    def __init__(self):
        super().__init__('pure_pursuit_node')
        self.stat = 'move'

        self.declare_parameter('goal_x', 0.0)
        self.declare_parameter('goal_y', 0.0)
        self.add_on_set_parameters_callback(self.parameter_callback)

        self.goal_x = self.get_parameter('goal_x').value
        self.goal_y = self.get_parameter('goal_y').value

        self.lookahead_distance = 0.45
        self.linear_velocity = 0.1
        self.goal_tolerance = 0.3
        self.obs_threshold = 0.3

        self.path = [[self.goal_x, self.goal_y]]
        self.current_waypoint_index = 0

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub = self.create_subscription(PoseWithCovarianceStamped,'/amcl_pose', self.pose_callback, qos_profile_sensor_data)
        self.scan_sub = self.create_subscription(LaserScan, '/scan',self.callback, qos_profile_sensor_data)

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        self.left_dist = 99.9
        self.front_dist = 99.9
        self.right_dist = 99.9
        self.is_localized = False

        self.timer = self.create_timer(0.5, self.control)
        self.get_logger().info("Pure Pursuit Node start")


    def callback(self, msg):
        front_ranges = msg.ranges[0:30] + msg.ranges[-30:]
        left_ranges = msg.ranges[20:70]
        right_ranges = msg.ranges[-70:-20]

        self.front_dist = self.get_min_dist(front_ranges)
        self.left_dist = self.get_min_dist(left_ranges)
        self.right_dist = self.get_min_dist(right_ranges)

    def get_min_dist(self, range_list):

        valid_values = [x for x in range_list if x > 0.05 and x < 10.0]
        if valid_values:
            return min(valid_values)
        return 99.9

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'goal_x':
                self.goal_x = param.value
                self.get_logger().info(f'goal_x updated: {self.goal_x}')
            if param.name == 'goal_y':
                self.goal_y = param.value
                self.get_logger().info(f'goal_y updated: {self.goal_y}')

        self.path = [[self.goal_x, self.goal_y]]
        self.current_waypoint_index = 0
        return SetParametersResult(successful=True)


    def pose_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        self.current_yaw = atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
        self.is_localized = True


    def control(self):
        if not self.is_localized:
            return

        if self.front_dist < self.obs_threshold:
            self.get_logger().warn(f"Obstacle Ahead! ({self.front_dist:.2f}m)")
            self.obstacle_move()
            return

        if self.current_waypoint_index >= len(self.path):
            self.stop_robot()
            return

        self.control_cmd()

    def obstacle_move(self):

        cmd = Twist()
        cmd.linear.x = 0.0
        if self.left_dist > self.right_dist:
            cmd.angular.z = 0.7 
            self.get_logger().info("Turn Left!")
        else:
            cmd.angular.z = -0.7
        
        self.pub.publish(cmd)


    def control_cmd(self):

        goal_x, goal_y = self.path[self.current_waypoint_index]

        dx = goal_x - self.current_x
        dy = goal_y - self.current_y
        distance = max(sqrt(dx * dx + dy * dy), 0.01)

        if distance < self.goal_tolerance:
            self.get_logger().info(f'Waypoint {self.current_waypoint_index} Reached')
            self.current_waypoint_index += 1
            return

        target_angle = atan2(dy, dx)
        alpha = target_angle - self.current_yaw

        if alpha > pi:
            alpha -= 2 * pi
        elif alpha < -pi:
            alpha += 2 * pi

        angular_velocity = self.linear_velocity * (2.0 * sin(alpha)) / distance

        cmd = Twist()   
        cmd.linear.x = self.linear_velocity
        cmd.angular.z = angular_velocity

        if cmd.angular.z > 1.0:
            cmd.angular.z = 1.0
        if cmd.angular.z < -1.0:
            cmd.angular.z = -1.0

        self.pub.publish(cmd)


    def stop_robot(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.pub.publish(cmd)
        self.get_logger().info('All waypoints completed. Robot Stop')


def main(args=None):
    rclpy.init()
    node = PurePursuit()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()