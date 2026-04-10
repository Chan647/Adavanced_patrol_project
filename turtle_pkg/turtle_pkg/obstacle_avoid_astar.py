import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import LaserScan
from math import pow, atan2, sqrt, sin, pi
from sensor_msgs.msg import CompressedImage
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory
from rclpy.qos import qos_profile_sensor_data
import cv2
import heapq
import numpy as np
import os

class NodeAStar:
    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position
        self.g = 0
        self.h = 0
        self.f = 0
    def __eq__(self, other): return self.position == other.position
    def __lt__(self, other): return self.f < other.f

class IntegratedNavigation(Node):
    def __init__(self):
        super().__init__('integrated_navigation')

        self.lookahead_dist = 0.5
        self.linear_vel = 0.12
        self.stop_tolerance = 0.3
        self.robot_radius = 0.24
        
        self.map_data = None
        self.map_resolution = 0.05
        self.map_origin = [0.0, 0.0]
        self.map_width = 0
        self.map_height = 0
        self.robot_radius_cells = int(self.robot_radius / self.map_resolution)
        
        self.current_pose = None
        self.current_yaw = 0.0
        self.global_path = []
        self.path_index = 0
        self.lid_detected = False

        self.left_dist = 99.9
        self.front_dist = 99.9
        self.right_dist = 99.9
        self.obs_threshold = 0.3

        pkg_path = get_package_share_directory('my_turtle_pkg')
        model_path = os.path.join(pkg_path, 'config', 'best.pt')
        self.model = YOLO(model_path)

        
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_path = self.create_publisher(Path, '/planned_path', 10)

        self.sub_map = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.sub_pose = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10)
        self.sub_goal = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.sub_image = self.create_subscription(CompressedImage, '/image_raw/compressed', self.image_callback, 10)
        self.sub_scan = self.create_subscription(LaserScan, '/scan',self.scan_callback, qos_profile_sensor_data)

        self.timer = self.create_timer(0.3, self.control_loop)
        self.get_logger().info("Let's Run!")

    def scan_callback(self, msg):
        front_ranges = msg.ranges[0:30] + msg.ranges[-30:]
        left_ranges = msg.ranges[20:70]
        right_ranges = msg.ranges[-70:-20]

        self.front_dist = self.get_min_dist(front_ranges)
        self.left_dist = self.get_min_dist(left_ranges)
        self.right_dist = self.get_min_dist(right_ranges)

    def map_callback(self, msg):
        self.map_resolution = msg.info.resolution
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_origin = [msg.info.origin.position.x, msg.info.origin.position.y]
        self.map_data = np.array(msg.data).reshape((self.map_height, self.map_width))

    def pose_callback(self, msg):
        self.current_pose = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        q = msg.pose.pose.orientation
        self.current_yaw = atan2(2.0*(q.w*q.z + q.x*q.y), 1.0-2.0*(q.y*q.y + q.z*q.z))

    def goal_callback(self, msg):
        if self.map_data is None or self.current_pose is None: return

        goal_pose = [msg.pose.position.x, msg.pose.position.y]
        start_grid = self.world_to_grid(self.current_pose)
        goal_grid = self.world_to_grid(goal_pose)
        
        self.get_logger().info("Calculating Path...")
        path_grid = self.run_astar(start_grid, goal_grid)
        
        if path_grid:
            self.global_path = [self.grid_to_world(p) for p in path_grid]
            self.path_index = 0
            self.publish_path_viz()
            self.get_logger().info("Path Found! Go!")
        else:
            self.get_logger().warn("No Path Found.")

    def run_astar(self, start, end):
        if not (0 <= start[0] < self.map_height and 0 <= start[1] < self.map_width): return None
        if not (0 <= end[0] < self.map_height and 0 <= end[1] < self.map_width): return None

        start_node = NodeAStar(None, start)
        end_node = NodeAStar(None, end)
        open_list = []
        heapq.heappush(open_list, start_node)
        visited = set()
        moves = [(0,1), (0,-1), (1,0), (-1,0), (1,1), (1,-1), (-1,1), (-1,-1)]

        while open_list:
            current_node = heapq.heappop(open_list)
            if current_node.position in visited: continue
            visited.add(current_node.position)

            if current_node.position == end_node.position:
                path = []
                current = current_node
                while current:
                    path.append(current.position)
                    current = current.parent
                return path[::-1]

            for move in moves:
                ny, nx = current_node.position[0] + move[0], current_node.position[1] + move[1]
                if not (0 <= ny < self.map_height and 0 <= nx < self.map_width): continue
                if self.map_data[ny][nx] != 0: continue

                too_close = False
                for dy in range(-self.robot_radius_cells, self.robot_radius_cells + 1):
                    for dx in range(-self.robot_radius_cells, self.robot_radius_cells + 1):
                        cy, cx = ny + dy, nx + dx
                        if 0 <= cy < self.map_height and 0 <= cx < self.map_width:
                            if self.map_data[cy][cx] != 0:
                                too_close = True
                                break
                    if too_close:
                        break

                if too_close:
                    continue
                
                new_node = NodeAStar(current_node, (ny, nx))
                new_node.g = current_node.g + 1
                new_node.h = sqrt((ny - end[0])**2 + (nx - end[1])**2)
                new_node.f = new_node.g + new_node.h
                heapq.heappush(open_list, new_node)
        return None

    def control_loop(self):
        if not self.global_path: return

        if self.front_dist < self.obs_threshold:
            self.get_logger().warn(f"Obstacle Ahead! ({self.front_dist:.2f}m)")
            self.obstacle_move()
            return

        final_goal = self.global_path[-1]

        dist_to_final = sqrt((final_goal[0]-self.current_pose[0])**2 + (final_goal[1]-self.current_pose[1])**2)

        if dist_to_final < self.stop_tolerance:
            self.global_path = []
            self.stop_robot()
            return

        target_x, target_y = final_goal

        for i in range(self.path_index, len(self.global_path)):
            px, py = self.global_path[i]
            dist = sqrt((px - self.current_pose[0])**2 + (py - self.current_pose[1])**2)

            if dist >= self.lookahead_dist:
                target_x, target_y = px, py
                self.path_index = i
                break

        dx = target_x - self.current_pose[0]
        dy = target_y - self.current_pose[1]
        alpha = atan2(dy, dx) - self.current_yaw
        
        if alpha > pi: alpha -= 2*pi
        elif alpha < -pi: alpha += 2*pi

        cmd = Twist()

        if abs(alpha) > 1.0:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.8 if alpha > 0 else -0.8

        elif self.lid_detected:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        else:
            angular_velocity = self.linear_vel * (2.0 * sin(alpha)) / self.lookahead_dist
            cmd.linear.x = self.linear_vel
            cmd.angular.z = angular_velocity
        
        if cmd.angular.z > 1.0: cmd.angular.z = 1.0
        if cmd.angular.z < -1.0: cmd.angular.z = -1.0
        
        self.pub_cmd.publish(cmd)

    def obstacle_move(self):

        cmd = Twist()
        cmd.linear.x = 0.0
        if self.left_dist > self.right_dist:
            cmd.angular.z = 0.7 
            self.get_logger().info("Turn Left!")
        else:
            cmd.angular.z = -0.7
        
        self.pub_cmd.publish(cmd)


    def world_to_grid(self, world):
        return (int((world[1]-self.map_origin[1])/self.map_resolution),
                int((world[0]-self.map_origin[0])/self.map_resolution))

    def grid_to_world(self, grid):
        return [(grid[1]*self.map_resolution)+self.map_origin[0],
                (grid[0]*self.map_resolution)+self.map_origin[1]]

    def publish_path_viz(self):
        msg = Path()
        msg.header.frame_id = 'map'
        for p in self.global_path:
            ps = PoseStamped()
            ps.pose.position.x, ps.pose.position.y = p[0], p[1]
            msg.poses.append(ps)
        self.pub_path.publish(msg)

    def get_min_dist(self, range_list):

        valid_values = [x for x in range_list if x > 0.05 and x < 10.0]
        if valid_values:
            return min(valid_values)
        return 99.9

    def stop_robot(self):
        self.pub_cmd.publish(Twist())

    def image_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        height, width, _ = frame.shape
        results = self.model(frame, verbose=False)
        self.lid_detected = False

        for result in results:
            for box in result.boxes:
                cls = int(box.cls[0])
                name = self.model.names[cls]

                if name == 'lid':
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    cx = (x1 + x2) // 2
                    cy = (y1 + y2) // 2

                    offset = (width // 2) - cx
                    cv2.circle(frame, (cx,cy), 10, (0, 255, 0), -1)
                    cv2.putText(frame, 'lid', (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
                    self.lid_detected = True

        cv2.imshow('lid_dector', frame)
        cv2.waitKey(50)

def main(args=None):
    rclpy.init(args=args)
    node = IntegratedNavigation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
