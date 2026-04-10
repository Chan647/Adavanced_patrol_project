import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Path
from math import atan2, sqrt, sin, pi, inf
import heapq
import numpy as np

class IntegratedNavigation(Node):
    def __init__(self):
        super().__init__('integrated_navigation')
        self.lookahead_dist = 0.5
        self.linear_vel = 0.2
        self.stop_tolerance = 0.15
        self.safety_margin = 4
        self.map_data = None
        self.prev_map_data = None
        self.map_resolution = 0.05
        self.map_origin = [0.0, 0.0]
        self.map_width = 0
        self.map_height = 0
        self.current_pose = None
        self.current_yaw = 0.0
        self.goal_grid = None
        self.g = {}
        self.rhs = {}
        self.U = []
        self.km = 0.0
        self.last_start = None
        self.global_path = []
        self.path_index = 0
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_path = self.create_publisher(Path, '/planned_path', 10)
        self.sub_map = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.sub_pose = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10)
        self.sub_goal = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Let's Run!")

    def heuristic(self, a, b):
        return sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

    def calculate_key(self, s, start_grid):
        min_grhs = min(self.g.get(s, inf), self.rhs.get(s, inf))
        return (min_grhs + self.heuristic(start_grid, s) + self.km, min_grhs)

    def get_neighbors(self, pos):
        moves = [(0,1,1), (0,-1,1), (1,0,1), (-1,0,1), (1,1,sqrt(2)), (1,-1,sqrt(2)), (-1,1,sqrt(2)), (-1,-1,sqrt(2))]
        neighbors = []
        y, x = pos
        for dy, dx, cost in moves:
            ny, nx = y + dy, x + dx
            if 0 <= ny < self.map_height and 0 <= nx < self.map_width:
                if self.map_data[ny, nx] == 0 and self.check_safety(ny, nx):
                    neighbors.append(((ny, nx), cost))
        return neighbors

    def check_safety(self, y, x):
        margin = self.safety_margin
        for r in range(y - margin, y + margin + 1):
            for c in range(x - margin, x + margin + 1):
                if 0 <= r < self.map_height and 0 <= c < self.map_width:
                    if self.map_data[r, c] != 0:
                        return False
        return True

    def update_vertex(self, u, start_grid):
        if u != self.goal_grid:
            self.rhs[u] = min((cost + self.g.get(succ, inf) for succ, cost in self.get_neighbors(u)), default=inf)
        self.U = [item for item in self.U if item[1] != u]
        if self.g.get(u, inf) != self.rhs.get(u, inf):
            heapq.heappush(self.U, (self.calculate_key(u, start_grid), u))

    def compute_shortest_path(self, start_grid):
        while self.U and (self.U[0][0] < self.calculate_key(start_grid, start_grid) or self.rhs.get(start_grid, inf) != self.g.get(start_grid, inf)):
            k_old, u = heapq.heappop(self.U)
            if k_old > self.calculate_key(u, start_grid):
                heapq.heappush(self.U, (self.calculate_key(u, start_grid), u))
            elif self.g.get(u, inf) > self.rhs.get(u, inf):
                self.g[u] = self.rhs[u]
                for pred, _ in self.get_neighbors(u):
                    self.update_vertex(pred, start_grid)
            else:
                self.g[u] = inf
                for pred, _ in self.get_neighbors(u) + [(u, 0)]:
                    self.update_vertex(pred, start_grid)

    def extract_path(self, start_grid):
        path = []
        current = start_grid
        while current != self.goal_grid:
            path.append(current)
            successors = self.get_neighbors(current)
            if not successors:
                return None
            current = min(successors, key=lambda s: s[1] + self.g.get(s[0], inf))[0]
        path.append(self.goal_grid)
        return path

    def map_callback(self, msg):
        self.map_resolution = msg.info.resolution
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_origin = [msg.info.origin.position.x, msg.info.origin.position.y]
        new_map = np.array(msg.data).reshape((self.map_height, self.map_width))
        if self.map_data is None:
            self.map_data = new_map
            return
        if self.goal_grid and self.current_pose:
            start_grid = self.world_to_grid(self.current_pose)
            changed = np.where(new_map != self.map_data)
            for y, x in zip(changed[0], changed[1]):
                pos = (y, x)
                self.update_vertex(pos, start_grid)
            self.km += self.heuristic(self.last_start or start_grid, start_grid)
            self.last_start = start_grid
            self.compute_shortest_path(start_grid)
            path_grid = self.extract_path(start_grid)
            if path_grid:
                self.global_path = [self.grid_to_world(p) for p in path_grid]
                self.publish_path_viz()
        self.prev_map_data = self.map_data.copy()
        self.map_data = new_map

    def pose_callback(self, msg):
        old_pose = self.current_pose
        self.current_pose = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        q = msg.pose.pose.orientation
        self.current_yaw = atan2(2.0*(q.w*q.z + q.x*q.y), 1.0-2.0*(q.y*q.y + q.z*q.z))
        if self.goal_grid and old_pose:
            old_start = self.world_to_grid(old_pose)
            new_start = self.world_to_grid(self.current_pose)
            if old_start != new_start:
                self.km += self.heuristic(old_start, new_start)
                self.last_start = new_start
                self.compute_shortest_path(new_start)
                path_grid = self.extract_path(new_start)
                if path_grid:
                    self.global_path = [self.grid_to_world(p) for p in path_grid]
                    self.publish_path_viz()

    def goal_callback(self, msg):
        if self.map_data is None or self.current_pose is None:
            return
        goal_pose = [msg.pose.position.x, msg.pose.position.y]
        self.goal_grid = self.world_to_grid(goal_pose)
        start_grid = self.world_to_grid(self.current_pose)
        self.g = {}
        self.rhs = {self.goal_grid: 0.0}
        self.U = []
        self.km = 0.0
        self.last_start = start_grid
        heapq.heappush(self.U, (self.calculate_key(self.goal_grid, start_grid), self.goal_grid))
        self.compute_shortest_path(start_grid)
        path_grid = self.extract_path(start_grid)
        if path_grid:
            self.global_path = [self.grid_to_world(p) for p in path_grid]
            self.path_index = 0
            self.publish_path_viz()
            self.get_logger().info("Path Found! Go!")
        else:
            self.get_logger().warn("No Path Found.")

    def control_loop(self):
        if not self.global_path or not self.current_pose:
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
        angular_velocity = self


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


if __name__ == '__main__':
    main()
