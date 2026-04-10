  #include <rclcpp/rclcpp.hpp>

  #include <geometry_msgs/msg/twist.hpp>
  #include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
  #include <geometry_msgs/msg/pose_stamped.hpp>
  #include <nav_msgs/msg/occupancy_grid.hpp>
  #include <nav_msgs/msg/path.hpp>
  #include <sensor_msgs/msg/laser_scan.hpp>

  #include <algorithm>
  #include <cmath>
  #include <cstdint>
  #include <fstream>
  #include <limits>
  #include <queue>
  #include <string>
  #include <utility>
  #include <vector>

  using std::placeholders::_1;
  using namespace std::chrono_literals;

  struct Waypoint {
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
  };

  static inline std::string trim(const std::string& s) {
    size_t b = s.find_first_not_of(" \t\r\n");
    if (b == std::string::npos) return "";
    size_t e = s.find_last_not_of(" \t\r\n");
    return s.substr(b, e - b + 1);
  }

  class PatrolNavigation : public rclcpp::Node {
  public:
    PatrolNavigation() : Node("patrol_navigation") {
      declare_parameter<std::string>("yaml_path", "/home/chan/lch_ws/src/pt_pkg/config/waypoints.yaml");

      // ===== Pure Pursuit / Tracking =====
      declare_parameter<double>("lookahead", 0.75);
      declare_parameter<double>("stop_tolerance", 0.30);
      declare_parameter<double>("skip_tolerance", 0.05);

      declare_parameter<double>("v_min", 0.20);
      declare_parameter<double>("v_max", 0.40);
      declare_parameter<double>("curv_slow_gain", 1.2);

      // 회전 너무 빠른 문제 방지
      declare_parameter<double>("max_ang", 1.3);
      declare_parameter<double>("max_ang_acc", 2.0);

      // ===== Obstacle avoid (soft) =====
      declare_parameter<double>("hard_stop_dist", 0.18);
      declare_parameter<double>("slow_down_dist", 0.70);
      declare_parameter<double>("avoid_ang_gain", 0.55);
      declare_parameter<double>("avoid_v_scale", 0.65);

      // ===== A* inflation =====
      declare_parameter<double>("robot_radius", 0.25);

      // scan sector (deg)
      declare_parameter<double>("scan_front_deg", 15.0);
      declare_parameter<double>("scan_front_left_deg1", 15.0);
      declare_parameter<double>("scan_front_left_deg2", 55.0);
      declare_parameter<double>("scan_front_right_deg1", -55.0);
      declare_parameter<double>("scan_front_right_deg2", -15.0);

      // load params
      get_parameter("lookahead", lookahead_);
      get_parameter("stop_tolerance", stop_tol_);
      get_parameter("skip_tolerance", skip_tol_);

      get_parameter("v_min", v_min_);
      get_parameter("v_max", v_max_);
      get_parameter("curv_slow_gain", curv_slow_gain_);

      get_parameter("max_ang", max_ang_);
      get_parameter("max_ang_acc", max_ang_acc_);

      get_parameter("hard_stop_dist", hard_stop_dist_);
      get_parameter("slow_down_dist", slow_down_dist_);
      get_parameter("avoid_ang_gain", avoid_ang_gain_);
      get_parameter("avoid_v_scale", avoid_v_scale_);

      get_parameter("robot_radius", robot_radius_);

      get_parameter("scan_front_deg", scan_front_deg_);
      get_parameter("scan_front_left_deg1", scan_fl1_deg_);
      get_parameter("scan_front_left_deg2", scan_fl2_deg_);
      get_parameter("scan_front_right_deg1", scan_fr1_deg_);
      get_parameter("scan_front_right_deg2", scan_fr2_deg_);

      cmd_pub_  = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
      path_pub_ = create_publisher<nav_msgs::msg::Path>("/planned_path", 10);

      map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", 10, std::bind(&PatrolNavigation::mapCallback, this, _1));

      pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/amcl_pose", 10, std::bind(&PatrolNavigation::poseCallback, this, _1));

      scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", rclcpp::SensorDataQoS(), std::bind(&PatrolNavigation::scanCallback, this, _1));

      control_timer_ = create_wall_timer(100ms, std::bind(&PatrolNavigation::controlLoop, this));
      wp_timer_      = create_wall_timer(500ms, std::bind(&PatrolNavigation::waypointLoop, this));

      std::string yaml_path;
      get_parameter("yaml_path", yaml_path);

      if (!loadWaypointsFromYaml(yaml_path)) {
        RCLCPP_ERROR(get_logger(), "Failed to load waypoints from: %s", yaml_path.c_str());
      } else {
        RCLCPP_INFO(get_logger(), "Loaded waypoints: %zu", waypoints_.size());
      }

      prev_cmd_time_ = now();
      RCLCPP_INFO(get_logger(), "PatrolNavigation started (A* + PurePursuit).");
    }

  private:
    // ================= callbacks =================
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
      map_width_  = static_cast<int>(msg->info.width);
      map_height_ = static_cast<int>(msg->info.height);
      resolution_ = msg->info.resolution;
      origin_x_   = msg->info.origin.position.x;
      origin_y_   = msg->info.origin.position.y;

      map_ = msg->data;
      map_ready_ = true;

      robot_radius_cells_ = std::max(1, static_cast<int>(robot_radius_ / resolution_));
    }

    void poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
      robot_x_ = msg->pose.pose.position.x;
      robot_y_ = msg->pose.pose.position.y;

      const auto& q = msg->pose.pose.orientation;
      robot_yaw_ = std::atan2(2.0 * (q.w * q.z), 1.0 - 2.0 * (q.z * q.z));

      pose_ready_ = true;
    }

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
      const int n = static_cast<int>(msg->ranges.size());
      if (n < 10) return;

      double angle = msg->angle_min;
      const double inc = msg->angle_increment;

      double f = 99.9, fl = 99.9, fr = 99.9;

      for (int i = 0; i < n; ++i) {
        const float rr = msg->ranges[i];
        if (!(rr > 0.05f && rr < 10.0f)) {
          angle += inc;
          continue;
        }

        const double deg = angle * 180.0 / M_PI;

        if (deg >= -scan_front_deg_ && deg <= scan_front_deg_) {
          f = std::min(f, (double)rr);
        } else if (deg > scan_fl1_deg_ && deg <= scan_fl2_deg_) {
          fl = std::min(fl, (double)rr);
        } else if (deg >= scan_fr1_deg_ && deg < scan_fr2_deg_) {
          fr = std::min(fr, (double)rr);
        }

        angle += inc;
      }

      front_dist_ = f;
      front_left_dist_ = fl;
      front_right_dist_ = fr;

      min_front_dist_ = std::min(front_dist_, std::min(front_left_dist_, front_right_dist_));
      scan_ready_ = true;
    }

    // ================= main loops =================
    void waypointLoop() {
      if (!map_ready_ || !pose_ready_) return;
      if (waypoints_.size() < 2) return;

      if (!global_path_.empty()) return;

      if (isCurrentGoalTriviallyReached()) {
        advanceWaypointIndex();
        return;
      }

      const bool ok = planToWaypoint();
      if (!ok) {
        RCLCPP_WARN(get_logger(), "A* failed for wp_index=%d, skipping.", wp_index_);
        advanceWaypointIndex();
      }
    }

    void controlLoop() {
      if (!map_ready_ || !pose_ready_) return;
      if (global_path_.empty()) return;

      // scan 없어도 출발은 하게 함
      followPathPurePursuit();
    }

    bool isCurrentGoalTriviallyReached() const {
      const auto& wp = waypoints_[wp_index_];
      const double d = std::hypot(wp.x - robot_x_, wp.y - robot_y_);
      return d < skip_tol_;
    }

    void advanceWaypointIndex() {
      const int N = static_cast<int>(waypoints_.size());
      if (N <= 1) return;

      int next = wp_index_ + dir_;
      if (next >= N) {
        dir_ = -1;
        next = N - 2;
      } else if (next < 0) {
        dir_ = +1;
        next = 1;
      }
      wp_index_ = next;
    }

    // ================= A* =================
    struct AStarCell {
      double g{std::numeric_limits<double>::infinity()};
      double f{std::numeric_limits<double>::infinity()};
      int py{-1}, px{-1};
      bool closed{false};
    };

    struct PQNode {
      int y, x;
      double f;
    };

    struct PQCompare {
      bool operator()(const PQNode& a, const PQNode& b) const {
        return a.f > b.f;
      }
    };

    bool planToWaypoint() {
      const auto start = worldToGrid(robot_x_, robot_y_);
      const auto goal  = worldToGrid(waypoints_[wp_index_].x, waypoints_[wp_index_].y);

      if (!inMap(start.first, start.second) || !inMap(goal.first, goal.second)) {
        RCLCPP_WARN(get_logger(), "Start/Goal out of map.");
        return false;
      }

      const auto path_grid = runAStar(start, goal);
      if (path_grid.empty()) return false;

      global_path_.clear();
      global_path_.reserve(path_grid.size());
      for (const auto& p : path_grid) {
        const auto w = gridToWorld(p.first, p.second);
        global_path_.push_back(w);
      }

      // 누적거리 s
      path_s_.clear();
      path_s_.reserve(global_path_.size());
      path_s_.push_back(0.0);
      for (size_t i = 1; i < global_path_.size(); ++i) {
        const double dx = global_path_[i].first - global_path_[i-1].first;
        const double dy = global_path_[i].second - global_path_[i-1].second;
        path_s_.push_back(path_s_.back() + std::hypot(dx, dy));
      }

      path_index_ = 0;
      progress_s_ = 0.0;

      publishPath();
      return true;
    }

    std::vector<std::pair<int,int>> runAStar(std::pair<int,int> start, std::pair<int,int> goal) {
      if (!isFreeWithInflation(start.first, start.second)) return {};
      if (!isFreeWithInflation(goal.first, goal.second)) return {};

      std::vector<AStarCell> cells(map_width_ * map_height_);
      auto idx = [&](int y, int x){ return y * map_width_ + x; };

      auto h = [&](int y, int x){
        return std::hypot(double(y - goal.first), double(x - goal.second));
      };

      std::priority_queue<PQNode, std::vector<PQNode>, PQCompare> open;

      cells[idx(start.first, start.second)].g = 0.0;
      cells[idx(start.first, start.second)].f = h(start.first, start.second);
      open.push(PQNode{start.first, start.second, cells[idx(start.first, start.second)].f});

      const std::vector<std::pair<int,int>> moves = {
        {0,1},{0,-1},{1,0},{-1,0},{1,1},{1,-1},{-1,1},{-1,-1}
      };

      while (!open.empty()) {
        const auto cur = open.top(); open.pop();

        auto& ccell = cells[idx(cur.y, cur.x)];
        if (ccell.closed) continue;
        ccell.closed = true;

        if (cur.y == goal.first && cur.x == goal.second) {
          std::vector<std::pair<int,int>> path;
          int y = goal.first, x = goal.second;
          path.push_back({y,x});
          while (!(y == start.first && x == start.second)) {
            auto& cell = cells[idx(y,x)];
            const int py = cell.py, px = cell.px;
            if (py < 0 || px < 0) return {};
            y = py; x = px;
            path.push_back({y,x});
          }
          std::reverse(path.begin(), path.end());
          return path;
        }

        for (const auto& mv : moves) {
          const int ny = cur.y + mv.first;
          const int nx = cur.x + mv.second;
          if (!inMap(ny, nx)) continue;
          if (!isFreeWithInflation(ny, nx)) continue;

          const double step = (mv.first != 0 && mv.second != 0) ? std::sqrt(2.0) : 1.0;
          const double ng = ccell.g + step;

          auto& ncell = cells[idx(ny,nx)];
          if (ncell.closed) continue;

          if (ng < ncell.g) {
            ncell.g = ng;
            ncell.f = ng + h(ny,nx);
            ncell.py = cur.y;
            ncell.px = cur.x;
            open.push(PQNode{ny, nx, ncell.f});
          }
        }
      }

      return {};
    }

    bool inMap(int y, int x) const {
      return (0 <= y && y < map_height_ && 0 <= x && x < map_width_);
    }

    bool isOccupiedRaw(int y, int x) const {
      const int i = y * map_width_ + x;
      const int8_t v = map_[i];
      if (v < 0) return false;
      if (v >= 50) return true;
      return false;
    }

    bool isFreeWithInflation(int y, int x) const {
      for (int dy = -robot_radius_cells_; dy <= robot_radius_cells_; ++dy) {
        for (int dx = -robot_radius_cells_; dx <= robot_radius_cells_; ++dx) {
          const int cy = y + dy;
          const int cx = x + dx;
          if (!inMap(cy,cx)) continue;
          if (isOccupiedRaw(cy,cx)) return false;
        }
      }
      return true;
    }

    static inline double normAngle(double a) {
      while (a > M_PI) a -= 2.0 * M_PI;
      while (a < -M_PI) a += 2.0 * M_PI;
      return a;
    } 

    // ================= Pure Pursuit =================
    void followPathPurePursuit() {
      if (global_path_.size() < 2) return;

      const auto& final_goal = global_path_.back();
      const double dist_to_final = std::hypot(final_goal.first - robot_x_, final_goal.second - robot_y_);

      if (dist_to_final < stop_tol_) {
        global_path_.clear();
        path_s_.clear();
        path_index_ = 0;
        progress_s_ = 0.0;
        stopRobot();
        advanceWaypointIndex();
        return;
      }

      // 1) progress_s_ 갱신 (뒤로는 절대 안 감) + path_index_ 갱신
      const double s_proj = projectToPathS(progress_s_);
      progress_s_ = std::max(progress_s_, s_proj);
      progress_s_ = std::min(progress_s_, path_s_.back());

      // 2) 목표 s
      const double L = lookahead_;
      const double s_target = std::min(progress_s_ + L, path_s_.back());

      // 3) 목표점 보간
      double tx = final_goal.first, ty = final_goal.second;
      sampleByS(s_target, tx, ty);

      // 4) 목표점을 로봇 좌표계로 변환
      const double dx = tx - robot_x_;
      const double dy = ty - robot_y_;
      const double ang_to_target = std::atan2(dy, dx);
      const double yaw_err = normAngle(ang_to_target - robot_yaw_);
      const double c = std::cos(robot_yaw_);
      const double s = std::sin(robot_yaw_);
      const double x_r =  c*dx + s*dy;
      const double y_r = -s*dx + c*dy;

      if (std::fabs(yaw_err) > M_PI / 2.0) {
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = 0.0;  // 무조건 정지

        cmd.angular.z = std::clamp(1.5 * yaw_err,
                                   -max_ang_, +max_ang_);

        cmd_pub_->publish(cmd);

        prev_cmd_ang_ = cmd.angular.z;
        prev_cmd_time_ = now();
        return;   // 🔥 여기서 종료
      }

      // 5) 곡률
      double curvature = 0.0;
      if (L > 1e-3) curvature = 2.0 * y_r / (L * L);

      // 6) 속도(직진은 빠르고, 곡률은 감속)
      double v = v_max_ / (1.0 + curv_slow_gain_ * std::fabs(curvature));
      v = std::clamp(v, v_min_, v_max_);

      // 7) 각속도
      double w_des = v * curvature;

      // 8) 회피(soft): 실제로 조향이 들어가게
      if (scan_ready_) {
        if (min_front_dist_ < hard_stop_dist_) {
          geometry_msgs::msg::Twist cmd;
          cmd.linear.x = 0.0;
          cmd.angular.z = (front_left_dist_ > front_right_dist_) ? +0.55 : -0.55;
          cmd_pub_->publish(cmd);
          return;
        }

        if (min_front_dist_ < slow_down_dist_) {
          const double strength = std::clamp(
            (slow_down_dist_ - min_front_dist_) / (slow_down_dist_ - hard_stop_dist_), 0.0, 1.0);

          // ROS에서 +w = 좌회전. 왼쪽이 더 가깝다면 오른쪽(-)으로 피해라.
          const double side_bias = std::clamp((front_left_dist_ - front_right_dist_) / 1.0, -1.0, 1.0);

          w_des += avoid_ang_gain_ * strength * side_bias;
          v *= (1.0 - (1.0 - avoid_v_scale_) * strength);
        }
      }

      // 9) 각속도 제한 + 각가속도 제한
      w_des = std::clamp(w_des, -max_ang_, max_ang_);

      const auto t_now = now();
      const double dt = (t_now - prev_cmd_time_).seconds();
      double w_cmd = w_des;

      if (dt > 1e-4) {
        const double max_dw = max_ang_acc_ * dt;
        const double dw = std::clamp(w_des - prev_cmd_ang_, -max_dw, +max_dw);
        w_cmd = prev_cmd_ang_ + dw;
      }

      prev_cmd_ang_ = w_cmd;
      prev_cmd_time_ = t_now;

      geometry_msgs::msg::Twist cmd;
      cmd.linear.x = v;
      cmd.angular.z = w_cmd;
      cmd_pub_->publish(cmd);
    }

    // ---- 경로 투영: path_index_부터 앞으로만 검색(중요)
    double projectToPathS(double s_from) {
      if (global_path_.size() < 2 || path_s_.size() < 2) return s_from;

      // s_from 기준으로 최소 인덱스 보정(너무 뒤 인덱스 방지)
      size_t idx_from = 0;
      while (idx_from + 1 < path_s_.size() && path_s_[idx_from + 1] < s_from) {
        ++idx_from;
      }

      // 실제 검색 시작 인덱스는 "현재 path_index_"와 "s_from 인덱스" 중 큰 값
      size_t start_i = std::max(path_index_, idx_from);
      if (start_i >= global_path_.size() - 1) start_i = global_path_.size() - 2;

      size_t end_i = std::min(global_path_.size() - 2, start_i + (size_t)40);

      double best_s = s_from;
      double best_d2 = std::numeric_limits<double>::infinity();

      for (size_t i = start_i; i <= end_i; ++i) {
        const double x1 = global_path_[i].first;
        const double y1 = global_path_[i].second;
        const double x2 = global_path_[i+1].first;
        const double y2 = global_path_[i+1].second;

        const double vx = x2 - x1;
        const double vy = y2 - y1;
        const double wx = robot_x_ - x1;
        const double wy = robot_y_ - y1;

        const double vv = vx*vx + vy*vy;
        if (vv < 1e-9) continue;

        double t = (wx*vx + wy*vy) / vv;
        t = std::clamp(t, 0.0, 1.0);

        const double px = x1 + t*vx;
        const double py = y1 + t*vy;

        const double dx = robot_x_ - px;
        const double dy = robot_y_ - py;
        const double d2 = dx*dx + dy*dy;

        if (d2 < best_d2) {
          best_d2 = d2;
          best_s = path_s_[i] + t * (path_s_[i+1] - path_s_[i]);
        }
      }

      // 🔥 여기서 path_index_ 갱신(이게 빠져서 네가 계속 틀어졌던 거)
      path_index_ = std::lower_bound(path_s_.begin(), path_s_.end(), best_s) - path_s_.begin();
      if (path_index_ >= global_path_.size() - 1) path_index_ = global_path_.size() - 2;

      return best_s;
    }

    void sampleByS(double s_target, double& out_x, double& out_y) const {
      if (path_s_.empty() || global_path_.empty()) return;

      if (s_target <= 0.0) {
        out_x = global_path_.front().first;
        out_y = global_path_.front().second;
        return;
      }
      if (s_target >= path_s_.back()) {
        out_x = global_path_.back().first;
        out_y = global_path_.back().second;
        return;
      }

      auto it = std::lower_bound(path_s_.begin(), path_s_.end(), s_target);
      size_t i = std::distance(path_s_.begin(), it);
      if (i == 0) i = 1;
      if (i >= path_s_.size()) i = path_s_.size() - 1;

      const double s0 = path_s_[i-1];
      const double s1 = path_s_[i];
      const double t = (s_target - s0) / std::max(1e-9, (s1 - s0));

      const double x0 = global_path_[i-1].first;
      const double y0 = global_path_[i-1].second;
      const double x1 = global_path_[i].first;
      const double y1 = global_path_[i].second;

      out_x = x0 + t * (x1 - x0);
      out_y = y0 + t * (y1 - y0);
    }

    // ================= utils =================
    std::pair<int,int> worldToGrid(double x, double y) const {
      const int gx = static_cast<int>((x - origin_x_) / resolution_);
      const int gy = static_cast<int>((y - origin_y_) / resolution_);
      return {gy, gx};
    }

    std::pair<double,double> gridToWorld(int y, int x) const {
      const double wx = x * resolution_ + origin_x_;
      const double wy = y * resolution_ + origin_y_;
      return {wx, wy};
    }

    void publishPath() {
      nav_msgs::msg::Path msg;
      msg.header.frame_id = "map";
      msg.header.stamp = now();

      msg.poses.reserve(global_path_.size());
      for (const auto& p : global_path_) {
        geometry_msgs::msg::PoseStamped ps;
        ps.header.frame_id = "map";
        ps.header.stamp = msg.header.stamp;
        ps.pose.position.x = p.first;
        ps.pose.position.y = p.second;
        ps.pose.orientation.w = 1.0;
        msg.poses.push_back(ps);
      }
      path_pub_->publish(msg);
    }

    void stopRobot() {
      cmd_pub_->publish(geometry_msgs::msg::Twist());
    }

    // ================= YAML =================
    bool loadWaypointsFromYaml(const std::string& path) {
      std::ifstream fin(path);
      if (!fin.is_open()) return false;

      std::vector<Waypoint> wps;
      Waypoint cur;
      bool in_item = false;

      std::string line;
      while (std::getline(fin, line)) {
        line = trim(line);
        if (line.empty()) continue;
        if (line.rfind("#", 0) == 0) continue;

        if (line.rfind("-", 0) == 0) {
          if (in_item) wps.push_back(cur);
          cur = Waypoint{};
          in_item = true;

          auto pos = line.find("x:");
          if (pos != std::string::npos) {
            cur.x = std::stod(trim(line.substr(pos + 2)));
          }
          continue;
        }

        if (!in_item) continue;

        if (line.rfind("x:", 0) == 0) {
          cur.x = std::stod(trim(line.substr(2)));
        } else if (line.rfind("y:", 0) == 0) {
          cur.y = std::stod(trim(line.substr(2)));
        } else if (line.rfind("yaw:", 0) == 0) {
          cur.yaw = std::stod(trim(line.substr(4)));
        }
      }

      if (in_item) wps.push_back(cur);

      if (wps.size() < 2) {
        RCLCPP_ERROR(get_logger(), "Need at least 2 waypoints in YAML.");
        return false;
      }

      waypoints_ = std::move(wps);
      wp_index_ = 0;
      dir_ = +1;
      return true;
    }

  private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::TimerBase::SharedPtr wp_timer_;

    // map
    std::vector<int8_t> map_;
    int map_width_{0}, map_height_{0};
    double resolution_{0.05};
    double origin_x_{0.0}, origin_y_{0.0};
    bool map_ready_{false};

    // pose
    double robot_x_{0.0}, robot_y_{0.0}, robot_yaw_{0.0};
    bool pose_ready_{false};

    // scan
    bool scan_ready_{false};
    double front_dist_{99.9};
    double front_left_dist_{99.9};
    double front_right_dist_{99.9};
    double min_front_dist_{99.9};

    // scan params
    double scan_front_deg_{15.0};
    double scan_fl1_deg_{15.0}, scan_fl2_deg_{55.0};
    double scan_fr1_deg_{-55.0}, scan_fr2_deg_{-15.0};

    // params
    double lookahead_{0.75};
    double stop_tol_{0.30};
    double skip_tol_{0.05};

    double v_min_{0.05};
    double v_max_{0.18};
    double curv_slow_gain_{2.2};

    double max_ang_{0.9};
    double max_ang_acc_{1.6};

    double hard_stop_dist_{0.18};
    double slow_down_dist_{0.70};
    double avoid_ang_gain_{0.55};
    double avoid_v_scale_{0.65};

    double robot_radius_{0.40};
    int robot_radius_cells_{3};

    // yaw rate limiter
    double prev_cmd_ang_{0.0};
    rclcpp::Time prev_cmd_time_;

    // patrol
    std::vector<Waypoint> waypoints_;
    int wp_index_{0};
    int dir_{+1};

    // path
    std::vector<std::pair<double,double>> global_path_;
    std::vector<double> path_s_;
    size_t path_index_{0};
    double progress_s_{0.0};
  };

  int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PatrolNavigation>());
    rclcpp::shutdown();
    return 0;
  }