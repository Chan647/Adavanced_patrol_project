#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <queue>
#include <string>
#include <utility>
#include <vector>

using std::placeholders::_1;

struct Waypoint {
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

static inline double normAngle(double a) {
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

static inline double dist2(double x1, double y1, double x2, double y2) {
  double dx = x1 - x2;
  double dy = y1 - y2;
  return dx * dx + dy * dy;
}

class PatrolNavNode : public rclcpp::Node {
public:
  PatrolNavNode() : Node("patrol_nav_node") {
    declare_parameter<std::string>("yaml_path", "/home/chan/lch_ws/src/pt_pkg/config/waypoints.yaml");

    declare_parameter<std::string>("map_topic", "/map");
    declare_parameter<std::string>("amcl_topic", "/amcl_pose");
    declare_parameter<std::string>("scan_topic", "/scan");
    declare_parameter<std::string>("cmd_topic", "/cmd_vel");
    declare_parameter<std::string>("path_topic", "/planned_path");

    declare_parameter<double>("control_hz", 20.0);

    declare_parameter<double>("goal_tolerance", 0.35);
    declare_parameter<double>("yaw_tolerance", 0.15);

    declare_parameter<double>("pp_lookahead_min", 0.35);
    declare_parameter<double>("pp_lookahead_max", 1.10);
    declare_parameter<double>("pp_lookahead_speed_gain", 0.7);

    declare_parameter<double>("v_nominal", 0.25);
    declare_parameter<double>("v_min", 0.08);
    declare_parameter<double>("w_max", 1.2);

    declare_parameter<double>("rotate_in_place_w", 0.7);
    declare_parameter<double>("rotate_trigger_deg", 55.0);

    declare_parameter<double>("obs_front_dist", 0.55);
    declare_parameter<double>("obs_front_fov_deg", 40.0);
    declare_parameter<double>("obs_side_fov_deg", 70.0);
    declare_parameter<double>("avoid_weight", 0.85);
    declare_parameter<double>("avoid_w_gain", 1.0);
    declare_parameter<double>("avoid_v_scale", 0.55);
    declare_parameter<double>("max_path_deviation", 0.80);

    declare_parameter<double>("w_lpf_alpha", 0.25);
    declare_parameter<double>("w_deadband", 0.03);

    loadWaypoints();

    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        get_parameter("map_topic").as_string(), rclcpp::QoS(1).transient_local().reliable(),
        std::bind(&PatrolNavNode::onMap, this, _1));

    amcl_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        get_parameter("amcl_topic").as_string(), 10,
        std::bind(&PatrolNavNode::onPose, this, _1));

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        get_parameter("scan_topic").as_string(), rclcpp::SensorDataQoS(),
        std::bind(&PatrolNavNode::onScan, this, _1));

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(get_parameter("cmd_topic").as_string(), 10);
    path_pub_ = create_publisher<nav_msgs::msg::Path>(get_parameter("path_topic").as_string(), 10);

    double hz = get_parameter("control_hz").as_double();
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / hz), std::bind(&PatrolNavNode::onTimer, this));
  }

private:
  enum class Mode { WAIT, PLAN, ROTATE, TRACK, AVOID };

  void loadWaypoints() {
    std::string yaml_path = get_parameter("yaml_path").as_string();
    YAML::Node root = YAML::LoadFile(yaml_path);
    if (!root["waypoints"] || !root["waypoints"].IsSequence()) {
      throw std::runtime_error("waypoints.yaml: 'waypoints' sequence not found");
    }

    waypoints_.clear();
    for (const auto& n : root["waypoints"]) {
      Waypoint w;
      w.x = n["x"].as<double>();
      w.y = n["y"].as<double>();
      w.yaw = n["yaw"] ? n["yaw"].as<double>() : 0.0;
      waypoints_.push_back(w);
    }
    if (waypoints_.size() < 2) {
      throw std::runtime_error("Need at least 2 waypoints");
    }

    seq_.clear();
    int N = static_cast<int>(waypoints_.size());
    for (int i = 0; i < N; i++) seq_.push_back(i);
    for (int i = N - 2; i >= 1; i--) seq_.push_back(i);

    seq_index_ = 0;
    target_wp_index_ = seq_[seq_index_];

    mode_ = Mode::WAIT;
  }

  void onMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    map_ = *msg;
    have_map_ = true;
  }

  void onPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    pose_x_ = msg->pose.pose.position.x;
    pose_y_ = msg->pose.pose.position.y;

    double qx = msg->pose.pose.orientation.x;
    double qy = msg->pose.pose.orientation.y;
    double qz = msg->pose.pose.orientation.z;
    double qw = msg->pose.pose.orientation.w;

    double siny_cosp = 2.0 * (qw * qz + qx * qy);
    double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    pose_yaw_ = std::atan2(siny_cosp, cosy_cosp);

    have_pose_ = true;
  }

  void onScan(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    scan_ = *msg;
    have_scan_ = true;
  }

  bool worldToMap(double wx, double wy, int& mx, int& my) const {
    if (!have_map_) return false;
    const auto& info = map_.info;
    double ox = info.origin.position.x;
    double oy = info.origin.position.y;
    double r = info.resolution;
    mx = static_cast<int>(std::floor((wx - ox) / r));
    my = static_cast<int>(std::floor((wy - oy) / r));
    if (mx < 0 || my < 0) return false;
    if (mx >= static_cast<int>(info.width) || my >= static_cast<int>(info.height)) return false;
    return true;
  }

  void mapToWorld(int mx, int my, double& wx, double& wy) const {
    const auto& info = map_.info;
    double ox = info.origin.position.x;
    double oy = info.origin.position.y;
    double r = info.resolution;
    wx = ox + (mx + 0.5) * r;
    wy = oy + (my + 0.5) * r;
  }

  bool isFree(int mx, int my) const {
    const auto& info = map_.info;
    int W = static_cast<int>(info.width);
    int H = static_cast<int>(info.height);

    int inflation = 3;

    for (int dy = -inflation; dy <= inflation; dy++) {
      for (int dx = -inflation; dx <= inflation; dx++) {

        int nx = mx + dx;
        int ny = my + dy;

        if (nx < 0 || ny < 0 || nx >= W || ny >= H)
          return false;

        int idx = ny * W + nx;
        int8_t v = map_.data[idx];

        if (v < 0) return false;
        if (v >= 50) return false;
      }
    }

    return true;
  }

  bool findNearestFree(int& mx, int& my, int radius) const {
    if (isFree(mx, my)) return true;

    int best_x = mx, best_y = my;
    double best_d2 = std::numeric_limits<double>::infinity();
    bool found = false;

    for (int dy = -radius; dy <= radius; dy++) {
      for (int dx = -radius; dx <= radius; dx++) {
        int nx = mx + dx;
        int ny = my + dy;
        // 맵 범위 체크는 isFree가 해주지만, 여기서도 해도 됨
        if (!isFree(nx, ny)) continue;

        double d2 = dx * dx + dy * dy;
        if (d2 < best_d2) {
          best_d2 = d2;
          best_x = nx;
          best_y = ny;
          found = true;
        }
      }
    }

    if (!found) return false;
    mx = best_x; my = best_y;
    return true;
  }

  struct ANode {
    int x{0}, y{0};
    double g{0.0};
    double f{0.0};
  };
  struct AComp {
    bool operator()(const ANode& a, const ANode& b) const { return a.f > b.f; }
  };

  bool aStarPlan(double sx, double sy, double gx, double gy, nav_msgs::msg::Path& out) {
    int sxi, syi, gxi, gyi;
    if (!worldToMap(sx, sy, sxi, syi)) return false;
    if (!worldToMap(gx, gy, gxi, gyi)) return false;

    const int W = static_cast<int>(map_.info.width);
    const int H = static_cast<int>(map_.info.height);

    auto h = [&](int x, int y) {
      double dx = static_cast<double>(x - gxi);
      double dy = static_cast<double>(y - gyi);
      return std::hypot(dx, dy);
    };

    std::vector<double> gscore(W * H, std::numeric_limits<double>::infinity());
    std::vector<int> parent(W * H, -1);
    std::vector<uint8_t> closed(W * H, 0);

    auto idx = [&](int x, int y) { return y * W + x; };

    int snap_r = 8;
    if (!findNearestFree(sxi, syi, snap_r)) return false;
    if (!findNearestFree(gxi, gyi, snap_r)) return false;

    std::priority_queue<ANode, std::vector<ANode>, AComp> pq;
    int sidx = idx(sxi, syi);
    gscore[sidx] = 0.0;
    pq.push(ANode{sxi, syi, 0.0, h(sxi, syi)});

    const int dx8[8] = {1, -1, 0, 0, 1, 1, -1, -1};
    const int dy8[8] = {0, 0, 1, -1, 1, -1, 1, -1};

    bool found = false;

    while (!pq.empty()) {
      ANode cur = pq.top();
      pq.pop();

      int cidx = idx(cur.x, cur.y);
      if (closed[cidx]) continue;
      closed[cidx] = 1;

      if (cur.x == gxi && cur.y == gyi) {
        found = true;
        break;
      }

      for (int k = 0; k < 8; k++) {
        int nx = cur.x + dx8[k];
        int ny = cur.y + dy8[k];
        if (nx < 0 || ny < 0 || nx >= W || ny >= H) continue;
        if (!isFree(nx, ny)) continue;

        double step = (k < 4) ? 1.0 : std::sqrt(2.0);
        int nidx = idx(nx, ny);
        if (closed[nidx]) continue;

        double ng = gscore[cidx] + step;
        if (ng < gscore[nidx]) {
          gscore[nidx] = ng;
          parent[nidx] = cidx;
          double nf = ng + h(nx, ny);
          pq.push(ANode{nx, ny, ng, nf});
        }
      }
    }

    if (!found) return false;

    std::vector<std::pair<int, int>> cells;
    int cur = idx(gxi, gyi);
    while (cur != -1 && cur != sidx) {
      int cx = cur % W;
      int cy = cur / W;
      cells.push_back({cx, cy});
      cur = parent[cur];
    }
    cells.push_back({sxi, syi});
    std::reverse(cells.begin(), cells.end());

    out.header.frame_id = map_.header.frame_id.empty() ? "map" : map_.header.frame_id;
    out.header.stamp = now();
    out.poses.clear();
    out.poses.reserve(cells.size());

    for (auto& c : cells) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = out.header;
      mapToWorld(c.first, c.second, ps.pose.position.x, ps.pose.position.y);
      ps.pose.orientation.w = 1.0;
      out.poses.push_back(ps);
    }

    return true;
  }

  double computeLookahead(double v) const {
    double lmin = get_parameter("pp_lookahead_min").as_double();
    double lmax = get_parameter("pp_lookahead_max").as_double();
    double k = get_parameter("pp_lookahead_speed_gain").as_double();
    double L = lmin + k * std::fabs(v);
    return std::clamp(L, lmin, lmax);
  }

  int findClosestIndexWindowed(const nav_msgs::msg::Path& path, int last, double x, double y) const {
    int n = static_cast<int>(path.poses.size());
    if (n == 0) return 0;

    int win = 40;
    int a = std::max(0, last - win);
    int b = std::min(n - 1, last + win);

    int best = last;
    double bestd = std::numeric_limits<double>::infinity();
    for (int i = a; i <= b; i++) {
      double px = path.poses[i].pose.position.x;
      double py = path.poses[i].pose.position.y;
      double d = dist2(px, py, x, y);
      if (d < bestd) {
        bestd = d;
        best = i;
      }
    }
    return best;
  }

  bool purePursuitCmd(const nav_msgs::msg::Path& path, double& v_out, double& w_out) {
    if (path.poses.size() < 2) return false;

    if (path_index_ < 0) path_index_ = 0;
    path_index_ = findClosestIndexWindowed(path, path_index_, pose_x_, pose_y_);

    double v_nom = get_parameter("v_nominal").as_double();
    double L = computeLookahead(v_nom);

    int n = static_cast<int>(path.poses.size());
    int look = path_index_;
    double accum = 0.0;
    while (look + 1 < n) {
      double x1 = path.poses[look].pose.position.x;
      double y1 = path.poses[look].pose.position.y;
      double x2 = path.poses[look + 1].pose.position.x;
      double y2 = path.poses[look + 1].pose.position.y;
      accum += std::hypot(x2 - x1, y2 - y1);
      look++;
      if (accum >= L) break;
    }
    look = std::clamp(look, path_index_, n - 1);

    double tx = path.poses[look].pose.position.x;
    double ty = path.poses[look].pose.position.y;

    double dx = tx - pose_x_;
    double dy = ty - pose_y_;
    double yaw = pose_yaw_;

    double lx =  std::cos(-yaw) * dx - std::sin(-yaw) * dy;
    double ly =  std::sin(-yaw) * dx + std::cos(-yaw) * dy;

    if (lx < 1e-6) return false;

    double kappa = 2.0 * ly / (lx * lx + ly * ly);
    double w = v_nom * kappa;

    double wmax = get_parameter("w_max").as_double();
    w = std::clamp(w, -wmax, wmax);

    v_out = v_nom;
    w_out = w;
    return true;
  }

  struct ObsInfo {
    bool front_blocked{false};
    double left_clear{0.0};
    double right_clear{0.0};
  };

  ObsInfo analyzeScan() const {
    ObsInfo o;
    if (!have_scan_ || scan_.ranges.empty()) return o;

    double front_dist = get_parameter("obs_front_dist").as_double();
    double fov_front = get_parameter("obs_front_fov_deg").as_double() * M_PI / 180.0;
    double fov_side  = get_parameter("obs_side_fov_deg").as_double() * M_PI / 180.0;

    auto clampRange = [&](float r) -> double {
      if (!std::isfinite(r))
        return static_cast<double>(scan_.range_max);

      return std::clamp(
          static_cast<double>(r),
          static_cast<double>(scan_.range_min),
          static_cast<double>(scan_.range_max));
    };

    double front_min = scan_.range_max;
    double left_min = scan_.range_max;
    double right_min = scan_.range_max;

    for (size_t i = 0; i < scan_.ranges.size(); i++) {
      double ang = scan_.angle_min + static_cast<double>(i) * scan_.angle_increment;
      double r = clampRange(scan_.ranges[i]);
      double ang_norm = ang;
      if (ang_norm > M_PI)
          ang_norm -= 2.0 * M_PI;

      if (std::fabs(ang_norm) <= fov_front) {
          front_min = std::min(front_min, r);
      }

      if (ang > 0.0 && ang <= fov_side) {
        left_min = std::min(left_min, r);
      } 
      else if (ang >= (2.0 * M_PI - fov_side) && ang <= 2.0 * M_PI) {
        right_min = std::min(right_min, r);
      }
    }

    o.front_blocked = (front_min < front_dist);
    o.left_clear  = left_min;
    o.right_clear = right_min;
    return o;
  }

  double pathDeviation() const {
    if (global_path_.poses.empty()) return 0.0;
    int idx = std::clamp(path_index_, 0, static_cast<int>(global_path_.poses.size()) - 1);
    double px = global_path_.poses[idx].pose.position.x;
    double py = global_path_.poses[idx].pose.position.y;
    return std::sqrt(dist2(px, py, pose_x_, pose_y_));
  }

  void publishZero() {
    geometry_msgs::msg::Twist cmd;
    cmd_pub_->publish(cmd);
  }

  void publishCmd(double v, double w) {
    double alpha = get_parameter("w_lpf_alpha").as_double();
    double dead = get_parameter("w_deadband").as_double();
    if (std::fabs(w) < dead) w = 0.0;
    w_filt_ = alpha * w + (1.0 - alpha) * w_filt_;

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = v;
    cmd.angular.z = w_filt_;
    cmd_pub_->publish(cmd);
  }

  bool reachedWaypoint(const Waypoint& wp) const {
    double tol = get_parameter("goal_tolerance").as_double();
    return std::sqrt(dist2(pose_x_, pose_y_, wp.x, wp.y)) <= tol;
  }

  double desiredHeadingToNext(const Waypoint& next) const {
    return std::atan2(next.y - pose_y_, next.x - pose_x_);
  }

  void advanceWaypoint() {
    seq_index_ = (seq_index_ + 1) % static_cast<int>(seq_.size());
    target_wp_index_ = seq_[seq_index_];
  }

  void planToTarget() {
    const Waypoint& tgt = waypoints_[target_wp_index_];
    nav_msgs::msg::Path path;
    if (!aStarPlan(pose_x_, pose_y_, tgt.x, tgt.y, path)) {
      RCLCPP_WARN(get_logger(), "A* failed, keep previous path");
      mode_ = Mode::TRACK;
      return;
    }
    global_path_ = path;
    path_pub_->publish(global_path_);
    path_index_ = 0;
    mode_ = Mode::TRACK;
  }

  void onTimer() {
    if (!have_map_ || !have_pose_) {
      publishZero();
      return;
    }

    const Waypoint& tgt = waypoints_[target_wp_index_];

    if (mode_ == Mode::WAIT) mode_ = Mode::PLAN;

    if (mode_ == Mode::PLAN) {
      planToTarget();
      return;
    }

    if (reachedWaypoint(tgt)) {
      advanceWaypoint();
      const Waypoint& nxt = waypoints_[target_wp_index_];
      double heading = desiredHeadingToNext(nxt);
      double dyaw = normAngle(heading - pose_yaw_);
      double trig = get_parameter("rotate_trigger_deg").as_double() * M_PI / 180.0;
      if (std::fabs(dyaw) >= trig) {
        rotate_target_yaw_ = heading;
        mode_ = Mode::ROTATE;
      } else {
        mode_ = Mode::PLAN;
      }
      publishZero();
      return;
    }

    if (mode_ == Mode::ROTATE) {
      double dyaw = normAngle(rotate_target_yaw_ - pose_yaw_);
      double tol = get_parameter("yaw_tolerance").as_double();
      if (std::fabs(dyaw) <= tol) {
        publishZero();
        mode_ = Mode::PLAN;
        return;
      }
      double w = get_parameter("rotate_in_place_w").as_double();
      w = (dyaw > 0.0) ? std::fabs(w) : -std::fabs(w);
      publishCmd(0.0, w);
      return;
    }

    if (global_path_.poses.size() < 2) {
      mode_ = Mode::PLAN;
      publishZero();
      return;
    }

    ObsInfo obs = analyzeScan();
    double dev = pathDeviation();
    double max_dev = get_parameter("max_path_deviation").as_double();

    double v_pp = 0.0, w_pp = 0.0;
    bool ok_pp = purePursuitCmd(global_path_, v_pp, w_pp);
    if (!ok_pp) {
      mode_ = Mode::PLAN;
      publishZero();
      return;
    }

    double side_thresh = 0.32;
    if (obs.left_clear < side_thresh || obs.right_clear < side_thresh) {
      mode_ = Mode::AVOID;
    }

    if (!obs.front_blocked && dev <= max_dev) {
      mode_ = Mode::TRACK;
      publishCmd(v_pp, w_pp);
      return;
    }

    if (obs.front_blocked) {
      mode_ = Mode::AVOID;
    }

    if (dev > max_dev) {
      mode_ = Mode::TRACK;
    }

    mode_ = Mode::AVOID;

    bool go_left = (obs.left_clear >= obs.right_clear);
    double sign = go_left ? +1.0 : -1.0;

    double w_gain = get_parameter("avoid_w_gain").as_double();
    double w_avoid = sign * w_gain * (1.0 / std::max(0.25, std::min(obs.left_clear, obs.right_clear)));

    double wmax = get_parameter("w_max").as_double();
    w_avoid = std::clamp(w_avoid, -wmax, wmax);

    double avoid_weight = get_parameter("avoid_weight").as_double();
    double w = (1.0 - avoid_weight) * w_pp + avoid_weight * w_avoid;

    double v = v_pp * get_parameter("avoid_v_scale").as_double();
    double vmin = get_parameter("v_min").as_double();
    v = std::max(v, vmin);

    publishCmd(v, w);
  }

private:
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

  rclcpp::TimerBase::SharedPtr timer_;

  nav_msgs::msg::OccupancyGrid map_;
  sensor_msgs::msg::LaserScan scan_;
  nav_msgs::msg::Path global_path_;

  bool have_map_{false};
  bool have_pose_{false};
  bool have_scan_{false};

  double pose_x_{0.0};
  double pose_y_{0.0};
  double pose_yaw_{0.0};

  std::vector<Waypoint> waypoints_;
  std::vector<int> seq_;
  int seq_index_{0};
  int target_wp_index_{0};

  Mode mode_{Mode::WAIT};

  int path_index_{0};
  double rotate_target_yaw_{0.0};

  double w_filt_{0.0};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PatrolNavNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}