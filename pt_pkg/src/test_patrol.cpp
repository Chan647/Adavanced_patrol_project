#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>
#include <turtlebot3_msgs/srv/sound.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>

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
  std::string label{""};
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

static inline double normAngle(double a) {
  while (a >  M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

static inline double dist2(double x1, double y1, double x2, double y2) {
  double dx = x1 - x2, dy = y1 - y2;
  return dx*dx + dy*dy;
}

class PatrolNavNode : public rclcpp::Node {
public:
  PatrolNavNode() : Node("patrol_nav_node") {
    declare_parameter<std::string>("yaml_path", "/home/cho/lch_ws/src/pt_pkg/config/waypoints.yaml");
    declare_parameter<std::string>("map_topic", "/map");
    declare_parameter<std::string>("amcl_topic", "/amcl_pose");
    declare_parameter<std::string>("scan_topic", "/scan");
    declare_parameter<std::string>("cmd_topic", "/cmd_vel");
    declare_parameter<std::string>("path_topic", "/planned_path");
    declare_parameter<double>("control_hz", 20.0);
    declare_parameter<int>("inflation_cells", 3);
    declare_parameter<double>("map_robot_radius", 0.32);
    declare_parameter<double>("goal_tolerance", 0.5);
    declare_parameter<double>("yaw_tolerance", 0.10);
    declare_parameter<double>("pp_lookahead_min", 0.35);
    declare_parameter<double>("pp_lookahead_max", 1.10);
    declare_parameter<double>("pp_lookahead_speed_gain", 0.7);
    declare_parameter<double>("v_nominal", 0.25);
    declare_parameter<double>("v_min", 0.08);
    declare_parameter<double>("w_max", 1.2);
    declare_parameter<double>("rotate_in_place_w", 0.6);
    declare_parameter<double>("rotate_trigger_deg", 55.0);
    declare_parameter<double>("obs_front_dist", 0.75);
    declare_parameter<double>("obs_front_fov_deg", 30.0);
    declare_parameter<double>("obs_side_fov_deg", 70.0);
    declare_parameter<double>("avoid_weight", 0.85);
    declare_parameter<double>("avoid_w_gain", 1.0);
    declare_parameter<double>("avoid_v_scale", 0.55);
    declare_parameter<double>("max_path_deviation", 1.50);
    declare_parameter<double>("w_lpf_alpha", 0.15);
    declare_parameter<double>("w_deadband", 0.03);

    loadWaypoints();

    double hz = get_parameter("control_hz").as_double();
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0/hz),
                               std::bind(&PatrolNavNode::onTimer, this));

    cmd_pub_       = create_publisher<geometry_msgs::msg::Twist>(get_parameter("cmd_topic").as_string(), 10);
    path_pub_      = create_publisher<nav_msgs::msg::Path>(get_parameter("path_topic").as_string(), 10);
    stopped_pub_   = create_publisher<std_msgs::msg::Bool>("/robot_stopped", 10);
    follow_pub_    = create_publisher<std_msgs::msg::Bool>("/robot_following", 10);
    init_pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);
    led_pub_ = create_publisher<std_msgs::msg::Int32>("/led_mode", 10);

    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        get_parameter("map_topic").as_string(), rclcpp::QoS(1).transient_local().reliable(),
        std::bind(&PatrolNavNode::onMap, this, _1));

    amcl_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        get_parameter("amcl_topic").as_string(), 10,
        std::bind(&PatrolNavNode::onPose, this, _1));

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        get_parameter("scan_topic").as_string(), rclcpp::SensorDataQoS(),
        std::bind(&PatrolNavNode::onScan, this, _1));

    person_sub_ = create_subscription<std_msgs::msg::Bool>("/person_detected", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg){ person_detected_ = msg->data; });

    bbox_sub_ = create_subscription<geometry_msgs::msg::Vector3>("/person_bbox", rclcpp::QoS(1).best_effort(),
        [this](const geometry_msgs::msg::Vector3::SharedPtr msg){
          person_bbox_x_ = msg->x; person_bbox_z_ = msg->z; });

    face_sub_ = create_subscription<std_msgs::msg::String>("/face_result", 10,
        [this](const std_msgs::msg::String::SharedPtr msg){
          recognized_name_ = msg->data;
          if (msg->data.find("침입자") != std::string::npos) {
            if (!buzzer_on_) { buzzer_on_ = true; playSound(2); RCLCPP_WARN(get_logger(), "침입자 감지 → 부저 ON"); }
          } else {
            face_recognized_ = true;
            RCLCPP_INFO(get_logger(), "얼굴 인식 결과: %s", msg->data.c_str());
          }
        });

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        "/imu", rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::Imu::SharedPtr msg){
          double w = std::abs(msg->orientation.w);
          theft_detected_ = (w > 1.00);
        });

    nav_cmd_sub_ = create_subscription<std_msgs::msg::String>(
        "/cpp_nav_command", 10,
        [this](const std_msgs::msg::String::SharedPtr msg)
        {

          std::string cmd = msg->data;

          // 1. 자율 주행 시작 (1회 / 무한반복)
          if (cmd == "once" || cmd == "loop") 
          {
            
            std::vector<Waypoint> old_wp = waypoints_;
            
            loadWaypoints();
            
            // 기존 경로에서 이어갈지(Resume), 처음부터 출발할지 판단하는 로직
            bool path_changed = false;
            if (old_wp.size() != waypoints_.size()) 
            {
                path_changed = true; 
            } 
            else if (!old_wp.empty() && !waypoints_.empty()) 
            {
                if (std::abs(old_wp.back().x - waypoints_.back().x) > 0.05 ||
                    std::abs(old_wp.back().y - waypoints_.back().y) > 0.05) 
                {
                    path_changed = true;
                }
            }

            if (path_changed) 
            {
                seq_index_ = 0;
                RCLCPP_INFO(get_logger(), "🚀 새로운 경로로 출발");
            } 
            else 
            {
                RCLCPP_INFO(get_logger(), "▶️ 기존 경로 이어서 주행 (Resume)");
            }

            if (cmd == "once") 
            {
              nav_mode_ = NavMode::ONCE;
            } 
            else 
            {
              nav_mode_ = NavMode::LOOP;
            }
            
            nav_once_done_ = false;
            target_wp_index_ = seq_[seq_index_];
            mode_ = Mode::PLAN;
          } 


          // 2. 정지 명령
          else if (cmd == "stop") 
          {
            nav_mode_ = NavMode::IDLE;
            publishZero();
            RCLCPP_INFO(get_logger(), "🛑 정지 명령 수신");
          } 

          else if (cmd == "charging") 
          {
            RCLCPP_INFO(get_logger(), "🔋 충전 모드 명령 수신 → 시작 위치로 이동");
            nav_mode_ = NavMode::CHARGE;
            target_wp_index_ = seq_[0]; // 시작 웨이포인트(0번)
            mode_ = Mode::PLAN;
            publishZero();
          }

          else if (cmd.find("goto:") == 0) 
          {
            std::string target_label = cmd.substr(5); // "goto:" 떼어내고 라벨만 추출
            
            loadWaypoints(); // 웹에서 새로 추가했을 수 있으니 최신 YAML 파일 다시 읽기

            int found_idx = -1;
            for (size_t i = 0; i < waypoints_.size(); i++) {
              if (waypoints_[i].label == target_label) {
                found_idx = i;
                break;
              }
            }

            if (found_idx != -1) 
            {
              target_wp_index_ = found_idx;
              nav_mode_ = NavMode::GOTO; // GOTO 모드로 전환
              mode_ = Mode::PLAN;        // A* 경로 계획 바로 시작
              RCLCPP_INFO(get_logger(), "🎯 단일 목적지 이동 명령 수신: [%s] (인덱스: %d)", target_label.c_str(), found_idx);
            } 
            else 
            {
              RCLCPP_WARN(get_logger(), "❌ 알 수 없는 목적지 라벨입니다: %s", target_label.c_str());
            }
          }
        });
    pan_angle_sub_ = create_subscription<std_msgs::msg::Float32>("/pan_angle", 10,
        [this](const std_msgs::msg::Float32::SharedPtr msg){ pan_angle_ = msg->data; });
}

private:

  enum class Mode { WAIT, PLAN, ROTATE, TRACK, AVOID, FOLLOW, SEARCH, SEARCH_BACKUP };

  void loadWaypoints() {
    YAML::Node root = YAML::LoadFile(get_parameter("yaml_path").as_string());
    if (!root["waypoints"] || !root["waypoints"].IsSequence())
      throw std::runtime_error("waypoints.yaml: 'waypoints' sequence not found");
    waypoints_.clear();
    for (const auto& n : root["waypoints"]) {
      Waypoint w;
      w.label = n["label"] ? n["label"].as<std::string>() : "";
      w.x = n["x"].as<double>(); w.y = n["y"].as<double>();
      w.yaw = n["yaw"] ? n["yaw"].as<double>() : 0.0;
      waypoints_.push_back(w);
    }
    if (waypoints_.size() < 2) throw std::runtime_error("Need at least 2 waypoints");
    seq_.clear();
    int N = static_cast<int>(waypoints_.size());
    for (int i = 0; i < N; i++) seq_.push_back(i);
    for (int i = N-2; i >= 1; i--) seq_.push_back(i);
    seq_index_ = 0; target_wp_index_ = seq_[0]; mode_ = Mode::WAIT;
    std::string seq_str = "seq: ";
    for (int i : seq_) seq_str += std::to_string(i) + " ";
    RCLCPP_INFO(get_logger(), "%s", seq_str.c_str());
  }

  void onMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) { map_ = *msg; have_map_ = true; }

  void onPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    pose_x_ = msg->pose.pose.position.x; pose_y_ = msg->pose.pose.position.y;
    double qx=msg->pose.pose.orientation.x, qy=msg->pose.pose.orientation.y;
    double qz=msg->pose.pose.orientation.z, qw=msg->pose.pose.orientation.w;
    pose_yaw_ = std::atan2(2.0*(qw*qz+qx*qy), 1.0-2.0*(qy*qy+qz*qz));
    if (!have_pose_) {
      geometry_msgs::msg::PoseWithCovarianceStamped init_pose;
      init_pose.header.frame_id = "map";
      init_pose.header.stamp = now();
      init_pose.pose.pose.position.x = pose_x_;
      init_pose.pose.pose.position.y = pose_y_;
      init_pose.pose.pose.orientation.z = std::sin(pose_yaw_ / 2.0);
      init_pose.pose.pose.orientation.w = std::cos(pose_yaw_ / 2.0);
      init_pose.pose.covariance[0]  = 0.25;
      init_pose.pose.covariance[7]  = 0.25;
      init_pose.pose.covariance[35] = 0.07;
      init_pose_pub_->publish(init_pose);
      RCLCPP_INFO(get_logger(), "초기 위치 자동 설정: x=%.2f y=%.2f", pose_x_, pose_y_);
    }
    have_pose_ = true;
  }

  void onScan(const sensor_msgs::msg::LaserScan::SharedPtr msg) { scan_ = *msg; have_scan_ = true; }

  bool worldToMap(double wx, double wy, int& mx, int& my) const {
    if (!have_map_) return false;
    const auto& info = map_.info;
    mx = static_cast<int>(std::floor((wx - info.origin.position.x) / info.resolution));
    my = static_cast<int>(std::floor((wy - info.origin.position.y) / info.resolution));
    if (mx<0||my<0||mx>=static_cast<int>(info.width)||my>=static_cast<int>(info.height)) return false;
    return true;
  }

  void mapToWorld(int mx, int my, double& wx, double& wy) const {
    wx = map_.info.origin.position.x + (mx+0.5)*map_.info.resolution;
    wy = map_.info.origin.position.y + (my+0.5)*map_.info.resolution;
  }

  bool isFree(int mx, int my, int inflation) const {
    int W=static_cast<int>(map_.info.width), H=static_cast<int>(map_.info.height);
    for (int dy=-inflation; dy<=inflation; dy++)
      for (int dx=-inflation; dx<=inflation; dx++) {
        int nx=mx+dx, ny=my+dy;
        if (nx<0||ny<0||nx>=W||ny>=H) return false;
        if (map_.data[ny*W+nx]<0||map_.data[ny*W+nx]>=50) return false;
      }
    return true;
  }

  bool findNearestFree(int& mx, int& my, int radius, int inflation) const {
    if (isFree(mx,my,inflation)) return true;
    int bx=mx, by=my; double bd=std::numeric_limits<double>::infinity(); bool found=false;
    for (int dy=-radius; dy<=radius; dy++)
      for (int dx=-radius; dx<=radius; dx++) {
        int nx=mx+dx, ny=my+dy;
        if (!isFree(nx,ny,inflation)) continue;
        double d2=dx*dx+dy*dy;
        if (d2<bd){bd=d2;bx=nx;by=ny;found=true;}
      }
    if (!found) return false;
    mx=bx; my=by; return true;
  }

  struct ANode { int x{0},y{0}; double g{0.0},f{0.0}; };
  struct AComp { bool operator()(const ANode& a,const ANode& b)const{return a.f>b.f;} };

  bool aStarPlan(double sx,double sy,double gx,double gy,nav_msgs::msg::Path& out) {
    int sxi,syi,gxi,gyi;
    if (!worldToMap(sx,sy,sxi,syi)||!worldToMap(gx,gy,gxi,gyi)) return false;
    int W=static_cast<int>(map_.info.width), H=static_cast<int>(map_.info.height);
    int inflation=static_cast<int>(std::ceil(get_parameter("map_robot_radius").as_double()/map_.info.resolution));
    auto h=[&](int x,int y){return std::hypot((double)(x-gxi),(double)(y-gyi));};
    auto idx=[&](int x,int y){return y*W+x;};
    std::vector<double> gs(W*H,std::numeric_limits<double>::infinity());
    std::vector<int> par(W*H,-1);
    std::vector<uint8_t> cl(W*H,0);
    if (!findNearestFree(sxi,syi,8,inflation)||!findNearestFree(gxi,gyi,8,inflation)) return false;
    std::priority_queue<ANode,std::vector<ANode>,AComp> pq;
    int sidx=idx(sxi,syi); gs[sidx]=0.0; pq.push(ANode{sxi,syi,0.0,h(sxi,syi)});
    const int dx8[8]={1,-1,0,0,1,1,-1,-1}, dy8[8]={0,0,1,-1,1,-1,1,-1};
    bool found=false;
    while (!pq.empty()) {
      ANode cur=pq.top(); pq.pop();
      int ci=idx(cur.x,cur.y);
      if (cl[ci]) continue; cl[ci]=1;
      if (cur.x==gxi&&cur.y==gyi){found=true;break;}
      for (int k=0;k<8;k++){
        int nx=cur.x+dx8[k],ny=cur.y+dy8[k];
        if (nx<0||ny<0||nx>=W||ny>=H||!isFree(nx,ny,inflation)) continue;
        double step=(k<4)?1.0:std::sqrt(2.0);
        int ni=idx(nx,ny); if(cl[ni]) continue;
        double ng=gs[ci]+step;
        if (ng<gs[ni]){gs[ni]=ng;par[ni]=ci;pq.push(ANode{nx,ny,ng,ng+h(nx,ny)});}
      }
    }
    if (!found) return false;
    std::vector<std::pair<int,int>> cells;
    int cur=idx(gxi,gyi);
    while (cur!=-1&&cur!=sidx){cells.push_back({cur%W,cur/W});cur=par[cur];}
    cells.push_back({sxi,syi}); std::reverse(cells.begin(),cells.end());
    out.header.frame_id=map_.header.frame_id.empty()?"map":map_.header.frame_id;
    out.header.stamp=now(); out.poses.clear(); out.poses.reserve(cells.size());
    for (auto& c:cells){
      geometry_msgs::msg::PoseStamped ps; ps.header=out.header;
      mapToWorld(c.first,c.second,ps.pose.position.x,ps.pose.position.y);
      ps.pose.orientation.w=1.0; out.poses.push_back(ps);
    }
    smoothPath(out); return true;
  }

  void smoothPath(nav_msgs::msg::Path& path,int iter=80,double alpha=0.3,double beta=0.15) {
    int n=static_cast<int>(path.poses.size()); if(n<3) return;
    std::vector<double> ox(n),oy(n);
    for (int i=0;i<n;i++){ox[i]=path.poses[i].pose.position.x;oy[i]=path.poses[i].pose.position.y;}
    for (int it=0;it<iter;it++)
      for (int i=1;i<n-1;i++){
        double cx=path.poses[i].pose.position.x,cy=path.poses[i].pose.position.y;
        path.poses[i].pose.position.x=cx+alpha*(ox[i]-cx)+beta*(path.poses[i-1].pose.position.x+path.poses[i+1].pose.position.x-2.0*cx);
        path.poses[i].pose.position.y=cy+alpha*(oy[i]-cy)+beta*(path.poses[i-1].pose.position.y+path.poses[i+1].pose.position.y-2.0*cy);
      }
  }

  double computeLookahead(double v) const {
    double lmin=get_parameter("pp_lookahead_min").as_double();
    double lmax=get_parameter("pp_lookahead_max").as_double();
    double k=get_parameter("pp_lookahead_speed_gain").as_double();
    return std::clamp(lmin+k*std::fabs(v),lmin,lmax);
  }

  int findClosestIndex(const nav_msgs::msg::Path& path,double x,double y) const {
    int n=static_cast<int>(path.poses.size()); if(n==0) return 0;
    int best=0; double bd=std::numeric_limits<double>::infinity();
    for (int i=0;i<n;i++){
      double d=dist2(path.poses[i].pose.position.x,path.poses[i].pose.position.y,x,y);
      if (d<bd){bd=d;best=i;}
    }
    return best;
  }

  bool purePursuitCmd(const nav_msgs::msg::Path& path,double& v_out,double& w_out) {
    if (path.poses.size()<2) return false;
    if (path_index_<0) path_index_=0;
    path_index_=findClosestIndex(path,pose_x_,pose_y_);
    double v_nom=get_parameter("v_nominal").as_double();
    double L=computeLookahead(v_nom);
    int n=static_cast<int>(path.poses.size()); int look=path_index_; double accum=0.0;
    while (look+1<n){
      accum+=std::hypot(path.poses[look+1].pose.position.x-path.poses[look].pose.position.x,
                        path.poses[look+1].pose.position.y-path.poses[look].pose.position.y);
      look++; if(accum>=L) break;
    }
    look=std::clamp(look,path_index_,n-1);
    double tx=path.poses[look].pose.position.x, ty=path.poses[look].pose.position.y;
    double dx=tx-pose_x_, dy=ty-pose_y_;
    double lx=std::cos(-pose_yaw_)*dx-std::sin(-pose_yaw_)*dy;
    double ly=std::sin(-pose_yaw_)*dx+std::cos(-pose_yaw_)*dy;
    if (lx<-0.3) return false;
    double kappa=2.0*ly/(lx*lx+ly*ly);
    v_out=v_nom; w_out=std::clamp(v_nom*kappa,-get_parameter("w_max").as_double(),get_parameter("w_max").as_double());
    return true;
  }

  struct ObsInfo { bool front_blocked{false}; double left_clear{0.0},right_clear{0.0}; };

  ObsInfo analyzeScan() const {
    ObsInfo o; if(!have_scan_||scan_.ranges.empty()) return o;
    double fd=get_parameter("obs_front_dist").as_double();
    double ff=get_parameter("obs_front_fov_deg").as_double()*M_PI/180.0;
    double fs=get_parameter("obs_side_fov_deg").as_double()*M_PI/180.0;
    double rmin=scan_.range_min,rmax=scan_.range_max;
    double fmn=rmax,lmn=rmax,rmn=rmax; int fc=0,lc=0,rc=0;
    for (size_t i=0;i<scan_.ranges.size();i++){
      float rf=scan_.ranges[i]; if(!std::isfinite(rf)) continue;
      double r=rf; if(r<rmin||r>rmax) continue;
      double ang=scan_.angle_min+i*scan_.angle_increment;
      while(ang> M_PI) ang-=2.0*M_PI; while(ang<-M_PI) ang+=2.0*M_PI;
      if(std::fabs(ang)<=ff){fmn=std::min(fmn,r);fc++;}
      if(ang>0.0&&ang<=fs){lmn=std::min(lmn,r);lc++;}
      else if(ang<0.0&&ang>=-fs){rmn=std::min(rmn,r);rc++;}
    }
    if(fc<8)fmn=0.0; if(lc<8)lmn=0.0; if(rc<8)rmn=0.0;
    o.front_blocked=(fmn<fd); o.left_clear=lmn; o.right_clear=rmn;
    return o;
  }

  double pathDeviation() const {
    if(global_path_.poses.empty()) return 0.0;
    int n=static_cast<int>(global_path_.poses.size());
    double best=std::numeric_limits<double>::infinity();
    int a=std::max(0,path_index_-20),b=std::min(n-1,path_index_+20);
    for (int i=a;i<=b;i++){
      double d=std::sqrt(dist2(global_path_.poses[i].pose.position.x,
                               global_path_.poses[i].pose.position.y,pose_x_,pose_y_));
      if(d<best) best=d;
    }
    return best;
  }

  double getFrontPersonDist() const {
    if (!have_scan_ || scan_.ranges.empty()) return 999.0;
    double fov = 15.0 * M_PI / 180.0;
    double min_dist = 999.0;
    for (size_t i = 0; i < scan_.ranges.size(); i++) {
        float r = scan_.ranges[i];
        if (!std::isfinite(r)) continue;
        double ang = scan_.angle_min + i * scan_.angle_increment;
        while (ang >  M_PI) ang -= 2.0 * M_PI;
        while (ang < -M_PI) ang += 2.0 * M_PI;
        if (std::fabs(ang) <= fov)
            min_dist = std::min(min_dist, (double)r);
    }
    return min_dist;
  }

  void publishZero() { geometry_msgs::msg::Twist cmd; cmd_pub_->publish(cmd); }

  void publishCmd(double v,double w) {
    double alpha=get_parameter("w_lpf_alpha").as_double();
    double dead=get_parameter("w_deadband").as_double();
    if(std::fabs(w)<dead) w=0.0;
    w_filt_=alpha*w+(1.0-alpha)*w_filt_;
    geometry_msgs::msg::Twist cmd; cmd.linear.x=v; cmd.angular.z=w_filt_;
    cmd_pub_->publish(cmd);
  }

  void publishRotateCmd(double w) {
    geometry_msgs::msg::Twist cmd; cmd.linear.x=0.0; cmd.angular.z=w;
    cmd_pub_->publish(cmd); w_filt_=w;
  }

  bool reachedWaypoint(const Waypoint& wp) const {
    return std::sqrt(dist2(pose_x_,pose_y_,wp.x,wp.y))<=get_parameter("goal_tolerance").as_double();
  }

  void advanceWaypoint() {
    int next_index = (seq_index_ + 1) % static_cast<int>(seq_.size());

    // 💡 [수정] 1회 주행(ONCE) 무한 루프 버그 완벽 해결
    if (nav_mode_ == NavMode::ONCE) {
      if (seq_index_ == static_cast<int>(seq_.size()) - 1) {
        // 마지막 웨이포인트를 찍고 시작점(0번)으로 복귀하기 직전에 플래그 ON
        nav_once_done_ = true; 
      } 
      else if (seq_index_ == 0 && nav_once_done_) {
        // 시작점에 최종 도착해서 다시 1번으로 출발하려는 순간, 강제 정지!
        nav_once_done_ = false;
        nav_mode_ = NavMode::IDLE;
        publishZero();
        RCLCPP_INFO(get_logger(), "🏁 1회 주행 완료 → 시작 위치 정지");
        return;
      }
    }

    seq_index_ = next_index;
    target_wp_index_ = seq_[seq_index_];
  }

  void triggerSectorFallback() {
    char blocked_sector = waypoints_[target_wp_index_].label.empty() ?
                          ' ' : waypoints_[target_wp_index_].label[0];
    std::vector<int> new_seq;
    int N = static_cast<int>(waypoints_.size());
    for (int i = seq_index_ - 1; i >= 0; i--)
        new_seq.push_back(seq_[i]);
    std::vector<int> blocked_fwd;
    for (int i = 0; i < N; i++) {
        if (!waypoints_[i].label.empty() &&
            waypoints_[i].label[0] == blocked_sector)
            blocked_fwd.push_back(i);
    }
    for (int idx : blocked_fwd) new_seq.push_back(idx);
    for (int i = static_cast<int>(blocked_fwd.size())-2; i >= 0; i--)
        new_seq.push_back(blocked_fwd[i]);
    for (int i = seq_index_ - 1; i >= 0; i--)
        new_seq.push_back(seq_[i]);
    for (int i = 0; i < N; i++) new_seq.push_back(i);
    for (int i = N-2; i >= 1; i--) new_seq.push_back(i);
    seq_ = new_seq;
    seq_index_ = 0;
    target_wp_index_ = seq_[0];
    astar_fail_count_ = 0;
    obs_blocked_start_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    global_path_.poses.clear();
    mode_ = Mode::PLAN;
    RCLCPP_WARN(get_logger(), "섹터 막힘 → 역순 복귀 후 막힌 섹터 우선 방문");
  }

  void planToTarget() {
    w_filt_=0.0; publishZero();
    const Waypoint& tgt=waypoints_[target_wp_index_];
    nav_msgs::msg::Path path;
    if (!aStarPlan(pose_x_,pose_y_,tgt.x,tgt.y,path)){
        RCLCPP_WARN(get_logger(),"A* failed, retry next cycle");
        astar_fail_count_++;
        if (astar_fail_count_ >= ASTAR_FAIL_MAX) triggerSectorFallback();
      return;
    }
    astar_fail_count_ = 0;
    global_path_=path; path_pub_->publish(global_path_); path_index_=0;
    int n=static_cast<int>(global_path_.poses.size());
    double lmin=get_parameter("pp_lookahead_min").as_double();
    double best_d=std::numeric_limits<double>::infinity();
    for (int i=0;i<n;i++){
      double px=global_path_.poses[i].pose.position.x, py=global_path_.poses[i].pose.position.y;
      double dx=px-pose_x_, dy=py-pose_y_;
      double lx=std::cos(-pose_yaw_)*dx-std::sin(-pose_yaw_)*dy;
      if(lx<0.0) continue;
      double d=dist2(px,py,pose_x_,pose_y_);
      if(std::sqrt(d)<lmin) continue;
      if(d<best_d){best_d=d;path_index_=i;}
    }
    mode_=Mode::TRACK;
  }

  void returnToPlanWithRotate() {
    w_filt_=0.0; publishZero(); global_path_.poses.clear(); path_index_=0;
    playSound(0);
    publishLED(3);
    const Waypoint& tgt=waypoints_[target_wp_index_];
    double heading=std::atan2(tgt.y-pose_y_,tgt.x-pose_x_);
    double dyaw=normAngle(heading-pose_yaw_);
    double trig=get_parameter("rotate_trigger_deg").as_double()*M_PI/180.0;
    std_msgs::msg::Bool fm; fm.data=false; follow_pub_->publish(fm);
    if(std::fabs(dyaw)>=trig){rotate_target_yaw_=heading;rotate_settle_count_=0;mode_=Mode::ROTATE;}
    else mode_=Mode::PLAN;
  }

  void playSound(int value) {
    if (!sound_client_->wait_for_service(std::chrono::milliseconds(100))) return;
    auto req=std::make_shared<turtlebot3_msgs::srv::Sound::Request>();
    req->value=value; sound_client_->async_send_request(req);
  }

  void tickBuzzer() {
    if (!buzzer_on_) return;
    if (last_buzzer_time_.nanoseconds()==0 || (now()-last_buzzer_time_).seconds()>=1.0) {
      playSound(2); last_buzzer_time_=now();
    }
  }

  bool isSearchDone() {
    double delta=std::fabs(normAngle(pose_yaw_-search_last_yaw_));
    search_accumulated_yaw_+=delta;
    search_last_yaw_=pose_yaw_;
    return (search_accumulated_yaw_ >= 2.0*M_PI - 0.15);
  }

  void publishLED(int mode) {
    std_msgs::msg::Int32 msg;
    msg.data = mode;
    led_pub_->publish(msg);
  }

  void enterSearchMode() {
      publishZero();
      search_accumulated_yaw_ = 0.0;
      search_last_yaw_ = pose_yaw_;
      search_was_stopped_ = robot_stopped_;

      if (!buzzer_on_) { buzzer_on_ = true; playSound(2); }

      double pan_offset = pan_angle_ - 80.0;
      if (std::fabs(pan_offset) > 10.0) {
          search_w_ = (pan_offset < 0) ? -0.4 : 0.4;
      } else {
          if (!stopped_scan_ranges_.empty() && have_scan_ &&
              stopped_scan_ranges_.size() == scan_.ranges.size()) {
              double ld=0.0, rd=0.0; int lc=0, rc=0;
              int n = static_cast<int>(scan_.ranges.size());
              for (int i = 0; i < n; i++) {
                  double ang = scan_.angle_min + i * scan_.angle_increment;
                  while (ang >  M_PI) ang -= 2.0 * M_PI;
                  while (ang < -M_PI) ang += 2.0 * M_PI;
                  if (!std::isfinite(stopped_scan_ranges_[i]) ||
                      !std::isfinite(scan_.ranges[i])) continue;
                  double delta = stopped_scan_ranges_[i] - scan_.ranges[i];
                  if (delta < SCAN_DELTA_THRESHOLD) continue;
                  if (ang > 0.0) { ld += delta; lc++; }
                  else if (ang < 0.0) { rd += delta; rc++; }
              }
              double la = (lc > 0) ? ld/lc : 0.0;
              double ra = (rc > 0) ? rd/rc : 0.0;
              if (la >= ra && la > 0.0) {
                  search_w_ = 0.4;
                  RCLCPP_INFO(get_logger(), "SEARCH: 왼쪽 스캔 변화 많음 → 왼쪽 회전");
              } else if (ra > la && ra > 0.0) {
                  search_w_ = -0.4;
                  RCLCPP_INFO(get_logger(), "SEARCH: 오른쪽 스캔 변화 많음 → 오른쪽 회전");
              } else {
                  search_w_ = 0.4;
                  RCLCPP_INFO(get_logger(), "SEARCH: 스캔 변화 없음 → 기본 왼쪽 회전");
              }
          } else {
              search_w_ = 0.4;
          }
      }

      if (search_was_stopped_ && !stopped_scan_ranges_.empty() && have_scan_) {
          double scan_ang = -(pan_offset / 100.0) * (M_PI / 2.0);
          double best_diff = std::numeric_limits<double>::infinity();
          double person_dist = 999.0;
          for (size_t i = 0; i < stopped_scan_ranges_.size(); i++) {
              if (!std::isfinite(stopped_scan_ranges_[i])) continue;
              double ang = scan_.angle_min + i * scan_.angle_increment;
              while (ang >  M_PI) ang -= 2.0 * M_PI;
              while (ang < -M_PI) ang += 2.0 * M_PI;
              double diff = std::fabs(normAngle(ang - scan_ang));
              if (diff < best_diff) { best_diff = diff; person_dist = stopped_scan_ranges_[i]; }
          }
          if (person_dist > 1.5) {
              std_msgs::msg::Bool sm; sm.data = false; stopped_pub_->publish(sm);
              std_msgs::msg::Bool fm; fm.data = false; follow_pub_->publish(fm);
              fm.data = true; follow_pub_->publish(fm);
              RCLCPP_INFO(get_logger(), "SEARCH: 사람 멀리(%.2fm) → tilt 35", person_dist);
          } else {
              std_msgs::msg::Bool sm; sm.data = true; stopped_pub_->publish(sm);
              RCLCPP_INFO(get_logger(), "SEARCH: 사람 가까이(%.2fm) → tilt 15", person_dist);
          }
      } else {
          std_msgs::msg::Bool sm; sm.data = false; stopped_pub_->publish(sm);
          std_msgs::msg::Bool fm; fm.data = false; follow_pub_->publish(fm);
          fm.data = true; follow_pub_->publish(fm);
      }

      robot_stopped_ = false;
      stop_time_ = rclcpp::Time(0);
      face_timeout_ = false;
      stop_settled_ = false;
      stop_settled_time_ = rclcpp::Time(0);
      person_detect_confirm_ = 0;
      mode_ = Mode::SEARCH;
      RCLCPP_WARN(get_logger(), "SEARCH 모드 진입 (was_stopped=%d, search_w=%.2f)",
                  search_was_stopped_, search_w_);
  }

  void onTimer() {
    if (theft_detected_) {
      if ((now() - last_theft_buzz_time_).seconds() >= 1.0) {
        playSound(3); last_theft_buzz_time_ = now();
      }
    } else {
      last_theft_buzz_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    }

    if (nav_mode_ == NavMode::IDLE) { publishZero(); return; }
    if (!have_map_||!have_pose_) { publishZero(); return; }

    const Waypoint& tgt=waypoints_[target_wp_index_];

    if (mode_==Mode::WAIT) mode_=Mode::PLAN;
    if (mode_==Mode::PLAN) { planToTarget(); return; }

    if (mode_!=Mode::ROTATE&&mode_!=Mode::FOLLOW&&mode_!=Mode::SEARCH&&mode_!=Mode::SEARCH_BACKUP&&reachedWaypoint(tgt)){
      if (nav_mode_ == NavMode::CHARGE || nav_mode_ == NavMode::GOTO) {
        double target_yaw = waypoints_[target_wp_index_].yaw;
        double dyaw = normAngle(target_yaw - pose_yaw_);
        double tol = get_parameter("yaw_tolerance").as_double();
        if (std::fabs(dyaw) >= tol) {
            rotate_target_yaw_ = target_yaw;
            rotate_settle_count_ = 0;
            mode_ = Mode::ROTATE;
            publishZero();
            return;
        }
        publishZero();
        nav_mode_ = NavMode::IDLE;
        RCLCPP_INFO(get_logger(), "목적지 도착 → 정지");
        publishLED(3);
        return;
      }
      advanceWaypoint();
      const Waypoint& nxt=waypoints_[target_wp_index_];
      const Waypoint& cur_wp=waypoints_[seq_[(seq_index_-1+static_cast<int>(seq_.size()))%seq_.size()]];
      double heading=std::atan2(nxt.y-cur_wp.y,nxt.x-cur_wp.x);
      double dyaw=normAngle(heading-pose_yaw_);
      double trig=get_parameter("rotate_trigger_deg").as_double()*M_PI/180.0;
      if(std::fabs(dyaw)>=trig){rotate_target_yaw_=heading;rotate_settle_count_=0;mode_=Mode::ROTATE;}
      else mode_=Mode::PLAN;
      publishZero(); return;
    }

    if (mode_==Mode::ROTATE){
      obs_blocked_start_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
      double dyaw=normAngle(rotate_target_yaw_-pose_yaw_);
      double tol=get_parameter("yaw_tolerance").as_double();
      if(std::fabs(dyaw)<=tol){
          publishZero(); w_filt_=0.0;
          if(++rotate_settle_count_>=5){
              rotate_settle_count_=0;
              // CHARGE / GOTO 모드면 회전 후 정지
              if (nav_mode_ == NavMode::CHARGE || nav_mode_ == NavMode::GOTO) {
                  nav_mode_ = NavMode::IDLE;
                  RCLCPP_INFO(get_logger(), "목적지 yaw 정렬 완료 → 정지");
                  publishLED(3);
              } else {
                  mode_ = Mode::PLAN;
              }
          }
          return;
      }
      rotate_settle_count_=0;
      double wmax=get_parameter("rotate_in_place_w").as_double();
      double w=std::clamp(1.5*dyaw,-wmax,wmax);
      if(std::fabs(w)<0.15) w=std::copysign(0.15,dyaw);
      publishRotateCmd(w); return;
    }

    if (global_path_.poses.size()<2&&mode_!=Mode::FOLLOW&&mode_!=Mode::SEARCH&&mode_!=Mode::SEARCH_BACKUP){
      mode_=Mode::PLAN; publishZero(); return;
    }

    // ── SEARCH 모드 ──────────────────────────────────────────────────
    if (mode_ == Mode::SEARCH) {
      tickBuzzer();
      publishLED(4);

      bool person_found = person_detected_ && person_bbox_z_ >= 0.10;

      if (person_found) {
        person_detect_confirm_++;
        if (person_detect_confirm_ >= 2) {
          double cur_dist = getFrontPersonDist();
          RCLCPP_INFO(get_logger(), "SEARCH 중 사람 발견! dist=%.2f", cur_dist);
          publishZero();
          search_accumulated_yaw_ = 0.0;
          stopped_scan_ranges_.clear();
          person_detect_confirm_ = 0;
          person_lost_time_ = rclcpp::Time(0);


          if (cur_dist <= 1.2 || person_bbox_z_ >= 0.25) {
            RCLCPP_INFO(get_logger(), "SEARCH → 가까움 → 후진 후 안면인식");
            mode_ = Mode::SEARCH_BACKUP;  // 새 모드
            search_backup_start_ = now();
            std_msgs::msg::Bool fm; fm.data = true; follow_pub_->publish(fm);
          } else {
            robot_stopped_ = false;
            std_msgs::msg::Bool sm; sm.data = false; stopped_pub_->publish(sm);
            std_msgs::msg::Bool fm; fm.data = true; follow_pub_->publish(fm);
            RCLCPP_INFO(get_logger(), "SEARCH → 멀리 → FOLLOW 추종");
            mode_ = Mode::FOLLOW;
            follow_reentry_time_ = now();
          }
          return;
        }
      } else {
        person_detect_confirm_ = 0;
      }

      if (isSearchDone()) {
        RCLCPP_INFO(get_logger(), "SEARCH 완료 → 사람 없음 → 경로 재설정");
        buzzer_on_ = false; last_buzzer_time_ = rclcpp::Time(0);
        face_success_done_ = false; face_recognized_ = false; face_timeout_ = false;
        robot_stopped_ = false;
        person_detect_confirm_ = 0;
        stopped_scan_ranges_.clear();
        std_msgs::msg::Bool sm; sm.data = false; stopped_pub_->publish(sm);
        std_msgs::msg::Bool fm; fm.data = false; follow_pub_->publish(fm);
        returnToPlanWithRotate();
        return;
      }

      publishRotateCmd(search_w_);
      return;
    }

    if (mode_ == Mode::SEARCH_BACKUP) {
      if ((now() - search_backup_start_).seconds() < 1.5) {
          geometry_msgs::msg::Twist cmd;
          cmd.linear.x = -0.1;
          cmd_pub_->publish(cmd);
          return;
      }

      publishZero();
      robot_stopped_ = true;
      stop_time_ = rclcpp::Time(0);
      face_timeout_ = false;
      stop_settled_ = false;
      stop_settled_time_ = rclcpp::Time(0);
      std_msgs::msg::Bool sm;
      sm.data = false; stopped_pub_->publish(sm);
      sm.data = true;  stopped_pub_->publish(sm);
      std_msgs::msg::Bool fm; fm.data = true; follow_pub_->publish(fm);
      RCLCPP_INFO(get_logger(), "후진 완료 → 정지 + 안면인식");
      from_search_backup_ = true;
      mode_ = Mode::FOLLOW;
      follow_reentry_time_ = now();
      search_backup_start_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
      return;
    }

    // ── TRACK 모드 ───────────────────────────────────────────────────
    if (mode_==Mode::TRACK){
      publishLED(5);
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
        "TRACK: person_detected=%d bbox_z=%.3f ignore=%d mode=%d",
        person_detected_, person_bbox_z_, 
        (int)(face_success_time_.nanoseconds()!=0&&(now()-face_success_time_).seconds()<FACE_SUCCESS_IGNORE),
        (int)mode_);
      ObsInfo obs=analyzeScan();
      if(obs.front_blocked){
        if (obs_blocked_start_.nanoseconds() == 0) obs_blocked_start_ = now();
        if ((now() - obs_blocked_start_).seconds() >= OBS_BLOCKED_TIMEOUT) {
          RCLCPP_WARN(get_logger(), "장애물 지속 감지 → 섹터 역순 복귀");
          triggerSectorFallback(); return;
        }
        bool gl=(obs.left_clear>=obs.right_clear); double sign=gl?+1.0:-1.0;
        double wg=get_parameter("avoid_w_gain").as_double();
        double wa=std::clamp(sign*wg*(1.0/std::max(0.25,std::min(obs.left_clear,obs.right_clear))),
                             -get_parameter("w_max").as_double(),get_parameter("w_max").as_double());
        publishCmd(get_parameter("v_min").as_double(),wa); return;
      } else {
        obs_blocked_start_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
      }
      bool ignore_person=(face_success_time_.nanoseconds()!=0&&
                          (now()-face_success_time_).seconds()<FACE_SUCCESS_IGNORE);
      double front_dist = getFrontPersonDist();
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                           "person_bbox_z=%.3f front_dist=%.2f", person_bbox_z_, front_dist);
      if(person_detected_ && !ignore_person && person_bbox_z_ >= 0.10 && front_dist <= 2.5){
        std_msgs::msg::Bool sm; sm.data=false; stopped_pub_->publish(sm);
        person_detect_count_++;
        if(person_detect_count_>=PERSON_DETECT_THRESHOLD){
          RCLCPP_INFO(get_logger(),"사람 감지 → FOLLOW 모드");
          person_detect_count_=0; face_success_done_=false;
          person_lost_time_=rclcpp::Time(0); person_detect_confirm_=0;
          mode_=Mode::FOLLOW;
          follow_reentry_time_ = now();
          std_msgs::msg::Bool fm; fm.data=true; follow_pub_->publish(fm);
        }
        if(mode_==Mode::FOLLOW) return;
      } else person_detect_count_=0;
    }

    // ── FOLLOW 모드 ──────────────────────────────────────────────────
    if (mode_ == Mode::FOLLOW) {
      double front_dist_follow = getFrontPersonDist();
      bool person_really_detected = person_detected_
                                    && person_bbox_z_ >= 0.08
                                    && front_dist_follow <= 2.5;

      if (person_really_detected) {
        person_detect_confirm_++;
        last_person_bbox_x_ = person_bbox_x_;
        if (person_detect_confirm_ >= 2) person_lost_time_ = rclcpp::Time(0);
        if (person_bbox_x_ < -0.15) publishLED(1); 
        else if (person_bbox_x_ > 0.15) publishLED(2);
      } else {
        person_detect_confirm_ = 0;
      }

      if (!person_really_detected) {
          if (follow_reentry_time_.nanoseconds() != 0 &&
              (now() - follow_reentry_time_).seconds() < 2.0) {
              publishZero(); return;
          }

          if (person_lost_time_.nanoseconds() == 0) {
              person_lost_time_ = now();
              publishLED(4);
              stopped_scan_ranges_ = std::vector<float>(scan_.ranges.begin(), scan_.ranges.end());
          }

          if (robot_stopped_ || from_search_backup_) {
              if ((now() - person_lost_time_).seconds() >= 0.3) {
                  from_search_backup_ = false;
                  enterSearchMode();
              } else {
                  publishZero();
              }
          } else {
              if ((now() - person_lost_time_).seconds() >= 2.0) {
                  enterSearchMode();
              } else {
                  double pan_error = (pan_angle_ - 80.0) / 100.0;
                  if (std::fabs(pan_error) < 0.15) pan_error = 0.0;
                  double w_cmd = 0.8 * pan_error;
                  publishCmd(0.15, w_cmd);
              }
          }
          return;
      }

      person_lost_time_ = rclcpp::Time(0);
      follow_reentry_time_ = rclcpp::Time(0);
      from_search_backup_ = false; 

      if (robot_stopped_ || (person_really_detected && front_dist_follow <= 1.0)) {
        ObsInfo obs = analyzeScan();
        if (obs.front_blocked) {
          bool gl = (obs.left_clear >= obs.right_clear);
          double sign = gl ? +1.0 : -1.0;
          double wg = get_parameter("avoid_w_gain").as_double();
          double wa = std::clamp(sign * wg * (1.0 / std::max(0.25, std::min(obs.left_clear, obs.right_clear))),
                                 -get_parameter("w_max").as_double(), get_parameter("w_max").as_double());
          publishCmd(get_parameter("v_min").as_double(), wa);
          return;
        }
        publishZero();
        if (!robot_stopped_) {
          std_msgs::msg::Bool sm; sm.data = true; stopped_pub_->publish(sm);
          robot_stopped_ = true; stop_settled_time_ = rclcpp::Time(0);
          stopped_scan_ranges_ = std::vector<float>(scan_.ranges.begin(), scan_.ranges.end());
        }
        if (!stop_settled_) {
          if (stop_settled_time_.nanoseconds() == 0) stop_settled_time_ = now();
          else if ((now() - stop_settled_time_).seconds() >= 1.0) stop_settled_ = true;
        }
        if (!face_timeout_ && stop_time_.nanoseconds() == 0) stop_time_ = now();
        if (!face_timeout_ && (now() - stop_time_).seconds() >= FACE_TIMEOUT) {
          face_timeout_ = true; buzzer_on_ = true; playSound(2);
          RCLCPP_WARN(get_logger(), "얼굴 인식 실패 → 부저 ON");
        }
        if (face_timeout_ && buzzer_on_) tickBuzzer();

        if (front_dist_follow > 1.4 && robot_stopped_ && person_really_detected) {
          std_msgs::msg::Bool sm; sm.data = false; stopped_pub_->publish(sm);
          robot_stopped_ = false; stop_settled_ = false; stop_settled_time_ = rclcpp::Time(0);
          stop_time_ = rclcpp::Time(0); face_timeout_ = false;
          last_buzzer_time_ = rclcpp::Time(0); buzzer_on_ = false;
          return;
        }

        if (face_recognized_) {
          std_msgs::msg::Bool sm; sm.data = false; stopped_pub_->publish(sm);
          robot_stopped_ = false; face_success_time_ = now(); face_success_done_ = true;
          buzzer_on_ = false; last_buzzer_time_ = rclcpp::Time(0);
          face_recognized_ = false; stop_settled_ = false; stop_settled_time_ = rclcpp::Time(0);
          recognized_name_ = ""; face_timeout_ = false; stop_time_ = rclcpp::Time(0);
          RCLCPP_INFO(get_logger(), "안면인식 완료 → seq_index=%d target_wp=%d",
                seq_index_, target_wp_index_);
          advanceWaypoint();
          RCLCPP_INFO(get_logger(), "advanceWaypoint 후 → seq_index=%d target_wp=%d",
                seq_index_, target_wp_index_); 
          returnToPlanWithRotate();
        }
      } else {
        if (robot_stopped_) { publishZero(); return; }
        face_recognized_ = false;
        ObsInfo obs = analyzeScan();
        if (obs.front_blocked) {
          bool gl = (obs.left_clear >= obs.right_clear);
          double sign = gl ? +1.0 : -1.0;
          double wg = get_parameter("avoid_w_gain").as_double();
          double wa = std::clamp(sign * wg * (1.0 / std::max(0.25, std::min(obs.left_clear, obs.right_clear))),
                                 -get_parameter("w_max").as_double(), get_parameter("w_max").as_double());
          double aw = get_parameter("avoid_weight").as_double();
          double w = (1.0 - aw) * (-0.3 * person_bbox_x_) + aw * wa;
          publishCmd(get_parameter("v_min").as_double(), w);
          RCLCPP_WARN(get_logger(), "FOLLOW 중 장애물 → 회피 기동");
        } else {
          constexpr double BBOX_Z_FAR  = 0.05;
          constexpr double BBOX_Z_NEAR = 0.25;
          constexpr double V_FOLLOW_MAX = 0.22;
          constexpr double V_FOLLOW_MIN = 0.15;
          double t = (person_bbox_z_ - BBOX_Z_FAR) / (BBOX_Z_NEAR - BBOX_Z_FAR);
          t = std::clamp(t, 0.0, 1.0);
          double v_cmd = V_FOLLOW_MAX + t * (V_FOLLOW_MIN - V_FOLLOW_MAX);
          double bbox_error = person_bbox_x_;
          if (std::fabs(bbox_error) < 0.05) bbox_error = 0.0;
          double w_cmd = -0.8 * bbox_error;
          publishCmd(v_cmd, w_cmd);
        }
      }
      return;
    }

    ObsInfo obs=analyzeScan();
    double dev=pathDeviation(), max_dev=get_parameter("max_path_deviation").as_double();
    double v_pp=0.0,w_pp=0.0;
    bool ok_pp=purePursuitCmd(global_path_,v_pp,w_pp);
    if(!ok_pp){
      int n=static_cast<int>(global_path_.poses.size());
      int look=std::min(path_index_+5,n-1);
      double tx=global_path_.poses[look].pose.position.x, ty=global_path_.poses[look].pose.position.y;
      double dyaw=normAngle(std::atan2(ty-pose_y_,tx-pose_x_)-pose_yaw_);
      double wr=std::clamp(1.5*dyaw,-get_parameter("rotate_in_place_w").as_double(),get_parameter("rotate_in_place_w").as_double());
      if(std::fabs(wr)<0.15) wr=std::copysign(0.15,dyaw);
      publishRotateCmd(wr); return;
    }
    if(!obs.front_blocked&&dev<=max_dev){mode_=Mode::TRACK;publishCmd(v_pp,w_pp);return;}
    if(dev>max_dev){RCLCPP_WARN(get_logger(),"Path deviation too large (%.2f) → replan",dev);mode_=Mode::PLAN;publishZero();return;}
    if(obs.front_blocked){
      mode_=Mode::AVOID;
      bool gl=(obs.left_clear>=obs.right_clear); double sign=gl?+1.0:-1.0;
      double wg=get_parameter("avoid_w_gain").as_double();
      double wa=std::clamp(sign*wg*(1.0/std::max(0.25,std::min(obs.left_clear,obs.right_clear))),
                           -get_parameter("w_max").as_double(),get_parameter("w_max").as_double());
      double aw=get_parameter("avoid_weight").as_double();
      double w=(1.0-aw)*w_pp+aw*wa;
      double v=std::max(v_pp*get_parameter("avoid_v_scale").as_double(),get_parameter("v_min").as_double());
      publishCmd(v,w); return;
    }
    mode_=Mode::TRACK; publishCmd(v_pp,w_pp);
  }

private:
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr              map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr               scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr                    cmd_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr                          path_pub_;
  rclcpp::TimerBase::SharedPtr                                               timer_;
  nav_msgs::msg::OccupancyGrid map_; sensor_msgs::msg::LaserScan scan_; nav_msgs::msg::Path global_path_;

  bool   have_map_{false},have_pose_{false},have_scan_{false};
  double pose_x_{0.0},pose_y_{0.0},pose_yaw_{0.0};

  std::vector<Waypoint> waypoints_; std::vector<int> seq_;
  int    seq_index_{0},target_wp_index_{0},rotate_settle_count_{0};
  Mode   mode_{Mode::WAIT}; int path_index_{0};
  double rotate_target_yaw_{0.0},w_filt_{0.0};

  bool   person_detected_{false}; double person_bbox_z_{0.0},person_bbox_x_{0.0};
  int    person_detect_count_{0};
  static constexpr int PERSON_DETECT_THRESHOLD=2;

  bool        face_recognized_{false}; std::string recognized_name_{""};
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr face_sub_;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr          stopped_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr       person_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr bbox_sub_;
  rclcpp::Client<turtlebot3_msgs::srv::Sound>::SharedPtr     sound_client_;

  rclcpp::Time stop_time_; bool buzzer_on_{false},face_timeout_{false};
  static constexpr double FACE_TIMEOUT=10.0;

  rclcpp::Time stop_settled_time_; bool stop_settled_{false};
  rclcpp::Time last_buzzer_time_;
  bool         face_success_done_{false};
  rclcpp::Time face_success_time_;
  static constexpr double FACE_SUCCESS_IGNORE=5.0;

  rclcpp::Time person_lost_time_;

  double search_w_{0.4};
  double search_accumulated_yaw_{0.0};
  double search_last_yaw_{0.0};
  bool   search_was_stopped_{false};

  std::vector<float> stopped_scan_ranges_;
  double last_person_bbox_x_{0.0};
  int    person_detect_confirm_{0};
  bool   search_back_{false};
  static constexpr double SCAN_DELTA_THRESHOLD=0.3;

  bool robot_stopped_{false};

  enum class NavMode { IDLE, LOOP, ONCE, CHARGE, GOTO } nav_mode_{NavMode::IDLE};
  bool nav_once_done_{false};
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr nav_cmd_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_pose_pub_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  bool theft_detected_{false};
  rclcpp::Time last_theft_buzz_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr follow_pub_;

  double pan_angle_{80.0};
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr pan_angle_sub_;

  int astar_fail_count_{0};
  static constexpr int ASTAR_FAIL_MAX = 10;
  rclcpp::Time obs_blocked_start_{0, 0, RCL_ROS_TIME};
  static constexpr double OBS_BLOCKED_TIMEOUT = 3.0;

  rclcpp::Time follow_reentry_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time search_backup_start_{0, 0, RCL_ROS_TIME};

  bool from_search_backup_{false};

  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr led_pub_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr charging_sub_;
};

int main(int argc,char** argv){
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<PatrolNavNode>());
  rclcpp::shutdown(); return 0;
}