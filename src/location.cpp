#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <vector>
#include <fstream>

struct Waypoint {
    double x, y, yaw;
};

class WaypointRecorder : public rclcpp::Node {
public:
    WaypointRecorder() : Node("waypoint_recorder") {
        sub_init_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 10,
            std::bind(&WaypointRecorder::initCallback, this, std::placeholders::_1));

        sub_goal_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10,
            std::bind(&WaypointRecorder::goalCallback, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(), "Waypoint Recorder Ready");
        RCLCPP_INFO(get_logger(), "Press ENTER to save YAML");
    }

private:
    void initCallback(
        const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        waypoints_.clear();
        waypoints_.push_back(toWaypoint(msg->pose.pose));
        RCLCPP_INFO(get_logger(), "Start point saved");
    }

    void goalCallback(
        const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        waypoints_.push_back(toWaypoint(msg->pose));
        RCLCPP_INFO(get_logger(), "Goal added (%zu)", waypoints_.size());
    }

    Waypoint toWaypoint(const geometry_msgs::msg::Pose& p) {
        double yaw = atan2(
            2.0 * (p.orientation.w * p.orientation.z),
            1.0 - 2.0 * p.orientation.z * p.orientation.z
        );
        return {p.position.x, p.position.y, yaw};
    }

public:
    void saveYaml() {
        std::ofstream file("/home/chan/lch_ws/src/pt_pkg/config/waypoints.yaml");
        file << "waypoints:\n";
        for (auto& wp : waypoints_) {
            file << "  - x: " << wp.x << "\n";
            file << "    y: " << wp.y << "\n";
            file << "    yaw: " << wp.yaw << "\n";
        }
        file.close();
        RCLCPP_INFO(get_logger(), "waypoints.yaml saved");
    }

private:
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_init_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_;
    std::vector<Waypoint> waypoints_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WaypointRecorder>();

    std::thread input_thread([&]() {
        std::cin.get();
        node->saveYaml();
    });

    rclcpp::spin(node);
    rclcpp::shutdown();
    input_thread.join();
    return 0;
}
