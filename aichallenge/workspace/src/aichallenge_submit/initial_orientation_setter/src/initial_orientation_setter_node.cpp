#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <cmath>
#include <vector>
#include <limits>

class InitialOrientationSetter : public rclcpp::Node
{
public:
    InitialOrientationSetter();

private:
    void gnss_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void trajectory_callback(const autoware_auto_planning_msgs::msg::Trajectory::SharedPtr msg);
    
    double calculate_yaw_from_trajectory(const geometry_msgs::msg::Point& current_pos);
    size_t find_nearest_waypoint_index(const geometry_msgs::msg::Point& current_pos);
    
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr gnss_sub_;
    rclcpp::Subscription<autoware_auto_planning_msgs::msg::Trajectory>::SharedPtr trajectory_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;
    
    autoware_auto_planning_msgs::msg::Trajectory::SharedPtr current_trajectory_;
    bool trajectory_received_ = false;
    int publish_count_ = 0;
    
    static constexpr double COVARIANCE_POSITION = 0.1;
    static constexpr double COVARIANCE_ORIENTATION = 0.1;
    static constexpr double INVALID_ORIENTATION_COVARIANCE = 100.0;
};

InitialOrientationSetter::InitialOrientationSetter()
: Node("initial_orientation_setter")
{
    const auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    const auto transient_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    
    gnss_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/sensing/gnss/pose_with_covariance", qos_profile,
        std::bind(&InitialOrientationSetter::gnss_callback, this, std::placeholders::_1));
    
    trajectory_sub_ = this->create_subscription<autoware_auto_planning_msgs::msg::Trajectory>(
        "/planning/scenario_planning/trajectory", qos_profile,
        std::bind(&InitialOrientationSetter::trajectory_callback, this, std::placeholders::_1));
    
    initial_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", transient_qos);
    
    RCLCPP_INFO(this->get_logger(), "Initial Orientation Setter node started");
}

void InitialOrientationSetter::trajectory_callback(const autoware_auto_planning_msgs::msg::Trajectory::SharedPtr msg)
{
    if (msg->points.empty()) {
        RCLCPP_WARN(this->get_logger(), "Received empty trajectory");
        return;
    }
    
    current_trajectory_ = msg;
    trajectory_received_ = true;
    RCLCPP_DEBUG(this->get_logger(), "Trajectory received with %zu points", msg->points.size());
}

void InitialOrientationSetter::gnss_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    if (!trajectory_received_ || !current_trajectory_) {
        RCLCPP_DEBUG(this->get_logger(), "Trajectory not yet received, skipping GNSS callback");
        return;
    }
    
    if (current_trajectory_->points.empty()) {
        RCLCPP_WARN(this->get_logger(), "Current trajectory is empty");
        return;
    }
    
    auto initial_pose = *msg;
    
    double yaw = calculate_yaw_from_trajectory(msg->pose.pose.position);
    
    initial_pose.pose.pose.orientation.x = 0.0;
    initial_pose.pose.pose.orientation.y = 0.0;
    initial_pose.pose.pose.orientation.z = sin(yaw * 0.5);
    initial_pose.pose.pose.orientation.w = cos(yaw * 0.5);
    
    initial_pose.pose.covariance[0] = COVARIANCE_POSITION;
    initial_pose.pose.covariance[7] = COVARIANCE_POSITION;
    initial_pose.pose.covariance[14] = COVARIANCE_POSITION;
    initial_pose.pose.covariance[21] = INVALID_ORIENTATION_COVARIANCE;
    initial_pose.pose.covariance[28] = INVALID_ORIENTATION_COVARIANCE;
    initial_pose.pose.covariance[35] = COVARIANCE_ORIENTATION;
    
    initial_pose_pub_->publish(initial_pose);
    publish_count_++;
    
    RCLCPP_INFO(this->get_logger(), 
        "Published initial pose (%d/3): position(%.3f, %.3f, %.3f), yaw=%.3f deg", 
        publish_count_,
        initial_pose.pose.pose.position.x,
        initial_pose.pose.pose.position.y,
        initial_pose.pose.pose.position.z,
        yaw * 180.0 / M_PI);
    
    if (publish_count_ >= 3) {
        RCLCPP_INFO(this->get_logger(), "Published 3 times. Shutting down...");
        rclcpp::shutdown();
    }
}

double InitialOrientationSetter::calculate_yaw_from_trajectory(const geometry_msgs::msg::Point& current_pos)
{
    size_t nearest_idx = find_nearest_waypoint_index(current_pos);
    
    if (nearest_idx >= current_trajectory_->points.size() - 1) {
        nearest_idx = current_trajectory_->points.size() - 2;
    }
    
    const auto& current_point = current_trajectory_->points[nearest_idx].pose.position;
    const auto& next_point = current_trajectory_->points[nearest_idx + 1].pose.position;
    
    double dx = next_point.x - current_point.x;
    double dy = next_point.y - current_point.y;
    
    return std::atan2(dy, dx);
}

size_t InitialOrientationSetter::find_nearest_waypoint_index(const geometry_msgs::msg::Point& current_pos)
{
    if (current_trajectory_->points.empty()) {
        return 0;
    }
    
    double min_distance_sq = std::numeric_limits<double>::max();
    size_t nearest_idx = 0;
    
    for (size_t i = 0; i < current_trajectory_->points.size(); ++i) {
        const auto& waypoint_pos = current_trajectory_->points[i].pose.position;
        
        double dx = current_pos.x - waypoint_pos.x;
        double dy = current_pos.y - waypoint_pos.y;
        double distance_sq = dx * dx + dy * dy;
        
        if (distance_sq < min_distance_sq) {
            min_distance_sq = distance_sq;
            nearest_idx = i;
        }
    }
    
    return nearest_idx;
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<InitialOrientationSetter>());
    rclcpp::shutdown();
    return 0;
}
