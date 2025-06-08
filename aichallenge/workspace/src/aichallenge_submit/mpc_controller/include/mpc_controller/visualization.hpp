#ifndef MPC_CONTROLLER_VISUALIZATION_HPP_
#define MPC_CONTROLLER_VISUALIZATION_HPP_

#include <vector>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>

namespace mpc_controller {

class Visualization {
public:
  explicit Visualization(rclcpp::Node::SharedPtr node);

  // 予測軌跡の可視化
  void publishTrajectory(
    const std::vector<double>& predicted_x,
    const std::vector<double>& predicted_y,
    const std::vector<double>& predicted_yaw,
    const double current_x,
    const double current_y,
    const double current_yaw,
    const double acceleration,
    const double steering_angle);

  // 車両の可視化（矢印）
  visualization_msgs::msg::Marker createVehicleMarker(
    const double x,
    const double y,
    const double yaw,
    const double scale = 1.0,
    const double r = 1.0,
    const double g = 0.0,
    const double b = 0.0);

  // 車両の可視化（四角形）
  visualization_msgs::msg::Marker createVehicleBoxMarker(
    const double x,
    const double y,
    const double yaw,
    const double length = 4.5,
    const double width = 1.8,
    const double r = 1.0,
    const double g = 0.0,
    const double b = 0.0);

  // 軌跡の可視化（線）
  visualization_msgs::msg::Marker createTrajectoryMarker(
    const std::vector<double>& x,
    const std::vector<double>& y,
    const double scale = 0.1,
    const double r = 0.0,
    const double g = 0.0,
    const double b = 1.0);

  // 制御入力のテキスト表示
  visualization_msgs::msg::Marker createControlInputTextMarker(
    const double x,
    const double y,
    const double acceleration,
    const double steering_angle,
    const double scale = 0.5);

private:
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Node::SharedPtr node_;
};

}  // namespace mpc_controller

#endif  // MPC_CONTROLLER_VISUALIZATION_HPP_ 