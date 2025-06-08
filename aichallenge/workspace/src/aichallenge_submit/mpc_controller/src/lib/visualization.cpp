#include "mpc_controller/visualization.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <iomanip>
#include <sstream>

namespace mpc_controller {

Visualization::Visualization(rclcpp::Node::SharedPtr node) : node_(node) {
  marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
    "mpc_controller/visualization", 10);
}

visualization_msgs::msg::Marker Visualization::createVehicleMarker(
  const double x,
  const double y,
  const double yaw,
  const double scale,
  const double r,
  const double g,
  const double b) {
  
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = node_->now();
  marker.ns = "vehicle";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = scale;  // 矢印の長さ
  marker.scale.y = scale * 0.2;  // 矢印の太さ
  marker.scale.z = scale * 0.2;  // 矢印の高さ
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = 1.0;

  // 矢印の開始点
  geometry_msgs::msg::Point start;
  start.x = x;
  start.y = y;
  start.z = 0.0;
  marker.points.push_back(start);

  // 矢印の終点（進行方向）
  geometry_msgs::msg::Point end;
  end.x = x + std::cos(yaw) * scale;
  end.y = y + std::sin(yaw) * scale;
  end.z = 0.0;
  marker.points.push_back(end);

  return marker;
}

visualization_msgs::msg::Marker Visualization::createVehicleBoxMarker(
  const double x,
  const double y,
  const double yaw,
  const double length,
  const double width,
  const double r,
  const double g,
  const double b) {
  
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = node_->now();
  marker.ns = "vehicle_box";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::CUBE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = length;
  marker.scale.y = width;
  marker.scale.z = 0.1;  // 高さは薄く
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = 0.8;  // 少し透明に

  // 位置と向きの設定
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = 0.0;

  // ヨー角の設定
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  marker.pose.orientation = tf2::toMsg(q);

  return marker;
}

visualization_msgs::msg::Marker Visualization::createTrajectoryMarker(
  const std::vector<double>& x,
  const std::vector<double>& y,
  const double scale,
  const double r,
  const double g,
  const double b) {
  
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = node_->now();
  marker.ns = "trajectory";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = scale;  // 線の太さ
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = 1.0;

  for (size_t i = 0; i < x.size(); ++i) {
    geometry_msgs::msg::Point p;
    p.x = x[i];
    p.y = y[i];
    p.z = 0.0;
    marker.points.push_back(p);
  }

  return marker;
}

visualization_msgs::msg::Marker Visualization::createControlInputTextMarker(
  const double x,
  const double y,
  const double acceleration,
  const double steering_angle,
  const double scale) {
  
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = node_->now();
  marker.ns = "control_inputs";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.z = scale;  // テキストのサイズ
  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 1.0;
  marker.color.a = 1.0;

  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = 1.0;

  std::stringstream ss;
  ss << "Accel: " << std::fixed << std::setprecision(2) << acceleration << " m/s²\n"
     << "Steer: " << std::fixed << std::setprecision(2) << steering_angle * 180.0 / M_PI << "°";
  marker.text = ss.str();

  return marker;
}

void Visualization::publishTrajectory(
  const std::vector<double>& predicted_x,
  const std::vector<double>& predicted_y,
  const std::vector<double>& predicted_yaw,
  const double current_x,
  const double current_y,
  const double current_yaw,
  const double acceleration,
  const double steering_angle) {
  
  visualization_msgs::msg::MarkerArray marker_array;

  // 予測軌跡の追加
  marker_array.markers.push_back(createTrajectoryMarker(predicted_x, predicted_y));

  // 車両の位置と向きの追加（矢印）
  marker_array.markers.push_back(createVehicleMarker(current_x, current_y, current_yaw));

  // 車両の形状の追加（四角形）
  marker_array.markers.push_back(createVehicleBoxMarker(current_x, current_y, current_yaw));

  // 制御入力のテキスト表示の追加
  marker_array.markers.push_back(createControlInputTextMarker(
    current_x, current_y, acceleration, steering_angle));

  marker_pub_->publish(marker_array);
}

}  // namespace mpc_controller 