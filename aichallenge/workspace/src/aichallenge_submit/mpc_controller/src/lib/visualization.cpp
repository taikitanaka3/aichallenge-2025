#include "mpc_controller/visualization.hpp"
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <iomanip>
#include <sstream>

namespace mpc_controller {

Visualization::Visualization(rclcpp::Node::SharedPtr node) : node_(node) {
  marker_array_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
    "mpc_controller/visualization/marker_array", 1);
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
  const std::string& ns,
  int id,
  double r, double g, double b, double a,
  const std::vector<double>& x,
  const std::vector<double>& y)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = node_->now();
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = 0.02;  // 線の太さをさらに細く
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = a;

  // 点を追加
  for (size_t i = 0; i < x.size(); ++i) {
    geometry_msgs::msg::Point point;
    point.x = x[i];
    point.y = y[i];
    point.z = 0.0;
    marker.points.push_back(point);
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

void Visualization::publishPredictedTrajectory(
  const std::vector<double>& predicted_x,
  const std::vector<double>& predicted_y,
  const std::vector<double>& predicted_yaw,
  double current_x,
  double current_y,
  double current_yaw,
  double current_accel,
  double current_steer)
{
  visualization_msgs::msg::MarkerArray marker_array;

  // 予測軌跡のマーカー（赤・細く・半透明）
  auto trajectory_marker = createTrajectoryMarker(
    "predicted_trajectory", 0,
    1.0, 0.0, 0.0, 0.8,  // 赤色・半透明
    predicted_x, predicted_y);
  marker_array.markers.push_back(trajectory_marker);

  // 車両のマーカー（青矢印）
  visualization_msgs::msg::Marker vehicle_marker;
  vehicle_marker.header.frame_id = "map";
  vehicle_marker.header.stamp = node_->now();
  vehicle_marker.ns = "vehicle";
  vehicle_marker.id = 2;
  vehicle_marker.type = visualization_msgs::msg::Marker::ARROW;
  vehicle_marker.action = visualization_msgs::msg::Marker::ADD;
  vehicle_marker.scale.x = 0.5;  // 矢印の長さ
  vehicle_marker.scale.y = 0.1;  // 矢印の太さ
  vehicle_marker.scale.z = 0.1;  // 矢印の高さ
  vehicle_marker.color.r = 0.0;
  vehicle_marker.color.g = 0.0;
  vehicle_marker.color.b = 1.0;
  vehicle_marker.color.a = 1.0;

  // 車両の位置と向きを設定
  vehicle_marker.pose.position.x = current_x;
  vehicle_marker.pose.position.y = current_y;
  vehicle_marker.pose.position.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0, 0, current_yaw);
  vehicle_marker.pose.orientation = tf2::toMsg(q);

  marker_array.markers.push_back(vehicle_marker);

  // マーカー配列をパブリッシュ
  marker_array_pub_->publish(marker_array);
}

void Visualization::publishReferenceTrajectory(
  const std::vector<double>& reference_x,
  const std::vector<double>& reference_y)
{
  visualization_msgs::msg::MarkerArray marker_array;

  // 参照軌跡のマーカー（緑・細く・半透明）
  auto reference_marker = createTrajectoryMarker(
    "reference_trajectory", 1,
    0.0, 1.0, 0.0, 0.8,  // 緑色・半透明
    reference_x, reference_y);
  marker_array.markers.push_back(reference_marker);

  // マーカー配列をパブリッシュ
  marker_array_pub_->publish(marker_array);
}

}  // namespace mpc_controller 