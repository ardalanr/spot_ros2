// Copyright (c) 2024 Robotics and AI Institute LLC dba RAI Institute. All rights reserved.

#include <spot_driver/local_grid/local_grid_middleware_handle.hpp>

namespace {
constexpr auto kPublisherHistoryDepth = 1;
constexpr auto kNodeName = "spot_local_grid_publisher";
constexpr auto kTerrainMapTopic = "local_grid/terrain";
}  // namespace

namespace spot_ros2 {

LocalGridMiddlewareHandle::LocalGridMiddlewareHandle(const std::shared_ptr<rclcpp::Node>& node)
    : node_{node},
      terrain_map_publisher_{node_->create_publisher<grid_map_msgs::msg::GridMap>(
          kTerrainMapTopic, makePublisherQoS(kPublisherHistoryDepth))} {}

LocalGridMiddlewareHandle::LocalGridMiddlewareHandle(const rclcpp::NodeOptions& node_options)
    : LocalGridMiddlewareHandle(std::make_shared<rclcpp::Node>(kNodeName, node_options)) {}

void LocalGridMiddlewareHandle::publishTerrainMap(const grid_map_msgs::msg::GridMap& grid_map) {
  terrain_map_publisher_->publish(grid_map);
}

}  // namespace spot_ros2
