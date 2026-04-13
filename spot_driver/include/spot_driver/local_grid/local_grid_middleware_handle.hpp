// Copyright (c) 2024 Robotics and AI Institute LLC dba RAI Institute. All rights reserved.

#pragma once

#include <grid_map_msgs/msg/grid_map.hpp>
#include <memory>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <spot_driver/local_grid/local_grid_publisher.hpp>

namespace spot_ros2 {

/**
 * @brief Production implementation of LocalGridPublisher::MiddlewareHandle.
 */
class LocalGridMiddlewareHandle : public LocalGridPublisher::MiddlewareHandle {
 public:
  /**
   * @brief Construct from an existing rclcpp::Node.
   */
  explicit LocalGridMiddlewareHandle(const std::shared_ptr<rclcpp::Node>& node);

  /**
   * @brief Construct by creating a new rclcpp::Node from NodeOptions.
   */
  explicit LocalGridMiddlewareHandle(const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions{});

  ~LocalGridMiddlewareHandle() override = default;

  void publishTerrainMap(const grid_map_msgs::msg::GridMap& grid_map) override;

 private:
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<rclcpp::Publisher<grid_map_msgs::msg::GridMap>> terrain_map_publisher_;
};

}  // namespace spot_ros2
