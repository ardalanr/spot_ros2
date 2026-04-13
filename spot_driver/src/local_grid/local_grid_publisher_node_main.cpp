// Copyright (c) 2024 Robotics and AI Institute LLC dba RAI Institute. All rights reserved.

#include <rclcpp/executors.hpp>
#include <spot_driver/api/spot_clock_sources.hpp>
#include <spot_driver/local_grid/local_grid_publisher_node.hpp>

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  spot_ros2::LocalGridPublisherNode node;

  spot_ros2::SetSpotSDKClockSource(node.get_clock());

  rclcpp::spin(node.get_node_base_interface());

  return 0;
}
