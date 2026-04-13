// Copyright (c) 2024 Robotics and AI Institute LLC dba RAI Institute. All rights reserved.

#pragma once

#include <memory>
#include <rclcpp/node_interfaces/node_base_interface.hpp>
#include <rclcpp/node_options.hpp>
#include <spot_driver/api/spot_api.hpp>
#include <spot_driver/interfaces/clock_interface_base.hpp>
#include <spot_driver/interfaces/logger_interface_base.hpp>
#include <spot_driver/interfaces/node_interface_base.hpp>
#include <spot_driver/interfaces/parameter_interface_base.hpp>
#include <spot_driver/interfaces/timer_interface_base.hpp>
#include <spot_driver/local_grid/local_grid_publisher.hpp>

namespace spot_ros2 {

/**
 * @brief Wraps LocalGridPublisher to allow using it as an rclcpp::Node or composable component.
 */
class LocalGridPublisherNode {
 public:
  /**
   * @brief Constructor for dependency-injected use (e.g. in tests).
   */
  LocalGridPublisherNode(std::unique_ptr<NodeInterfaceBase> node_base_interface, std::unique_ptr<SpotApi> spot_api,
                         std::unique_ptr<LocalGridPublisher::MiddlewareHandle> middleware_handle,
                         std::unique_ptr<ParameterInterfaceBase> parameter_interface,
                         std::unique_ptr<LoggerInterfaceBase> logger_interface,
                         std::unique_ptr<TimerInterfaceBase> timer_interface,
                         std::unique_ptr<ClockInterfaceBase> clock_interface);

  /**
   * @brief Constructor that creates the rclcpp::Node and all rclcpp-specialized interfaces.
   */
  explicit LocalGridPublisherNode(const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions{});

  std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> get_node_base_interface();
  std::shared_ptr<rclcpp::Clock> get_clock();

 private:
  void initialize(std::unique_ptr<SpotApi> spot_api,
                  std::unique_ptr<LocalGridPublisher::MiddlewareHandle> middleware_handle,
                  std::unique_ptr<ParameterInterfaceBase> parameter_interface,
                  std::unique_ptr<LoggerInterfaceBase> logger_interface,
                  std::unique_ptr<TimerInterfaceBase> timer_interface);

  std::unique_ptr<NodeInterfaceBase> node_base_interface_;
  std::unique_ptr<ClockInterfaceBase> clock_interface_;
  std::unique_ptr<SpotApi> spot_api_;
  std::unique_ptr<LocalGridPublisher> internal_;
};

}  // namespace spot_ros2
