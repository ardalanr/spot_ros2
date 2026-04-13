// Copyright (c) 2024 Robotics and AI Institute LLC dba RAI Institute. All rights reserved.

#include <spot_driver/local_grid/local_grid_publisher_node.hpp>

#include <memory>
#include <utility>

#include <spot_driver/api/default_spot_api.hpp>
#include <spot_driver/interfaces/rclcpp_clock_interface.hpp>
#include <spot_driver/interfaces/rclcpp_logger_interface.hpp>
#include <spot_driver/interfaces/rclcpp_node_interface.hpp>
#include <spot_driver/interfaces/rclcpp_parameter_interface.hpp>
#include <spot_driver/interfaces/rclcpp_wall_timer_interface.hpp>
#include <spot_driver/local_grid/local_grid_middleware_handle.hpp>

namespace {
constexpr auto kDefaultSDKName = "local_grid_publisher_node";
}

namespace spot_ros2 {

LocalGridPublisherNode::LocalGridPublisherNode(std::unique_ptr<NodeInterfaceBase> node_base_interface,
                                               std::unique_ptr<SpotApi> spot_api,
                                               std::unique_ptr<LocalGridPublisher::MiddlewareHandle> middleware_handle,
                                               std::unique_ptr<ParameterInterfaceBase> parameter_interface,
                                               std::unique_ptr<LoggerInterfaceBase> logger_interface,
                                               std::unique_ptr<TimerInterfaceBase> timer_interface,
                                               std::unique_ptr<ClockInterfaceBase> clock_interface)
    : node_base_interface_{std::move(node_base_interface)}, clock_interface_{std::move(clock_interface)} {
  initialize(std::move(spot_api), std::move(middleware_handle), std::move(parameter_interface),
             std::move(logger_interface), std::move(timer_interface));
}

LocalGridPublisherNode::LocalGridPublisherNode(const rclcpp::NodeOptions& node_options) {
  const auto node = std::make_shared<rclcpp::Node>("local_grid_publisher", node_options);
  node_base_interface_ = std::make_unique<RclcppNodeInterface>(node->get_node_base_interface());
  clock_interface_ = std::make_unique<RclcppClockInterface>(node->get_node_clock_interface());
  auto mw_handle = std::make_unique<LocalGridMiddlewareHandle>(node);
  auto parameter_interface = std::make_unique<RclcppParameterInterface>(node);
  auto logger_interface = std::make_unique<RclcppLoggerInterface>(node->get_logger());
  auto timer_interface = std::make_unique<RclcppWallTimerInterface>(node);

  const auto timesync_timeout = parameter_interface->getTimeSyncTimeout();
  auto spot_api =
      std::make_unique<DefaultSpotApi>(kDefaultSDKName, timesync_timeout, parameter_interface->getCertificate());

  initialize(std::move(spot_api), std::move(mw_handle), std::move(parameter_interface), std::move(logger_interface),
             std::move(timer_interface));
}

void LocalGridPublisherNode::initialize(std::unique_ptr<SpotApi> spot_api,
                                        std::unique_ptr<LocalGridPublisher::MiddlewareHandle> middleware_handle,
                                        std::unique_ptr<ParameterInterfaceBase> parameter_interface,
                                        std::unique_ptr<LoggerInterfaceBase> logger_interface,
                                        std::unique_ptr<TimerInterfaceBase> timer_interface) {
  spot_api_ = std::move(spot_api);

  const auto hostname = parameter_interface->getHostname();
  const auto port = parameter_interface->getPort();
  const auto username = parameter_interface->getUsername();
  const auto password = parameter_interface->getPassword();
  const auto frame_prefix = parameter_interface->getFramePrefixWithDefaultFallback();

  if (const auto result = spot_api_->createRobot(hostname, port, frame_prefix); !result) {
    const auto error_msg = std::string{"Failed to create interface to robot: "}.append(result.error());
    logger_interface->logError(error_msg);
    throw std::runtime_error(error_msg);
  }

  if (const auto result = spot_api_->authenticate(username, password); !result) {
    const auto error_msg = std::string{"Failed to authenticate robot: "}.append(result.error());
    logger_interface->logError(error_msg);
    throw std::runtime_error(error_msg);
  }

  internal_ = std::make_unique<LocalGridPublisher>(spot_api_->localGridClientInterface(),
                                                   spot_api_->timeSyncInterface(), std::move(middleware_handle),
                                                   std::move(parameter_interface), std::move(logger_interface),
                                                   std::move(timer_interface));
}

std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> LocalGridPublisherNode::get_node_base_interface() {
  return node_base_interface_->getNodeBaseInterface();
}

std::shared_ptr<rclcpp::Clock> LocalGridPublisherNode::get_clock() {
  return clock_interface_->getClock();
}

}  // namespace spot_ros2
