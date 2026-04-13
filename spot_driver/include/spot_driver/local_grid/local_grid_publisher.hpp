// Copyright (c) 2024 Robotics and AI Institute LLC dba RAI Institute. All rights reserved.

#pragma once

#include <grid_map_msgs/msg/grid_map.hpp>
#include <memory>
#include <string>

#include <spot_driver/api/local_grid_client_interface.hpp>
#include <spot_driver/api/middleware_handle_base.hpp>
#include <spot_driver/api/time_sync_api.hpp>
#include <spot_driver/interfaces/logger_interface_base.hpp>
#include <spot_driver/interfaces/parameter_interface_base.hpp>
#include <spot_driver/interfaces/timer_interface_base.hpp>

namespace spot_ros2 {

/**
 * @brief Requests the "terrain" local grid from Spot, converts it to a GridMap, and publishes it.
 */
class LocalGridPublisher {
 public:
  /**
   * @brief Handle that enables dependency injection of ROS and rclcpp::Node operations.
   */
  class MiddlewareHandle : public MiddlewareHandleBase {
   public:
    virtual ~MiddlewareHandle() = default;
    /**
     * @brief Publish the terrain elevation GridMap.
     */
    virtual void publishTerrainMap(const grid_map_msgs::msg::GridMap& grid_map) = 0;
  };

  /**
   * @brief Construct a LocalGridPublisher.
   *
   * @param local_grid_client Interface to retrieve local grid data from the Spot SDK.
   * @param time_sync_api Converts robot timestamps to local time.
   * @param middleware_handle Owns and invokes the ROS publisher.
   * @param parameter_interface Supplies the publish rate and frame prefix.
   * @param logger_interface Logs errors encountered during polling.
   * @param timer_interface Drives periodic polling via timerCallback().
   */
  LocalGridPublisher(std::shared_ptr<LocalGridClientInterface> local_grid_client,
                     std::shared_ptr<TimeSyncApi> time_sync_api,
                     std::unique_ptr<MiddlewareHandle> middleware_handle,
                     std::unique_ptr<ParameterInterfaceBase> parameter_interface,
                     std::unique_ptr<LoggerInterfaceBase> logger_interface,
                     std::unique_ptr<TimerInterfaceBase> timer_interface);

 private:
  void timerCallback();

  std::string frame_prefix_;

  std::shared_ptr<LocalGridClientInterface> local_grid_client_;
  std::shared_ptr<TimeSyncApi> time_sync_api_;
  std::unique_ptr<MiddlewareHandle> middleware_handle_;
  std::unique_ptr<ParameterInterfaceBase> parameter_interface_;
  std::unique_ptr<LoggerInterfaceBase> logger_interface_;
  std::unique_ptr<TimerInterfaceBase> timer_interface_;
};

}  // namespace spot_ros2
