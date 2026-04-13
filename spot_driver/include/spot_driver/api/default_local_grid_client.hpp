// Copyright (c) 2024 Robotics and AI Institute LLC dba RAI Institute. All rights reserved.

#pragma once

#include <bosdyn/client/local_grid/local_grid_client.h>
#include <spot_driver/api/local_grid_client_interface.hpp>

namespace spot_ros2 {

/**
 * @brief Production implementation of LocalGridClientInterface that wraps the Bosdyn SDK LocalGridClient.
 */
class DefaultLocalGridClient : public LocalGridClientInterface {
 public:
  /**
   * @brief Construct a DefaultLocalGridClient from a raw Bosdyn SDK client pointer.
   * @param client Raw pointer to a LocalGridClient. The Robot object owns this client's lifetime.
   */
  explicit DefaultLocalGridClient(::bosdyn::client::LocalGridClient* client);

  tl::expected<bosdyn::api::GetLocalGridsResponse, std::string> getLocalGrids(
      const std::vector<std::string>& grid_type_names) override;

 private:
  ::bosdyn::client::LocalGridClient* client_;
};

}  // namespace spot_ros2
