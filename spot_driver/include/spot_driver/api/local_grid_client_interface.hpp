// Copyright (c) 2024 Robotics and AI Institute LLC dba RAI Institute. All rights reserved.

#pragma once

#include <bosdyn/api/local_grid.pb.h>
#include <tl_expected/expected.hpp>

#include <string>
#include <vector>

namespace spot_ros2 {

/**
 * @brief Interface class to interact with Spot SDK's Local Grid Client.
 */
class LocalGridClientInterface {
 public:
  // LocalGridClientInterface is move-only
  LocalGridClientInterface() = default;
  LocalGridClientInterface(LocalGridClientInterface&&) = default;
  LocalGridClientInterface(const LocalGridClientInterface&) = delete;
  LocalGridClientInterface& operator=(LocalGridClientInterface&&) = default;
  LocalGridClientInterface& operator=(const LocalGridClientInterface&) = delete;

  virtual ~LocalGridClientInterface() = default;

  /**
   * @brief Request a set of local grids by type name.
   * @param grid_type_names List of grid type name strings to request (e.g. {"terrain"}).
   * @return Returns an expected containing the GetLocalGridsResponse if successful, or an error string on failure.
   */
  virtual tl::expected<bosdyn::api::GetLocalGridsResponse, std::string> getLocalGrids(
      const std::vector<std::string>& grid_type_names) = 0;
};

}  // namespace spot_ros2
