// Copyright (c) 2024 Robotics and AI Institute LLC dba RAI Institute. All rights reserved.

#include <spot_driver/api/default_local_grid_client.hpp>

namespace spot_ros2 {

DefaultLocalGridClient::DefaultLocalGridClient(::bosdyn::client::LocalGridClient* client) : client_{client} {}

tl::expected<bosdyn::api::GetLocalGridsResponse, std::string> DefaultLocalGridClient::getLocalGrids(
    const std::vector<std::string>& grid_type_names) {
  const auto result = client_->GetLocalGridsAsync(grid_type_names).get();
  if (!result.status) {
    return tl::make_unexpected("Failed to get local grids: " + result.status.DebugString());
  }
  return result.response;
}

}  // namespace spot_ros2
