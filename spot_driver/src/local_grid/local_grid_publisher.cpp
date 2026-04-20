// Copyright (c) 2024 Robotics and AI Institute LLC dba RAI Institute. All rights reserved.

#include <spot_driver/local_grid/local_grid_publisher.hpp>
#include <spot_driver/conversions/local_grid.hpp>

#include <chrono>
#include <utility>

namespace {
// The string name of the Spot terrain local grid type.
constexpr auto kTerrainGridTypeName = "terrain";
}  // namespace

namespace spot_ros2 {

LocalGridPublisher::LocalGridPublisher(std::shared_ptr<LocalGridClientInterface> local_grid_client,
                                       std::shared_ptr<TimeSyncApi> time_sync_api,
                                       std::unique_ptr<MiddlewareHandle> middleware_handle,
                                       std::unique_ptr<ParameterInterfaceBase> parameter_interface,
                                       std::unique_ptr<LoggerInterfaceBase> logger_interface,
                                       std::unique_ptr<TimerInterfaceBase> timer_interface)
    : local_grid_client_{std::move(local_grid_client)},
      time_sync_api_{std::move(time_sync_api)},
      middleware_handle_{std::move(middleware_handle)},
      parameter_interface_{std::move(parameter_interface)},
      logger_interface_{std::move(logger_interface)},
      timer_interface_{std::move(timer_interface)} {
  frame_prefix_ = parameter_interface_->getFramePrefixWithDefaultFallback();

  const double rate = parameter_interface_->getLocalGridRate();
  const auto period = std::chrono::duration<double>{1.0 / rate};
  timer_interface_->setTimer(period, [this] { timerCallback(); });
}

void LocalGridPublisher::timerCallback() {
  const auto clock_skew_result = time_sync_api_->getClockSkew();
  if (!clock_skew_result) {
    logger_interface_->logError(std::string{"Failed to get clock skew: "}.append(clock_skew_result.error()));
    return;
  }

  const auto response = local_grid_client_->getLocalGrids({kTerrainGridTypeName});
  if (!response) {
    logger_interface_->logError(std::string{"Failed to get local grids: "}.append(response.error()));
    return;
  }

  // Find the terrain grid in the response.
  for (const auto& local_grid_response : response->local_grid_responses()) {
    if (local_grid_response.status() != bosdyn::api::LocalGridResponse_Status_STATUS_OK) {
      logger_interface_->logError("Local grid response status not OK for type: " +
                                  local_grid_response.local_grid().local_grid_type_name());
      continue;
    }

    // One-time diagnostic: log frame_name_local_grid_data and all frames in the transforms_snapshot.
    if (!logged_snapshot_frames_) {
      logged_snapshot_frames_ = true;
      const auto& grid = local_grid_response.local_grid();
      const auto& edge_map = grid.transforms_snapshot().child_to_parent_edge_map();
      logger_interface_->logWarn("frame_name_local_grid_data='" + grid.frame_name_local_grid_data() + "'");
      for (const auto& kv : edge_map) {
        const auto& edge = kv.second;
        std::string msg = "  '" + kv.first + "' -> '" + edge.parent_frame_name() + "'";
        if (edge.has_parent_tform_child()) {
          const auto& p = edge.parent_tform_child().position();
          const auto& r = edge.parent_tform_child().rotation();
          msg += " | t=(" + std::to_string(p.x()) + ", " + std::to_string(p.y()) + ", " + std::to_string(p.z()) + ")";
          msg += " | q=(w=" + std::to_string(r.w()) + ", x=" + std::to_string(r.x()) +
                 ", y=" + std::to_string(r.y()) + ", z=" + std::to_string(r.z()) + ")";
        }
        logger_interface_->logWarn(msg);
      }
    }

    const auto maybe_grid_map = getTerrainMap(local_grid_response.local_grid(), clock_skew_result.value(), frame_prefix_);
    if (!maybe_grid_map) {
      logger_interface_->logError("Failed to convert terrain local grid to GridMap.");
      continue;
    }

    middleware_handle_->publishTerrainMap(maybe_grid_map.value());
  }
}

}  // namespace spot_ros2
