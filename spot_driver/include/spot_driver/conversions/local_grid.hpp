// Copyright (c) 2024 Robotics and AI Institute LLC dba RAI Institute. All rights reserved.

#pragma once

#include <bosdyn/api/local_grid.pb.h>
#include <google/protobuf/duration.pb.h>
#include <grid_map_msgs/msg/grid_map.hpp>

#include <optional>
#include <string>

namespace spot_ros2 {

/**
 * @brief Convert a Spot "terrain" LocalGrid protobuf into a grid_map_msgs::msg::GridMap.
 *
 * @details The output GridMap always contains an "elevation" layer (float32 heights in meters).
 * If terrain_valid is provided, a "terrain_valid" layer is added as a float32 mask (1.0 = known,
 * 0.0 = unknown). Both layers share the same header, info, and column-major layout.
 *
 * The grid is expressed in the vision frame, prefixed with frame_prefix. Bosdyn stores terrain
 * cells with x (forward) as the inner dimension and y (left) as the outer dimension, which
 * matches grid_map's column-major Eigen storage directly (rows=x, cols=y).
 *
 * Handles both ENCODING_RAW and ENCODING_RLE, and CELL_FORMAT_FLOAT32, CELL_FORMAT_INT16, and
 * CELL_FORMAT_UINT8. For INT16, applies: height = cell_value * cell_value_scale + cell_value_offset.
 *
 * @param local_grid A single "terrain" LocalGrid protobuf from the GetLocalGridsResponse.
 * @param clock_skew Clock skew to apply when converting the acquisition timestamp to local time.
 * @param frame_prefix Prefix to prepend to the grid's reference frame ID.
 * @param terrain_valid Optional pointer to the "terrain_valid" LocalGrid (CELL_FORMAT_UINT8 mask).
 *                      If non-null and decodable, a "terrain_valid" layer is appended to the GridMap.
 * @return A GridMap with an "elevation" layer (and optionally "terrain_valid"), or nullopt if the
 *         input is invalid or has an unsupported cell format or encoding.
 */
std::optional<grid_map_msgs::msg::GridMap> getTerrainMap(const bosdyn::api::LocalGrid& local_grid,
                                                         const google::protobuf::Duration& clock_skew,
                                                         const std::string& frame_prefix,
                                                         const bosdyn::api::LocalGrid* terrain_valid = nullptr);

}  // namespace spot_ros2
