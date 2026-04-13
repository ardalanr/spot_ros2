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
 * @details The output GridMap contains a single layer named "elevation" with float32 height values in
 * meters. The grid is expressed in the frame given by LocalGrid::frame_name_local_grid_data, prefixed
 * with frame_prefix. Bosdyn stores terrain cells with x (forward) as the inner dimension and y (left)
 * as the outer dimension, which matches grid_map's column-major Eigen storage directly (rows=x, cols=y).
 *
 * Handles both ENCODING_RAW and ENCODING_RLE, and both CELL_FORMAT_FLOAT32 and CELL_FORMAT_INT16.
 * For integer formats, applies: height = cell_value * cell_value_scale + cell_value_offset.
 *
 * @param local_grid A single LocalGrid protobuf from the GetLocalGridsResponse.
 * @param clock_skew Clock skew to apply when converting the acquisition timestamp to local time.
 * @param frame_prefix Prefix to prepend to the grid's reference frame ID.
 * @return A GridMap with an "elevation" layer, or nullopt if the input is invalid or has an
 *         unsupported cell format or encoding.
 */
std::optional<grid_map_msgs::msg::GridMap> getTerrainMap(const bosdyn::api::LocalGrid& local_grid,
                                                         const google::protobuf::Duration& clock_skew,
                                                         const std::string& frame_prefix);

}  // namespace spot_ros2
