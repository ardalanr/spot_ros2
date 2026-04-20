// Copyright (c) 2024 Robotics and AI Institute LLC dba RAI Institute. All rights reserved.

#include <spot_driver/conversions/local_grid.hpp>
#include <spot_driver/conversions/time.hpp>

#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/multi_array_dimension.hpp>
#include <std_msgs/msg/multi_array_layout.hpp>

#include <cstdint>
#include <cstring>
#include <cmath>

namespace spot_ros2 {

namespace {

// grid_map stores data in column-major Eigen matrices (Eigen's default).
// The conventions are: rows = x direction (forward), columns = y direction (left).
// The MultiArray layout labels used by grid_map_ros to identify column-major storage:
constexpr auto kOuterDimLabel = "column_index";  // outer dim = columns (y direction)
constexpr auto kInnerDimLabel = "row_index";     // inner dim = rows    (x direction)
constexpr auto kTargetFrame = "vision";

// Minimal SE3 math for composing frame transforms from the FrameTreeSnapshot.
struct Vec3d {
  double x, y, z;
};

struct Quatd {
  double w, x, y, z;
};

// Hamilton quaternion product.
Quatd quatMul(const Quatd& a, const Quatd& b) {
  return {a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
          a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
          a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
          a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w};
}

// Rotate a 3D point by a unit quaternion: q * p * q^-1.
Vec3d rotatePoint(const Quatd& q, const Vec3d& p) {
  const Quatd pq{0.0, p.x, p.y, p.z};
  const Quatd qconj{q.w, -q.x, -q.y, -q.z};
  const Quatd result = quatMul(quatMul(q, pq), qconj);
  return {result.x, result.y, result.z};
}

struct SE3d {
  Vec3d t;  // translation
  Quatd r;  // rotation (unit quaternion)
};

// Compose two SE3 transforms: T_A_C = T_A_B * T_B_C.
SE3d composeSE3(const SE3d& T_AB, const SE3d& T_BC) {
  const Vec3d rotated = rotatePoint(T_AB.r, T_BC.t);
  return {{T_AB.t.x + rotated.x, T_AB.t.y + rotated.y, T_AB.t.z + rotated.z}, quatMul(T_AB.r, T_BC.r)};
}

// Inverse of an SE3 transform (valid only for unit-quaternion rotation).
SE3d inverseSE3(const SE3d& T) {
  const Quatd r_inv{T.r.w, -T.r.x, -T.r.y, -T.r.z};
  const Vec3d t_inv = rotatePoint(r_inv, {-T.t.x, -T.t.y, -T.t.z});
  return {t_inv, r_inv};
}

using EdgeMap = google::protobuf::Map<std::string, bosdyn::api::FrameTreeSnapshot_ParentEdge>;

struct FramePath {
  SE3d T_root_frame;   // transform: root → start_frame
  std::string root_name;
};

/**
 * @brief Walk from start_frame up to the tree root (the frame with an empty parent), accumulating
 * T_root_start along the way.
 *
 * In the Bosdyn local grid FrameTreeSnapshot the root is typically "body", with "odom" and "vision"
 * as children of the root — not ancestors. Walking purely up the parent chain from
 * terrain_local_grid_corner therefore never reaches "odom" directly.
 */
constexpr int kMaxDepth = 32;

std::optional<FramePath> walkToRoot(const EdgeMap& edge_map, const std::string& start_frame) {
  SE3d accumulated{{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0, 0.0}};
  std::string current = start_frame;

  for (int depth = 0; depth < kMaxDepth; ++depth) {
    const auto it = edge_map.find(current);
    if (it == edge_map.end()) {
      return std::nullopt;
    }
    const auto& edge = it->second;
    if (edge.parent_frame_name().empty()) {
      // current is the root
      return FramePath{accumulated, current};
    }
    if (!edge.has_parent_tform_child()) {
      return std::nullopt;
    }
    const auto& pose = edge.parent_tform_child();
    const SE3d T_parent_current{
        {pose.position().x(), pose.position().y(), pose.position().z()},
        {pose.rotation().w(), pose.rotation().x(), pose.rotation().y(), pose.rotation().z()}};
    accumulated = composeSE3(T_parent_current, accumulated);
    current = edge.parent_frame_name();
  }
  return std::nullopt;
}

/**
 * @brief Compute T_target_child using two paths through the common tree root.
 *
 * The Bosdyn snapshot for local grids uses "body" as the root, with both "odom" and
 * "terrain_local_grid_corner" as descendants (in different branches). To get T_odom_corner we:
 *   1. Walk corner → root to get T_root_corner.
 *   2. Walk odom → root to get T_root_odom.
 *   3. T_odom_corner = inv(T_root_odom) * T_root_corner.
 *
 * @return {T_target_child, target_frame_name}. Falls back to root frame if target_frame is not
 *         reachable. Returns nullopt if child_frame is not found at all.
 */
std::optional<std::pair<SE3d, std::string>> computeTargetTChild(
    const bosdyn::api::FrameTreeSnapshot& snapshot, const std::string& child_frame,
    const std::string& target_frame) {
  const auto& edge_map = snapshot.child_to_parent_edge_map();

  const auto child_path = walkToRoot(edge_map, child_frame);
  if (!child_path) {
    return std::nullopt;
  }

  // If target IS the root, we already have T_root_child = T_target_child.
  if (child_path->root_name == target_frame) {
    return std::make_pair(child_path->T_root_frame, target_frame);
  }

  // Walk from target up to root: gives T_root_target.
  const auto target_path = walkToRoot(edge_map, target_frame);
  if (!target_path || target_path->root_name != child_path->root_name) {
    // target not reachable from the same root — fall back to root frame
    return std::make_pair(child_path->T_root_frame, child_path->root_name);
  }

  // T_target_child = inv(T_root_target) * T_root_child
  const SE3d T_target_child = composeSE3(inverseSE3(target_path->T_root_frame), child_path->T_root_frame);
  return std::make_pair(T_target_child, target_frame);
}

/**
 * @brief Decode a Bosdyn local grid's raw byte buffer into float32 heights.
 *
 * @details Bosdyn stores cells with x (forward) as the inner dimension and y (left) as the outer
 * dimension: data[y_idx * num_cells_x + x_idx]. This matches grid_map's column-major storage
 * (data[col * num_rows + row] = data[y_idx * num_cells_x + x_idx]), so the decoded values can be
 * copied directly without reordering.
 *
 * @param local_grid Input LocalGrid protobuf.
 * @param out Output vector of float32 height values, length == num_cells_x * num_cells_y.
 * @return true on success, false if the cell format or encoding is unsupported.
 */
bool decodeGridData(const bosdyn::api::LocalGrid& local_grid, std::vector<float>& out) {
  const int num_cells_x = local_grid.extent().num_cells_x();
  const int num_cells_y = local_grid.extent().num_cells_y();
  const int total_cells = num_cells_x * num_cells_y;
  const auto& raw = local_grid.data();
  const auto encoding = local_grid.encoding();
  const auto cell_format = local_grid.cell_format();

  // Determine byte stride per cell and whether to apply scale/offset.
  size_t byte_stride = 0;
  bool is_float32 = false;
  switch (cell_format) {
    case bosdyn::api::LocalGrid_CellFormat_CELL_FORMAT_FLOAT32:
      byte_stride = sizeof(float);
      is_float32 = true;
      break;
    case bosdyn::api::LocalGrid_CellFormat_CELL_FORMAT_INT16:
      byte_stride = sizeof(int16_t);
      break;
    default:
      return false;  // unsupported format
  }

  const float scale = static_cast<float>(local_grid.cell_value_scale());
  const float offset = static_cast<float>(local_grid.cell_value_offset());

  out.clear();
  out.reserve(total_cells);

  auto decode_value = [&](const char* src) -> float {
    if (is_float32) {
      float v;
      std::memcpy(&v, src, sizeof(float));
      return v;
    } else {
      int16_t v;
      std::memcpy(&v, src, sizeof(int16_t));
      return static_cast<float>(v) * scale + offset;
    }
  };

  if (encoding == bosdyn::api::LocalGrid_Encoding_ENCODING_RAW) {
    if (static_cast<int>(raw.size()) < total_cells * static_cast<int>(byte_stride)) {
      return false;
    }
    for (int i = 0; i < total_cells; ++i) {
      out.push_back(decode_value(raw.data() + i * byte_stride));
    }
  } else if (encoding == bosdyn::api::LocalGrid_Encoding_ENCODING_RLE) {
    for (int run = 0; run < local_grid.rle_counts_size(); ++run) {
      const size_t byte_offset = static_cast<size_t>(run) * byte_stride;
      if (byte_offset + byte_stride > raw.size()) {
        return false;
      }
      const float height = decode_value(raw.data() + byte_offset);
      const int count = local_grid.rle_counts(run);
      for (int k = 0; k < count; ++k) {
        out.push_back(height);
      }
    }
  } else {
    return false;
  }

  return static_cast<int>(out.size()) == total_cells;
}

}  // namespace

std::optional<grid_map_msgs::msg::GridMap> getTerrainMap(const bosdyn::api::LocalGrid& local_grid,
                                                         const google::protobuf::Duration& clock_skew,
                                                         const std::string& frame_prefix) {
  const int num_cells_x = local_grid.extent().num_cells_x();
  const int num_cells_y = local_grid.extent().num_cells_y();
  const double cell_size = local_grid.extent().cell_size();

  if (num_cells_x <= 0 || num_cells_y <= 0 || cell_size <= 0.0) {
    return std::nullopt;
  }

  std::vector<float> heights;
  if (!decodeGridData(local_grid, heights)) {
    return std::nullopt;
  }

  // Bosdyn stores data[y * num_x + x] with (x=0, y=0) at the minimum-corner cell.
  // grid_map's column-major layout places index 0 at the maximum-(x,y) corner.
  // Reversing the flat array corrects both axes simultaneously.
  std::reverse(heights.begin(), heights.end());

  grid_map_msgs::msg::GridMap grid_map;

  // Compute the grid center in odom frame.
  // frame_name_local_grid_data origin is at the grid corner (minimum x,y cell center).
  // The center is offset by half the grid extent in the +x and +y directions of the corner frame.
  const double length_x = num_cells_x * cell_size;
  const double length_y = num_cells_y * cell_size;

  // The Bosdyn snapshot for local grids uses "body" as the root, with "odom" and the corner frame
  // in different branches. computeTargetTChild handles the two-path traversal through the root.
  const auto transform_result = computeTargetTChild(local_grid.transforms_snapshot(),
                                                    local_grid.frame_name_local_grid_data(), kTargetFrame);
  if (!transform_result) {
    return std::nullopt;
  }

  const auto& [T_target_corner, target_frame_name] = *transform_result;

  const Vec3d center_in_corner{length_x / 2.0, length_y / 2.0, 0.0};
  const Vec3d center_rotated = rotatePoint(T_target_corner.r, center_in_corner);
  const Vec3d center_in_target{T_target_corner.t.x + center_rotated.x, T_target_corner.t.y + center_rotated.y,
                                T_target_corner.t.z + center_rotated.z};

  // Header: frame and timestamp.
  grid_map.header.frame_id = frame_prefix + target_frame_name;
  grid_map.header.stamp = robotTimeToLocalTime(local_grid.acquisition_time(), clock_skew);

  // Info: resolution, map size, and pose of the map center in the target frame.
  grid_map.info.resolution = cell_size;
  grid_map.info.length_x = length_x;
  grid_map.info.length_y = length_y;
  grid_map.info.pose.position.x = center_in_target.x;
  grid_map.info.pose.position.y = center_in_target.y;
  grid_map.info.pose.position.z = center_in_target.z;
  grid_map.info.pose.orientation.w = T_target_corner.r.w;
  grid_map.info.pose.orientation.x = T_target_corner.r.x;
  grid_map.info.pose.orientation.y = T_target_corner.r.y;
  grid_map.info.pose.orientation.z = T_target_corner.r.z;

  // Layer metadata.
  grid_map.layers = {"elevation"};
  grid_map.basic_layers = {"elevation"};
  grid_map.outer_start_index = 0;
  grid_map.inner_start_index = 0;

  // Data: one Float32MultiArray per layer, in grid_map column-major layout.
  // dim[0] = outer = columns (y direction), dim[1] = inner = rows (x direction).
  // Bosdyn's data[y * num_x + x] maps directly to grid_map's data[col * nRows + row].
  std_msgs::msg::Float32MultiArray layer;
  layer.layout.dim.resize(2);
  layer.layout.dim[0].label = kOuterDimLabel;
  layer.layout.dim[0].size = static_cast<uint32_t>(num_cells_y);
  layer.layout.dim[0].stride = static_cast<uint32_t>(num_cells_y * num_cells_x);
  layer.layout.dim[1].label = kInnerDimLabel;
  layer.layout.dim[1].size = static_cast<uint32_t>(num_cells_x);
  layer.layout.dim[1].stride = static_cast<uint32_t>(num_cells_x);
  layer.data = std::move(heights);

  grid_map.data.push_back(std::move(layer));

  return grid_map;
}

}  // namespace spot_ros2
