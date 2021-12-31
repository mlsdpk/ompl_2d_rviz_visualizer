/******************************************************************************
  BSD 2-Clause License

  Copyright (c) 2021, Phone Thiha Kyaw
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:

  1. Redistributions of source code must retain the above copyright notice, this
    list of conditions and the following disclaimer.

  2. Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#pragma once

#define MAP_UTILS_DEBUG 0

#include <nav_msgs/OccupancyGrid.h>

namespace ompl_2d_rviz_visualizer_ros
{
/**
 * @brief OccupancyGrid data constants
 */
static constexpr int8_t OCC_GRID_UNKNOWN = -1;
static constexpr int8_t OCC_GRID_FREE = 0;
static constexpr int8_t OCC_GRID_OCCUPIED = 100;

namespace map_utils
{
/**
 * @brief Generate realvector bounds of an 2D occupancy grid map
 * @param min_x Lower map bound in x-axis
 * @param max_x Higher map bound in x-axis
 * @param min_y Lower map bound in y-axis
 * @param max_y Higher map bound in y-axis
 * @return true if can generate bounds
 */
inline bool getBounds(double& min_x, double& max_x, double& min_y, double& max_y, const nav_msgs::OccupancyGrid& ogm)
{
  // extract map parameters
  unsigned int cells_size_x = ogm.info.width;
  unsigned int cells_size_y = ogm.info.height;
  double resolution = static_cast<double>(ogm.info.resolution);
  double origin_x = ogm.info.origin.position.x;
  double origin_y = ogm.info.origin.position.y;

  double map_size_x = cells_size_x * resolution;
  double map_size_y = cells_size_y * resolution;

  min_x = origin_x;
  min_y = origin_y;
  max_x = map_size_x - fabs(origin_x);
  max_y = map_size_y - fabs(origin_y);

#if MAP_UTILS_DEBUG
  std::cout << "[DEBUG] [map_utils] Map size in meters: " << map_size_x << " X " << map_size_y << std::endl;
  std::cout << "[DEBUG] [map_utils] Map bounds (lower-left): " << min_x << ", " << min_y << std::endl;
  std::cout << "[DEBUG] [map_utils] Map bounds (upper-right): " << max_x << ", " << max_y << std::endl;
#endif

  return true;
}

}  // namespace map_utils
}  // namespace ompl_2d_rviz_visualizer_ros