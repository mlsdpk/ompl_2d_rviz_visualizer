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

#include <nav_msgs/OccupancyGrid.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/config.h>
#include <ompl_2d_rviz_visualizer_ros/map_utils.h>

namespace ob = ompl::base;

namespace ompl_2d_rviz_visualizer_ros
{
class CollisionChecker
{
public:
  CollisionChecker(std::shared_ptr<nav_msgs::OccupancyGrid> ogm_map) : map_{ std::move(ogm_map) }
  {
    if (!map_utils::getBounds(min_bound_x_, max_bound_x_, min_bound_y_, max_bound_y_, *map_))
    {
      std::cerr << "Fail to generate bounds in the occupancy grid map." << std::endl;
      exit(1);
    }

    map_cell_size_x_ = map_->info.width;
    map_cell_size_y_ = map_->info.height;
    map_resolution_ = map_->info.resolution;
    map_origin_x_ = map_->info.origin.position.x;
    map_origin_y_ = map_->info.origin.position.y;
  }

  ~CollisionChecker();

  bool isValid(const ob::State* state)
  {
    if (!state)
    {
      throw std::runtime_error("No state found for vertex");
    }

    // Convert to RealVectorStateSpace
    const ob::RealVectorStateSpace::StateType* real_state =
        static_cast<const ob::RealVectorStateSpace::StateType*>(state);

    double x = real_state->values[0];
    double y = real_state->values[1];

    return isValid(x, y);
  }

  bool isValid(double x, double y)
  {
    // find cell index in the map
    unsigned int idx = mapToCellIndex(x, y);
    auto occ = map_->data[idx];

    if (occ == OCC_GRID_FREE)
      return true;
    return false;
  }

private:
  unsigned int mapToCellIndex(double x, double y)
  {
    // check if the state is within bounds
    if (!(x >= min_bound_x_ && x <= max_bound_x_ && y >= min_bound_y_ && y <= max_bound_y_))
    {
      throw std::runtime_error("State must be within the bounds.");
    }

    int cell_x = (int)((x - map_origin_x_) / map_resolution_);
    int cell_y = (int)((y - map_origin_y_) / map_resolution_);

    unsigned int idx = cell_y * map_cell_size_x_ + cell_x;
    return idx;
  }

  std::shared_ptr<nav_msgs::OccupancyGrid> map_;

  double min_bound_x_;
  double min_bound_y_;
  double max_bound_x_;
  double max_bound_y_;

  unsigned int map_cell_size_x_;
  unsigned int map_cell_size_y_;
  double map_resolution_;
  double map_origin_x_;
  double map_origin_y_;
};

}  // namespace ompl_2d_rviz_visualizer_ros