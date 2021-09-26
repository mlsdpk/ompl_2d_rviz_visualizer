/* Copyright 2019 Rover Robotics
 * Copyright 2010 Brian Gerkey
 * Copyright (c) 2008, Willow Garage, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <ORGANIZATION> nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Modified by: Phone Thiha Kyaw, Sai Htet Moe Swe
 */

#pragma once

#include <SDL/SDL_image.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <boost/filesystem.hpp>

#include "ros/ros.h"
#include "yaml-cpp/yaml.h"

namespace ompl_2d_rviz_visualizer_ros {

/**
 * @brief OccupancyGrid data constants
 */
static constexpr int8_t OCC_GRID_UNKNOWN = -1;
static constexpr int8_t OCC_GRID_FREE = 0;
static constexpr int8_t OCC_GRID_OCCUPIED = 100;

enum MapMode { TRINARY, SCALE, RAW };

struct LoadParameters {
  std::string image_file_name;
  double resolution{0};
  std::vector<double> origin{0, 0, 0};
  double free_thresh;
  double occupied_thresh;
  MapMode mode;
  bool negate;
};

class MapLoader {
 public:
  MapLoader(const std::string& map_frame_name,
            const std::string& map_topic_name,
            ros::NodeHandle nh = ros::NodeHandle("~"));

  ~MapLoader();

  bool loadMapFromYaml(const std::string& path_to_yaml,
                       nav_msgs::OccupancyGrid& ogm_map);

 private:
  template <typename T>
  T yamlGetValue(const YAML::Node& node, const std::string& key) {
    try {
      return node[key].as<T>();
    } catch (YAML::Exception& e) {
      std::stringstream ss;
      ss << "Failed to parse YAML tag '" << key << "' for reason: " << e.msg;
      throw YAML::Exception(e.mark, ss.str());
    }
  }

  bool loadMapYaml(LoadParameters& load_parameters,
                   const std::string& path_to_yaml);

  void loadMapFromFile(nav_msgs::OccupancyGrid& map,
                       const LoadParameters& load_parameters);

  const char* mapModeToString(MapMode map_mode);
  MapMode mapModeFromString(std::string map_mode_name);

  ros::NodeHandle nh_;
  ros::Publisher map_pub_;
  std::string map_frame_name_;
  std::string map_topic_name_;
};
}  // namespace ompl_2d_rviz_visualizer_ros
