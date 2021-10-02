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

#include <ompl_2d_rviz_visualizer_ros/map_loader.h>

namespace ompl_2d_rviz_visualizer_ros {

MapLoader::MapLoader(const std::string &map_frame_name,
                     const std::string &map_topic_name, ros::NodeHandle nh)
    : map_frame_name_{map_frame_name},
      map_topic_name_{map_topic_name},
      nh_{nh} {
  map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>(map_topic_name, 1, true);
}

MapLoader::~MapLoader() {}

bool MapLoader::loadMapFromYaml(const std::string &path_to_yaml,
                                nav_msgs::OccupancyGrid &map) {
  // first check path is empty or not
  if (path_to_yaml.empty()) {
    std::cerr << "[ERROR] [map_loader]: YAML file name is empty, can't load!"
              << std::endl;
    return false;
  }

  std::cout << "[ INFO] [map_loader]: Loading yaml file: " << path_to_yaml
            << std::endl;

  LoadParameters load_parameters;
  try {
    loadMapYaml(load_parameters, path_to_yaml);
  } catch (YAML::Exception &e) {
    std::cerr << "[ERROR] [map_loader]: Failed processing YAML file "
              << path_to_yaml << " at position (" << e.mark.line << ":"
              << e.mark.column << ") for reason: " << e.what() << std::endl;
    return false;
  } catch (std::exception &e) {
    std::cerr
        << "[ERROR] [map_loader]: Failed to parse map YAML loaded from file "
        << path_to_yaml << " for reason: " << e.what() << std::endl;
    return false;
  }

  try {
    loadMapFromFile(map, load_parameters);
  } catch (std::exception &e) {
    std::cerr << "[ERROR] [map_io]: Failed to load image file "
              << load_parameters.image_file_name << " for reason: " << e.what()
              << std::endl;
    return false;
  }

  return true;
}

bool MapLoader::loadMapYaml(LoadParameters &load_parameters,
                            const std::string &path_to_yaml) {
  YAML::Node doc = YAML::LoadFile(path_to_yaml);

  auto image_file_name = yamlGetValue<std::string>(doc, "image");
  if (image_file_name.empty()) {
    throw YAML::Exception(doc["image"].Mark(), "The image tag was empty.");
  }

  boost::filesystem::path mapfpath(image_file_name);
  if (!mapfpath.is_absolute()) {
    boost::filesystem::path dir(path_to_yaml);
    dir = dir.parent_path();
    mapfpath = dir / mapfpath;
    image_file_name = mapfpath.string();
  }

  load_parameters.image_file_name = image_file_name;

  load_parameters.resolution = yamlGetValue<double>(doc, "resolution");
  load_parameters.origin = yamlGetValue<std::vector<double>>(doc, "origin");
  if (load_parameters.origin.size() != 3) {
    throw YAML::Exception(
        doc["origin"].Mark(),
        "value of the 'origin' tag should have 3 elements, not " +
            std::to_string(load_parameters.origin.size()));
  }

  load_parameters.free_thresh = yamlGetValue<double>(doc, "free_thresh");
  load_parameters.occupied_thresh =
      yamlGetValue<double>(doc, "occupied_thresh");

  auto map_mode_node = doc["mode"];
  if (!map_mode_node.IsDefined()) {
    load_parameters.mode = MapMode::TRINARY;
  } else {
    load_parameters.mode = mapModeFromString(map_mode_node.as<std::string>());
  }

  try {
    load_parameters.negate = yamlGetValue<int>(doc, "negate");
  } catch (YAML::Exception &) {
    load_parameters.negate = yamlGetValue<bool>(doc, "negate");
  }

#if MAPLOADER_DEBUG
  std::cout << "[DEBUG] [map_loader]: resolution: "
            << load_parameters.resolution << std::endl;
  std::cout << "[DEBUG] [map_loader]: origin[0]: " << load_parameters.origin[0]
            << std::endl;
  std::cout << "[DEBUG] [map_loader]: origin[1]: " << load_parameters.origin[1]
            << std::endl;
  std::cout << "[DEBUG] [map_loader]: origin[2]: " << load_parameters.origin[2]
            << std::endl;
  std::cout << "[DEBUG] [map_loader]: free_thresh: "
            << load_parameters.free_thresh << std::endl;
  std::cout << "[DEBUG] [map_loader]: occupied_thresh: "
            << load_parameters.occupied_thresh << std::endl;
  std::cout << "[DEBUG] [map_loader]: mode: "
            << mapModeToString(load_parameters.mode) << std::endl;
  std::cout << "[DEBUG] [map_loader]: negate: " << load_parameters.negate
            << std::endl;  // NOLINT
#endif

  return true;
}

void MapLoader::loadMapFromFile(nav_msgs::OccupancyGrid &map,
                                const LoadParameters &load_parameters) {
  SDL_Surface *img;

  // Load the image using SDL. If we get NULL back, the image load failed.
  if (!(img = IMG_Load(load_parameters.image_file_name.c_str()))) {
    std::string errmsg = std::string("failed to open image file \"") +
                         std::string(load_parameters.image_file_name.c_str()) +
                         std::string("\": ") + IMG_GetError();
    throw std::runtime_error(errmsg);
  }

  nav_msgs::OccupancyGrid msg;
  msg.info.width = img->w;
  msg.info.height = img->h;
  msg.info.resolution = static_cast<float>(load_parameters.resolution);
  msg.info.origin.position.x = load_parameters.origin[0];
  msg.info.origin.position.y = load_parameters.origin[1];
  msg.info.origin.position.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0, 0, load_parameters.origin[2]);
  msg.info.origin.orientation = tf2::toMsg(q);

  // Allocate space to hold the data
  msg.data.resize(msg.info.width * msg.info.height);

  // Get values that we'll need to iterate through the pixels
  unsigned int rowstride, n_channels, avg_channels;
  rowstride = img->pitch;
  n_channels = img->format->BytesPerPixel;

  // NOTE: Trinary mode still overrides here to preserve existing behavior.
  // Alpha will be averaged in with color channels when using trinary mode.
  if (load_parameters.mode == MapMode::TRINARY || !img->format->Amask)
    avg_channels = n_channels;
  else
    avg_channels = n_channels - 1;

  // Copy pixel data into the map structure
  unsigned char *pixels, *p;
  int color_sum, alpha;
  double occ;
  pixels = (unsigned char *)(img->pixels);
  for (auto j = 0; j < msg.info.height; j++) {
    for (auto i = 0; i < msg.info.width; i++) {
      // Compute mean of RGB for this pixel
      p = pixels + j * rowstride + i * n_channels;
      color_sum = 0;
      for (auto k = 0; k < avg_channels; k++) color_sum += *(p + (k));
      occ = color_sum / static_cast<double>(avg_channels);

      if (n_channels == 1)
        alpha = 1;
      else
        alpha = *(p + n_channels - 1);

      if (load_parameters.negate) occ = 255.0 - occ;

      int8_t map_cell;
      switch (load_parameters.mode) {
        case MapMode::RAW:
          if (OCC_GRID_FREE <= occ && occ <= OCC_GRID_OCCUPIED) {
            map_cell = static_cast<int8_t>(occ);
          } else {
            map_cell = OCC_GRID_UNKNOWN;
          }
          break;

        case MapMode::TRINARY:
          // If negate is true, we consider blacker pixels free, and whiter
          // pixels occupied.  Otherwise, it's vice versa.
          occ = (255 - occ) / 255.0;
          if (load_parameters.occupied_thresh < occ) {
            map_cell = OCC_GRID_OCCUPIED;
          } else if (occ < load_parameters.free_thresh) {
            map_cell = OCC_GRID_FREE;
          } else {
            map_cell = OCC_GRID_UNKNOWN;
          }
          break;

        case MapMode::SCALE:
          occ = (255 - occ) / 255.0;
          if (alpha < 1.0) {
            map_cell = OCC_GRID_UNKNOWN;
          } else if (load_parameters.occupied_thresh < occ) {
            map_cell = OCC_GRID_OCCUPIED;
          } else if (occ < load_parameters.free_thresh) {
            map_cell = OCC_GRID_FREE;
          } else {
            map_cell = std::rint((occ - load_parameters.free_thresh) /
                                 (load_parameters.occupied_thresh -
                                  load_parameters.free_thresh) *
                                 100.0);
          }
          break;

        default:
          throw std::runtime_error("Invalid map mode");
      }
      msg.data[msg.info.width * (msg.info.height - j - 1) + i] = map_cell;
    }
  }
  SDL_FreeSurface(img);

  ros::Time::waitForValid();
  msg.info.map_load_time = ros::Time::now();
  msg.header.frame_id = map_frame_name_;
  msg.header.stamp = ros::Time::now();

#if MAPLOADER_DEBUG
  std::cout << "[DEBUG] [map_loader]: Read map "
            << load_parameters.image_file_name << ": " << msg.info.width
            << " X " << msg.info.height << " map @ " << msg.info.resolution
            << " m/cell" << std::endl;
#endif

  map = msg;
  map_pub_.publish(msg);
}

const char *MapLoader::mapModeToString(MapMode map_mode) {
  switch (map_mode) {
    case MapMode::TRINARY:
      return "trinary";
    case MapMode::SCALE:
      return "scale";
    case MapMode::RAW:
      return "raw";
    default:
      throw std::invalid_argument("map_mode");
  }
}

MapMode MapLoader::mapModeFromString(std::string map_mode_name) {
  for (auto &c : map_mode_name) {
    c = tolower(c);
  }

  if (map_mode_name == "scale") {
    return MapMode::SCALE;
  } else if (map_mode_name == "raw") {
    return MapMode::RAW;
  } else if (map_mode_name == "trinary") {
    return MapMode::TRINARY;
  } else {
    throw std::invalid_argument("map_mode_name");
  }
}

}  // namespace ompl_2d_rviz_visualizer_ros