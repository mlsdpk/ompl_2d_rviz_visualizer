#pragma once

#include "ros/ros.h"
#include "ros/console.h"

#include "nav_msgs/GetMap.h"
#include <nav_msgs/OccupancyGrid.h>
#include <fstream>
#include "yaml-cpp/yaml.h"


#include <cstring>
#include <stdexcept>

#include <stdlib.h>
#include <stdio.h>

#include <SDL/SDL_image.h>
#include <LinearMath/btQuaternion.h>
#include <boost/filesystem.hpp>

enum MapMode {TRINARY, SCALE, RAW};
// compute linear index for given map coords
#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))


namespace ompl_2d_rviz_visualizer_ros {
class MapLoader {
  public:

    MapLoader(ros::NodeHandle nh = ros::NodeHandle("~"));

    ~MapLoader();

    bool loadMapFromYaml(const std::string &path_to_yaml, nav_msgs::OccupancyGrid &map);

    

  private:

    //Publishers
    // ros::NodeHandle nh_;
    // ros::Publisher map_pub_;
    // ros::Publisher metadata_pub_;

    // // // Latched publisher for metadata
    // ros::Publisher metadata_pub_ = nh_.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);

    // // // Latched publisher for data
    // ros::Publisher map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("map", 1, true);


    void loadMapFromFile(nav_msgs::GetMap::Response* resp,
                     const char* fname, double res, bool negate,
                     double occ_th, double free_th, double* origin,
                     MapMode mode=TRINARY);
    bool loadMapFromValues(const std::string &map_file_name, double resolution,
                           int negate, double occ_th, double free_th,
                           double origin[3], MapMode mode);

    ros::NodeHandle nh_;
    ros::Publisher map_pub_;
    ros::Publisher metadata_pub_;

    nav_msgs::GetMap::Response map_resp_;
    nav_msgs::MapMetaData meta_data_message_;

	};
}
