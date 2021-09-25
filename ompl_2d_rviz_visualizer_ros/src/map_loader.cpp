#include <ompl_2d_rviz_visualizer_ros/map_loader.h>

namespace ompl_2d_rviz_visualizer_ros {

MapLoader::MapLoader(ros::NodeHandle nh): nh_{nh}
{ 
  metadata_pub_ = nh_.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
  map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
}

MapLoader::~MapLoader(){}


template<typename T>
T yaml_get_value(const YAML::Node & node, const std::string & key)
{
  try {
    return node[key].as<T>();
  } catch (YAML::Exception & e) {
    std::stringstream ss;
    ss << "Failed to parse YAML tag '" << key << "' for reason: " << e.msg;
    throw YAML::Exception(e.mark, ss.str());
  }
}

bool MapLoader::loadMapFromYaml(const std::string &path_to_yaml, nav_msgs::OccupancyGrid &map)
    {
      std::cout<<"Print 1"<<std::endl;
      std::string mapfname;
      MapMode mode;
      double res;
      int negate;
      double occ_th;
      double free_th;
      double origin[3];
      std::ifstream fin(path_to_yaml.c_str());
      if (fin.fail()) {
        ROS_ERROR("Map_server could not open %s.", path_to_yaml.c_str());
        return false;
      }
      // The document loading process changed in yaml-cpp 0.5.
      YAML::Node doc = YAML::Load(fin);
      
      std::cout<<"Print 2"<<std::endl;

      try {
        res = yaml_get_value<double>(doc, "resolution");
        // res = doc["resolution"].as<double>();
      } catch (YAML::InvalidScalar &) {
        ROS_ERROR("The map does not contain a resolution tag or it is invalid.");
        return false;
      }
      try {
        negate = yaml_get_value<double>(doc, "negate");
      } catch (YAML::InvalidScalar &) {
        ROS_ERROR("The map does not contain a negate tag or it is invalid.");
        return false;
      }
      try {
        occ_th = yaml_get_value<double>(doc, "occupied_thresh");
      } catch (YAML::InvalidScalar &) {
        ROS_ERROR("The map does not contain an occupied_thresh tag or it is invalid.");
        return false;
      }
      try {
        free_th = yaml_get_value<double>(doc, "free_thresh");
      } catch (YAML::InvalidScalar &) {
        ROS_ERROR("The map does not contain a free_thresh tag or it is invalid.");
        return false;
      }
      try {
        std::string modeS = "";
      
        modeS = yaml_get_value<double>(doc, "mode");


        if(modeS=="trinary")
          mode = TRINARY;
        else if(modeS=="scale")
          mode = SCALE;
        else if(modeS=="raw")
          mode = RAW;
        else{
          ROS_ERROR("Invalid mode tag \"%s\".", modeS.c_str());
          return false;
        }
      } catch (YAML::Exception &) {
        ROS_DEBUG("The map does not contain a mode tag or it is invalid... assuming Trinary");
        mode = TRINARY;
      }
      std::cout<<"Print 3"<<std::endl;

      try {
        // origin[0] = yaml_get_value<double>(doc, "origin"[0]);
        origin[0] = doc["origin"][0].as<double>();
        origin[1] = doc["origin"][1].as<double>();
        origin[0] = doc["origin"][2].as<double>();
        // doc["origin"][0] >> origin[0];
        // doc["origin"][1] >> origin[1];
        // doc["origin"][2] >> origin[2];
      } catch (YAML::InvalidScalar &) {
        ROS_ERROR("The map does not contain an origin tag or it is invalid.");
        return false;
      }

      std::cout<<"Print 4"<<std::endl;
      try {
      
        mapfname = yaml_get_value<std::string>(doc, "image");
    
        // TODO: make this path-handling more robust
        if(mapfname.size() == 0)
        {
          ROS_ERROR("The image tag cannot be an empty string.");
          return false;
        }

        boost::filesystem::path mapfpath(mapfname);
        if (!mapfpath.is_absolute())
        {
          boost::filesystem::path dir(path_to_yaml);
          dir = dir.parent_path();
          mapfpath = dir / mapfpath;
          mapfname = mapfpath.string();
        }
      } catch (YAML::InvalidScalar &) {
        ROS_ERROR("The map does not contain an image tag or it is invalid.");
        return false;
      }
      std::cout<<"Print 5"<<std::endl;

      loadMapFromValues(mapfname, res, negate, occ_th, free_th, origin, mode);
      map = map_resp_.map;
      return true;
}


bool MapLoader::loadMapFromValues(const std::string &map_file_name, double resolution,
                           int negate, double occ_th, double free_th,
                           double origin[3], MapMode mode)
    {

      

      ROS_INFO("Loading map from image \"%s\"", map_file_name.c_str());
      std::cout<<"jaha"<<std::endl;
      try {
        loadMapFromFile(&map_resp_, map_file_name.c_str(),
                                    resolution, negate, occ_th, free_th,
                                    origin, mode);
      } catch (std::runtime_error& e) {
        ROS_ERROR("%s", e.what());
        return false;
      }
      std::cout<<"asdsa"<<std::endl;

      // To make sure get a consistent time in simulation
      ros::Time::waitForValid();
      map_resp_.map.info.map_load_time = ros::Time::now();
      map_resp_.map.header.frame_id = "map";
      map_resp_.map.header.stamp = ros::Time::now();
      ROS_INFO("Read a %d X %d map @ %.3lf m/cell",
               map_resp_.map.info.width,
               map_resp_.map.info.height,
               map_resp_.map.info.resolution);
      meta_data_message_ = map_resp_.map.info;

      std::cout<<"Print 6"<<std::endl;
      //Publish latched topics
      metadata_pub_.publish( meta_data_message_ );
      map_pub_.publish( map_resp_.map );
      std::cout<<"Print 7"<<std::endl;
      return true;
}



void
MapLoader::loadMapFromFile(nav_msgs::GetMap::Response* resp,
                const char* fname, double res, bool negate,
                double occ_th, double free_th, double* origin,
                MapMode mode)
{
  SDL_Surface* img;

  unsigned char* pixels;
  unsigned char* p;
  unsigned char value;
  int rowstride, n_channels, avg_channels;
  unsigned int i,j;
  int k;
  double occ;
  int alpha;
  int color_sum;
  double color_avg;

  // Load the image using SDL.  If we get NULL back, the image load failed.
  if(!(img = IMG_Load(fname)))
  {
    std::string errmsg = std::string("failed to open image file \"") +
            std::string(fname) + std::string("\": ") + IMG_GetError();
    throw std::runtime_error(errmsg);
  }

  // Copy the image data into the map structure
  resp->map.info.width = img->w;
  resp->map.info.height = img->h;
  resp->map.info.resolution = res;
  resp->map.info.origin.position.x = *(origin);
  resp->map.info.origin.position.y = *(origin+1);
  resp->map.info.origin.position.z = 0.0;
  btQuaternion q;
  // setEulerZYX(yaw, pitch, roll)
  q.setEulerZYX(*(origin+2), 0, 0);
  resp->map.info.origin.orientation.x = q.x();
  resp->map.info.origin.orientation.y = q.y();
  resp->map.info.origin.orientation.z = q.z();
  resp->map.info.origin.orientation.w = q.w();

  // Allocate space to hold the data
  resp->map.data.resize(resp->map.info.width * resp->map.info.height);

  // Get values that we'll need to iterate through the pixels
  rowstride = img->pitch;
  n_channels = img->format->BytesPerPixel;

  // NOTE: Trinary mode still overrides here to preserve existing behavior.
  // Alpha will be averaged in with color channels when using trinary mode.
  if (mode==TRINARY || !img->format->Amask)
    avg_channels = n_channels;
  else
    avg_channels = n_channels - 1;

  // Copy pixel data into the map structure
  pixels = (unsigned char*)(img->pixels);
  for(j = 0; j < resp->map.info.height; j++)
  {
    for (i = 0; i < resp->map.info.width; i++)
    {
      // Compute mean of RGB for this pixel
      p = pixels + j*rowstride + i*n_channels;
      color_sum = 0;
      for(k=0;k<avg_channels;k++)
        color_sum += *(p + (k));
      color_avg = color_sum / (double)avg_channels;

      if (n_channels == 1)
          alpha = 1;
      else
          alpha = *(p+n_channels-1);

      if(negate)
        color_avg = 255 - color_avg;

      if(mode==RAW){
          value = color_avg;
          resp->map.data[MAP_IDX(resp->map.info.width,i,resp->map.info.height - j - 1)] = value;
          continue;
      }


      // If negate is true, we consider blacker pixels free, and whiter
      // pixels occupied.  Otherwise, it's vice versa.
      occ = (255 - color_avg) / 255.0;

      // Apply thresholds to RGB means to determine occupancy values for
      // map.  Note that we invert the graphics-ordering of the pixels to
      // produce a map with cell (0,0) in the lower-left corner.
      if(occ > occ_th)
        value = +100;
      else if(occ < free_th)
        value = 0;
      else if(mode==TRINARY || alpha < 1.0)
        value = -1;
      else {
        double ratio = (occ - free_th) / (occ_th - free_th);
        value = 1 + 98 * ratio;
      }

      resp->map.data[MAP_IDX(resp->map.info.width,i,resp->map.info.height - j - 1)] = value;
    }
  }

  SDL_FreeSurface(img);
}
}