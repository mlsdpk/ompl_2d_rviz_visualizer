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

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/config.h>
#include <ompl/geometric/SimpleSetup.h>
#include <rviz_visual_tools/rviz_visual_tools.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace rvt = rviz_visual_tools;

namespace ompl_2d_rviz_visualizer_ros {

class RvizRenderer {
 public:
  /**
   * @brief Constructor
   * @param base_frame common base for all visualization markers, usually
   * "/world" or "/odom"
   * @param marker_topic rostopic to publish markers on Rviz
   * @param nh optional ros node handle (default: "~")
   */
  RvizRenderer(const std::string& base_frame, const std::string& marker_topic,
               ros::NodeHandle nh = ros::NodeHandle("~"));

  /**
   * @brief Destructor
   */
  ~RvizRenderer();

  /**
   * @brief Clear all the published markers on Rviz.
   */
  bool deleteAllMarkers();

  /**
   * @brief Render state on rviz. State is basically a sphere with adjustable
   * size and color.
   * @param state ompl abstract state
   * @param color rviz_visual_tools color
   * @param scale rviz_visual_tools scale
   * @param ns namespace of marker
   */
  bool renderState(const ob::State* state, const rvt::colors& color,
                   const rvt::scales& scale, const std::string& ns,
                   std::size_t id = 0);

  /**
   * @brief Render state on rviz. State is basically a sphere with adjustable
   * size and color.
   * @param point eigen vector3D
   * @param color rviz_visual_tools color
   * @param scale rviz_visual_tools scale
   * @param ns namespace of marker
   */
  bool renderState(const Eigen::Vector3d& point, const rvt::colors& color,
                   const rvt::scales& scale, const std::string& ns,
                   std::size_t id = 0);

  /**
   * @brief Render 2D geometric path on rviz. Path is basically a cylinder with
   * adjustable radius and color.
   * @param path ompl geometric 2D path
   * @param color rviz_visual_tools color
   * @param radius geometry of cylinder
   * @param ns namespace of marker
   */
  bool renderPath(const og::PathGeometric& path, const rvt::colors& color,
                  const double radius, const std::string& ns);

  /**
   * @brief Render ompl planner data graph on rviz.
   * @param planner_data ompl planner data
   * @param color rviz_visual_tools color
   * @param radius geometry of cylinder
   */
  bool renderGraph(const ob::PlannerDataPtr planner_data,
                   const rvt::colors& color, const double radius,
                   const std::string& ns);

 private:
  /**
   * @brief Convert ompl abstract state to eigen vector3D.
   * @param state ompl abstract state
   * @return eigen vector3D
   */
  Eigen::Vector3d stateToPoint(const ob::State* state);

  geometry_msgs::Point stateToPointMsg(const ob::State* state);

  bool loadMapFromYaml(std::string path_to_yaml);

  std::string base_frame_;
  rvt::RvizVisualToolsPtr visual_tools_;
};

typedef std::shared_ptr<RvizRenderer> RvizRendererPtr;
typedef std::shared_ptr<const RvizRenderer> RvizRendererConstPtr;

}  // namespace ompl_2d_rviz_visualizer_ros