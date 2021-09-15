#pragma once

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/config.h>
#include <ompl/geometric/SimpleSetup.h>
#include <rviz_visual_tools/rviz_visual_tools.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace rvt = rviz_visual_tools;

namespace ompl_2d_rviz_visualizer {

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
                   const rvt::scales& scale, const std::string& ns);

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
                   const rvt::colors& color, const double radius);

 private:
  /**
   * @brief Convert ompl abstract state to eigen vector3D.
   * @param state ompl abstract state
   * @return eigen vector3D
   */
  Eigen::Vector3d stateToPoint(const ob::State* state);

  geometry_msgs::Point stateToPointMsg(const ob::State* state);

  rvt::RvizVisualToolsPtr visual_tools_;
};

typedef std::shared_ptr<RvizRenderer> RvizRendererPtr;
typedef std::shared_ptr<const RvizRenderer> RvizRendererConstPtr;

}  // namespace ompl_2d_rviz_visualizer