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

#include <nodelet/nodelet.h>
#include <ompl/base/DiscreteMotionValidator.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/objectives/MaximizeMinClearanceObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/config.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl_2d_rviz_visualizer_msgs/MapBounds.h>
#include <ompl_2d_rviz_visualizer_msgs/Plan.h>
#include <ompl_2d_rviz_visualizer_msgs/Reset.h>
#include <ompl_2d_rviz_visualizer_msgs/State.h>
#include <ompl_2d_rviz_visualizer_ros/collision_checker.h>
#include <ompl_2d_rviz_visualizer_ros/map_loader.h>
#include <ompl_2d_rviz_visualizer_ros/map_utils.h>
#include <ompl_2d_rviz_visualizer_ros/rviz_renderer.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <std_msgs/UInt8.h>

#include <atomic>
#include <memory>
#include <mutex>
#include <thread>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace ompl_2d_rviz_visualizer_ros {

enum PLANNERS_IDS { INVALID, RRT_CONNECT, RRT_STAR };

enum PLANNING_OBJS_IDS { PATH_LENGTH, MAXMIN_CLEARANCE };

const std::vector<std::string> PLANNER_NAMES{"invalid", "rrt_connect",
                                             "rrt_star"};

class VisualizerNodelet : public nodelet::Nodelet {
 public:
  VisualizerNodelet() {}
  virtual ~VisualizerNodelet() {}

  void onInit() override {
    NODELET_DEBUG("initializing ompl_2d_rviz_visualizer nodelet...");

    // initialize node handlers
    nh_ = getNodeHandle();
    mt_nh_ = getMTNodeHandle();
    private_nh_ = getPrivateNodeHandle();

    // initialize rviz renderer object
    rviz_renderer_ = std::make_shared<RvizRenderer>(
        "map", "/ompl_2d_rviz_visualizer_nodelet/rviz_visual_markers", mt_nh_);

    // ompl related
    space_ = std::make_shared<ob::RealVectorStateSpace>(2u);

    // initialize map loader and occupancy grid map objects
    ogm_map_ = std::make_shared<nav_msgs::OccupancyGrid>();
    map_loader_ = std::make_shared<MapLoader>(
        "map", "/ompl_2d_rviz_visualizer_nodelet/map", mt_nh_);

    if (!private_nh_.hasParam("map_file_path")) {
      ROS_ERROR("map_file_path does not exist in parameter server. Exiting...");
      exit(1);
    }

    std::string map_file_path;
    private_nh_.getParam("map_file_path", map_file_path);
    map_loader_->loadMapFromYaml(map_file_path, *ogm_map_);

    ////////////////////////////////////////////////////////////////
    // set the bounds for the R^2
    if (!map_utils::getBounds(map_bounds_.min_x, map_bounds_.max_x,
                              map_bounds_.min_y, map_bounds_.max_y,
                              *ogm_map_)) {
      ROS_ERROR("Fail to generate bounds in the occupancy grid map.");
      exit(1);
    }

    ob::RealVectorBounds bounds(2);
    bounds.setLow(0, map_bounds_.min_x);
    bounds.setLow(1, map_bounds_.min_y);
    bounds.setHigh(0, map_bounds_.max_x);
    bounds.setHigh(1, map_bounds_.max_y);

    space_->as<ob::RealVectorStateSpace>()->setBounds(bounds);
    ////////////////////////////////////////////////////////////////

    space_->setup();

    ss_ = std::make_shared<og::SimpleSetup>(space_);
    si_ = ss_->getSpaceInformation();

    start_state_ = std::make_shared<ob::ScopedState<>>(space_);
    goal_state_ = std::make_shared<ob::ScopedState<>>(space_);

    // custom State Validity Checker class need to be implemented which uses
    // occupancy grid maps for collison checking
    collision_checker_ = std::make_shared<CollisionChecker>(ogm_map_);

    ss_->setStateValidityChecker([this](const ob::State* state) {
      return collision_checker_->isValid(state);
    });

    enable_planning_ = false;
    solution_found_ = false;
    rendering_finished_ = false;
    planner_id_ = PLANNERS_IDS::INVALID;
    planning_obj_id_ = PLANNING_OBJS_IDS::PATH_LENGTH;
    planning_duration_ = 0.0;

    // service servers
    start_state_setter_srv_ = mt_nh_.advertiseService(
        "/ompl_2d_rviz_visualizer_nodelet/set_start_state",
        &VisualizerNodelet::setStartStateService, this);

    goal_state_setter_srv_ = mt_nh_.advertiseService(
        "/ompl_2d_rviz_visualizer_nodelet/set_goal_state",
        &VisualizerNodelet::setGoalStateService, this);

    plan_request_srv_ =
        mt_nh_.advertiseService("/ompl_2d_rviz_visualizer_nodelet/plan_request",
                                &VisualizerNodelet::planRequestService, this);

    reset_request_srv_ = mt_nh_.advertiseService(
        "/ompl_2d_rviz_visualizer_nodelet/reset_request",
        &VisualizerNodelet::resetRequestService, this);

    map_bounds_srv_ = mt_nh_.advertiseService(
        "/ompl_2d_rviz_visualizer_nodelet/get_map_bounds",
        &VisualizerNodelet::getMapBoundsService, this);

    // timers
    planning_timer_ = mt_nh_.createWallTimer(
        ros::WallDuration(0.001), &VisualizerNodelet::planningTimerCB, this);

    rendering_timer_ = mt_nh_.createWallTimer(
        ros::WallDuration(0.001), &VisualizerNodelet::renderingTimerCB, this);
  }

  bool setStartStateService(ompl_2d_rviz_visualizer_msgs::StateRequest& req,
                            ompl_2d_rviz_visualizer_msgs::StateResponse& res) {
    if (!(collision_checker_->isValid(req.x, req.y))) {
      ROS_INFO("The start state is in collision.");
      res.success = false;
      return true;
    }

    (*start_state_)[0] = req.x;
    (*start_state_)[1] = req.y;
    rviz_renderer_->renderState((*start_state_).get(), rviz_visual_tools::GREEN,
                                rviz_visual_tools::XLARGE, "start_goal_states",
                                1);
    res.success = true;
    return true;
  }

  bool setGoalStateService(ompl_2d_rviz_visualizer_msgs::StateRequest& req,
                           ompl_2d_rviz_visualizer_msgs::StateResponse& res) {
    (*goal_state_)[0] = req.x;
    (*goal_state_)[1] = req.y;
    rviz_renderer_->renderState((*goal_state_).get(), rviz_visual_tools::RED,
                                rviz_visual_tools::XLARGE, "start_goal_states",
                                2);
    res.success = true;
    return true;
  }

  bool planRequestService(ompl_2d_rviz_visualizer_msgs::PlanRequest& req,
                          ompl_2d_rviz_visualizer_msgs::PlanResponse& res) {
    ROS_INFO("Planning request received.");
    ROS_INFO("Planner ID: %s", PLANNER_NAMES[req.planner_id].c_str());

    planner_id_ = req.planner_id;
    planning_obj_id_ = req.objective_id;
    planning_duration_ = req.duration;

    solution_found_ = false;
    rendering_finished_ = false;
    enable_planning_ = true;

    res.success = true;
    return true;
  }

  bool resetRequestService(ompl_2d_rviz_visualizer_msgs::ResetRequest& req,
                           ompl_2d_rviz_visualizer_msgs::ResetResponse& res) {
    ROS_INFO("Resetting the planner and clearing the graph markers.");

    if (req.clear_graph) {
      // first clear all the markers
      rviz_renderer_->deleteAllMarkers();
      // then redraw the start and goal states
      rviz_renderer_->renderState(
          (*start_state_).get(), rviz_visual_tools::GREEN,
          rviz_visual_tools::XXXLARGE, "start_goal_states", 1);
      rviz_renderer_->renderState((*goal_state_).get(), rviz_visual_tools::RED,
                                  rviz_visual_tools::XXXLARGE,
                                  "start_goal_states", 2);
    }
    res.success = true;
    return true;
  }

  bool getMapBoundsService(
      ompl_2d_rviz_visualizer_msgs::MapBoundsRequest& req,
      ompl_2d_rviz_visualizer_msgs::MapBoundsResponse& res) {
    res.min_x = map_bounds_.min_x;
    res.min_y = map_bounds_.min_y;
    res.max_x = map_bounds_.max_x;
    res.max_y = map_bounds_.max_y;
    return true;
  }

  void planningTimerCB(const ros::WallTimerEvent& event) {
    if (enable_planning_) {
      // choose the optimization objective
      switch (planning_obj_id_) {
        case PATH_LENGTH:
          optimization_objective_ =
              std::make_shared<ob::PathLengthOptimizationObjective>(si_);
          break;

        case MAXMIN_CLEARANCE:
          optimization_objective_ =
              std::make_shared<ob::MaximizeMinClearanceObjective>(si_);
          break;

        default:
          ROS_ERROR("Error setting OMPL optimization objective.");
          exit(-1);
          break;
      }
      ss_->setOptimizationObjective(optimization_objective_);

      // choose the planner
      switch (planner_id_) {
        case RRT_CONNECT:
          ss_->setPlanner(
              ob::PlannerPtr(std::make_shared<og::RRTConnect>(si_)));
          break;
        case RRT_STAR:
          ss_->setPlanner(ob::PlannerPtr(std::make_shared<og::RRTstar>(si_)));
          break;
        default:
          break;
      }

      // get latest planner params from the parameter server
      std::vector<std::string> param_names;
      ss_->getPlanner()->params().getParamNames(param_names);
      std::map<std::string, std::string> updated_param_names_values;
      for (const auto& n : param_names) {
        std::string param_name =
            "ompl_planner_parameters/" + PLANNER_NAMES[planner_id_] + "/" + n;
        XmlRpc::XmlRpcValue param;
        if (private_nh_.hasParam(param_name)) {
          private_nh_.getParam(param_name, param);

          if (param.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
            updated_param_names_values[n] =
                std::to_string(static_cast<double>(param));
          } else if (param.getType() == XmlRpc::XmlRpcValue::TypeInt) {
            updated_param_names_values[n] =
                std::to_string(static_cast<int>(param));
          } else if (param.getType() == XmlRpc::XmlRpcValue::TypeBoolean) {
            updated_param_names_values[n] =
                std::to_string(static_cast<bool>(param));
          }
        }
      }
      ss_->getPlanner()->params().setParams(updated_param_names_values);

      ROS_INFO("Planner parameters updated.");

      // Create the termination condition
      ob::PlannerTerminationCondition ptc =
          ob::timedPlannerTerminationCondition(planning_duration_, 0.01);

      ss_->clear();
      ss_->clearStartStates();

      // set the start and goal states
      ss_->setStartAndGoalStates(*start_state_, *goal_state_);

      // attempt to solve the problem within x seconds of planning time
      ob::PlannerStatus solved;
      solved = ss_->solve(ptc);

      if (solved) {
        enable_planning_ = false;
        solution_found_ = true;
      }
    }
  }

  void renderingTimerCB(const ros::WallTimerEvent& event) {
    if (solution_found_ && !rendering_finished_) {
      // render graph
      const ob::PlannerDataPtr planner_data(
          std::make_shared<ob::PlannerData>(si_));
      ss_->getPlannerData(*planner_data);

      ROS_INFO("Number of start vertices: %d",
               planner_data->numStartVertices());
      ROS_INFO("Number of goal vertices: %d", planner_data->numGoalVertices());
      ROS_INFO("Number of vertices: %d", planner_data->numVertices());
      ROS_INFO("Number of edges: %d", planner_data->numEdges());

      rviz_renderer_->renderGraph(planner_data, rviz_visual_tools::BLUE, 0.005,
                                  "planner_graph");

      // render path
      ss_->getSolutionPath().interpolate();

      rviz_renderer_->renderPath(ss_->getSolutionPath(),
                                 rviz_visual_tools::PURPLE, 0.02,
                                 "final_solution");
      rendering_finished_ = true;
    }
  }

 private:
  // ROS related
  // node handles
  ros::NodeHandle nh_;
  ros::NodeHandle mt_nh_;
  ros::NodeHandle private_nh_;

  // subscribers
  // publishers

  // service servers
  ros::ServiceServer plan_request_srv_;
  ros::ServiceServer reset_request_srv_;
  ros::ServiceServer start_state_setter_srv_;
  ros::ServiceServer goal_state_setter_srv_;
  ros::ServiceServer map_bounds_srv_;

  // timers
  ros::WallTimer planning_timer_;
  ros::WallTimer rendering_timer_;

  // rendering stuffs
  RvizRendererPtr rviz_renderer_;

  // ogm related
  struct MapBounds {
    double min_x;
    double min_y;
    double max_x;
    double max_y;
  };

  MapBounds map_bounds_;

  // ompl related
  ob::StateSpacePtr space_;
  og::SimpleSetupPtr ss_;
  ob::SpaceInformationPtr si_;
  std::shared_ptr<ob::OptimizationObjective> optimization_objective_;

  std::shared_ptr<nav_msgs::OccupancyGrid> ogm_map_;
  std::shared_ptr<MapLoader> map_loader_;
  std::shared_ptr<CollisionChecker> collision_checker_;

  ob::ScopedStatePtr start_state_;
  ob::ScopedStatePtr goal_state_;

  int planner_id_;
  int planning_obj_id_;
  double planning_duration_;

  // flags
  std::atomic_bool enable_planning_;
  std::atomic_bool solution_found_;
  std::atomic_bool rendering_finished_;
};
}  // namespace ompl_2d_rviz_visualizer_ros

PLUGINLIB_EXPORT_CLASS(ompl_2d_rviz_visualizer_ros::VisualizerNodelet,
                       nodelet::Nodelet)