#include <nodelet/nodelet.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/config.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
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

namespace ompl_2d_rviz_visualizer {
class VisualizerNodelet : public nodelet::Nodelet {
 public:
  VisualizerNodelet() {}
  virtual ~VisualizerNodelet() {}

  void onInit() override {
    NODELET_DEBUG("initializing ompl_2d_rviz_visualizer nodelet...");

    nh_ = getNodeHandle();
    mt_nh_ = getMTNodeHandle();
    private_nh_ = getPrivateNodeHandle();

    visual_tools_ = std::make_shared<rviz_visual_tools::RvizVisualTools>(
        "base_frame", "/rviz_visual_markers", mt_nh_);
    visual_tools_->loadMarkerPub();
    visual_tools_->deleteAllMarkers();
    visual_tools_->enableBatchPublishing();

    /////////////////////////////////////////////////////////
    // ompl related
    space_ = std::make_shared<ob::RealVectorStateSpace>(2u);

    // set the bounds for the R^2
    ob::RealVectorBounds bounds(2);
    bounds.setLow(-5);
    bounds.setHigh(5);
    space_->as<ob::RealVectorStateSpace>()->setBounds(bounds);
    space_->setup();

    ss_ = std::make_shared<og::SimpleSetup>(space_);
    si_ = ss_->getSpaceInformation();

    ss_->setPlanner(ob::PlannerPtr(std::make_shared<og::RRTstar>(si_)));
    ss_->setStateValidityChecker([](const ob::State* state) { return true; });

    path_length_objective_ =
        std::make_shared<ob::PathLengthOptimizationObjective>(si_);
    /////////////////////////////////////////////////////////

    enable_planning_ = false;
    solution_found_ = false;
    rendering_finished_ = false;

    // subscriber for rviz panel
    rviz_panel_sub_ = nh_.subscribe("/ompl_2d_rviz_visualizer_control_panel", 1,
                                    &VisualizerNodelet::rvizPanelCB, this);

    // timers
    planning_timer_ = mt_nh_.createWallTimer(
        ros::WallDuration(0.001), &VisualizerNodelet::planningTimerCB, this);

    rendering_timer_ = mt_nh_.createWallTimer(
        ros::WallDuration(0.001), &VisualizerNodelet::renderingTimerCB, this);
  }

  void rvizPanelCB(const std_msgs::UInt8& msg) {
    ROS_INFO("Panel msg received: %d", msg.data);

    if (msg.data == 1) {
      visual_tools_->deleteAllMarkers();
      visual_tools_->trigger();
    } else if (msg.data == 2) {
      solution_found_ = false;
      rendering_finished_ = false;
      enable_planning_ = true;
    }
  }

  void planningTimerCB(const ros::WallTimerEvent& event) {
    if (enable_planning_) {
      visual_tools_->deleteAllMarkers();
      // Create the termination condition
      double seconds = 0.1;
      ob::PlannerTerminationCondition ptc =
          ob::timedPlannerTerminationCondition(seconds, 0.01);

      // Create random start and goal space
      ob::ScopedState<> start(space_);
      ob::ScopedState<> goal(space_);
      start.random();
      goal.random();

      // show start and goal in rviz
      visual_tools_->publishSphere(
          stateToPoint(start), rviz_visual_tools::GREEN,
          rviz_visual_tools::XXXLARGE, "plan_start_goal");
      visual_tools_->publishSphere(stateToPoint(goal), rviz_visual_tools::RED,
                                   rviz_visual_tools::XXXLARGE,
                                   "plan_start_goal");
      visual_tools_->trigger();

      ss_->clear();
      ss_->clearStartStates();

      // set the start and goal states
      ss_->setStartAndGoalStates(start, goal);

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
      // publish path
      ss_->getSolutionPath().interpolate();

      publish2DPath(ss_->getSolutionPath(), rviz_visual_tools::PURPLE, 0.05,
                    "final_solution");
      visual_tools_->trigger();

      // publish graph
      const ob::PlannerDataPtr planner_data(
          std::make_shared<ob::PlannerData>(si_));
      ss_->getPlannerData(*planner_data);

      ROS_INFO("Number of start vertices: %d",
               planner_data->numStartVertices());
      ROS_INFO("Number of goal vertices: %d", planner_data->numGoalVertices());
      ROS_INFO("Number of vertices: %d", planner_data->numVertices());
      ROS_INFO("Number of edges: %d", planner_data->numEdges());

      for (std::size_t vertex_id = 1; vertex_id < planner_data->numVertices();
           ++vertex_id) {
        ob::PlannerDataVertex* vertex = &planner_data->getVertex(vertex_id);
        auto this_vertex = stateToPointMsg(vertex->getState());

        // Get the out edges from the current vertex
        std::vector<unsigned int> edge_list;
        planner_data->getEdges(vertex_id, edge_list);

        // Now loop through each edge
        for (std::vector<unsigned int>::const_iterator edge_it =
                 edge_list.begin();
             edge_it != edge_list.end(); ++edge_it) {
          // Convert vertex id to next coordinates
          ob::PlannerDataVertex* v = &planner_data->getVertex(*edge_it);
          auto next_vertex = stateToPointMsg(v->getState());

          visual_tools_->publishLine(this_vertex, next_vertex,
                                     rviz_visual_tools::BLUE,
                                     rviz_visual_tools::SMALL);
        }
      }
      visual_tools_->trigger();
      rendering_finished_ = true;
    }
  }

 private:
  Eigen::Vector3d stateToPoint(const ob::ScopedState<> state) {
    return stateToPoint(state.get());
  }

  Eigen::Vector3d stateToPoint(const ob::State* state) {
    if (!state) {
      ROS_FATAL_NAMED("ompl_example_2d", "No state found for vertex");
      exit(1);
    }

    // Convert to RealVectorStateSpace
    const ob::RealVectorStateSpace::StateType* real_state =
        static_cast<const ob::RealVectorStateSpace::StateType*>(state);

    // Create point
    Eigen::Vector3d temp_eigen_point_;
    temp_eigen_point_.x() = real_state->values[0];
    temp_eigen_point_.y() = real_state->values[1];
    temp_eigen_point_.z() = 0.0;

    return temp_eigen_point_;
  }

  geometry_msgs::Point stateToPointMsg(const ob::State* state) {
    if (!state) {
      ROS_FATAL_NAMED("ompl_example_2d", "No state found for vertex");
      exit(1);
    }

    // Convert to RealVectorStateSpace
    const ob::RealVectorStateSpace::StateType* real_state =
        static_cast<const ob::RealVectorStateSpace::StateType*>(state);

    // Create point
    geometry_msgs::Point temp_point;
    temp_point.x = real_state->values[0];
    temp_point.y = real_state->values[1];
    temp_point.z = 0.0;

    return temp_point;
  }

  bool publish2DPath(const og::PathGeometric& path,
                     const rviz_visual_tools::colors& color,
                     const double thickness, const std::string& ns) {
    // Error check
    if (path.getStateCount() <= 0) {
      ROS_WARN_STREAM_NAMED("ompl_example_2d", "No states found in path");
      return false;
    }

    // Initialize first vertex
    Eigen::Vector3d prev_vertex = stateToPoint(path.getState(0));
    Eigen::Vector3d this_vertex;

    // Convert path coordinates
    for (std::size_t i = 1; i < path.getStateCount(); ++i) {
      // Get current coordinates
      this_vertex = stateToPoint(path.getState(i));

      // Create line
      visual_tools_->publishCylinder(prev_vertex, this_vertex, color, thickness,
                                     ns);

      // Save these coordinates for next line
      prev_vertex = this_vertex;
    }

    return true;
  }

  // ROS related
  // node handles
  ros::NodeHandle nh_;
  ros::NodeHandle mt_nh_;
  ros::NodeHandle private_nh_;

  // subscribers
  ros::Subscriber rviz_panel_sub_;

  // publishers

  // timers
  ros::WallTimer planning_timer_;
  ros::WallTimer rendering_timer_;

  rviz_visual_tools::RvizVisualToolsPtr visual_tools_;

  // ompl related
  ob::StateSpacePtr space_;
  og::SimpleSetupPtr ss_;
  ob::SpaceInformationPtr si_;
  std::shared_ptr<ob::PathLengthOptimizationObjective> path_length_objective_;

  // flags
  std::atomic_bool enable_planning_;
  std::atomic_bool solution_found_;
  std::atomic_bool rendering_finished_;
};
}  // namespace ompl_2d_rviz_visualizer

PLUGINLIB_EXPORT_CLASS(ompl_2d_rviz_visualizer::VisualizerNodelet,
                       nodelet::Nodelet)