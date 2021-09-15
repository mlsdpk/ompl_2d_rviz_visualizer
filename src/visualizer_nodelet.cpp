#include <nodelet/nodelet.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/config.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl_2d_rviz_visualizer/rviz_renderer.h>
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

    // initialize node handlers
    nh_ = getNodeHandle();
    mt_nh_ = getMTNodeHandle();
    private_nh_ = getPrivateNodeHandle();

    // initialize rviz renderer object
    rviz_renderer_ = std::make_shared<RvizRenderer>(
        "base_frame", "/rviz_visual_markers", mt_nh_);

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
      rviz_renderer_->deleteAllMarkers();
    } else if (msg.data == 2) {
      solution_found_ = false;
      rendering_finished_ = false;
      enable_planning_ = true;
    }
  }

  void planningTimerCB(const ros::WallTimerEvent& event) {
    if (enable_planning_) {
      rviz_renderer_->deleteAllMarkers();
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
      rviz_renderer_->renderState(start.get(), rviz_visual_tools::GREEN,
                                  rviz_visual_tools::XXXLARGE,
                                  "plan_start_goal");
      rviz_renderer_->renderState(goal.get(), rviz_visual_tools::RED,
                                  rviz_visual_tools::XXXLARGE,
                                  "plan_start_goal");

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
      // render graph
      const ob::PlannerDataPtr planner_data(
          std::make_shared<ob::PlannerData>(si_));
      ss_->getPlannerData(*planner_data);

      ROS_INFO("Number of start vertices: %d",
               planner_data->numStartVertices());
      ROS_INFO("Number of goal vertices: %d", planner_data->numGoalVertices());
      ROS_INFO("Number of vertices: %d", planner_data->numVertices());
      ROS_INFO("Number of edges: %d", planner_data->numEdges());

      rviz_renderer_->renderGraph(planner_data, rviz_visual_tools::BLUE, 0.01);

      // render path
      ss_->getSolutionPath().interpolate();

      rviz_renderer_->renderPath(ss_->getSolutionPath(),
                                 rviz_visual_tools::PURPLE, 0.05,
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
  ros::Subscriber rviz_panel_sub_;

  // publishers

  // timers
  ros::WallTimer planning_timer_;
  ros::WallTimer rendering_timer_;

  // rendering stuffs
  RvizRendererPtr rviz_renderer_;

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