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

#include <ompl_2d_rviz_visualizer_ros/rviz_renderer.h>

namespace ompl_2d_rviz_visualizer_ros {

RvizRenderer::RvizRenderer(const std::string& base_frame,
                           const std::string& marker_topic, ros::NodeHandle nh)
    : base_frame_{base_frame} {
  visual_tools_ =
      std::make_shared<rvt::RvizVisualTools>(base_frame_, marker_topic, nh);
  visual_tools_->loadMarkerPub();
  visual_tools_->deleteAllMarkers();
  visual_tools_->enableBatchPublishing();
}

RvizRenderer::~RvizRenderer() {}

bool RvizRenderer::deleteAllMarkers() {
  visual_tools_->deleteAllMarkers();
  return visual_tools_->trigger();
}

bool RvizRenderer::renderState(const ob::State* state, const rvt::colors& color,
                               const rvt::scales& scale, const std::string& ns,
                               std::size_t id) {
  visual_tools_->publishSphere(stateToPoint(state), color, scale, ns, id);
  return visual_tools_->trigger();
}

bool RvizRenderer::renderState(const Eigen::Vector3d& point,
                               const rvt::colors& color,
                               const rvt::scales& scale, const std::string& ns,
                               std::size_t id) {
  visual_tools_->publishSphere(point, color, scale, ns, id);
  return visual_tools_->trigger();
}

bool RvizRenderer::renderPath(const og::PathGeometric& path,
                              const rvt::colors& color, const double radius,
                              const std::string& ns) {
  if (path.getStateCount() <= 0) {
    ROS_WARN_STREAM_NAMED("rviz_renderer", "No states found in path");
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
    visual_tools_->publishCylinder(prev_vertex, this_vertex, color, radius, ns);

    // Save these coordinates for next line
    prev_vertex = this_vertex;
  }
  return visual_tools_->trigger();
}

bool RvizRenderer::renderGraph(const ob::PlannerDataPtr planner_data,
                               const rvt::colors& color, const double radius,
                               const std::string& ns) {
  graph_msgs::GeometryGraph graph;

  for (std::size_t vertex_id = 0; vertex_id < planner_data->numVertices();
       ++vertex_id) {
    ob::PlannerDataVertex* vertex = &planner_data->getVertex(vertex_id);
    auto this_vertex = stateToPointMsg(vertex->getState());
    graph.nodes.push_back(this_vertex);

    graph_msgs::Edges edge_msg;
    // Get the out edges from the current vertex
    std::vector<unsigned int> edge_list;
    planner_data->getEdges(vertex_id, edge_msg.node_ids);
    graph.edges.push_back(edge_msg);
  }

  visual_tools_->publishGraph(graph, color, radius);

  return visual_tools_->trigger();
}

Eigen::Vector3d RvizRenderer::stateToPoint(const ob::State* state) {
  if (!state) {
    ROS_FATAL_NAMED("rviz_renderer", "No state found for vertex");
    exit(1);
  }
  // Convert to RealVectorStateSpace
  const ob::RealVectorStateSpace::StateType* real_state =
      static_cast<const ob::RealVectorStateSpace::StateType*>(state);

  // Create point
  Eigen::Vector3d temp_eigen_point;
  temp_eigen_point.x() = real_state->values[0];
  temp_eigen_point.y() = real_state->values[1];
  temp_eigen_point.z() = 0.0;

  return temp_eigen_point;
}

geometry_msgs::Point RvizRenderer::stateToPointMsg(const ob::State* state) {
  if (!state) {
    ROS_FATAL_NAMED("rviz_renderer", "No state found for vertex");
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

}  // namespace ompl_2d_rviz_visualizer_ros