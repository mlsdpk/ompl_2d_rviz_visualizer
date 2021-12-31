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

namespace ompl_2d_rviz_visualizer_ros
{
RvizRenderer::RvizRenderer(const std::string& base_frame, const std::string& marker_topic, ros::NodeHandle nh)
  : base_frame_{ base_frame }
{
  visual_tools_ = std::make_shared<rvt::RvizVisualTools>(base_frame_, marker_topic, nh);
  visual_tools_->loadMarkerPub();
  visual_tools_->deleteAllMarkers();
  visual_tools_->enableBatchPublishing();
}

RvizRenderer::~RvizRenderer()
{
}

bool RvizRenderer::deleteAllMarkers()
{
  visual_tools_->deleteAllMarkers();
  return visual_tools_->trigger();
}

void RvizRenderer::addToMarkerIDs(const std::string& ns, std::size_t id)
{
  auto marker_ids_it = marker_ids_.find(ns);
  if (marker_ids_it != marker_ids_.end())
  {
    marker_ids_it->second.insert(id);
  }
  else
  {
    marker_ids_[ns] = std::unordered_set<std::size_t>{ id };
  }
}

bool RvizRenderer::deleteAllMarkersInNS(const std::string& ns)
{
  // check specific namespace has already been published
  auto marker_ids_it = marker_ids_.find(ns);
  if (marker_ids_it != marker_ids_.end())
  {
    // if so then we create temporary marker and publish it with delete action
    visualization_msgs::Marker temp_marker;
    temp_marker.header.frame_id = base_frame_;
    temp_marker.ns = ns;
    temp_marker.action = visualization_msgs::Marker::DELETE;
    for (const auto id : marker_ids_it->second)
    {
      temp_marker.id = id;
      temp_marker.header.stamp = ros::Time::now();
      visual_tools_->publishMarker(temp_marker);
    }

    // don't forget to remove the ns from marker_ids
    marker_ids_.erase(ns);
    return visual_tools_->trigger();
  }
  else
  {
    ROS_DEBUG("Namespace %s not found in marker IDs.", ns.c_str());
    return false;
  }
}

bool RvizRenderer::deleteMarkerInNSAndID(const std::string& ns, std::size_t id)
{
  // check specific namespace has already been published
  auto marker_ids_ns_it = marker_ids_.find(ns);
  if (marker_ids_ns_it != marker_ids_.end())
  {
    // if so, we find the specific id
    auto marker_ids_id_it = marker_ids_ns_it->second.find(id);
    if (marker_ids_id_it != marker_ids_ns_it->second.end())
    {
      // if the id is found,
      // publish temporary marker with delete action
      visualization_msgs::Marker temp_marker;
      temp_marker.header.frame_id = base_frame_;
      temp_marker.ns = ns;
      temp_marker.action = visualization_msgs::Marker::DELETE;
      temp_marker.id = id;
      temp_marker.header.stamp = ros::Time::now();
      visual_tools_->publishMarker(temp_marker);

      // don't forget to remove the id from marker_ids ns
      marker_ids_ns_it->second.erase(id);
      return visual_tools_->trigger();
    }
    else
    {
      ROS_DEBUG("ID %ld not found in marker namespace %s.", id, ns.c_str());
      return false;
    }
  }
  else
  {
    ROS_DEBUG("Namespace %s not found in marker IDs.", ns.c_str());
    return false;
  }
}

bool RvizRenderer::renderState(const ob::State* state, const rvt::colors& color, const rvt::scales& scale,
                               const std::string& ns, std::size_t id)
{
  visual_tools_->publishSphere(stateToPoint(state), color, scale, ns, id);

  // remember this marker
  addToMarkerIDs(ns, id);

  return visual_tools_->trigger();
}

bool RvizRenderer::renderState(const Eigen::Vector3d& point, const rvt::colors& color, const rvt::scales& scale,
                               const std::string& ns, std::size_t id)
{
  visual_tools_->publishSphere(point, color, scale, ns, id);

  // remember this marker
  addToMarkerIDs(ns, id);

  return visual_tools_->trigger();
}

bool RvizRenderer::renderPath(const og::PathGeometric& path, const rvt::colors& color, const double radius,
                              const std::string& ns)
{
  if (path.getStateCount() <= 0)
  {
    ROS_WARN("No states found in path");
    return false;
  }

  // Initialize first vertex
  Eigen::Vector3d prev_vertex = stateToPoint(path.getState(0));
  Eigen::Vector3d this_vertex;

  std::size_t id = 1;

  // Convert path coordinates
  for (std::size_t i = 1; i < path.getStateCount(); ++i)
  {
    // Get current coordinates
    this_vertex = stateToPoint(path.getState(i));

    // Create line
    publishCylinder(prev_vertex, this_vertex, color, radius, ns, id);
    addToMarkerIDs(ns, id);
    id++;

    // Save these coordinates for next line
    prev_vertex = this_vertex;
  }
  return visual_tools_->trigger();
}

bool RvizRenderer::renderGraph(const ob::PlannerDataPtr planner_data, const rvt::colors& color, const double radius,
                               const std::string& ns)
{
  graph_msgs::GeometryGraph graph;

  for (std::size_t vertex_id = 0; vertex_id < planner_data->numVertices(); ++vertex_id)
  {
    ob::PlannerDataVertex* vertex = &planner_data->getVertex(vertex_id);
    auto this_vertex = stateToPointMsg(vertex->getState());
    graph.nodes.push_back(this_vertex);

    graph_msgs::Edges edge_msg;
    // Get the out edges from the current vertex
    std::vector<unsigned int> edge_list;
    planner_data->getEdges(vertex_id, edge_msg.node_ids);
    graph.edges.push_back(edge_msg);
  }

  // Track which pairs of nodes we've already connected since graph is
  // bi-directional
  std::size_t id = 0;
  typedef std::pair<std::size_t, std::size_t> node_ids;
  std::set<node_ids> added_edges;
  std::pair<std::set<node_ids>::iterator, bool> return_value;
  Eigen::Vector3d a, b;
  for (std::size_t i = 0; i < graph.nodes.size(); ++i)
  {
    for (std::size_t j = 0; j < graph.edges[i].node_ids.size(); ++j)
    {
      // Check if we've already added this pair of nodes (edge)
      return_value = added_edges.insert(node_ids(i, j));
      if (!return_value.second)
      {
        // Element already existed in set, so don't add a new collision object
      }
      else
      {
        // Create a cylinder from two points
        a = visual_tools_->convertPoint(graph.nodes[i]);
        b = visual_tools_->convertPoint(graph.nodes[graph.edges[i].node_ids[j]]);
        publishCylinder(a, b, color, radius, ns, id);
        addToMarkerIDs(ns, id);
        id++;
      }
    }
  }
  visual_tools_->trigger();

  // We are currently overwriting IDs with ADD option (which is still ok)
  // But if our graph is pruned, there will be left-over IDs with higher
  // numbers, which needs to be deleted (remove from the rviz scene).
  // One way to solve this might be - delete unused IDs from this current
  // namespace using stored marker_ids_.

  // delete left-over markers
  // run until marker id not found in namespace
  while (deleteMarkerInNSAndID(ns, id++))
  {
  }

  return true;
}

bool RvizRenderer::publishCylinder(const Eigen::Vector3d& point1, const Eigen::Vector3d& point2,
                                   const rvt::colors& color, const double radius, const std::string& ns, std::size_t id)
{
  visualization_msgs::Marker cylinder_marker;
  cylinder_marker.lifetime = ros::Duration(0.0);
  cylinder_marker.header.frame_id = base_frame_;
  // Set the marker action.  Options are ADD and DELETE
  cylinder_marker.action = visualization_msgs::Marker::ADD;
  // Set the marker type.
  cylinder_marker.type = visualization_msgs::Marker::CYLINDER;

  // Distance between two points
  double height = (point1 - point2).lpNorm<2>();

  // Find center point
  Eigen::Vector3d pt_center = visual_tools_->getCenterPoint(point1, point2);

  // Create vector
  Eigen::Isometry3d pose;
  pose = visual_tools_->getVectorBetweenPoints(pt_center, point2);

  // Convert pose to be normal to cylindar axis
  Eigen::Isometry3d rotation;
  rotation = Eigen::AngleAxisd(0.5 * M_PI, Eigen::Vector3d::UnitY());
  pose = pose * rotation;

  // Set the timestamp
  cylinder_marker.header.stamp = ros::Time::now();
  cylinder_marker.ns = ns;
  cylinder_marker.id = id;

  // Set the pose
  cylinder_marker.pose = visual_tools_->convertPose(pose);

  // Set marker size
  cylinder_marker.scale.x = radius;
  cylinder_marker.scale.y = radius;
  cylinder_marker.scale.z = height;

  // Set marker color
  cylinder_marker.color = visual_tools_->getColor(color);

  return visual_tools_->publishMarker(cylinder_marker);
}

Eigen::Vector3d RvizRenderer::stateToPoint(const ob::State* state)
{
  if (!state)
  {
    ROS_FATAL("No state found for vertex");
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

geometry_msgs::Point RvizRenderer::stateToPointMsg(const ob::State* state)
{
  if (!state)
  {
    ROS_FATAL("No state found for vertex");
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