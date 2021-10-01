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

#include <ros/ros.h>
#include <rviz/panel.h>

#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QDoubleValidator>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QRadioButton>
#include <QRegExp>
#include <QRegExpValidator>
#include <QScrollArea>
#include <QSlider>
#include <QStandardItemModel>
#include <QString>
#include <QStringList>
#include <QVBoxLayout>
#include <random>
#include <sstream>
#include <type_traits>
#include <variant>

namespace ompl_2d_rviz_visualizer_ros {

static const std::string PARAMETERS_NS = "ompl_planner_parameters/";

static const std::string PARAMETERS_RANGE_NS = "ompl_planner_parameters_range/";

static const QStringList START_GOAL_COMBO_BOX_ITEMS = {
    "Generate Random Point", "Type Manually", "Rviz Clicked Point Tool (WIP)"};

static const QStringList PLANNERS = {"<Not specified>", "rrt_connect",
                                     "rrt_star"};

static const QStringList PTCS = {"Duration in seconds", "Iteration number"};

static const QStringList PLANNING_OBJS = {"Minimum path length",
                                          "Maximize minimum clearance"};

static const QStringList COMBO_BOX_BOOLEAN_LIST = {"false", "true"};

enum START_GOAL_COMBO_BOX_IDS { Random, Manual, Clicked };

enum PLANNERS_IDS { INVALID, RRT_CONNECT, RRT_STAR };

enum PLANNING_OBJS_IDS { PATH_LENGTH, MAXMIN_CLEARANCE };

struct PlannerParameter {
  std::string name;
  std::variant<double, int, bool> value;
  std::string range;
};

class OMPL_ControlPanel : public rviz::Panel {
  Q_OBJECT
 public:
  OMPL_ControlPanel(QWidget *parent = 0);

  virtual void load(const rviz::Config &config);
  virtual void save(rviz::Config config) const;

 public Q_SLOTS:

 protected Q_SLOTS:
  void startCheckBoxStateChanged(bool checked);
  void goalCheckBoxStateChanged(bool checked);
  void startComboBoxActivated(int index);
  void goalComboBoxActivated(int index);
  void plannerComboBoxActivated(int index);
  void planningObjectiveComboBoxActivated(int index);
  void animationModeCheckBoxStateChanged(int state);
  void ptcComboBoxActivated(int index);
  void btn_start_clicked();
  void btn_goal_clicked();
  void reset();
  void plan();

 protected:
  using PlannerParameterList = std::vector<PlannerParameter>;

  void loadPlannerParameters();
  void setPlannerParameter(PlannerParameter &param, const std::string &name,
                           const XmlRpc::XmlRpcValue &value,
                           const std::string &range);
  bool updatePlannerParamsLayoutList(unsigned int id);

  template <typename T>
  void get_range(T &min, T &step, T &max, const std::string &range) {
    std::istringstream ss(range);
    std::string s_min, s_step, s_max;
    std::getline(ss, s_min, ':');
    std::getline(ss, s_step, ':');
    std::getline(ss, s_max);
    if constexpr (std::is_same_v<T, int>) {
      min = std::stoi(s_min);
      step = std::stoi(s_step);
      max = std::stoi(s_max);
    } else if constexpr (std::is_same_v<T, double>) {
      min = std::stod(s_min);
      step = std::stod(s_step);
      max = std::stod(s_max);
    }
  };

  void generateRandomPoint(double &x, double &y);

  QRadioButton *start_check_box_;
  QRadioButton *goal_check_box_;
  QComboBox *start_combo_box_;
  QComboBox *goal_combo_box_;
  QDoubleSpinBox *start_x_spin_box_;
  QDoubleSpinBox *start_y_spin_box_;
  QDoubleSpinBox *goal_x_spin_box_;
  QDoubleSpinBox *goal_y_spin_box_;
  QPushButton *btn_start_;
  QPushButton *btn_goal_;
  QComboBox *planner_combo_box_;
  QComboBox *planner_params_combo_box_;
  QList<QHBoxLayout *> planner_params_layout_list_;
  QVBoxLayout *planner_params_v_layout_;
  QPushButton *btn_reset_;
  QPushButton *btn_plan_;
  QComboBox *planning_objective_combo_box_;
  QDoubleSpinBox *planning_duration_spin_box_;
  QSpinBox *ptc_iteration_number_spin_box_;
  QCheckBox *animation_mode_check_box_;
  QSpinBox *animate_every_spin_box_;
  QSlider *animation_speed_slider_;
  QComboBox *ptc_combo_box_;

  std::vector<PlannerParameterList> planners_param_list_;

  int planner_id_;
  int planning_obj_id_;
  int planning_mode_;

  bool start_state_exists_;
  bool goal_state_exists_;

  double min_bound_x_;
  double max_bound_x_;
  double min_bound_y_;
  double max_bound_y_;

  std::mt19937 rn_gen_;

  // ROS related
  ros::NodeHandle nh_;
  ros::NodeHandle prv_nh_;
  ros::ServiceClient plan_request_client_;
  ros::ServiceClient reset_request_client_;
  ros::ServiceClient start_state_setter_client_;
  ros::ServiceClient goal_state_setter_client_;
  ros::ServiceClient map_bounds_client_;
};  // namespace ompl_2d_rviz_visualizer_ros

}  // namespace ompl_2d_rviz_visualizer_ros
