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

  std::vector<PlannerParameterList> planners_param_list_;

  int planner_id_;
  int planning_obj_id_;

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
};

}  // namespace ompl_2d_rviz_visualizer_ros
