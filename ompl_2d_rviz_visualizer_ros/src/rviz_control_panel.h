#pragma once
#include <ompl_2d_rviz_visualizer_msgs/State.h>
#include <ros/ros.h>
#include <rviz/panel.h>

#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QRadioButton>
#include <QString>
#include <QStringList>
#include <QVBoxLayout>
#include <random>
#include <variant>

namespace ompl_2d_rviz_visualizer_ros {

static const QStringList START_GOAL_COMBO_BOX_ITEMS = {
    "Generate Random Point", "Type Manually", "Rviz Clicked Point Tool (WIP)"};

static const QStringList PLANNERS = {"<Not specified>", "rrt_connect",
                                     "rrt_star"};

struct AnyGet {
  std::string operator()(bool value) { return value ? "true" : "false"; }
  std::string operator()(int value) { return std::to_string(value); }
  std::string operator()(double value) { return std::to_string(value); }
};

enum PLANNERS_IDS { INVALID, RRT_CONNECT, RRT_STAR };

enum StartGoalComboBox { Random, Manual, Clicked };

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
  void btn_start_clicked();
  void btn_goal_clicked();
  void reset();
  void plan();

 protected:
  bool updatePlannerParamsLayoutList(unsigned int id);
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

  std::vector<std::map<QString, std::variant<double, int, bool>>>
      planner_params_;
  int planner_id_;

  // ROS related
  ros::NodeHandle nh_;
  ros::NodeHandle prv_nh_;
  ros::Publisher data_publisher_;
  ros::Publisher start_state_publisher_;
  ros::Publisher goal_state_publisher_;

  bool start_state_exists_;
  bool goal_state_exists_;

  double min_bound_x_;
  double max_bound_x_;
  double min_bound_y_;
  double max_bound_y_;

  std::mt19937 rn_gen_;
};

}  // namespace ompl_2d_rviz_visualizer_ros
