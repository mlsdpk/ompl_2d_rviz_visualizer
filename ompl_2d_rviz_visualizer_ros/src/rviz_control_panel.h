#pragma once

#include <ompl_2d_rviz_visualizer_msgs/State.h>
#include <ros/ros.h>
#include <rviz/panel.h>

#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QPushButton>
#include <QStringList>
#include <random>

namespace ompl_2d_rviz_visualizer_ros {

static const QStringList START_GOAL_COMBO_BOX_ITEMS = {
    "Generate Random Point", "Type Manually", "Rviz Clicked Point Tool (WIP)"};

enum StartGoalComboBox { Random, Manual, Clicked };

class OMPL_ControlPanel : public rviz::Panel {
  Q_OBJECT
 public:
  OMPL_ControlPanel(QWidget *parent = 0);

  virtual void load(const rviz::Config &config);
  virtual void save(rviz::Config config) const;

 public Q_SLOTS:

 protected Q_SLOTS:
  void startCheckBoxStateChanged(int state);
  void goalCheckBoxStateChanged(int state);
  void startComboBoxActivated(int index);
  void goalComboBoxActivated(int index);
  void btn_start_clicked();
  void btn_goal_clicked();
  void reset();
  void plan();

 protected:
  void generateRandomPoint(double &x, double &y);

  QCheckBox *start_check_box_;
  QCheckBox *goal_check_box_;
  QComboBox *start_combo_box_;
  QComboBox *goal_combo_box_;
  QDoubleSpinBox *start_x_spin_box_;
  QDoubleSpinBox *start_y_spin_box_;
  QDoubleSpinBox *goal_x_spin_box_;
  QDoubleSpinBox *goal_y_spin_box_;
  QPushButton *btn_start_;
  QPushButton *btn_goal_;

  QPushButton *btn_reset_;
  QPushButton *btn_plan_;

  // ROS related
  ros::NodeHandle nh_;
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
