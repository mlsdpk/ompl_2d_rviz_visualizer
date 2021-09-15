#include "rviz_control_panel.h"

#include <std_msgs/UInt8.h>

#include <QHBoxLayout>
#include <QLabel>

namespace ompl_2d_rviz_visualizer {
OMPL_ControlPanel::OMPL_ControlPanel(QWidget* parent) : rviz::Panel(parent) {
  // Create a push button
  btn_reset_ = new QPushButton(this);
  btn_reset_->setText("Reset");
  connect(btn_reset_, SIGNAL(clicked()), this, SLOT(reset()));

  btn_plan_ = new QPushButton(this);
  btn_plan_->setText("Plan");
  connect(btn_plan_, SIGNAL(clicked()), this, SLOT(plan()));

  // Horizontal Layout
  QHBoxLayout* hlayout = new QHBoxLayout;
  hlayout->addWidget(btn_reset_);
  hlayout->addWidget(btn_plan_);

  setLayout(hlayout);

  data_publisher_ = nh_.advertise<std_msgs::UInt8>(
      "/ompl_2d_rviz_visualizer_control_panel", 1);

  btn_reset_->setEnabled(false);
  btn_plan_->setEnabled(true);
}

void OMPL_ControlPanel::reset() {
  ROS_INFO_STREAM_NAMED("ompl_control_panel", "RESET button pressed.");

  std_msgs::UInt8 msg;
  msg.data = 1;
  data_publisher_.publish(msg);
  btn_reset_->setEnabled(false);
  btn_plan_->setEnabled(true);
}

void OMPL_ControlPanel::plan() {
  ROS_INFO_STREAM_NAMED("ompl_control_panel", "PLAN button pressed.");

  std_msgs::UInt8 msg;
  msg.data = 2;
  data_publisher_.publish(msg);
  btn_reset_->setEnabled(true);
  btn_plan_->setEnabled(false);
}

void OMPL_ControlPanel::save(rviz::Config config) const {
  rviz::Panel::save(config);
}

void OMPL_ControlPanel::load(const rviz::Config& config) {
  rviz::Panel::load(config);
}
}  // namespace ompl_2d_rviz_visualizer

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(ompl_2d_rviz_visualizer::OMPL_ControlPanel, rviz::Panel)