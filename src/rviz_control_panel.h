#pragma once

#include <ros/ros.h>
#include <rviz/panel.h>

#include <QPushButton>

namespace ompl_2d_rviz_visualizer {

class OMPL_ControlPanel : public rviz::Panel {
  Q_OBJECT
 public:
  OMPL_ControlPanel(QWidget *parent = 0);

  virtual void load(const rviz::Config &config);
  virtual void save(rviz::Config config) const;

 public Q_SLOTS:

  // Here we declare some internal slots.
 protected Q_SLOTS:
  void reset();
  void plan();

 protected:
  QPushButton *btn_reset_;
  QPushButton *btn_plan_;

  // ROS related
  ros::NodeHandle nh_;
  ros::Publisher data_publisher_;
};

}  // namespace ompl_2d_rviz_visualizer
