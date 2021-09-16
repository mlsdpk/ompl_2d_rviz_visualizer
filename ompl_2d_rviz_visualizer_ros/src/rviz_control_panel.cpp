#include "rviz_control_panel.h"

#include <std_msgs/UInt8.h>

#include <QHBoxLayout>
#include <QLabel>
#include <QVBoxLayout>

namespace ompl_2d_rviz_visualizer_ros {
OMPL_ControlPanel::OMPL_ControlPanel(QWidget* parent) : rviz::Panel(parent) {
  ////////////////////
  // Start group box
  ////////////////////
  QHBoxLayout* start_hlayout = new QHBoxLayout;

  start_check_box_ = new QCheckBox("Start", this);
  connect(start_check_box_, SIGNAL(stateChanged(int)), this,
          SLOT(startCheckBoxStateChanged(int)));

  start_combo_box_ = new QComboBox(this);
  start_combo_box_->addItems(START_GOAL_COMBO_BOX_ITEMS);
  connect(start_combo_box_, SIGNAL(activated(int)), this,
          SLOT(startComboBoxActivated(int)));

  start_x_spin_box_ = new QDoubleSpinBox(this);
  QHBoxLayout* start_x_hlayout = new QHBoxLayout;
  start_x_hlayout->addWidget(new QLabel(QString("X:")));
  start_x_hlayout->addWidget(start_x_spin_box_);

  start_y_spin_box_ = new QDoubleSpinBox(this);
  QHBoxLayout* start_y_hlayout = new QHBoxLayout;
  start_y_hlayout->addWidget(new QLabel(QString("Y:")));
  start_y_hlayout->addWidget(start_y_spin_box_);

  btn_start_ = new QPushButton(this);
  btn_start_->setText("Send");
  connect(btn_start_, SIGNAL(clicked()), this, SLOT(btn_start_clicked()));

  start_hlayout->addWidget(start_check_box_);
  start_hlayout->addWidget(start_combo_box_);
  start_hlayout->addLayout(start_x_hlayout);
  start_hlayout->addLayout(start_y_hlayout);
  start_hlayout->addWidget(btn_start_);

  start_combo_box_->setEnabled(false);
  start_x_spin_box_->setEnabled(false);
  start_y_spin_box_->setEnabled(false);
  btn_start_->setEnabled(false);
  ////////////////////

  ////////////////////
  // Goal group box
  ////////////////////
  QHBoxLayout* goal_hlayout = new QHBoxLayout;

  goal_check_box_ = new QCheckBox("Goal", this);
  connect(goal_check_box_, SIGNAL(stateChanged(int)), this,
          SLOT(goalCheckBoxStateChanged(int)));

  goal_combo_box_ = new QComboBox(this);
  goal_combo_box_->addItems(START_GOAL_COMBO_BOX_ITEMS);
  connect(goal_combo_box_, SIGNAL(activated(int)), this,
          SLOT(goalComboBoxActivated(int)));

  goal_x_spin_box_ = new QDoubleSpinBox(this);
  QHBoxLayout* goal_x_hlayout = new QHBoxLayout;
  goal_x_hlayout->addWidget(new QLabel(QString("X:")));
  goal_x_hlayout->addWidget(goal_x_spin_box_);

  goal_y_spin_box_ = new QDoubleSpinBox(this);
  QHBoxLayout* goal_y_hlayout = new QHBoxLayout;
  goal_y_hlayout->addWidget(new QLabel(QString("Y:")));
  goal_y_hlayout->addWidget(goal_y_spin_box_);

  btn_goal_ = new QPushButton(this);
  btn_goal_->setText("Send");
  connect(btn_goal_, SIGNAL(clicked()), this, SLOT(btn_goal_clicked()));

  goal_hlayout->addWidget(goal_check_box_);
  goal_hlayout->addWidget(goal_combo_box_);
  goal_hlayout->addLayout(goal_x_hlayout);
  goal_hlayout->addLayout(goal_y_hlayout);
  goal_hlayout->addWidget(btn_goal_);

  goal_combo_box_->setEnabled(false);
  goal_x_spin_box_->setEnabled(false);
  goal_y_spin_box_->setEnabled(false);
  btn_goal_->setEnabled(false);
  ////////////////////

  /////////////////////////////////////////
  // Horizontal Layout for reset and plan
  /////////////////////////////////////////
  btn_reset_ = new QPushButton(this);
  btn_reset_->setText("Reset");
  connect(btn_reset_, SIGNAL(clicked()), this, SLOT(reset()));

  btn_plan_ = new QPushButton(this);
  btn_plan_->setText("Plan");
  connect(btn_plan_, SIGNAL(clicked()), this, SLOT(plan()));

  QHBoxLayout* hlayout = new QHBoxLayout;
  hlayout->addWidget(btn_reset_);
  hlayout->addWidget(btn_plan_);
  /////////////////////////////////////////

  // Final vertical layout
  QVBoxLayout* layout = new QVBoxLayout;
  layout->addLayout(start_hlayout);
  layout->addLayout(goal_hlayout);
  layout->addLayout(hlayout);

  setLayout(layout);
  //////////////////////////////////////////////

  start_state_exists_ = false;
  goal_state_exists_ = false;

  // min and max bounds must obtain from map in the future
  min_bound_x_ = -5.0;
  max_bound_x_ = 5.0;
  min_bound_y_ = -5.0;
  max_bound_y_ = 5.0;

  start_x_spin_box_->setRange(min_bound_x_, max_bound_x_);
  start_y_spin_box_->setRange(min_bound_y_, max_bound_y_);
  goal_x_spin_box_->setRange(min_bound_x_, max_bound_x_);
  goal_y_spin_box_->setRange(min_bound_y_, max_bound_y_);

  data_publisher_ = nh_.advertise<std_msgs::UInt8>(
      "/ompl_2d_rviz_visualizer_control_panel", 1);

  start_state_publisher_ =
      nh_.advertise<ompl_2d_rviz_visualizer_msgs::State>("/start_state", 1);

  goal_state_publisher_ =
      nh_.advertise<ompl_2d_rviz_visualizer_msgs::State>("/goal_state", 1);

  btn_reset_->setEnabled(false);
  btn_plan_->setEnabled(true);
}

void OMPL_ControlPanel::startCheckBoxStateChanged(int state) {
  if (state == Qt::Checked) {
    start_combo_box_->setEnabled(true);
    btn_start_->setEnabled(true);

    // make sure spinbox is enabled
    if (start_combo_box_->currentIndex() == Manual) {
      start_x_spin_box_->setEnabled(true);
      start_y_spin_box_->setEnabled(true);
    }
  } else if (state == Qt::Unchecked) {
    start_combo_box_->setEnabled(false);
    start_x_spin_box_->setEnabled(false);
    start_y_spin_box_->setEnabled(false);
    btn_start_->setEnabled(false);
  }
}

void OMPL_ControlPanel::goalCheckBoxStateChanged(int state) {
  if (state == Qt::Checked) {
    goal_combo_box_->setEnabled(true);
    btn_goal_->setEnabled(true);

    // make sure spinbox is enabled
    if (goal_combo_box_->currentIndex() == Manual) {
      goal_x_spin_box_->setEnabled(true);
      goal_y_spin_box_->setEnabled(true);
    }
  } else if (state == Qt::Unchecked) {
    goal_combo_box_->setEnabled(false);
    goal_x_spin_box_->setEnabled(false);
    goal_y_spin_box_->setEnabled(false);
    btn_goal_->setEnabled(false);
  }
}

void OMPL_ControlPanel::startComboBoxActivated(int index) {
  if (index == Random) {
    start_x_spin_box_->setEnabled(false);
    start_y_spin_box_->setEnabled(false);
  } else if (index == Manual) {
    start_x_spin_box_->setEnabled(true);
    start_y_spin_box_->setEnabled(true);
  } else if (index == Clicked) {
    start_x_spin_box_->setEnabled(false);
    start_y_spin_box_->setEnabled(false);
  }
}

void OMPL_ControlPanel::goalComboBoxActivated(int index) {
  if (index == Random) {
    goal_x_spin_box_->setEnabled(false);
    goal_y_spin_box_->setEnabled(false);
  } else if (index == Manual) {
    goal_x_spin_box_->setEnabled(true);
    goal_y_spin_box_->setEnabled(true);
  } else if (index == Clicked) {
    goal_x_spin_box_->setEnabled(false);
    goal_y_spin_box_->setEnabled(false);
  }
}

void OMPL_ControlPanel::generateRandomPoint(double& x, double& y) {
  std::random_device rd;
  rn_gen_ = std::mt19937(rd());
  std::uniform_real_distribution<double> dis_x(min_bound_x_, max_bound_x_);
  std::uniform_real_distribution<double> dis_y(min_bound_y_, max_bound_y_);
  x = dis_x(rn_gen_);
  y = dis_y(rn_gen_);
}

void OMPL_ControlPanel::btn_start_clicked() {
  // the point needs to be collision-free
  ompl_2d_rviz_visualizer_msgs::State msg;
  if (start_combo_box_->currentIndex() == Random) {
    generateRandomPoint(msg.x, msg.y);
  } else if (start_combo_box_->currentIndex() == Manual) {
    //
  } else if (start_combo_box_->currentIndex() == Clicked) {
    //
  }
  start_x_spin_box_->setValue(msg.x);
  start_y_spin_box_->setValue(msg.y);
  start_state_publisher_.publish(msg);
  start_state_exists_ = true;
}

void OMPL_ControlPanel::btn_goal_clicked() {
  ompl_2d_rviz_visualizer_msgs::State msg;
  if (goal_combo_box_->currentIndex() == Random) {
    generateRandomPoint(msg.x, msg.y);
  } else if (goal_combo_box_->currentIndex() == Manual) {
    //
  } else if (goal_combo_box_->currentIndex() == Clicked) {
    //
  }
  goal_x_spin_box_->setValue(msg.x);
  goal_y_spin_box_->setValue(msg.y);
  goal_state_publisher_.publish(msg);
  goal_state_exists_ = true;
}

void OMPL_ControlPanel::reset() {
  ROS_INFO_STREAM_NAMED("ompl_control_panel", "RESET button pressed.");

  std_msgs::UInt8 msg;
  msg.data = 1;
  data_publisher_.publish(msg);
  btn_reset_->setEnabled(false);
  btn_plan_->setEnabled(true);

  start_check_box_->setEnabled(true);
  goal_check_box_->setEnabled(true);
  start_state_exists_ = false;
  goal_state_exists_ = false;
}

void OMPL_ControlPanel::plan() {
  ROS_INFO_STREAM_NAMED("ompl_control_panel", "PLAN button pressed.");

  if (start_state_exists_ && goal_state_exists_) {
    std_msgs::UInt8 msg;
    msg.data = 2;
    data_publisher_.publish(msg);
    btn_reset_->setEnabled(true);
    btn_plan_->setEnabled(false);

    start_check_box_->setEnabled(false);
    start_combo_box_->setEnabled(false);
    start_x_spin_box_->setEnabled(false);
    start_y_spin_box_->setEnabled(false);
    btn_start_->setEnabled(false);
    goal_check_box_->setEnabled(false);
    goal_combo_box_->setEnabled(false);
    goal_x_spin_box_->setEnabled(false);
    goal_y_spin_box_->setEnabled(false);
    btn_goal_->setEnabled(false);
    start_check_box_->setChecked(false);
    goal_check_box_->setChecked(false);
  }
}

void OMPL_ControlPanel::save(rviz::Config config) const {
  rviz::Panel::save(config);
}

void OMPL_ControlPanel::load(const rviz::Config& config) {
  rviz::Panel::load(config);
}
}  // namespace ompl_2d_rviz_visualizer_ros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(ompl_2d_rviz_visualizer_ros::OMPL_ControlPanel,
                       rviz::Panel)