#include "rviz_control_panel.h"

#include <std_msgs/UInt8.h>

namespace ompl_2d_rviz_visualizer_ros {
OMPL_ControlPanel::OMPL_ControlPanel(QWidget* parent)
    : rviz::Panel(parent),
      nh_{"ompl_2d_rviz_visualizer_nodelet/ompl_planner_parameters"},
      prv_nh_{"~ompl_controlpanel"},
      planner_id_{0} {
  ////////////////////
  // Start layout
  ////////////////////
  QHBoxLayout* start_hlayout = new QHBoxLayout;

  start_check_box_ = new QRadioButton("Start", this);
  connect(start_check_box_, SIGNAL(toggled(bool)), this,
          SLOT(startCheckBoxStateChanged(bool)));

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
  // Goal layout
  ////////////////////
  QHBoxLayout* goal_hlayout = new QHBoxLayout;

  goal_check_box_ = new QRadioButton("Goal", this);
  connect(goal_check_box_, SIGNAL(toggled(bool)), this,
          SLOT(goalCheckBoxStateChanged(bool)));

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

  /////////////////////////////
  // Start and Goal Group Box
  /////////////////////////////
  QGroupBox* query_gp_box = new QGroupBox(QString("Query"));
  QVBoxLayout* query_v_layout = new QVBoxLayout;
  query_v_layout->addLayout(start_hlayout);
  query_v_layout->addLayout(goal_hlayout);
  query_gp_box->setLayout(query_v_layout);
  /////////////////////////////

  ////////////////////////////////////////
  // Planner related layout
  ////////////////////////////////////////
  planner_combo_box_ = new QComboBox;
  planner_combo_box_->addItems(PLANNERS);
  connect(planner_combo_box_, SIGNAL(activated(int)), this,
          SLOT(plannerComboBoxActivated(int)));

  QGroupBox* planner_params_gp_box = new QGroupBox(QString("Parameters"));
  planner_params_v_layout_ = new QVBoxLayout;
  planner_params_gp_box->setLayout(planner_params_v_layout_);

  QVBoxLayout* planner_vlayout = new QVBoxLayout;
  planner_vlayout->addWidget(planner_combo_box_);
  planner_vlayout->addWidget(planner_params_gp_box);

  QGroupBox* planner_gp_box = new QGroupBox(QString("Planner"));
  planner_gp_box->setLayout(planner_vlayout);
  ////////////////////////////////////////

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
  layout->addWidget(query_gp_box);
  layout->addWidget(planner_gp_box);
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

  // hardcoded planner related parameters
  // TODO: Read from the config yaml file or parameter server and
  // store in the vector
  std::map<QString, std::variant<double, int, bool>> rrt_connect_params;
  rrt_connect_params["maxDistance"] = 0.0;
  rrt_connect_params["addIntermediateStates"] = false;

  std::map<QString, std::variant<double, int, bool>> rrt_star_params;
  rrt_star_params["maxDistance"] = 0.0;
  rrt_star_params["rewireFactor"] = 0.0;
  rrt_star_params["goalBias"] = 0.0;

  planner_params_.push_back(rrt_connect_params);
  planner_params_.push_back(rrt_star_params);
  ////////////////////////////////////////////////////////

  data_publisher_ = prv_nh_.advertise<std_msgs::UInt8>("plan_request", 1);

  start_state_publisher_ =
      prv_nh_.advertise<ompl_2d_rviz_visualizer_msgs::State>("start_state", 1);

  goal_state_publisher_ =
      prv_nh_.advertise<ompl_2d_rviz_visualizer_msgs::State>("goal_state", 1);

  btn_reset_->setEnabled(false);
  btn_plan_->setEnabled(true);
}

void OMPL_ControlPanel::startCheckBoxStateChanged(bool checked) {
  if (checked) {
    start_combo_box_->setEnabled(true);
    btn_start_->setEnabled(true);

    // make sure spinbox is enabled
    if (start_combo_box_->currentIndex() == Manual) {
      start_x_spin_box_->setEnabled(true);
      start_y_spin_box_->setEnabled(true);
    }
  } else {
    start_combo_box_->setEnabled(false);
    start_x_spin_box_->setEnabled(false);
    start_y_spin_box_->setEnabled(false);
    btn_start_->setEnabled(false);
  }
}

void OMPL_ControlPanel::goalCheckBoxStateChanged(bool checked) {
  if (checked) {
    goal_combo_box_->setEnabled(true);
    btn_goal_->setEnabled(true);

    // make sure spinbox is enabled
    if (goal_combo_box_->currentIndex() == Manual) {
      goal_x_spin_box_->setEnabled(true);
      goal_y_spin_box_->setEnabled(true);
    }
  } else {
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

void OMPL_ControlPanel::plannerComboBoxActivated(int index) {
  // update planner id
  planner_id_ = index;
  // delete the parameters
  for (const auto layout : planner_params_layout_list_) {
    QLayoutItem* item;
    while ((item = layout->takeAt(0)) != nullptr) {
      delete item->widget();
      delete item;
    }
    delete layout;
  }
  planner_params_layout_list_.clear();
  updatePlannerParamsLayoutList(static_cast<unsigned>(index));
}

bool OMPL_ControlPanel::updatePlannerParamsLayoutList(unsigned int id) {
  if (id == PLANNERS_IDS::INVALID) return false;

  for (auto it = planner_params_[id - 1].begin();
       it != planner_params_[id - 1].end(); it++) {
    QLabel* label = new QLabel(it->first);
    // TODO: Custom validators need to be implemented for line edits
    QLineEdit* line_edit = new QLineEdit;
    line_edit->setFixedWidth(150);
    line_edit->setText(
        QString::fromStdString(std::visit(AnyGet{}, it->second)));
    QHBoxLayout* params_hlayout = new QHBoxLayout;
    params_hlayout->addWidget(label);
    params_hlayout->addWidget(line_edit);
    planner_params_layout_list_.append(params_hlayout);
  }
  for (const auto layout : planner_params_layout_list_) {
    planner_params_v_layout_->addLayout(layout);
  }
  return true;
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
    // read the parameters and update the parameter server
    for (const auto layout : planner_params_layout_list_) {
      auto param_name = static_cast<QLabel*>(layout->itemAt(0)->widget())
                            ->text()
                            .toStdString();
      const auto param_val =
          static_cast<QLineEdit*>(layout->itemAt(1)->widget())->text();
      param_name = PLANNERS[planner_id_].toStdString() + "/" + param_name;

      if (param_val == "true")
        nh_.setParam(param_name, true);
      else if (param_val == "false")
        nh_.setParam(param_name, false);
      else
        nh_.setParam(param_name, std::stod(param_val.toStdString()));
    }

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

std::string to_string(const std::variant<double, int, bool>& in) {
  return std::visit(AnyGet{}, in);
}
}  // namespace ompl_2d_rviz_visualizer_ros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(ompl_2d_rviz_visualizer_ros::OMPL_ControlPanel,
                       rviz::Panel)