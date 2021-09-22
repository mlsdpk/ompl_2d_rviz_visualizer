#include "rviz_control_panel.h"

#include <ompl_2d_rviz_visualizer_msgs/Plan.h>
#include <ompl_2d_rviz_visualizer_msgs/Reset.h>
#include <ompl_2d_rviz_visualizer_msgs/State.h>

namespace ompl_2d_rviz_visualizer_ros {
OMPL_ControlPanel::OMPL_ControlPanel(QWidget* parent)
    : rviz::Panel(parent),
      nh_{"ompl_2d_rviz_visualizer_nodelet"},
      prv_nh_{"~ompl_controlpanel"},
      planner_id_{PLANNERS_IDS::INVALID},
      planning_obj_id_{PLANNING_OBJS_IDS::PATH_LENGTH} {
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

  start_check_box_->setChecked(true);
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

  QScrollArea* scroll_area = new QScrollArea;
  scroll_area->setWidgetResizable(true);
  QWidget* temp = new QWidget;
  scroll_area->setWidget(temp);
  planner_params_v_layout_ = new QVBoxLayout;
  temp->setLayout(planner_params_v_layout_);

  QVBoxLayout* planner_vlayout = new QVBoxLayout;
  planner_vlayout->addWidget(planner_combo_box_);
  planner_vlayout->addWidget(new QLabel(QString("Parameters")));
  planner_vlayout->addWidget(scroll_area);

  QGroupBox* planner_gp_box = new QGroupBox(QString("Planner"));
  planner_gp_box->setLayout(planner_vlayout);

  ////////////////////////////////////////

  /////////////////////////////////////////
  // Planning objective and duration
  /////////////////////////////////////////
  planning_objective_combo_box_ = new QComboBox;
  planning_objective_combo_box_->addItems(PLANNING_OBJS);
  connect(planning_objective_combo_box_, SIGNAL(activated(int)), this,
          SLOT(planningObjectiveComboBoxActivated(int)));

  planning_duration_spin_box_ = new QDoubleSpinBox;
  planning_duration_spin_box_->setMinimum(0.0);
  planning_duration_spin_box_->setSingleStep(0.1);
  planning_duration_spin_box_->setFixedWidth(150);

  QHBoxLayout* planning_hlayout = new QHBoxLayout;
  planning_hlayout->addWidget(new QLabel(QString("Planning objective:")));
  planning_hlayout->addWidget(planning_objective_combo_box_);
  planning_hlayout->addWidget(new QLabel(QString("Planning duration:")));
  planning_hlayout->addWidget(planning_duration_spin_box_);
  /////////////////////////////////////////

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
  layout->addLayout(planning_hlayout);
  layout->addLayout(hlayout);

  setLayout(layout);
  startCheckBoxStateChanged(true);
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

  loadPlannerParameters();

  btn_reset_->setEnabled(false);
  btn_plan_->setEnabled(true);
  ///////////////////////////////////////////////////////////////

  // ROS related
  plan_request_client_ =
      nh_.serviceClient<ompl_2d_rviz_visualizer_msgs::Plan>("plan_request");

  reset_request_client_ =
      nh_.serviceClient<ompl_2d_rviz_visualizer_msgs::Reset>("reset_request");

  start_state_setter_client_ =
      nh_.serviceClient<ompl_2d_rviz_visualizer_msgs::State>("set_start_state");

  goal_state_setter_client_ =
      nh_.serviceClient<ompl_2d_rviz_visualizer_msgs::State>("set_goal_state");

  ROS_INFO_STREAM_NAMED("Control Panel",
                        "Waiting for service servers to be connected...");
  plan_request_client_.waitForExistence();
  reset_request_client_.waitForExistence();
  start_state_setter_client_.waitForExistence();
  goal_state_setter_client_.waitForExistence();
  ROS_INFO_STREAM_NAMED("Control Panel", "Service servers found.");
}

void OMPL_ControlPanel::loadPlannerParameters() {
  std::vector<std::string> ros_param_names;
  nh_.getParamNames(ros_param_names);
  for (auto i = 1; i < PLANNERS.length(); ++i) {
    PlannerParameterList planner_param_list;
    for (const auto& n : ros_param_names) {
      std::size_t found;
      if ((found = n.find(PARAMETERS_NS)) != std::string::npos) {
        if (n.find(PLANNERS[i].toStdString()) == std::string::npos) continue;

        std::string ompl_param_name;
        ompl_param_name = n.substr(found + PARAMETERS_NS.length() +
                                   PLANNERS[i].toStdString().length() + 1);

        // get the planner parameter
        XmlRpc::XmlRpcValue xml_param;
        nh_.getParam(n, xml_param);

        // get the planner parameter range
        std::string parameter_range_ros_param = PARAMETERS_RANGE_NS +
                                                PLANNERS[i].toStdString() +
                                                "/" + ompl_param_name;
        std::string planner_param_range;
        nh_.getParam(parameter_range_ros_param, planner_param_range);

        // set planner parameter
        PlannerParameter planner_param;
        setPlannerParameter(planner_param, ompl_param_name, xml_param,
                            planner_param_range);

        planner_param_list.push_back(planner_param);
      }
    }
    planners_param_list_.push_back(planner_param_list);
  }
}

void OMPL_ControlPanel::setPlannerParameter(PlannerParameter& param,
                                            const std::string& name,
                                            const XmlRpc::XmlRpcValue& value,
                                            const std::string& range) {
  param.name = name;
  if (value.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
    param.value = static_cast<double>(value);
  } else if (value.getType() == XmlRpc::XmlRpcValue::TypeInt) {
    param.value = static_cast<int>(value);
  } else if (value.getType() == XmlRpc::XmlRpcValue::TypeBoolean) {
    param.value = static_cast<bool>(value);
  }
  param.range = range;
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

void OMPL_ControlPanel::planningObjectiveComboBoxActivated(int index) {
  planning_obj_id_ = index;
}

bool OMPL_ControlPanel::updatePlannerParamsLayoutList(unsigned int id) {
  if (id == PLANNERS_IDS::INVALID) return false;

  for (const auto& param : planners_param_list_[id - 1]) {
    QHBoxLayout* params_hlayout = new QHBoxLayout;
    QLabel* label = new QLabel(QString::fromStdString(param.name));
    params_hlayout->addWidget(label);

    // double spin box for doubles
    if (const double* pval = std::get_if<double>(&param.value)) {
      QDoubleSpinBox* param_widget = new QDoubleSpinBox;
      double min, step, max;
      get_range<double>(min, step, max, param.range);
      param_widget->setRange(min, max);
      param_widget->setValue(*pval);
      param_widget->setSingleStep(step);
      params_hlayout->addWidget(param_widget);
    }
    // spin box for ints
    else if (const int* pval = std::get_if<int>(&param.value)) {
      QSpinBox* param_widget = new QSpinBox;
      int min, step, max;
      get_range<int>(min, step, max, param.range);
      param_widget->setRange(min, max);
      param_widget->setStepType(QAbstractSpinBox::DefaultStepType);
      param_widget->setValue(*pval);
      param_widget->setSingleStep(step);
      params_hlayout->addWidget(param_widget);
    }
    // combo box for booleans
    else if (const bool* pval = std::get_if<bool>(&param.value)) {
      QComboBox* param_widget = new QComboBox;
      param_widget->addItems(COMBO_BOX_BOOLEAN_LIST);
      param_widget->setCurrentIndex(*pval ? 1 : 0);
      params_hlayout->addWidget(param_widget);
    }

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
    generateRandomPoint(msg.request.x, msg.request.y);
  } else if (start_combo_box_->currentIndex() == Manual) {
    //
  } else if (start_combo_box_->currentIndex() == Clicked) {
    //
  }
  start_x_spin_box_->setValue(msg.request.x);
  start_y_spin_box_->setValue(msg.request.y);
  if (start_state_setter_client_.call(msg)) {
    if (msg.response.success)
      ROS_DEBUG("set_start_state service calling succeeded.");
  } else {
    ROS_ERROR("Failed to call set_start_state service.");
    exit(-1);
  }
  start_state_exists_ = true;
}

void OMPL_ControlPanel::btn_goal_clicked() {
  ompl_2d_rviz_visualizer_msgs::State msg;
  if (goal_combo_box_->currentIndex() == Random) {
    generateRandomPoint(msg.request.x, msg.request.y);
  } else if (goal_combo_box_->currentIndex() == Manual) {
    //
  } else if (goal_combo_box_->currentIndex() == Clicked) {
    //
  }
  goal_x_spin_box_->setValue(msg.request.x);
  goal_y_spin_box_->setValue(msg.request.y);
  if (goal_state_setter_client_.call(msg)) {
    if (msg.response.success)
      ROS_DEBUG("set_goal_state service calling succeeded.");
  } else {
    ROS_ERROR("Failed to call set_goal_state service.");
    exit(-1);
  }
  goal_state_exists_ = true;
}

void OMPL_ControlPanel::reset() {
  ROS_INFO_STREAM_NAMED("ompl_control_panel", "RESET button pressed.");

  ompl_2d_rviz_visualizer_msgs::Reset msg;
  msg.request.clear_graph = true;
  if (reset_request_client_.call(msg)) {
    if (msg.response.success) ROS_DEBUG("Reset Service calling succeeded.");
  } else {
    ROS_ERROR("Failed to call reset_request service.");
    exit(-1);
  }
  btn_reset_->setEnabled(false);
  btn_plan_->setEnabled(true);

  start_check_box_->setEnabled(true);
  goal_check_box_->setEnabled(true);
  start_check_box_->setChecked(true);
  startCheckBoxStateChanged(true);
}

void OMPL_ControlPanel::plan() {
  ROS_INFO_STREAM_NAMED("ompl_control_panel", "PLAN button pressed.");

  if (start_state_exists_ && goal_state_exists_ &&
      planner_id_ != PLANNERS_IDS::INVALID) {
    // read the parameters and update the parameter server
    for (const auto layout : planner_params_layout_list_) {
      auto param_name = static_cast<QLabel*>(layout->itemAt(0)->widget())
                            ->text()
                            .toStdString();
      param_name = "ompl_planner_parameters/" +
                   PLANNERS[planner_id_].toStdString() + "/" + param_name;
      // get the widget type
      std::string widget_type(
          layout->itemAt(1)->widget()->metaObject()->className());
      if (widget_type == "QDoubleSpinBox") {
        const auto param_val =
            static_cast<QDoubleSpinBox*>(layout->itemAt(1)->widget())->value();
        nh_.setParam(param_name, param_val);
      } else if (widget_type == "QSpinBox") {
        const auto param_val =
            static_cast<QSpinBox*>(layout->itemAt(1)->widget())->value();
        nh_.setParam(param_name, param_val);
      } else if (widget_type == "QComboBox") {
        const auto idx = static_cast<QComboBox*>(layout->itemAt(1)->widget())
                             ->currentIndex();
        if (COMBO_BOX_BOOLEAN_LIST[idx] == "false")
          nh_.setParam(param_name, false);
        else
          nh_.setParam(param_name, true);
      }
    }

    ompl_2d_rviz_visualizer_msgs::Plan msg;
    msg.request.planner_id = planner_id_;
    msg.request.objective_id = planning_obj_id_;
    msg.request.duration = planning_duration_spin_box_->value();
    if (plan_request_client_.call(msg)) {
      if (msg.response.success) ROS_DEBUG("Plan Service calling succeeded.");
    } else {
      ROS_ERROR("Failed to call plan_request service.");
      exit(-1);
    }

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