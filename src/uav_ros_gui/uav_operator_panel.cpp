#include <qgridlayout.h>
#include <qgroupbox.h>
#include <qnamespace.h>
#include <qspinbox.h>
#include <uav_ros_gui/uav_operator_panel.hpp>

#include <pluginlib/class_list_macros.h>

#include <QThread>
#include <QTimer>
#include <QSpinBox>
#include <QTextEdit>
#include <QPushButton>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGroupBox>
#include <QDateEdit>
#include <QCheckBox>
#include <QMessageBox>

namespace uav_ros_gui {

/* ========================================================================== */
/* Constructor/Destructor                                                     */
/* ========================================================================== */

UAVOperatorPanel::UAVOperatorPanel() : rqt_gui_cpp::Plugin(), widget_(nullptr)
{

  setObjectName("UAVOperatorPanel");
}

/* ========================================================================== */
/* Initialize/Shutdown                                                        */
/* ========================================================================== */

void UAVOperatorPanel::initPlugin(qt_gui_cpp::PluginContext& context)
{
  widget_ = new QWidget();
  if (context.serialNumber() > 1) {
    widget_->setWindowTitle(widget_->windowTitle() + " ("
                            + QString::number(context.serialNumber()) + ")");
  }

  // Add a size policy
  QSizePolicy sp(QSizePolicy::Preferred, QSizePolicy::Preferred);
  sp.setHorizontalStretch(1);
  sp.setVerticalStretch(1);

  // Styles
  const QString border_style("border: 1px solid black;");
  const QString box_style("font-size: 14px;");

  /* UAV CONTROL */

  // UAV control panel buttons
  auto takeoff_slider_label  = new QLabel("Takeoff Slider");
  m_takeoff_slider           = new QSlider(Qt::Orientation::Horizontal);
  auto land_button           = new QPushButton("Land");
  auto pos_hold_button       = new QPushButton("Position Hold");
  auto tracker_enable_button = new QPushButton("Tracker Enable");
  auto tracker_reset_button  = new QPushButton("Tracker Stop");

  // Make a takeoff slider panel
  auto takeoff_vlayout = new QHBoxLayout();
  takeoff_vlayout->addWidget(takeoff_slider_label);
  takeoff_vlayout->addWidget(m_takeoff_slider);

  // UAV control panel layout
  auto uav_control_panel_vlayout = new QVBoxLayout;
  uav_control_panel_vlayout->addLayout(takeoff_vlayout);
  uav_control_panel_vlayout->addWidget(land_button);
  uav_control_panel_vlayout->addWidget(pos_hold_button);
  uav_control_panel_vlayout->addWidget(tracker_enable_button);
  uav_control_panel_vlayout->addWidget(tracker_reset_button);

  // UAV control panel group
  auto uav_control_panel = new QGroupBox(tr("UAV Control Panel"));
  uav_control_panel->setLayout(uav_control_panel_vlayout);
  uav_control_panel->setStyleSheet(box_style);
  uav_control_panel->setSizePolicy(sp);

  // UAV control panel signals/slots
  connect(m_takeoff_slider,
          &QSlider::sliderReleased,
          this,
          &UAVOperatorPanel::takeoff_slider_released);
  connect(
    land_button, &QPushButton::released, this, &UAVOperatorPanel::land_button_released);
  connect(
    pos_hold_button, &QPushButton::released, this, &UAVOperatorPanel::pos_hold_released);
  connect(tracker_enable_button,
          &QPushButton::released,
          this,
          &UAVOperatorPanel::tracker_enable_released);
  connect(tracker_reset_button,
          &QPushButton::released,
          this,
          &UAVOperatorPanel::tracker_reset_released);
  /* MISSION CONTROL */

  // Mission Control widgets
  auto mission_info_label = new QLabel(tr("Mission Info"));
  m_mission_info_text     = new QLabel("NO MESSAGES");
  m_mission_info_text->setStyleSheet(border_style);
  auto mission_start_button = new QPushButton("Start Mission");
  auto mission_stop_button  = new QPushButton("Stop Mission");
  auto task_confirm_button  = new QPushButton("Task Confirm");
  auto task_refute_button   = new QPushButton("Task Refute");

  // Mission Control panel layout
  auto mission_control_panel_grid = new QGridLayout();
  mission_control_panel_grid->addWidget(mission_start_button, 1, 0);
  mission_control_panel_grid->addWidget(mission_stop_button, 1, 1);
  mission_control_panel_grid->addWidget(mission_info_label, 0, 0, Qt::AlignCenter);
  mission_control_panel_grid->addWidget(m_mission_info_text, 0, 1);
  mission_control_panel_grid->addWidget(task_confirm_button, 2, 0);
  mission_control_panel_grid->addWidget(task_refute_button, 2, 1);

  // Mission control panel group
  auto mission_control_panel = new QGroupBox(tr("Mission Control Panel"));
  mission_control_panel->setLayout(mission_control_panel_grid);
  mission_control_panel->setStyleSheet(box_style);
  mission_control_panel->setSizePolicy(sp);

  // Mission control signals

  connect(mission_start_button,
          &QPushButton::released,
          this,
          &UAVOperatorPanel::start_mission_released);
  connect(mission_stop_button,
          &QPushButton::released,
          this,
          &UAVOperatorPanel::clear_mission_released);
  connect(task_confirm_button,
          &QPushButton::released,
          this,
          &UAVOperatorPanel::task_confirm_released);
  connect(task_refute_button,
          &QPushButton::released,
          this,
          &UAVOperatorPanel::task_refute_released);
  /* STATUS PANEL */

  // Status panel widgets
  auto carrot_status_label  = new QLabel(tr("Carrot Status"));
  auto tracker_status_label = new QLabel(tr("Tracker Status"));
  auto mission_status_label = new QLabel(tr("Mission Status"));
  auto task_status_label    = new QLabel(tr("Task Status"));
  auto task_info_label      = new QLabel(tr("Task Info"));

  m_carrot_status_text  = new QLabel(tr("NO MESSAGES"));
  m_tracker_status_text = new QLabel(tr("NO MESSAGES"));
  m_mission_status_text = new QLabel(tr("NO MESSAGES"));
  m_task_status_text    = new QLabel(tr("NO MESSAGES"));
  m_task_info_text      = new QLabel(tr("NO MESSAGES"));

  m_carrot_status_text->setStyleSheet(border_style);
  m_tracker_status_text->setStyleSheet(border_style);
  m_mission_status_text->setStyleSheet(border_style);
  m_task_status_text->setStyleSheet(border_style);
  m_task_info_text->setStyleSheet(border_style);

  // Status panel layout
  auto status_panel_layout = new QGridLayout();
  status_panel_layout->addWidget(carrot_status_label, 0, 0, Qt::AlignCenter);
  status_panel_layout->addWidget(m_carrot_status_text, 0, 1);
  status_panel_layout->addWidget(tracker_status_label, 1, 0, Qt::AlignCenter);
  status_panel_layout->addWidget(m_tracker_status_text, 1, 1);
  status_panel_layout->addWidget(mission_status_label, 2, 0, Qt::AlignCenter);
  status_panel_layout->addWidget(m_mission_status_text, 2, 1);
  status_panel_layout->addWidget(task_status_label, 3, 0, Qt::AlignCenter);
  status_panel_layout->addWidget(m_task_status_text, 3, 1);
  status_panel_layout->addWidget(task_info_label, 4, 0, Qt::AlignCenter);
  status_panel_layout->addWidget(m_task_info_text, 4, 1);

  // Status panel group
  auto status_panel = new QGroupBox(tr("Status Panel"));
  status_panel->setStyleSheet(box_style);
  status_panel->setSizePolicy(sp);
  status_panel->setLayout(status_panel_layout);

  // Status panel connections

  connect(this,
          &UAVOperatorPanel::update_carrot_status,
          m_carrot_status_text,
          &QLabel::setText);

  /* MAIN SETUP */

  // Start a thread for updating labels
  auto label_update_timer_thread = new QThread;
  label_update_timer_thread->start();

  // Make a timer and move it to thread to update labels
  auto label_update_timer = new QTimer;
  label_update_timer->setInterval(1000);
  connect(
    label_update_timer, &QTimer::timeout, this, &UAVOperatorPanel::update_status_labels);
  label_update_timer->start();
  label_update_timer->moveToThread(label_update_timer_thread);

  // Make a top row layout
  auto main_grid = new QGridLayout;
  main_grid->addWidget(uav_control_panel, 0, 0);
  main_grid->addWidget(mission_control_panel, 0, 1, 2, 1);
  main_grid->addWidget(status_panel, 1, 0);

  widget_->setLayout(main_grid);
  context.addWidget(widget_);
}

void UAVOperatorPanel::update_status_labels()
{
  m_carrot_status_text->setText(QString::fromStdString(m_uav_handle.getCarrotStatus()));
  m_tracker_status_text->setText(QString::fromStdString(m_uav_handle.getTrackerStatus()));
  m_mission_status_text->setText(QString::fromStdString(m_uav_handle.getMissionStatus()));
  m_task_status_text->setText(QString::fromStdString(m_uav_handle.getTaskStatus()));
  m_task_info_text->setText(QString::fromStdString(m_uav_handle.getTaskInfo()));
  m_mission_info_text->setText(
    QString::fromStdString(std::get<0>(m_uav_handle.getWaypointStatus())));
}

void UAVOperatorPanel::task_refute_released()
{
  ROS_INFO("[UAVOperatorPanel] task refute released");
  auto [success, message] = m_uav_handle.refuteTask();
  make_a_simple_msg_box("Task refute response", message);
}

void UAVOperatorPanel::task_confirm_released()
{
  ROS_INFO("[UAVOperatorPanel] task confirm released");
  auto [success, message] = m_uav_handle.confirmTask();
  make_a_simple_msg_box("Task confirm response", message);
}

void UAVOperatorPanel::tracker_enable_released()
{
  ROS_INFO("[UAVOperatorPanel] Tracker enable released");
  auto [success, message] = m_uav_handle.enableTracker();
  make_a_simple_msg_box("Tracker enable response", message);
}

void UAVOperatorPanel::tracker_reset_released()
{
  ROS_INFO("[UAVOperatorPanel] Tracker reset released");
  auto [success, message] = m_uav_handle.resetTracker();
  make_a_simple_msg_box("Tracker reset response", message);
}

void UAVOperatorPanel::start_mission_released()
{
  ROS_INFO("[UAVOperatorPanel] Start mission released");
  auto [success, message] = m_uav_handle.publishWaypoints();
  make_a_simple_msg_box("Start mission response", message);
}
void UAVOperatorPanel::clear_mission_released()
{
  ROS_INFO("[UAVOperatorPanel] Clear mission released");
  auto [success, message] = m_uav_handle.clearWaypoints();
  make_a_simple_msg_box("Clear mission response", message);
}
void UAVOperatorPanel::pos_hold_released()
{
  ROS_INFO("[UAVOperatorPanel] Position hold released");
  auto [success, message] = m_uav_handle.enablePositionHold();
  make_a_simple_msg_box("Position hold response", message);
}

void UAVOperatorPanel::land_button_released()
{
  auto force_land = new QCheckBox("Force Land");
  force_land->setChecked(true);
  force_land->setToolTip(
    "True if you want to cancel mission / trajectory, otherwise false");

  QMessageBox msgBox;
  msgBox.setText("DANGER: Land!");
  msgBox.setInformativeText("Do you want to land.");
  msgBox.setStandardButtons(QMessageBox::Yes | QMessageBox::Cancel);
  msgBox.setDefaultButton(QMessageBox::Cancel);
  msgBox.setWindowTitle("Landing Box");
  msgBox.setCheckBox(force_land);

  int ret = msgBox.exec();
  if (ret != QMessageBox::Yes) { return; }

  const auto force_land_check = force_land->isChecked();
  auto [success, message]     = m_uav_handle.land(force_land_check);

  make_a_simple_msg_box("Land Response", message);
}

void UAVOperatorPanel::takeoff_slider_released()
{
  const auto slider_value = m_takeoff_slider->value();
  ROS_INFO("[UAVOperatorPanel] takeoff slider released, value %d", slider_value);
  m_takeoff_slider->setValue(0);

  const auto slider_max = m_takeoff_slider->maximum();
  if (slider_value < slider_max) { return; }

  ROS_INFO("[UAVOperatorPanel] Takeoff requested");

  auto        carrot_checkbox   = new QCheckBox("Enable Carrot");
  auto        offboard_checkbox = new QCheckBox("Set Offboard");
  QMessageBox msgBox;
  msgBox.setText("DANGER: Takeoff!");
  msgBox.setInformativeText("Do you know what you are doing?");
  msgBox.setStandardButtons(QMessageBox::Yes | QMessageBox::Cancel);
  msgBox.setDefaultButton(QMessageBox::Cancel);
  msgBox.setWindowTitle("DANGER: TAKEOFF!");
  carrot_checkbox->setToolTip(
    "Enable Carrot by publishing the correct Joy message, DO NOT CLICK THIS CHECKBOX "
    "UNLESS YOU ARE IN SIMULATION!");
  offboard_checkbox->setToolTip(
    "Set GUIDED_NOGPS mode, DO NOT CLICK THIS CHECKBOX UNLESS YOU ARE IN SIMULATION!");
  msgBox.setCheckBox(carrot_checkbox);

  auto alt_label    = new QLabel(tr("Takeoff Relative Alt"));
  auto alt_spin_box = new QDoubleSpinBox;
  alt_spin_box->setRange(0, 20);
  alt_spin_box->setSingleStep(0.1);
  alt_spin_box->setValue(1.2);

  QGridLayout* grid  = qobject_cast<QGridLayout*>(msgBox.layout());
  int          index = grid->indexOf(carrot_checkbox);
  int          row, column, rowSpan, columnSpan;
  grid->getItemPosition(index, &row, &column, &rowSpan, &columnSpan);
  grid->addWidget(offboard_checkbox, row, column + 1, rowSpan, columnSpan);
  grid->addWidget(alt_label, row + 1, column, rowSpan, columnSpan);
  grid->addWidget(alt_spin_box, row + 1, column + 1, rowSpan, columnSpan);

  int ret = msgBox.exec();
  if (ret != QMessageBox::Yes) { return; }

  const auto enable_carrot = carrot_checkbox->isChecked();
  const auto set_offboard  = offboard_checkbox->isChecked();
  const auto alt_height    = alt_spin_box->value();
  ROS_INFO(
    "[UAVOperatorPanel] Takeoff [%d, %d, %.2f]", enable_carrot, set_offboard, alt_height);

  auto [success, message] =
    m_uav_handle.armAndTakeoff(alt_height, enable_carrot, set_offboard);

  make_a_simple_msg_box("Takeoff Response", message);
}

void UAVOperatorPanel::make_a_simple_msg_box(const std::string& title,
                                             const std::string& text)
{
  QMessageBox msgBox;
  msgBox.setText(QString::fromStdString(title));
  msgBox.setInformativeText(QString::fromStdString(text));
  msgBox.exec();
}

void UAVOperatorPanel::shutdownPlugin() {}

/* ========================================================================== */
/* Settings                                                                   */
/* ========================================================================== */

void UAVOperatorPanel::saveSettings(qt_gui_cpp::Settings& plugin_settings,
                                    qt_gui_cpp::Settings& instance_settings) const
{}

void UAVOperatorPanel::restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
                                       const qt_gui_cpp::Settings& instance_settings)
{}

/* ========================================================================== */
/* Methods                                                                    */
/* ========================================================================== */


/* ========================================================================== */
/* Events                                                                     */
/* ========================================================================== */


/* ========================================================================== */
/* Callbacks                                                                  */
/* ========================================================================== */


/* ========================================================================== */
/* Slots                                                                      */
/* ========================================================================== */


/* ========================================================================== */
/* Signals                                                                    */
/* ========================================================================== */


}// namespace uav_ros_gui

PLUGINLIB_EXPORT_CLASS(uav_ros_gui::UAVOperatorPanel, rqt_gui_cpp::Plugin)
