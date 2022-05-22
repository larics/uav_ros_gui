#include <qgridlayout.h>
#include <qgroupbox.h>
#include <qnamespace.h>
#include <uav_ros_gui/uav_operator_panel.hpp>

#include <pluginlib/class_list_macros.h>

#include <QTextEdit>
#include <QPushButton>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QLabel>
#include <QGroupBox>
#include <QDateEdit>
#include <QSlider>

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
  // ui_.setupUi(widget_);
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
  auto takeoff_slider        = new QSlider(Qt::Orientation::Horizontal);
  auto land_button           = new QPushButton("Land");
  auto pos_hold_button       = new QPushButton("Position Hold");
  auto tracker_enable_button = new QPushButton("Tracker Enable");

  // Make a takeoff slider panel
  auto takeoff_vlayout = new QHBoxLayout();
  takeoff_vlayout->addWidget(takeoff_slider_label);
  takeoff_vlayout->addWidget(takeoff_slider);

  // UAV control panel layout
  auto uav_control_panel_vlayout = new QVBoxLayout;
  uav_control_panel_vlayout->addLayout(takeoff_vlayout);
  uav_control_panel_vlayout->addWidget(land_button);
  uav_control_panel_vlayout->addWidget(pos_hold_button);
  uav_control_panel_vlayout->addWidget(tracker_enable_button);

  // UAV control panel group
  auto uav_control_panel = new QGroupBox(tr("UAV Control Panel"));
  uav_control_panel->setLayout(uav_control_panel_vlayout);
  uav_control_panel->setStyleSheet(box_style);
  uav_control_panel->setSizePolicy(sp);

  /* MISSION CONTROL */

  // Mission Control widgets
  auto mission_info_label = new QLabel(tr("Mission Info"));
  auto mission_info_text  = new QLabel("NO MESSAGES");
  mission_info_text->setStyleSheet(border_style);
  auto mission_start_button = new QPushButton("Start Mission");
  auto mission_stop_button  = new QPushButton("Stop Mission");

  // Mission Control panel layout
  auto mission_control_panel_grid = new QGridLayout();
  mission_control_panel_grid->addWidget(mission_start_button, 1, 0);
  mission_control_panel_grid->addWidget(mission_stop_button, 1, 1);
  mission_control_panel_grid->addWidget(mission_info_label, 0, 0, Qt::AlignCenter);
  mission_control_panel_grid->addWidget(mission_info_text, 0, 1);

  // Mission control panel group
  auto mission_control_panel = new QGroupBox(tr("Mission Control Panel"));
  mission_control_panel->setLayout(mission_control_panel_grid);
  mission_control_panel->setStyleSheet(box_style);
  mission_control_panel->setSizePolicy(sp);

  /* STATUS PANEL */

  // Status panel widgets
  auto carrot_status_label  = new QLabel(tr("Carrot Status"));
  auto tracker_status_label = new QLabel(tr("Tracker Status"));
  auto mission_status_label = new QLabel(tr("Mission Status"));
  auto task_status_label    = new QLabel(tr("Task Status"));

  auto carrot_status_text  = new QLabel(tr("NO MESSAGES"));
  auto tracker_status_text = new QLabel(tr("NO MESSAGES"));
  auto mission_status_text = new QLabel(tr("NO MESSAGES"));
  auto task_status_text    = new QLabel(tr("NO MESSAGES"));

  carrot_status_text->setStyleSheet(border_style);
  tracker_status_text->setStyleSheet(border_style);
  mission_status_text->setStyleSheet(border_style);
  task_status_text->setStyleSheet(border_style);

  // Status panel layout
  auto status_panel_layout = new QGridLayout();
  status_panel_layout->addWidget(carrot_status_label, 0, 0, Qt::AlignCenter);
  status_panel_layout->addWidget(carrot_status_text, 0, 1);
  status_panel_layout->addWidget(tracker_status_label, 1, 0, Qt::AlignCenter);
  status_panel_layout->addWidget(tracker_status_text, 1, 1);
  status_panel_layout->addWidget(mission_status_label, 2, 0, Qt::AlignCenter);
  status_panel_layout->addWidget(mission_status_text, 2, 1);
  status_panel_layout->addWidget(task_status_label, 3, 0, Qt::AlignCenter);
  status_panel_layout->addWidget(task_status_text, 3, 1);

  // Status panel group
  auto status_panel = new QGroupBox(tr("Status Panel"));
  status_panel->setStyleSheet(box_style);
  status_panel->setSizePolicy(sp);
  status_panel->setLayout(status_panel_layout);

  /* MAIN SETUP */

  // Outer Layer


  // Make a top row layout
  auto main_grid = new QGridLayout;
  main_grid->addWidget(uav_control_panel, 0, 0);
  main_grid->addWidget(mission_control_panel, 0, 1);
  main_grid->addWidget(status_panel, 1, 0, 1, 2);

  widget_->setLayout(main_grid);
  context.addWidget(widget_);
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
