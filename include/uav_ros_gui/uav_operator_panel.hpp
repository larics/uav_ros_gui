#pragma once

#include <rqt_gui_cpp/plugin.h>
#include <uav_ros_gui/uav_ros_api.hpp>

#include <ros/ros.h>

#include <QLabel>
#include <QWidget>
#include <QObject>
#include <QSlider>

namespace uav_ros_gui {

class UAVOperatorPanel : public rqt_gui_cpp::Plugin
{
  Q_OBJECT

public:
  /* ======================================================================== */
  /* Constructor/Destructor                                                   */
  /* ======================================================================== */

  UAVOperatorPanel();

  /* ======================================================================== */
  /* Initialize/Shutdown                                                      */
  /* ======================================================================== */

  void initPlugin(qt_gui_cpp::PluginContext& context) override;

  void shutdownPlugin() override;

  /* ======================================================================== */
  /* Settings                                                                 */
  /* ======================================================================== */

  void saveSettings(qt_gui_cpp::Settings& plugin_settings,
                    qt_gui_cpp::Settings& instance_settings) const override;

  void restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
                       const qt_gui_cpp::Settings& instance_settings) override;

private:
  /* ======================================================================== */
  /* Constants                                                                */
  /* ======================================================================== */

  const std::string TAG = "UAVOperatorPanel";

  /* ======================================================================== */
  /* Variables                                                                */
  /* ======================================================================== */

  QWidget* widget_;

  ros::NodeHandle nh_;

  QSlider*         m_takeoff_slider;
  QTimer*          m_label_update_timer;
  uav_ros_api::UAV m_uav_handle;

  QLabel* m_carrot_status_text;
  QLabel* m_tracker_status_text;
  QLabel* m_mission_status_text;
  QLabel* m_task_status_text;
  QLabel* m_task_info_text;
  QLabel* m_mission_info_text;
  QLabel* m_safety_status_text;
  QLabel* m_fence_status_text;

  /* ======================================================================== */
  /* Methods                                                                  */
  /* ======================================================================== */

  void make_a_simple_msg_box(const std::string& title, const std::string& text);

  /* ======================================================================== */
  /* Events                                                                   */
  /* ======================================================================== */


  /* ======================================================================== */
  /* Callbacks                                                                */
  /* ======================================================================== */


protected slots:

  /* ======================================================================== */
  /* Slots                                                                    */
  /* ======================================================================== */

  void takeoff_slider_released();
  void land_button_released();
  void tracker_enable_released();
  void tracker_reset_released();
  void pos_hold_released();
  void start_mission_released();
  void clear_mission_released();
  void task_confirm_released();
  void task_refute_released();
  void update_status_labels();
  void manipulator_retract_released();
  void manipulator_expand_released();
  void take_wall_point_released();
  void take_brick_point_released();
  void challenge_started_released();
  void safety_override_released();
  void safety_off_released();
  void fence_on_released();
  void fence_off_released();
  void clear_clicker_released();
  void upload_clicker_released();

signals:

  /* ======================================================================== */
  /* Signals                                                                  */
  /* ======================================================================== */
  void update_carrot_status(QString);
};

}// namespace uav_ros_gui
