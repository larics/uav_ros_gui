#pragma once

#include <rqt_gui_cpp/plugin.h>
#include <uav_ros_gui/uav_ros_api.hpp>

#include <ros/ros.h>

#include <QWidget>
#include <QObject>
#include <QSlider>

namespace uav_ros_gui {

class UAVOperatorPanel : public rqt_gui_cpp::Plugin {
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

  QSlider* m_takeoff_slider;

  uav_ros_api::UAV m_uav_handle;

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
signals:

  /* ======================================================================== */
  /* Signals                                                                  */
  /* ======================================================================== */



};

} // namespace
