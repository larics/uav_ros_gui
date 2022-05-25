#include <uav_ros_gui/uav_ros_api.hpp>
#include <uav_ros_msgs/ArmAndTakeoff.h>
#include <uav_ros_msgs/Land.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>

uav_ros_api::UAV::UAV()
{
  ROS_INFO("[UAV] A new UAV api is created");
  m_takeoff_client =
    m_nh.serviceClient<uav_ros_msgs::ArmAndTakeoff>("uav_manager/takeoff");
  m_land_client = m_nh.serviceClient<uav_ros_msgs::Land>("uav_manager/land");

  m_tracker_enable  = m_nh.serviceClient<std_srvs::SetBool>("tracker/enable");
  m_tracker_reset   = m_nh.serviceClient<std_srvs::Empty>("tracker/reset");
  m_pos_hold_client = m_nh.serviceClient<std_srvs::Empty>("position_hold");

  m_carrot_status_handler =
    ros_util::CreateTopicHandlerMutexed<std_msgs::String>(m_nh, "carrot/status");

  ROS_INFO("[UAV] Nemaspace %s", m_nh.getNamespace().c_str());
}

std::string uav_ros_api::UAV::getCarrotStatus()
{
  if (m_carrot_status_handler == nullptr
      || !m_carrot_status_handler->isMessageRecieved()) {
    return "NO MESSAGES";
  }
  return m_carrot_status_handler->getData().data;
}

std::tuple<bool, std::string> uav_ros_api::UAV::enablePositionHold()
{

  std_srvs::Empty pos_hold_srv;
  auto            success = m_pos_hold_client.call(pos_hold_srv);
  if (!success) {
    ROS_WARN("[UAV] Unable to activate position hold.");
    return std::make_tuple<bool, std::string>(false, "Unable to activate position hold.");
  }

  return std::make_tuple<bool, std::string>(true, "Position hold activated");
}
std::tuple<bool, std::string> uav_ros_api::UAV::enableTracker()
{
  std_srvs::SetBool tracker_srv;
  tracker_srv.request.data = true;
  auto success             = m_tracker_enable.call(tracker_srv);
  if (!success) {
    ROS_WARN("[UAV] Unable to activate tracker.");
    return std::make_tuple<bool, std::string>(false, "Unable to activate tracker");
  }

  return std::make_tuple<bool, std::string>(tracker_srv.response.success,
                                            std::string(tracker_srv.response.message));
}


std::tuple<bool, std::string> uav_ros_api::UAV::resetTracker()
{
  std_srvs::Empty tracker_srv;
  auto            success = m_tracker_reset.call(tracker_srv);
  if (!success) {
    ROS_WARN("[UAV] Unable to deactivate tracker.");
    return std::make_tuple<bool, std::string>(false, "Unable to deactivate tracker");
  }

  return std::make_tuple<bool, std::string>(true, "Tracker deactivated");
}


std::tuple<bool, std::string> uav_ros_api::UAV::armAndTakeoff(
  double relative_altitude = 1,
  bool   enable_carrot     = false,
  bool   set_offboard      = false)
{
  uav_ros_msgs::ArmAndTakeoff takeoff_request;
  takeoff_request.request.enable_carrot = enable_carrot;
  takeoff_request.request.set_offboard  = set_offboard;
  takeoff_request.request.rel_alt       = relative_altitude;
  takeoff_request.request.timeout       = TAKEOFF_TIMEOUT;
  auto success                          = m_takeoff_client.call(takeoff_request);
  if (!success) {
    ROS_WARN("[UAV] Arm and Takeoff called unsuccesfully");
    return std::make_tuple<bool, std::string>(false, "Takeoff call failed");
  }

  ROS_INFO("[UAV] Takeoff: %d -  %s",
           takeoff_request.response.success,
           takeoff_request.response.message.c_str());
  return std::make_tuple<bool, std::string>(
    takeoff_request.response.success, std::string(takeoff_request.response.message));
}

std::tuple<bool, std::string> uav_ros_api::UAV::land(bool force_land)
{
  uav_ros_msgs::Land land_srv;
  land_srv.request.force_land = force_land;
  auto success                = m_land_client.call(land_srv);
  if (!success) {
    ROS_WARN("[UAV] Land called unsuccesfully");
    return std::make_tuple<bool, std::string>(false, "Land call failed");
  }

  ROS_INFO(
    "[UAV] Land: %d - %s", land_srv.response.success, land_srv.response.message.c_str());
  return std::make_tuple<bool, std::string>(land_srv.response.success,
                                            std::string(land_srv.response.message));
}
