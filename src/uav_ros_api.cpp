#include <std_msgs/Bool.h>
#include "dynamic_reconfigure/BoolParameter.h"
#include "std_msgs/String.h"
#include "uav_ros_msgs/Waypoint.h"
#include "uav_ros_msgs/WaypointStatus.h"
#include <iomanip>
#include <ios>
#include <tuple>
#include <uav_ros_gui/uav_ros_api.hpp>
#include <uav_ros_msgs/ArmAndTakeoff.h>
#include <uav_ros_msgs/Land.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include <sstream>
#include <dynamic_reconfigure/Reconfigure.h>

uav_ros_api::UAV::UAV()
{
  ROS_INFO("[UAV] A new UAV api is created");
  m_takeoff_client =
    m_nh.serviceClient<uav_ros_msgs::ArmAndTakeoff>("uav_manager/takeoff");
  m_land_client = m_nh.serviceClient<uav_ros_msgs::Land>("uav_manager/land");

  m_tracker_enable  = m_nh.serviceClient<std_srvs::SetBool>("tracker/enable");
  m_tracker_reset   = m_nh.serviceClient<std_srvs::Empty>("tracker/reset");
  m_pos_hold_client = m_nh.serviceClient<std_srvs::Empty>("position_hold");
  m_start_mission_client =
    m_nh.serviceClient<std_srvs::Empty>("mission_loader/publish_waypoints");
  m_clear_mission_client = m_nh.serviceClient<std_srvs::SetBool>("clear_waypoints");
  m_task_confirm_client  = m_nh.serviceClient<std_srvs::SetBool>("pickup_task/confirm");
  m_take_wall_oriting_client = m_nh.serviceClient<std_srvs::Empty>("take_wall");
  m_take_pickup_point_client = m_nh.serviceClient<std_srvs::Empty>("take_pickup");
  m_delta_retract =
    m_nh.serviceClient<dynamic_reconfigure::Reconfigure>("/Stabilizer/set_parameters");
  m_safety_client = m_nh.serviceClient<std_srvs::SetBool>("safety/override");

  m_carrot_status_handler =
    ros_util::CreateTopicHandlerMutexed<std_msgs::String>(m_nh, "carrot/status");
  m_tracker_status_handler =
    ros_util::CreateTopicHandlerMutexed<std_msgs::String>(m_nh, "tracker/status");
  m_mission_status_handler = ros_util::CreateTopicHandlerMutexed<std_msgs::String>(
    m_nh, "waypoint_publisher/state");
  m_task_status_handler =
    ros_util::CreateTopicHandlerMutexed<std_msgs::String>(m_nh, "task/state");
  m_task_info_handler =
    ros_util::CreateTopicHandlerMutexed<std_msgs::String>(m_nh, "task/status");
  m_wpinfo_handler = ros_util::CreateTopicHandlerMutexed<uav_ros_msgs::WaypointStatus>(
    m_nh, "waypoint_status");
  m_safety_status_handler =
    ros_util::CreateTopicHandlerMutexed<std_msgs::String>(m_nh, "safety/status");

  m_challenge_started_pub = m_nh.advertise<std_msgs::Bool>("challenge_started", 1);
  ROS_INFO("[UAV] Namespace %s", m_nh.getNamespace().c_str());
}


std::string uav_ros_api::UAV::getSafetyStatus()
{
  return get_status(m_safety_status_handler);
}

std::string uav_ros_api::UAV::getCarrotStatus()
{
  return get_status(m_carrot_status_handler);
}
std::string uav_ros_api::UAV::getTrackerStatus()
{
  return get_status(m_tracker_status_handler);
}
std::string uav_ros_api::UAV::getMissionStatus()
{
  return get_status(m_mission_status_handler);
}
std::string uav_ros_api::UAV::getTaskStatus()
{
  return get_status(m_task_status_handler);
}
std::string uav_ros_api::UAV::getTaskInfo() { return get_status(m_task_info_handler); }

std::tuple<bool, std::string> uav_ros_api::UAV::startChallenge()
{
  std_msgs::Bool msg;
  msg.data = true;
  m_challenge_started_pub.publish(msg);
  return std::make_tuple<bool, std::string>(true, "Challenge started");
}

std::tuple<bool, std::string> uav_ros_api::UAV::safetyOverride() {

  std_srvs::SetBool safety_srv;
  safety_srv.request.data = true;
  auto success          = m_safety_client.call(safety_srv);
  if (!success) {
    ROS_WARN("[UAV] Unable to do safety override");
    return std::make_tuple<bool, std::string>(false, "Unable to do safety override");
  }

  return std::make_tuple<bool, std::string>(safety_srv.response.success,
                                            std::string(safety_srv.response.message));
}
std::tuple<bool, std::string> uav_ros_api::UAV::safetyOff() {

  std_srvs::SetBool safety_srv;
  safety_srv.request.data = false;
  auto success          = m_safety_client.call(safety_srv);
  if (!success) {
    ROS_WARN("[UAV] Unable to turn safety off");
    return std::make_tuple<bool, std::string>(false, "Unable to turn safety off");
  }

  return std::make_tuple<bool, std::string>(safety_srv.response.success,
                                            std::string(safety_srv.response.message));
}

std::tuple<bool, std::string> uav_ros_api::UAV::takeWallOrigin()
{

  std_srvs::Empty take_srv;
  auto            success = m_take_wall_oriting_client.call(take_srv);
  if (!success) {
    ROS_WARN("[UAV] Unable take wall origin.");
    return std::make_tuple<bool, std::string>(false, "Unable to take wall origin");
  }

  return std::make_tuple<bool, std::string>(true, "Wall origin - taken.");
}

std::tuple<bool, std::string> uav_ros_api::UAV::takePickupPoint()
{
  std_srvs::Empty take_srv;
  auto            success = m_take_pickup_point_client.call(take_srv);
  if (!success) {
    ROS_WARN("[UAV] Unable take pickup origin.");
    return std::make_tuple<bool, std::string>(false, "Unable to take pickup origin");
  }

  return std::make_tuple<bool, std::string>(true, "Pickup origin - taken.");
}

std::tuple<bool, std::string> uav_ros_api::UAV::deltaRetract()
{
  dynamic_reconfigure::Reconfigure   reconf_srv;
  dynamic_reconfigure::BoolParameter bool_param;
  bool_param.name  = "retract";
  bool_param.value = true;
  reconf_srv.request.config.bools.push_back(bool_param);
  auto success = m_delta_retract.call(reconf_srv);
  if (!success) {
    ROS_WARN("[UAV] Unable to retract manipulator.");
    return std::make_tuple<bool, std::string>(false, "Unable to retract manipulator");
  }
  return std::make_tuple<bool, std::string>(true, "Manipulator retracted");
}

std::tuple<bool, std::string> uav_ros_api::UAV::deltaExpand()
{
  dynamic_reconfigure::Reconfigure   reconf_srv;
  dynamic_reconfigure::BoolParameter bool_param;
  bool_param.name  = "retract";
  bool_param.value = false;
  reconf_srv.request.config.bools.push_back(bool_param);
  auto success = m_delta_retract.call(reconf_srv);
  if (!success) {
    ROS_WARN("[UAV] Unable to expand manipulator.");
    return std::make_tuple<bool, std::string>(false, "Unable to expand manipulator");
  }
  return std::make_tuple<bool, std::string>(true, "Manipulator expanded");
}

std::tuple<std::string, uav_ros_msgs::WaypointStatus>
  uav_ros_api::UAV::getWaypointStatus()
{
  if (m_wpinfo_handler == nullptr || !m_wpinfo_handler->isMessageRecieved()) {
    return std::make_tuple<std::string, uav_ros_msgs::WaypointStatus>("NO MESSAGES", {});
  }

  std::stringstream ss;
  auto              data = m_wpinfo_handler->getData();
  ss << std::setprecision(3) << "Position: [ " << data.current_wp.pose.pose.position.x
     << ", " << data.current_wp.pose.pose.position.x << ", "
     << data.current_wp.pose.pose.position.z << " ]"
     << "\n";
  ss << std::setprecision(3) << "Orientation: [ "
     << data.current_wp.pose.pose.orientation.x << ", "
     << data.current_wp.pose.pose.orientation.x << ", "
     << data.current_wp.pose.pose.orientation.z << ", "
     << data.current_wp.pose.pose.orientation.w << " ]"
     << "\n";
  ss << "Tasks:\n";
  for (const auto& task : data.current_wp.tasks) {
    ss << " - " << task.name << ", id = " << task.id << "\n";
  }
  ss << "Status:\n";
  ss << "Distance to waypoint: " << data.distance_to_wp << "\n";
  ss << std::boolalpha << "Waiting: " << static_cast<bool>(data.waiting_at_wp) << "\n";
  ss << std::boolalpha << "Flying: " << static_cast<bool>(data.flying_to_wp) << "\n";
  return std::make_tuple<std::string, uav_ros_msgs::WaypointStatus>(ss.str(),
                                                                    std::move(data));
}

std::tuple<bool, std::string> uav_ros_api::UAV::confirmTask()
{
  std_srvs::SetBool task_srv;
  task_srv.request.data = true;
  auto success          = m_task_confirm_client.call(task_srv);
  if (!success) {
    ROS_WARN("[UAV] Unable to confirm task");
    return std::make_tuple<bool, std::string>(false, "Unable to confirm task");
  }

  return std::make_tuple<bool, std::string>(task_srv.response.success,
                                            std::string(task_srv.response.message));
}
std::tuple<bool, std::string> uav_ros_api::UAV::refuteTask()
{
  std_srvs::SetBool task_srv;
  task_srv.request.data = false;
  auto success          = m_task_confirm_client.call(task_srv);
  if (!success) {
    ROS_WARN("[UAV] Unable to confirm task");
    return std::make_tuple<bool, std::string>(false, "Unable to confirm task");
  }

  return std::make_tuple<bool, std::string>(task_srv.response.success,
                                            std::string(task_srv.response.message));
}

std::tuple<bool, std::string> uav_ros_api::UAV::publishWaypoints()
{
  std_srvs::Empty wp_srv;
  auto            success = m_start_mission_client.call(wp_srv);
  if (!success) {
    ROS_WARN("[UAV] Unable to start mission.");
    return std::make_tuple<bool, std::string>(false, "Unable to start mission");
  }

  return std::make_tuple<bool, std::string>(true, "Mission started.");
}

std::tuple<bool, std::string> uav_ros_api::UAV::clearWaypoints()
{
  std_srvs::SetBool clear_srv;
  clear_srv.request.data = true;
  auto success           = m_clear_mission_client.call(clear_srv);
  if (!success) {
    ROS_WARN("[UAV] Unable to clear mission.");
    return std::make_tuple<bool, std::string>(false, "Unable to clear mission");
  }

  return std::make_tuple<bool, std::string>(clear_srv.response.success,
                                            std::string(clear_srv.response.message));
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
