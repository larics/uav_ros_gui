#include <uav_ros_gui/uav_ros_api.hpp>
#include <uav_ros_msgs/ArmAndTakeoff.h>

uav_ros_api::UAV::UAV()
{
  ROS_INFO("[UAV] A new UAV api is created");
  m_takeoff_client =
    m_nh.serviceClient<uav_ros_msgs::ArmAndTakeoff>("uav_manager/takeoff");

  //TODO: Add namespace
  ROS_INFO("[UAV] Nemaspace %s", m_nh.getNamespace().c_str());
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