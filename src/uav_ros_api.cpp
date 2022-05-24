#include <uav_ros_gui/uav_ros_api.hpp>
#include <uav_ros_msgs/ArmAndTakeoff.h>
#include <uav_ros_msgs/Land.h>

uav_ros_api::UAV::UAV()
{
  ROS_INFO("[UAV] A new UAV api is created");
  m_takeoff_client =
    m_nh.serviceClient<uav_ros_msgs::ArmAndTakeoff>("uav_manager/takeoff");
  m_land_client = m_nh.serviceClient<uav_ros_msgs::Land>("uav_manager/land");

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
