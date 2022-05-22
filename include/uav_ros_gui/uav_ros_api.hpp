#ifndef UAV_ROS_API_HPP
#define UAV_ROS_API_HPP

#include "ros/node_handle.h"
#include <ros/ros.h>

namespace uav_ros_api {
class UAV
{
public:
  UAV();

  std::tuple<bool, std::string>  armAndTakeoff(double relative_altitude, bool enable_carrot, bool set_offboard);

private:
  ros::NodeHandle m_nh;
  ros::NodeHandle m_nh_private{ "~" };

  /* Service */
  ros::ServiceClient    m_takeoff_client;
  static constexpr auto TAKEOFF_TIMEOUT = 2.0;
};
}// namespace uav_ros_api
#endif /* UAV_ROS_API */
