#ifndef UAV_ROS_API_HPP
#define UAV_ROS_API_HPP

#include "ros/node_handle.h"
#include "ros/service_client.h"
#include <ros/ros.h>
#include <uav_ros_lib/topic_handler.hpp>
#include <std_msgs/String.h>
#include <uav_ros_msgs/WaypointStatus.h>

namespace uav_ros_api {
using StringTopicHandler = ros_util::TopicHandlerMutexed<std_msgs::String>;
using WPInfoHandler      = ros_util::TopicHandlerMutexed<uav_ros_msgs::WaypointStatus>;

class UAV
{
public:
  UAV();

  std::tuple<bool, std::string> armAndTakeoff(double relative_altitude,
                                              bool   enable_carrot,
                                              bool   set_offboard);

  std::tuple<bool, std::string> land(bool force_land = true);
  std::tuple<bool, std::string> enableTracker();
  std::tuple<bool, std::string> resetTracker();
  std::tuple<bool, std::string> enablePositionHold();
  std::tuple<bool, std::string> publishWaypoints();
  std::tuple<bool, std::string> clearWaypoints();
  std::tuple<bool, std::string> confirmTask();
  std::tuple<bool, std::string> refuteTask();

  std::string                                           getCarrotStatus();
  std::string                                           getTrackerStatus();
  std::string                                           getMissionStatus();
  std::string                                           getTaskStatus();
  std::string                                           getTaskInfo();
  std::tuple<std::string, uav_ros_msgs::WaypointStatus> getWaypointStatus();

private:
  template<typename T> std::string get_status(T& handler)
  {
    if (handler == nullptr || !handler->isMessageRecieved()) { return "NO MESSAGES"; }
    return handler->getData().data;
  }

  ros::NodeHandle m_nh;
  ros::NodeHandle m_nh_private{ "~" };

  /* Service */
  ros::ServiceClient m_takeoff_client;
  ros::ServiceClient m_land_client;
  ros::ServiceClient m_tracker_enable;
  ros::ServiceClient m_tracker_reset;
  ros::ServiceClient m_pos_hold_client;
  ros::ServiceClient m_start_mission_client;
  ros::ServiceClient m_clear_mission_client;
  ros::ServiceClient m_task_confirm_client;

  StringTopicHandler::Ptr m_carrot_status_handler;
  StringTopicHandler::Ptr m_tracker_status_handler;
  StringTopicHandler::Ptr m_mission_status_handler;
  StringTopicHandler::Ptr m_task_status_handler;
  StringTopicHandler::Ptr m_task_info_handler;
  WPInfoHandler::Ptr      m_wpinfo_handler;

  static constexpr auto TAKEOFF_TIMEOUT = 2.0;
};
}// namespace uav_ros_api
#endif /* UAV_ROS_API */
