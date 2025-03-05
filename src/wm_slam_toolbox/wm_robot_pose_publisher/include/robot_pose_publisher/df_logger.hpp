//
// Created by changun on 25. 2. 19.
//

#ifndef WM_ROBOT_POSE_PUBLISHER_INCLUDE_ROBOT_POSE_PUBLISHER_DF_LOGGER_HPP_
#define WM_ROBOT_POSE_PUBLISHER_INCLUDE_ROBOT_POSE_PUBLISHER_DF_LOGGER_HPP_

#include "rclcpp/rclcpp.hpp"

#define RCL_LOG_INFO(logger, format, ...)  RCLCPP_INFO(logger, "%s():%d:" format, __func__, __LINE__, ##__VA_ARGS__)
#define RCL_LOG_WARN(logger, format, ...)  RCLCPP_WARN(logger, "%s():%d:" format, __func__, __LINE__, ##__VA_ARGS__)
#define RCL_LOG_DEBUG(logger, format, ...) RCLCPP_DEBUG(logger, "%s():%d:" format, __func__, __LINE__, ##__VA_ARGS__)
#define RCL_LOG_ERROR(logger, format, ...) RCLCPP_ERROR(logger, "%s():%d:" format, __func__, __LINE__, ##__VA_ARGS__)
#define RCL_LOG_FATAL(logger, format, ...) RCLCPP_FATAL(logger, "%s():%d:" format, __func__, __LINE__, ##__VA_ARGS__)


#endif //WM_ROBOT_POSE_PUBLISHER_INCLUDE_ROBOT_POSE_PUBLISHER_DF_LOGGER_HPP_
