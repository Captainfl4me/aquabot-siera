#ifndef _SLROS_BUSMSG_CONVERSION_H_
#define _SLROS_BUSMSG_CONVERSION_H_

#include "rclcpp/rclcpp.hpp"
#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/msg/region_of_interest.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/header.hpp>
#include "aquabot_view_types.h"
#include "slros_msgconvert_utils.h"


void convertFromBus(builtin_interfaces::msg::Time& msgPtr, SL_Bus_builtin_interfaces_Time const* busPtr);
void convertToBus(SL_Bus_builtin_interfaces_Time* busPtr, const builtin_interfaces::msg::Time& msgPtr);

void convertFromBus(geometry_msgs::msg::Point& msgPtr, SL_Bus_geometry_msgs_Point const* busPtr);
void convertToBus(SL_Bus_geometry_msgs_Point* busPtr, const geometry_msgs::msg::Point& msgPtr);

void convertFromBus(geometry_msgs::msg::Pose& msgPtr, SL_Bus_geometry_msgs_Pose const* busPtr);
void convertToBus(SL_Bus_geometry_msgs_Pose* busPtr, const geometry_msgs::msg::Pose& msgPtr);

void convertFromBus(geometry_msgs::msg::PoseStamped& msgPtr, SL_Bus_geometry_msgs_PoseStamped const* busPtr);
void convertToBus(SL_Bus_geometry_msgs_PoseStamped* busPtr, const geometry_msgs::msg::PoseStamped& msgPtr);

void convertFromBus(geometry_msgs::msg::Quaternion& msgPtr, SL_Bus_geometry_msgs_Quaternion const* busPtr);
void convertToBus(SL_Bus_geometry_msgs_Quaternion* busPtr, const geometry_msgs::msg::Quaternion& msgPtr);

void convertFromBus(sensor_msgs::msg::CameraInfo& msgPtr, SL_Bus_sensor_msgs_CameraInfo const* busPtr);
void convertToBus(SL_Bus_sensor_msgs_CameraInfo* busPtr, const sensor_msgs::msg::CameraInfo& msgPtr);

void convertFromBus(sensor_msgs::msg::Image& msgPtr, SL_Bus_sensor_msgs_Image const* busPtr);
void convertToBus(SL_Bus_sensor_msgs_Image* busPtr, const sensor_msgs::msg::Image& msgPtr);

void convertFromBus(sensor_msgs::msg::PointCloud2& msgPtr, SL_Bus_sensor_msgs_PointCloud2 const* busPtr);
void convertToBus(SL_Bus_sensor_msgs_PointCloud2* busPtr, const sensor_msgs::msg::PointCloud2& msgPtr);

void convertFromBus(sensor_msgs::msg::PointField& msgPtr, SL_Bus_sensor_msgs_PointField const* busPtr);
void convertToBus(SL_Bus_sensor_msgs_PointField* busPtr, const sensor_msgs::msg::PointField& msgPtr);

void convertFromBus(sensor_msgs::msg::RegionOfInterest& msgPtr, SL_Bus_sensor_msgs_RegionOfInterest const* busPtr);
void convertToBus(SL_Bus_sensor_msgs_RegionOfInterest* busPtr, const sensor_msgs::msg::RegionOfInterest& msgPtr);

void convertFromBus(std_msgs::msg::Bool& msgPtr, SL_Bus_std_msgs_Bool const* busPtr);
void convertToBus(SL_Bus_std_msgs_Bool* busPtr, const std_msgs::msg::Bool& msgPtr);

void convertFromBus(std_msgs::msg::Float64& msgPtr, SL_Bus_std_msgs_Float64 const* busPtr);
void convertToBus(SL_Bus_std_msgs_Float64* busPtr, const std_msgs::msg::Float64& msgPtr);

void convertFromBus(std_msgs::msg::Header& msgPtr, SL_Bus_std_msgs_Header const* busPtr);
void convertToBus(SL_Bus_std_msgs_Header* busPtr, const std_msgs::msg::Header& msgPtr);


#endif
