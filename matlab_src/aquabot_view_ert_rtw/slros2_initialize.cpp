// Copyright 2022-2023 The MathWorks, Inc.
// Generated 01-Dec-2023 10:24:32
#include "slros2_initialize.h"
// aquabot_view/Publish
SimulinkPublisher<std_msgs::msg::Bool,SL_Bus_std_msgs_Bool> Pub_aquabot_view_38;
// aquabot_view/Publish1
SimulinkPublisher<std_msgs::msg::Float64,SL_Bus_std_msgs_Float64> Pub_aquabot_view_45;
// aquabot_view/Publish2
SimulinkPublisher<geometry_msgs::msg::PoseStamped,SL_Bus_geometry_msgs_PoseStamped> Pub_aquabot_view_48;
// aquabot_view/Detect boat/Subsystem/Subscribe
SimulinkSubscriber<sensor_msgs::msg::CameraInfo,SL_Bus_sensor_msgs_CameraInfo> Sub_aquabot_view_26;
// aquabot_view/SubLidar3D
SimulinkSubscriber<sensor_msgs::msg::PointCloud2,SL_Bus_sensor_msgs_PointCloud2> Sub_aquabot_view_11;
// aquabot_view/Subscribe
SimulinkSubscriber<sensor_msgs::msg::Image,SL_Bus_sensor_msgs_Image> Sub_aquabot_view_12;
