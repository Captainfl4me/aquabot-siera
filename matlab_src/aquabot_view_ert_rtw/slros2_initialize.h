// Copyright 2022-2023 The MathWorks, Inc.
// Generated 01-Dec-2023 10:24:32
#ifndef _SLROS2_INITIALIZE_H_
#define _SLROS2_INITIALIZE_H_
#include "aquabot_view_types.h"
// Generic pub-sub header
#include "slros2_generic_pubsub.h"
// Generic service header
#include "slros2_generic_service.h"
extern rclcpp::Node::SharedPtr SLROSNodePtr;
#ifndef SET_QOS_VALUES
#define SET_QOS_VALUES(qosStruct, hist, dep, dur, rel)  \
    {                                                   \
        qosStruct.history = hist;                       \
        qosStruct.depth = dep;                          \
        qosStruct.durability = dur;                     \
        qosStruct.reliability = rel;                    \
    }
#endif
inline rclcpp::QoS getQOSSettingsFromRMW(const rmw_qos_profile_t& qosProfile) {
    rclcpp::QoS qos(rclcpp::QoSInitialization::from_rmw(qosProfile));
    if (RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL == qosProfile.durability) {
        qos.transient_local();
    } else {
        qos.durability_volatile();
    }
    if (RMW_QOS_POLICY_RELIABILITY_RELIABLE == qosProfile.reliability) {
        qos.reliable();
    } else {
        qos.best_effort();
    }
    return qos;
}
// aquabot_view/Publish
extern SimulinkPublisher<std_msgs::msg::Bool,SL_Bus_std_msgs_Bool> Pub_aquabot_view_38;
// aquabot_view/Publish1
extern SimulinkPublisher<std_msgs::msg::Float64,SL_Bus_std_msgs_Float64> Pub_aquabot_view_45;
// aquabot_view/Publish2
extern SimulinkPublisher<geometry_msgs::msg::PoseStamped,SL_Bus_geometry_msgs_PoseStamped> Pub_aquabot_view_48;
// aquabot_view/Detect boat/Subsystem/Subscribe
extern SimulinkSubscriber<sensor_msgs::msg::CameraInfo,SL_Bus_sensor_msgs_CameraInfo> Sub_aquabot_view_26;
// aquabot_view/SubLidar3D
extern SimulinkSubscriber<sensor_msgs::msg::PointCloud2,SL_Bus_sensor_msgs_PointCloud2> Sub_aquabot_view_11;
// aquabot_view/Subscribe
extern SimulinkSubscriber<sensor_msgs::msg::Image,SL_Bus_sensor_msgs_Image> Sub_aquabot_view_12;
#endif
