#include "slros_busmsg_conversion.h"


// Conversions between SL_Bus_builtin_interfaces_Time and builtin_interfaces::msg::Time

void convertFromBus(builtin_interfaces::msg::Time& msgPtr, SL_Bus_builtin_interfaces_Time const* busPtr)
{
  const std::string rosMessageType("builtin_interfaces/Time");

  msgPtr.nanosec =  busPtr->nanosec;
  msgPtr.sec =  busPtr->sec;
}

void convertToBus(SL_Bus_builtin_interfaces_Time* busPtr, const builtin_interfaces::msg::Time& msgPtr)
{
  const std::string rosMessageType("builtin_interfaces/Time");

  busPtr->nanosec =  msgPtr.nanosec;
  busPtr->sec =  msgPtr.sec;
}


// Conversions between SL_Bus_geometry_msgs_Point and geometry_msgs::msg::Point

void convertFromBus(geometry_msgs::msg::Point& msgPtr, SL_Bus_geometry_msgs_Point const* busPtr)
{
  const std::string rosMessageType("geometry_msgs/Point");

  msgPtr.x =  busPtr->x;
  msgPtr.y =  busPtr->y;
  msgPtr.z =  busPtr->z;
}

void convertToBus(SL_Bus_geometry_msgs_Point* busPtr, const geometry_msgs::msg::Point& msgPtr)
{
  const std::string rosMessageType("geometry_msgs/Point");

  busPtr->x =  msgPtr.x;
  busPtr->y =  msgPtr.y;
  busPtr->z =  msgPtr.z;
}


// Conversions between SL_Bus_geometry_msgs_Pose and geometry_msgs::msg::Pose

void convertFromBus(geometry_msgs::msg::Pose& msgPtr, SL_Bus_geometry_msgs_Pose const* busPtr)
{
  const std::string rosMessageType("geometry_msgs/Pose");

  convertFromBus(msgPtr.orientation, &busPtr->orientation);
  convertFromBus(msgPtr.position, &busPtr->position);
}

void convertToBus(SL_Bus_geometry_msgs_Pose* busPtr, const geometry_msgs::msg::Pose& msgPtr)
{
  const std::string rosMessageType("geometry_msgs/Pose");

  convertToBus(&busPtr->orientation, msgPtr.orientation);
  convertToBus(&busPtr->position, msgPtr.position);
}


// Conversions between SL_Bus_geometry_msgs_PoseStamped and geometry_msgs::msg::PoseStamped

void convertFromBus(geometry_msgs::msg::PoseStamped& msgPtr, SL_Bus_geometry_msgs_PoseStamped const* busPtr)
{
  const std::string rosMessageType("geometry_msgs/PoseStamped");

  convertFromBus(msgPtr.header, &busPtr->header);
  convertFromBus(msgPtr.pose, &busPtr->pose);
}

void convertToBus(SL_Bus_geometry_msgs_PoseStamped* busPtr, const geometry_msgs::msg::PoseStamped& msgPtr)
{
  const std::string rosMessageType("geometry_msgs/PoseStamped");

  convertToBus(&busPtr->header, msgPtr.header);
  convertToBus(&busPtr->pose, msgPtr.pose);
}


// Conversions between SL_Bus_geometry_msgs_Quaternion and geometry_msgs::msg::Quaternion

void convertFromBus(geometry_msgs::msg::Quaternion& msgPtr, SL_Bus_geometry_msgs_Quaternion const* busPtr)
{
  const std::string rosMessageType("geometry_msgs/Quaternion");

  msgPtr.w =  busPtr->w;
  msgPtr.x =  busPtr->x;
  msgPtr.y =  busPtr->y;
  msgPtr.z =  busPtr->z;
}

void convertToBus(SL_Bus_geometry_msgs_Quaternion* busPtr, const geometry_msgs::msg::Quaternion& msgPtr)
{
  const std::string rosMessageType("geometry_msgs/Quaternion");

  busPtr->w =  msgPtr.w;
  busPtr->x =  msgPtr.x;
  busPtr->y =  msgPtr.y;
  busPtr->z =  msgPtr.z;
}


// Conversions between SL_Bus_sensor_msgs_CameraInfo and sensor_msgs::msg::CameraInfo

void convertFromBus(sensor_msgs::msg::CameraInfo& msgPtr, SL_Bus_sensor_msgs_CameraInfo const* busPtr)
{
  const std::string rosMessageType("sensor_msgs/CameraInfo");

  msgPtr.binning_x =  busPtr->binning_x;
  msgPtr.binning_y =  busPtr->binning_y;
  convertFromBusVariablePrimitiveArray(msgPtr.d, busPtr->d, busPtr->d_SL_Info);
  convertFromBusVariablePrimitiveArray(msgPtr.distortion_model, busPtr->distortion_model, busPtr->distortion_model_SL_Info);
  convertFromBus(msgPtr.header, &busPtr->header);
  msgPtr.height =  busPtr->height;
  convertFromBusFixedPrimitiveArray(msgPtr.k, busPtr->k);
  convertFromBusFixedPrimitiveArray(msgPtr.p, busPtr->p);
  convertFromBusFixedPrimitiveArray(msgPtr.r, busPtr->r);
  convertFromBus(msgPtr.roi, &busPtr->roi);
  msgPtr.width =  busPtr->width;
}

void convertToBus(SL_Bus_sensor_msgs_CameraInfo* busPtr, const sensor_msgs::msg::CameraInfo& msgPtr)
{
  const std::string rosMessageType("sensor_msgs/CameraInfo");

  busPtr->binning_x =  msgPtr.binning_x;
  busPtr->binning_y =  msgPtr.binning_y;
  convertToBusVariablePrimitiveArray(busPtr->d, busPtr->d_SL_Info, msgPtr.d, slros::EnabledWarning(rosMessageType, "d"));
  convertToBusVariablePrimitiveArray(busPtr->distortion_model, busPtr->distortion_model_SL_Info, msgPtr.distortion_model, slros::EnabledWarning(rosMessageType, "distortion_model"));
  convertToBus(&busPtr->header, msgPtr.header);
  busPtr->height =  msgPtr.height;
  convertToBusFixedPrimitiveArray(busPtr->k, msgPtr.k, slros::NoopWarning());
  convertToBusFixedPrimitiveArray(busPtr->p, msgPtr.p, slros::NoopWarning());
  convertToBusFixedPrimitiveArray(busPtr->r, msgPtr.r, slros::NoopWarning());
  convertToBus(&busPtr->roi, msgPtr.roi);
  busPtr->width =  msgPtr.width;
}


// Conversions between SL_Bus_sensor_msgs_Image and sensor_msgs::msg::Image

void convertFromBus(sensor_msgs::msg::Image& msgPtr, SL_Bus_sensor_msgs_Image const* busPtr)
{
  const std::string rosMessageType("sensor_msgs/Image");

  convertFromBusVariablePrimitiveArray(msgPtr.data, busPtr->data, busPtr->data_SL_Info);
  convertFromBusVariablePrimitiveArray(msgPtr.encoding, busPtr->encoding, busPtr->encoding_SL_Info);
  convertFromBus(msgPtr.header, &busPtr->header);
  msgPtr.height =  busPtr->height;
  msgPtr.is_bigendian =  busPtr->is_bigendian;
  msgPtr.step =  busPtr->step;
  msgPtr.width =  busPtr->width;
}

void convertToBus(SL_Bus_sensor_msgs_Image* busPtr, const sensor_msgs::msg::Image& msgPtr)
{
  const std::string rosMessageType("sensor_msgs/Image");

  convertToBusVariablePrimitiveArray(busPtr->data, busPtr->data_SL_Info, msgPtr.data, slros::EnabledWarning(rosMessageType, "data"));
  convertToBusVariablePrimitiveArray(busPtr->encoding, busPtr->encoding_SL_Info, msgPtr.encoding, slros::EnabledWarning(rosMessageType, "encoding"));
  convertToBus(&busPtr->header, msgPtr.header);
  busPtr->height =  msgPtr.height;
  busPtr->is_bigendian =  msgPtr.is_bigendian;
  busPtr->step =  msgPtr.step;
  busPtr->width =  msgPtr.width;
}


// Conversions between SL_Bus_sensor_msgs_PointCloud2 and sensor_msgs::msg::PointCloud2

void convertFromBus(sensor_msgs::msg::PointCloud2& msgPtr, SL_Bus_sensor_msgs_PointCloud2 const* busPtr)
{
  const std::string rosMessageType("sensor_msgs/PointCloud2");

  convertFromBusVariablePrimitiveArray(msgPtr.data, busPtr->data, busPtr->data_SL_Info);
  convertFromBusVariableNestedArray(msgPtr.fields, busPtr->fields, busPtr->fields_SL_Info);
  convertFromBus(msgPtr.header, &busPtr->header);
  msgPtr.height =  busPtr->height;
  msgPtr.is_bigendian =  busPtr->is_bigendian;
  msgPtr.is_dense =  busPtr->is_dense;
  msgPtr.point_step =  busPtr->point_step;
  msgPtr.row_step =  busPtr->row_step;
  msgPtr.width =  busPtr->width;
}

void convertToBus(SL_Bus_sensor_msgs_PointCloud2* busPtr, const sensor_msgs::msg::PointCloud2& msgPtr)
{
  const std::string rosMessageType("sensor_msgs/PointCloud2");

  convertToBusVariablePrimitiveArray(busPtr->data, busPtr->data_SL_Info, msgPtr.data, slros::EnabledWarning(rosMessageType, "data"));
  convertToBusVariableNestedArray(busPtr->fields, busPtr->fields_SL_Info, msgPtr.fields, slros::EnabledWarning(rosMessageType, "fields"));
  convertToBus(&busPtr->header, msgPtr.header);
  busPtr->height =  msgPtr.height;
  busPtr->is_bigendian =  msgPtr.is_bigendian;
  busPtr->is_dense =  msgPtr.is_dense;
  busPtr->point_step =  msgPtr.point_step;
  busPtr->row_step =  msgPtr.row_step;
  busPtr->width =  msgPtr.width;
}


// Conversions between SL_Bus_sensor_msgs_PointField and sensor_msgs::msg::PointField

void convertFromBus(sensor_msgs::msg::PointField& msgPtr, SL_Bus_sensor_msgs_PointField const* busPtr)
{
  const std::string rosMessageType("sensor_msgs/PointField");

  msgPtr.count =  busPtr->count;
  msgPtr.datatype =  busPtr->datatype;
  convertFromBusVariablePrimitiveArray(msgPtr.name, busPtr->name, busPtr->name_SL_Info);
  msgPtr.offset =  busPtr->offset;
}

void convertToBus(SL_Bus_sensor_msgs_PointField* busPtr, const sensor_msgs::msg::PointField& msgPtr)
{
  const std::string rosMessageType("sensor_msgs/PointField");

  busPtr->count =  msgPtr.count;
  busPtr->datatype =  msgPtr.datatype;
  convertToBusVariablePrimitiveArray(busPtr->name, busPtr->name_SL_Info, msgPtr.name, slros::EnabledWarning(rosMessageType, "name"));
  busPtr->offset =  msgPtr.offset;
}


// Conversions between SL_Bus_sensor_msgs_RegionOfInterest and sensor_msgs::msg::RegionOfInterest

void convertFromBus(sensor_msgs::msg::RegionOfInterest& msgPtr, SL_Bus_sensor_msgs_RegionOfInterest const* busPtr)
{
  const std::string rosMessageType("sensor_msgs/RegionOfInterest");

  msgPtr.do_rectify =  busPtr->do_rectify;
  msgPtr.height =  busPtr->height;
  msgPtr.width =  busPtr->width;
  msgPtr.x_offset =  busPtr->x_offset;
  msgPtr.y_offset =  busPtr->y_offset;
}

void convertToBus(SL_Bus_sensor_msgs_RegionOfInterest* busPtr, const sensor_msgs::msg::RegionOfInterest& msgPtr)
{
  const std::string rosMessageType("sensor_msgs/RegionOfInterest");

  busPtr->do_rectify =  msgPtr.do_rectify;
  busPtr->height =  msgPtr.height;
  busPtr->width =  msgPtr.width;
  busPtr->x_offset =  msgPtr.x_offset;
  busPtr->y_offset =  msgPtr.y_offset;
}


// Conversions between SL_Bus_std_msgs_Bool and std_msgs::msg::Bool

void convertFromBus(std_msgs::msg::Bool& msgPtr, SL_Bus_std_msgs_Bool const* busPtr)
{
  const std::string rosMessageType("std_msgs/Bool");

  msgPtr.data =  busPtr->data;
}

void convertToBus(SL_Bus_std_msgs_Bool* busPtr, const std_msgs::msg::Bool& msgPtr)
{
  const std::string rosMessageType("std_msgs/Bool");

  busPtr->data =  msgPtr.data;
}


// Conversions between SL_Bus_std_msgs_Float64 and std_msgs::msg::Float64

void convertFromBus(std_msgs::msg::Float64& msgPtr, SL_Bus_std_msgs_Float64 const* busPtr)
{
  const std::string rosMessageType("std_msgs/Float64");

  msgPtr.data =  busPtr->data;
}

void convertToBus(SL_Bus_std_msgs_Float64* busPtr, const std_msgs::msg::Float64& msgPtr)
{
  const std::string rosMessageType("std_msgs/Float64");

  busPtr->data =  msgPtr.data;
}


// Conversions between SL_Bus_std_msgs_Header and std_msgs::msg::Header

void convertFromBus(std_msgs::msg::Header& msgPtr, SL_Bus_std_msgs_Header const* busPtr)
{
  const std::string rosMessageType("std_msgs/Header");

  convertFromBusVariablePrimitiveArray(msgPtr.frame_id, busPtr->frame_id, busPtr->frame_id_SL_Info);
  convertFromBus(msgPtr.stamp, &busPtr->stamp);
}

void convertToBus(SL_Bus_std_msgs_Header* busPtr, const std_msgs::msg::Header& msgPtr)
{
  const std::string rosMessageType("std_msgs/Header");

  convertToBusVariablePrimitiveArray(busPtr->frame_id, busPtr->frame_id_SL_Info, msgPtr.frame_id, slros::EnabledWarning(rosMessageType, "frame_id"));
  convertToBus(&busPtr->stamp, msgPtr.stamp);
}

