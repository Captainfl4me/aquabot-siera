/*
 * aquabot_view_types.h
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "aquabot_view".
 *
 * Model version              : 1.110
 * Simulink Coder version : 23.2 (R2023b) 01-Aug-2023
 * C++ source code generated on : Fri Dec  1 10:24:26 2023
 *
 * Target selection: ert.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Linux 64)
 * Code generation objective: Execution efficiency
 * Validation result: Not run
 */

#ifndef RTW_HEADER_aquabot_view_types_h_
#define RTW_HEADER_aquabot_view_types_h_
#include "rtwtypes.h"
#include "coder_array.h"
#include "coder_bounded_array.h"
#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_std_msgs_Bool_
#define DEFINED_TYPEDEF_FOR_SL_Bus_std_msgs_Bool_

struct SL_Bus_std_msgs_Bool
{
  boolean_T data;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_std_msgs_Float64_
#define DEFINED_TYPEDEF_FOR_SL_Bus_std_msgs_Float64_

struct SL_Bus_std_msgs_Float64
{
  real_T data;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_builtin_interfaces_Time_
#define DEFINED_TYPEDEF_FOR_SL_Bus_builtin_interfaces_Time_

struct SL_Bus_builtin_interfaces_Time
{
  int32_T sec;
  uint32_T nanosec;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_ROSVariableLengthArrayInfo_
#define DEFINED_TYPEDEF_FOR_SL_Bus_ROSVariableLengthArrayInfo_

struct SL_Bus_ROSVariableLengthArrayInfo
{
  uint32_T CurrentLength;
  uint32_T ReceivedLength;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_std_msgs_Header_
#define DEFINED_TYPEDEF_FOR_SL_Bus_std_msgs_Header_

struct SL_Bus_std_msgs_Header
{
  SL_Bus_builtin_interfaces_Time stamp;
  uint8_T frame_id[128];
  SL_Bus_ROSVariableLengthArrayInfo frame_id_SL_Info;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_geometry_msgs_Point_
#define DEFINED_TYPEDEF_FOR_SL_Bus_geometry_msgs_Point_

struct SL_Bus_geometry_msgs_Point
{
  real_T x;
  real_T y;
  real_T z;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_geometry_msgs_Quaternion_
#define DEFINED_TYPEDEF_FOR_SL_Bus_geometry_msgs_Quaternion_

struct SL_Bus_geometry_msgs_Quaternion
{
  real_T x;
  real_T y;
  real_T z;
  real_T w;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_geometry_msgs_Pose_
#define DEFINED_TYPEDEF_FOR_SL_Bus_geometry_msgs_Pose_

struct SL_Bus_geometry_msgs_Pose
{
  SL_Bus_geometry_msgs_Point position;
  SL_Bus_geometry_msgs_Quaternion orientation;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_geometry_msgs_PoseStamped_
#define DEFINED_TYPEDEF_FOR_SL_Bus_geometry_msgs_PoseStamped_

struct SL_Bus_geometry_msgs_PoseStamped
{
  SL_Bus_std_msgs_Header header;
  SL_Bus_geometry_msgs_Pose pose;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_sensor_msgs_RegionOfInterest_
#define DEFINED_TYPEDEF_FOR_SL_Bus_sensor_msgs_RegionOfInterest_

struct SL_Bus_sensor_msgs_RegionOfInterest
{
  uint32_T x_offset;
  uint32_T y_offset;
  uint32_T height;
  uint32_T width;
  boolean_T do_rectify;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_sensor_msgs_CameraInfo_
#define DEFINED_TYPEDEF_FOR_SL_Bus_sensor_msgs_CameraInfo_

struct SL_Bus_sensor_msgs_CameraInfo
{
  SL_Bus_std_msgs_Header header;
  uint32_T height;
  uint32_T width;
  uint8_T distortion_model[128];
  SL_Bus_ROSVariableLengthArrayInfo distortion_model_SL_Info;
  real_T d[128];
  SL_Bus_ROSVariableLengthArrayInfo d_SL_Info;
  real_T k[9];
  real_T r[9];
  real_T p[12];
  uint32_T binning_x;
  uint32_T binning_y;
  SL_Bus_sensor_msgs_RegionOfInterest roi;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_sensor_msgs_PointField_
#define DEFINED_TYPEDEF_FOR_SL_Bus_sensor_msgs_PointField_

struct SL_Bus_sensor_msgs_PointField
{
  uint8_T name[128];
  SL_Bus_ROSVariableLengthArrayInfo name_SL_Info;
  uint32_T offset;
  uint8_T datatype;
  uint32_T count;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_sensor_msgs_PointCloud2_
#define DEFINED_TYPEDEF_FOR_SL_Bus_sensor_msgs_PointCloud2_

struct SL_Bus_sensor_msgs_PointCloud2
{
  SL_Bus_std_msgs_Header header;
  uint32_T height;
  uint32_T width;
  SL_Bus_sensor_msgs_PointField fields[16];
  SL_Bus_ROSVariableLengthArrayInfo fields_SL_Info;
  boolean_T is_bigendian;
  uint32_T point_step;
  uint32_T row_step;
  uint8_T data[960000];
  SL_Bus_ROSVariableLengthArrayInfo data_SL_Info;
  boolean_T is_dense;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_sensor_msgs_Image_
#define DEFINED_TYPEDEF_FOR_SL_Bus_sensor_msgs_Image_

struct SL_Bus_sensor_msgs_Image
{
  SL_Bus_std_msgs_Header header;
  uint32_T height;
  uint32_T width;
  uint8_T encoding[128];
  SL_Bus_ROSVariableLengthArrayInfo encoding_SL_Info;
  uint8_T is_bigendian;
  uint32_T step;
  uint8_T data[1244160];
  SL_Bus_ROSVariableLengthArrayInfo data_SL_Info;
};

#endif

/* Custom Type definition for MATLAB Function: '<S4>/MATLAB Function1' */
#ifndef struct_c_vision_internal_calibration_T
#define struct_c_vision_internal_calibration_T

struct c_vision_internal_calibration_T
{
  int32_T __dummy;
};

#endif                              /* struct_c_vision_internal_calibration_T */

#ifndef struct_cameraParameters_aquabot_view_T
#define struct_cameraParameters_aquabot_view_T

struct cameraParameters_aquabot_view_T
{
  c_vision_internal_calibration_T UndistortMap;
};

#endif                              /* struct_cameraParameters_aquabot_view_T */

#ifndef struct_c_vision_internal_codegen_cam_T
#define struct_c_vision_internal_codegen_cam_T

struct c_vision_internal_codegen_cam_T
{
  int32_T __dummy;
};

#endif                              /* struct_c_vision_internal_codegen_cam_T */

#ifndef struct_c_images_geotrans_internal_ri_T
#define struct_c_images_geotrans_internal_ri_T

struct c_images_geotrans_internal_ri_T
{
  int32_T __dummy;
};

#endif                              /* struct_c_images_geotrans_internal_ri_T */

#ifndef struct_c_pointclouds_internal_codege_T
#define struct_c_pointclouds_internal_codege_T

struct c_pointclouds_internal_codege_T
{
  int32_T __dummy;
};

#endif                              /* struct_c_pointclouds_internal_codege_T */

/* Custom Type definition for MATLAB Function: '<S4>/MATLAB Function2' */
#ifndef struct_sBCDbgMZyfjRVWzdpNZ1R9_aquabo_T
#define struct_sBCDbgMZyfjRVWzdpNZ1R9_aquabo_T

struct sBCDbgMZyfjRVWzdpNZ1R9_aquabo_T
{
  real_T Area;
};

#endif                              /* struct_sBCDbgMZyfjRVWzdpNZ1R9_aquabo_T */

/* Custom Type definition for MATLABSystem: '<S9>/SourceBlock' */
#include "rmw/qos_profiles.h"
#include "rmw/types.h"
#include "rmw/types.h"
#include "rmw/types.h"
#ifndef struct_ros_slros2_internal_block_Sub_T
#define struct_ros_slros2_internal_block_Sub_T

struct ros_slros2_internal_block_Sub_T
{
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  boolean_T isSetupComplete;
};

#endif                              /* struct_ros_slros2_internal_block_Sub_T */

#ifndef struct_ros_slros2_internal_block_Pub_T
#define struct_ros_slros2_internal_block_Pub_T

struct ros_slros2_internal_block_Pub_T
{
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  boolean_T isSetupComplete;
};

#endif                              /* struct_ros_slros2_internal_block_Pub_T */

#ifndef struct_ros_slros2_internal_block_Rea_T
#define struct_ros_slros2_internal_block_Rea_T

struct ros_slros2_internal_block_Rea_T
{
  int32_T isInitialized;
};

#endif                              /* struct_ros_slros2_internal_block_Rea_T */

#ifndef struct_c_ros_slros2_internal_block_P_T
#define struct_c_ros_slros2_internal_block_P_T

struct c_ros_slros2_internal_block_P_T
{
  SL_Bus_sensor_msgs_PointCloud2 BusStruct;
  uint32_T NumFields;
};

#endif                              /* struct_c_ros_slros2_internal_block_P_T */

/* Custom Type definition for MATLAB Function: '<S4>/MATLAB Function' */
#ifndef struct_s8812E1qfIkP21aQTfYt8iB_aquab_T
#define struct_s8812E1qfIkP21aQTfYt8iB_aquab_T

struct s8812E1qfIkP21aQTfYt8iB_aquab_T
{
  real_T Centroid[2];
};

#endif                              /* struct_s8812E1qfIkP21aQTfYt8iB_aquab_T */

#ifndef struct_ros_slros2_internal_block_R_m_T
#define struct_ros_slros2_internal_block_R_m_T

struct ros_slros2_internal_block_R_m_T
{
  int32_T isInitialized;
  uint8_T Image[1244160];
  uint32_T ImageSize[2];
};

#endif                              /* struct_ros_slros2_internal_block_R_m_T */

/* Custom Type definition for MATLAB Function: '<S4>/MATLAB Function2' */

/* Custom Type definition for MATLAB Function: '<S4>/MATLAB Function' */
#ifndef struct_cell_wrap_6_aquabot_view_T
#define struct_cell_wrap_6_aquabot_view_T

struct cell_wrap_6_aquabot_view_T
{
  coder::array<real_T, 1U> f1;
};

#endif                                 /* struct_cell_wrap_6_aquabot_view_T */

/* Custom Type definition for MATLAB Function: '<S4>/MATLAB Function2' */

/* Custom Type definition for MATLAB Function: '<S4>/MATLAB Function1' */
#ifndef struct_pointCloud_aquabot_view_T
#define struct_pointCloud_aquabot_view_T

struct pointCloud_aquabot_view_T
{
  boolean_T matlabCodegenIsDeleted;
  real32_T Location[90000];
  coder::array<uint8_T, 2U> Color;
  coder::array<real32_T, 2U> Normal;
  coder::array<real32_T, 2U> Intensity;
  coder::array<real32_T, 2U> RangeData;
  void* Kdtree;
  void* LocationHandle;
  boolean_T HasKdtreeConstructed;
  boolean_T HasLocationHandleAllocated;
  coder::array<c_pointclouds_internal_codege_T, 2U> PointCloudArrayData;
};

#endif                                 /* struct_pointCloud_aquabot_view_T */

/* Custom Type definition for MATLAB Function: '<S4>/MATLAB Function2' */

/* Custom Type definition for MATLAB Function: '<S4>/MATLAB Function' */

/* Custom Type definition for MATLAB Function: '<S4>/MATLAB Function2' */

/* Custom Type definition for MATLAB Function: '<S4>/MATLAB Function2' */
#ifndef struct_s_K3X5z0rBQ6KcDg7d33fBWG_aqua_T
#define struct_s_K3X5z0rBQ6KcDg7d33fBWG_aqua_T

struct s_K3X5z0rBQ6KcDg7d33fBWG_aqua_T
{
  real_T Area;
  real_T Centroid[2];
  real_T BoundingBox[4];
  real_T MajorAxisLength;
  real_T MinorAxisLength;
  real_T Eccentricity;
  real_T Orientation;
  coder::empty_bounded_array<boolean_T, 2U> Image;
  coder::empty_bounded_array<boolean_T, 2U> FilledImage;
  real_T FilledArea;
  real_T EulerNumber;
  real_T Extrema[16];
  real_T EquivDiameter;
  real_T Extent;
  coder::array<real_T, 1U> PixelIdxList;
  coder::empty_bounded_array<real_T, 2U> PixelList;
  real_T Perimeter;
  real_T Circularity;
  coder::empty_bounded_array<real_T, 1U> PixelValues;
  real_T WeightedCentroid[2];
  real_T MeanIntensity;
  real_T MinIntensity;
  real_T MaxIntensity;
  coder::empty_bounded_array<real_T, 2U> SubarrayIdx;
  real_T SubarrayIdxLengths[2];
};

#endif                              /* struct_s_K3X5z0rBQ6KcDg7d33fBWG_aqua_T */

/* Custom Type definition for MATLAB Function: '<S4>/MATLAB Function2' */

/* Custom Type definition for MATLAB Function: '<S4>/MATLAB Function' */
#ifndef struct_s_48rvm6kxzsa25RQQXhDnzF_aqua_T
#define struct_s_48rvm6kxzsa25RQQXhDnzF_aqua_T

struct s_48rvm6kxzsa25RQQXhDnzF_aqua_T
{
  real_T Area;
  real_T Centroid[2];
  real_T BoundingBox[4];
  real_T MajorAxisLength;
  real_T MinorAxisLength;
  real_T Eccentricity;
  real_T Orientation;
  coder::empty_bounded_array<boolean_T, 2U> Image;
  coder::empty_bounded_array<boolean_T, 2U> FilledImage;
  real_T FilledArea;
  real_T EulerNumber;
  real_T Extrema[16];
  real_T EquivDiameter;
  real_T Extent;
  coder::array<real_T, 1U> PixelIdxList;
  coder::array<real_T, 2U> PixelList;
  real_T Perimeter;
  real_T Circularity;
  coder::empty_bounded_array<real_T, 1U> PixelValues;
  real_T WeightedCentroid[2];
  real_T MeanIntensity;
  real_T MinIntensity;
  real_T MaxIntensity;
  coder::empty_bounded_array<real_T, 2U> SubarrayIdx;
  real_T SubarrayIdxLengths[2];
};

#endif                              /* struct_s_48rvm6kxzsa25RQQXhDnzF_aqua_T */

/* Custom Type definition for MATLAB Function: '<S4>/MATLAB Function' */

/* Parameters (default storage) */
typedef struct P_aquabot_view_T_ P_aquabot_view_T;

/* Forward declaration for rtModel */
typedef struct tag_RTM_aquabot_view_T RT_MODEL_aquabot_view_T;

#endif                                 /* RTW_HEADER_aquabot_view_types_h_ */
