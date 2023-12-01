/*
 * aquabot_view.cpp
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

#include "aquabot_view.h"
#include "rtwtypes.h"
#include "coder_array.h"
#include "aquabot_view_types.h"
#include <string.h>
#include "aquabot_view_private.h"

extern "C"
{

#include "rt_nonfinite.h"

}

#include <stddef.h>
#include <math.h>
#include "rmw/qos_profiles.h"
#include "rmw/types.h"

void aquabot_view::aquabot_v_SystemCore_setup_myfs
  (ros_slros2_internal_block_Sub_T *obj)
{
  rmw_qos_durability_policy_t durability;
  rmw_qos_history_policy_t history;
  rmw_qos_profile_t qos_profile;
  rmw_qos_reliability_policy_t reliability;
  static const char_T tmp[45] = { '/', 'w', 'a', 'm', 'v', '/', 's', 'e', 'n',
    's', 'o', 'r', 's', '/', 'l', 'i', 'd', 'a', 'r', 's', '/', 'l', 'i', 'd',
    'a', 'r', '_', 'w', 'a', 'm', 'v', '_', 's', 'e', 'n', 's', 'o', 'r', '/',
    'p', 'o', 'i', 'n', 't', 's' };

  obj->isInitialized = 1;
  qos_profile = rmw_qos_profile_default;
  history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
  SET_QOS_VALUES(qos_profile, history, (size_t)1.0, durability, reliability);
  for (int32_T i = 0; i < 45; i++) {
    aquabot_view_B.b_zeroDelimTopic_m[i] = tmp[i];
  }

  aquabot_view_B.b_zeroDelimTopic_m[45] = '\x00';
  Sub_aquabot_view_11.createSubscriber(&aquabot_view_B.b_zeroDelimTopic_m[0],
    qos_profile);
  obj->isSetupComplete = true;
}

void aquabot_view::aquabot__SystemCore_setup_myfsq
  (ros_slros2_internal_block_Sub_T *obj)
{
  rmw_qos_durability_policy_t durability;
  rmw_qos_history_policy_t history;
  rmw_qos_profile_t qos_profile;
  rmw_qos_reliability_policy_t reliability;
  static const char_T tmp[58] = { '/', 'w', 'a', 'm', 'v', '/', 's', 'e', 'n',
    's', 'o', 'r', 's', '/', 'c', 'a', 'm', 'e', 'r', 'a', 's', '/', 'm', 'a',
    'i', 'n', '_', 'c', 'a', 'm', 'e', 'r', 'a', '_', 's', 'e', 'n', 's', 'o',
    'r', '/', 'o', 'p', 't', 'i', 'c', 'a', 'l', '/', 'i', 'm', 'a', 'g', 'e',
    '_', 'r', 'a', 'w' };

  obj->isInitialized = 1;
  qos_profile = rmw_qos_profile_default;
  history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
  SET_QOS_VALUES(qos_profile, history, (size_t)1.0, durability, reliability);
  for (int32_T i = 0; i < 58; i++) {
    aquabot_view_B.b_zeroDelimTopic_n[i] = tmp[i];
  }

  aquabot_view_B.b_zeroDelimTopic_n[58] = '\x00';
  Sub_aquabot_view_12.createSubscriber(&aquabot_view_B.b_zeroDelimTopic_n[0],
    qos_profile);
  obj->isSetupComplete = true;
}

void aquabot_view::aquabot_view_SystemCore_setup(ros_slros2_internal_block_Sub_T
  *obj)
{
  rmw_qos_durability_policy_t durability;
  rmw_qos_history_policy_t history;
  rmw_qos_profile_t qos_profile;
  rmw_qos_reliability_policy_t reliability;
  static const char_T tmp[60] = { '/', 'w', 'a', 'm', 'v', '/', 's', 'e', 'n',
    's', 'o', 'r', 's', '/', 'c', 'a', 'm', 'e', 'r', 'a', 's', '/', 'm', 'a',
    'i', 'n', '_', 'c', 'a', 'm', 'e', 'r', 'a', '_', 's', 'e', 'n', 's', 'o',
    'r', '/', 'o', 'p', 't', 'i', 'c', 'a', 'l', '/', 'c', 'a', 'm', 'e', 'r',
    'a', '_', 'i', 'n', 'f', 'o' };

  obj->isInitialized = 1;
  qos_profile = rmw_qos_profile_default;
  history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
  SET_QOS_VALUES(qos_profile, history, (size_t)1.0, durability, reliability);
  for (int32_T i = 0; i < 60; i++) {
    aquabot_view_B.b_zeroDelimTopic[i] = tmp[i];
  }

  aquabot_view_B.b_zeroDelimTopic[60] = '\x00';
  Sub_aquabot_view_26.createSubscriber(&aquabot_view_B.b_zeroDelimTopic[0],
    qos_profile);
  obj->isSetupComplete = true;
}

void aquabot_view::aquabot_view_SystemCore_setup_m
  (ros_slros2_internal_block_Pub_T *obj)
{
  rmw_qos_durability_policy_t durability;
  rmw_qos_history_policy_t history;
  rmw_qos_profile_t qos_profile;
  rmw_qos_reliability_policy_t reliability;
  char_T b_zeroDelimTopic[24];
  static const char_T tmp[23] = { '/', 's', 'i', 'e', 'r', 'a', '/', 'v', 'i',
    'e', 'w', '/', 'i', 's', '_', 'i', 'n', '_', 's', 'i', 'g', 'h', 't' };

  obj->isInitialized = 1;
  qos_profile = rmw_qos_profile_default;
  history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
  SET_QOS_VALUES(qos_profile, history, (size_t)1.0, durability, reliability);
  for (int32_T i = 0; i < 23; i++) {
    b_zeroDelimTopic[i] = tmp[i];
  }

  b_zeroDelimTopic[23] = '\x00';
  Pub_aquabot_view_38.createPublisher(&b_zeroDelimTopic[0], qos_profile);
  obj->isSetupComplete = true;
}

void aquabot_view::aquabot_vie_SystemCore_setup_my
  (ros_slros2_internal_block_Pub_T *obj)
{
  rmw_qos_durability_policy_t durability;
  rmw_qos_history_policy_t history;
  rmw_qos_profile_t qos_profile;
  rmw_qos_reliability_policy_t reliability;
  char_T b_zeroDelimTopic[20];
  static const char_T tmp[19] = { '/', 's', 'i', 'e', 'r', 'a', '/', 'v', 'i',
    'e', 'w', '/', 'r', 'e', 'l', '_', 'y', 'a', 'w' };

  obj->isInitialized = 1;
  qos_profile = rmw_qos_profile_default;
  history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
  SET_QOS_VALUES(qos_profile, history, (size_t)1.0, durability, reliability);
  for (int32_T i = 0; i < 19; i++) {
    b_zeroDelimTopic[i] = tmp[i];
  }

  b_zeroDelimTopic[19] = '\x00';
  Pub_aquabot_view_45.createPublisher(&b_zeroDelimTopic[0], qos_profile);
  obj->isSetupComplete = true;
}

void aquabot_view::aquabot_vi_SystemCore_setup_myf
  (ros_slros2_internal_block_Pub_T *obj)
{
  rmw_qos_durability_policy_t durability;
  rmw_qos_history_policy_t history;
  rmw_qos_profile_t qos_profile;
  rmw_qos_reliability_policy_t reliability;
  char_T b_zeroDelimTopic[24];
  static const char_T tmp[23] = { '/', 's', 'i', 'e', 'r', 'a', '/', 'v', 'i',
    'e', 'w', '/', 'e', 'n', 'n', 'e', 'm', 'y', '_', 'p', 'o', 's', 'e' };

  obj->isInitialized = 1;
  qos_profile = rmw_qos_profile_default;
  history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
  SET_QOS_VALUES(qos_profile, history, (size_t)1.0, durability, reliability);
  for (int32_T i = 0; i < 23; i++) {
    b_zeroDelimTopic[i] = tmp[i];
  }

  b_zeroDelimTopic[23] = '\x00';
  Pub_aquabot_view_48.createPublisher(&b_zeroDelimTopic[0], qos_profile);
  obj->isSetupComplete = true;
}

void aquabot_view::aquabot_vie_ReadImage_resetImpl
  (ros_slros2_internal_block_R_m_T *obj)
{
  memset(&obj->Image[0], 0, 1244160U * sizeof(uint8_T));
  obj->ImageSize[0] = 576U;
  obj->ImageSize[1] = 720U;
}

void aquabot_view::aquabot_view_SystemCore_step_m(boolean_T *varargout_1,
  int32_T *varargout_2_header_stamp_sec, uint32_T
  *varargout_2_header_stamp_nanose, uint8_T varargout_2_header_frame_id[128],
  uint32_T *varargout_2_header_frame_id_SL_, uint32_T
  *varargout_2_header_frame_id_S_0, uint32_T *varargout_2_height, uint32_T
  *varargout_2_width, SL_Bus_sensor_msgs_PointField varargout_2_fields[16],
  uint32_T *varargout_2_fields_SL_Info_Curr, uint32_T
  *varargout_2_fields_SL_Info_Rece, boolean_T *varargout_2_is_bigendian,
  uint32_T *varargout_2_point_step, uint32_T *varargout_2_row_step, uint8_T
  varargout_2_data[960000], uint32_T *varargout_2_data_SL_Info_Curren, uint32_T *
  varargout_2_data_SL_Info_Receiv, boolean_T *varargout_2_is_dense)
{
  *varargout_1 = Sub_aquabot_view_11.getLatestMessage
    (&aquabot_view_B.b_varargout_2_l);
  *varargout_2_header_stamp_sec =
    aquabot_view_B.b_varargout_2_l.header.stamp.sec;
  *varargout_2_header_stamp_nanose =
    aquabot_view_B.b_varargout_2_l.header.stamp.nanosec;
  memcpy(&varargout_2_header_frame_id[0],
         &aquabot_view_B.b_varargout_2_l.header.frame_id[0], sizeof(uint8_T) <<
         7U);
  *varargout_2_header_frame_id_SL_ =
    aquabot_view_B.b_varargout_2_l.header.frame_id_SL_Info.CurrentLength;
  *varargout_2_header_frame_id_S_0 =
    aquabot_view_B.b_varargout_2_l.header.frame_id_SL_Info.ReceivedLength;
  *varargout_2_height = aquabot_view_B.b_varargout_2_l.height;
  *varargout_2_width = aquabot_view_B.b_varargout_2_l.width;
  memcpy(&varargout_2_fields[0], &aquabot_view_B.b_varargout_2_l.fields[0],
         sizeof(SL_Bus_sensor_msgs_PointField) << 4U);
  *varargout_2_fields_SL_Info_Curr =
    aquabot_view_B.b_varargout_2_l.fields_SL_Info.CurrentLength;
  *varargout_2_fields_SL_Info_Rece =
    aquabot_view_B.b_varargout_2_l.fields_SL_Info.ReceivedLength;
  *varargout_2_is_bigendian = aquabot_view_B.b_varargout_2_l.is_bigendian;
  *varargout_2_point_step = aquabot_view_B.b_varargout_2_l.point_step;
  *varargout_2_row_step = aquabot_view_B.b_varargout_2_l.row_step;
  memcpy(&varargout_2_data[0], &aquabot_view_B.b_varargout_2_l.data[0], 960000U *
         sizeof(uint8_T));
  *varargout_2_data_SL_Info_Curren =
    aquabot_view_B.b_varargout_2_l.data_SL_Info.CurrentLength;
  *varargout_2_data_SL_Info_Receiv =
    aquabot_view_B.b_varargout_2_l.data_SL_Info.ReceivedLength;
  *varargout_2_is_dense = aquabot_view_B.b_varargout_2_l.is_dense;
}

uint32_T aquabot_view::PointCloud2BusWrapper_getFieldI(const
  c_ros_slros2_internal_block_P_T *obj)
{
  int32_T b_i;
  uint32_T fieldIdx;
  char_T a[3];
  char_T b[3];
  boolean_T exitg1;
  fieldIdx = 0U;
  b_i = 0;
  exitg1 = false;
  while ((!exitg1) && (b_i <= static_cast<int32_T>(obj->NumFields) - 1)) {
    int32_T ret;
    b[0] = static_cast<int8_T>(obj->BusStruct.fields[b_i].name[0]);
    a[0] = 'r';
    b[1] = static_cast<int8_T>(obj->BusStruct.fields[b_i].name[1]);
    a[1] = 'g';
    b[2] = static_cast<int8_T>(obj->BusStruct.fields[b_i].name[2]);
    a[2] = 'b';
    ret = std::memcmp(&a[0], &b[0], 3);
    if (ret == 0) {
      fieldIdx = static_cast<uint32_T>(b_i) + 1U;
      exitg1 = true;
    } else {
      b_i++;
    }
  }

  return fieldIdx;
}

uint32_T aquabot_view::PointCloud2BusWrapper_getFiel_m(const
  c_ros_slros2_internal_block_P_T *obj)
{
  int32_T b_i;
  uint32_T fieldIdx;
  char_T a[9];
  char_T b[9];
  static const char_T tmp[9] = { 'i', 'n', 't', 'e', 'n', 's', 'i', 't', 'y' };

  boolean_T exitg1;
  fieldIdx = 0U;
  b_i = 0;
  exitg1 = false;
  while ((!exitg1) && (b_i <= static_cast<int32_T>(obj->NumFields) - 1)) {
    int32_T ret;
    for (ret = 0; ret < 9; ret++) {
      b[ret] = static_cast<int8_T>(obj->BusStruct.fields[b_i].name[ret]);
      a[ret] = tmp[ret];
    }

    ret = std::memcmp(&a[0], &b[0], 9);
    if (ret == 0) {
      fieldIdx = static_cast<uint32_T>(b_i) + 1U;
      exitg1 = true;
    } else {
      b_i++;
    }
  }

  return fieldIdx;
}

void aquabot_view::aquabot_view_bsxfun(const coder::array<real_T, 1U> &a, const
  real_T b_data[], const int32_T b_size[2], coder::array<real_T, 2U> &c)
{
  c.set_size(a.size(0), b_size[1]);
  if (a.size(0) != 0) {
    int32_T acoef;
    int32_T bcoef;
    int32_T d;
    bcoef = (b_size[1] != 1);
    d = b_size[1];
    acoef = (a.size(0) != 1);
    for (int32_T k = 0; k < d; k++) {
      int32_T d_0;
      int32_T ib;
      ib = bcoef * k;
      d_0 = c.size(0);
      for (int32_T k_0 = 0; k_0 < d_0; k_0++) {
        c[k_0 + c.size(0) * k] = a[acoef * k_0] + b_data[ib];
      }
    }
  }
}

void aquabot_view::PointCloud2Reader_getByteIndexF(const
  c_ros_slros2_internal_block_P_T *pc, uint32_T fieldIdx, coder::array<real_T,
  2U> &byteIdx, boolean_T pointIdxIsValid[30000])
{
  int32_T i;
  int32_T loop_ub;
  int32_T loop_ub_0;
  int32_T numBytes;
  uint32_T qY;
  aquabot_view_B.u1 = static_cast<uint64_T>(pc->BusStruct.height) *
    pc->BusStruct.width;
  if (aquabot_view_B.u1 > 4294967295UL) {
    aquabot_view_B.u1 = 4294967295UL;
  }

  if (pc->BusStruct.data_SL_Info.CurrentLength < 1U) {
    i = 0;
  } else {
    i = static_cast<int32_T>(pc->BusStruct.data_SL_Info.CurrentLength);
  }

  aquabot_view_B.u = static_cast<real_T>(i) / static_cast<real_T>
    (pc->BusStruct.point_step);
  if (aquabot_view_B.u < 0.0) {
    aquabot_view_B.u = ceil(aquabot_view_B.u);
  } else {
    aquabot_view_B.u = floor(aquabot_view_B.u);
  }

  if ((!rtIsNaN(aquabot_view_B.u)) && (static_cast<uint32_T>(aquabot_view_B.u1) >
       aquabot_view_B.u)) {
    if (aquabot_view_B.u >= 0.0) {
      aquabot_view_B.numPointsActual = static_cast<uint32_T>(aquabot_view_B.u);
    } else {
      aquabot_view_B.numPointsActual = 0U;
    }
  } else {
    aquabot_view_B.numPointsActual = static_cast<uint32_T>(aquabot_view_B.u1);
  }

  switch (pc->BusStruct.fields[static_cast<int32_T>(fieldIdx) - 1].datatype) {
   case 1U:
    numBytes = 1;
    break;

   case 2U:
    numBytes = 1;
    break;

   case 3U:
    numBytes = 2;
    break;

   case 4U:
    numBytes = 2;
    break;

   case 5U:
    numBytes = 4;
    break;

   case 6U:
    numBytes = 4;
    break;

   case 7U:
    numBytes = 4;
    break;

   case 8U:
    numBytes = 8;
    break;
  }

  byteIdx.set_size(30000, numBytes);
  loop_ub = 30000 * numBytes;
  for (i = 0; i < loop_ub; i++) {
    byteIdx[i] = 0.0;
  }

  for (i = 0; i < 30000; i++) {
    pointIdxIsValid[i] = (static_cast<uint32_T>(i + 1) <=
                          aquabot_view_B.numPointsActual);
  }

  aquabot_view_B.y_size[0] = 1;
  aquabot_view_B.y_size[1] = numBytes;
  loop_ub = numBytes - 1;
  for (i = 0; i <= loop_ub; i++) {
    aquabot_view_B.y_data[i] = static_cast<real_T>(i) + 1.0;
  }

  loop_ub = 0;
  for (i = 0; i < 30000; i++) {
    if (pointIdxIsValid[i]) {
      loop_ub++;
    }
  }

  numBytes = loop_ub;
  loop_ub = 0;
  for (i = 0; i < 30000; i++) {
    if (pointIdxIsValid[i]) {
      aquabot_view_B.tmp_data_b[loop_ub] = static_cast<int16_T>(i);
      loop_ub++;
    }
  }

  aquabot_view_B.pc.set_size(numBytes);
  for (i = 0; i < numBytes; i++) {
    aquabot_view_B.numPointsActual = pc->BusStruct.fields[static_cast<int32_T>
      (fieldIdx) - 1].offset;
    aquabot_view_B.u1 = pc->BusStruct.point_step * static_cast<uint64_T>
      (aquabot_view_B.tmp_data_b[i]);
    if (aquabot_view_B.u1 > 4294967295UL) {
      aquabot_view_B.u1 = 4294967295UL;
    }

    qY = aquabot_view_B.numPointsActual + static_cast<uint32_T>
      (aquabot_view_B.u1);
    if (qY < aquabot_view_B.numPointsActual) {
      qY = MAX_uint32_T;
    }

    aquabot_view_B.pc[i] = qY;
  }

  aquabot_view_bsxfun(aquabot_view_B.pc, aquabot_view_B.y_data,
                      aquabot_view_B.y_size, aquabot_view_B.r3);
  loop_ub = aquabot_view_B.r3.size(1);
  for (i = 0; i < loop_ub; i++) {
    loop_ub_0 = aquabot_view_B.r3.size(0);
    for (numBytes = 0; numBytes < loop_ub_0; numBytes++) {
      byteIdx[aquabot_view_B.tmp_data_b[numBytes] + 30000 * i] =
        aquabot_view_B.r3[aquabot_view_B.r3.size(0) * i + numBytes];
    }
  }
}

void aquabot_view::PointCloud2Reader_readFieldFrom(const coder::array<uint8_T,
  1U> &data, const coder::array<real_T, 2U> &byteIdx, const boolean_T
  pointIdxIsValid[30000], coder::array<real32_T, 1U> &fieldPoints)
{
  coder::array<real32_T, 1U> d;
  int32_T ny;
  int32_T tmp_size_idx_1;
  int32_T trueCount;
  boolean_T exitg1;
  boolean_T y;
  trueCount = 0;
  for (ny = 0; ny < 30000; ny++) {
    if (pointIdxIsValid[ny]) {
      trueCount++;
    }
  }

  tmp_size_idx_1 = trueCount;
  trueCount = 0;
  for (ny = 0; ny < 30000; ny++) {
    if (pointIdxIsValid[ny]) {
      aquabot_view_B.tmp_data_j[trueCount] = static_cast<int16_T>(ny);
      trueCount++;
    }
  }

  aquabot_view_B.b_b.set_size(byteIdx.size(1), tmp_size_idx_1);
  for (ny = 0; ny < tmp_size_idx_1; ny++) {
    int32_T loop_ub;
    loop_ub = byteIdx.size(1);
    for (trueCount = 0; trueCount < loop_ub; trueCount++) {
      aquabot_view_B.b_b[trueCount + aquabot_view_B.b_b.size(0) * ny] = byteIdx
        [30000 * trueCount + aquabot_view_B.tmp_data_j[ny]];
    }
  }

  trueCount = aquabot_view_B.b_b.size(0) * aquabot_view_B.b_b.size(1);
  aquabot_view_B.rawData.set_size(trueCount);
  for (ny = 0; ny < trueCount; ny++) {
    aquabot_view_B.rawData[ny] = data[static_cast<int32_T>(aquabot_view_B.b_b[ny])
      - 1];
  }

  if (trueCount == 0) {
    ny = 0;
  } else {
    ny = trueCount >> 2;
  }

  d.set_size(ny);
  memcpy((void *)&(d.data())[0], (void *)&(aquabot_view_B.rawData.data())[0],
         (uint32_T)((size_t)ny * sizeof(real32_T)));
  for (ny = 0; ny < 30000; ny++) {
    aquabot_view_B.x_j[ny] = !pointIdxIsValid[ny];
  }

  y = false;
  ny = 0;
  exitg1 = false;
  while ((!exitg1) && (ny < 30000)) {
    if (aquabot_view_B.x_j[ny]) {
      y = true;
      exitg1 = true;
    } else {
      ny++;
    }
  }

  if (y) {
    fieldPoints.set_size(30000);
    for (ny = 0; ny < 30000; ny++) {
      fieldPoints[ny] = (rtNaNF);
    }

    trueCount = 0;
    for (ny = 0; ny < 30000; ny++) {
      if (pointIdxIsValid[ny]) {
        aquabot_view_B.tmp_data_f[trueCount] = static_cast<int16_T>(ny);
        trueCount++;
      }
    }

    trueCount = d.size(0);
    for (ny = 0; ny < trueCount; ny++) {
      fieldPoints[static_cast<int32_T>(aquabot_view_B.tmp_data_f[ny])] = d[ny];
    }
  } else {
    trueCount = d.size(0);
    fieldPoints.set_size(d.size(0));
    for (ny = 0; ny < trueCount; ny++) {
      fieldPoints[ny] = d[ny];
    }
  }
}

void aquabot_view::aquab_PointCloud2Reader_readXYZ(const
  c_ros_slros2_internal_block_P_T *pc, uint32_T xIdx, uint32_T yIdx, uint32_T
  zIdx, coder::array<real32_T, 2U> &xyz)
{
  if (pc->BusStruct.data_SL_Info.CurrentLength < 1U) {
    aquabot_view_B.c_o4 = 0;
  } else {
    aquabot_view_B.c_o4 = static_cast<int32_T>
      (pc->BusStruct.data_SL_Info.CurrentLength);
  }

  aquabot_view_B.data_g.set_size(aquabot_view_B.c_o4);
  for (aquabot_view_B.i4 = 0; aquabot_view_B.i4 < aquabot_view_B.c_o4;
       aquabot_view_B.i4++) {
    aquabot_view_B.data_g[aquabot_view_B.i4] = pc->
      BusStruct.data[aquabot_view_B.i4];
  }

  PointCloud2Reader_getByteIndexF(pc, xIdx, aquabot_view_B.byteIdx,
    aquabot_view_B.pointIdxIsValid);
  aquabot_view_B.zOff = pc->BusStruct.fields[static_cast<int32_T>(xIdx) - 1].
    offset;
  aquabot_view_B.yOff = static_cast<real_T>(pc->BusStruct.fields
    [static_cast<int32_T>(yIdx) - 1].offset) - aquabot_view_B.zOff;
  aquabot_view_B.zOff = static_cast<real_T>(pc->BusStruct.fields
    [static_cast<int32_T>(zIdx) - 1].offset) - aquabot_view_B.zOff;
  PointCloud2Reader_readFieldFrom(aquabot_view_B.data_g, aquabot_view_B.byteIdx,
    aquabot_view_B.pointIdxIsValid, aquabot_view_B.varargin_1);
  aquabot_view_B.byteIdx_g.set_size(30000, aquabot_view_B.byteIdx.size(1));
  aquabot_view_B.c_o4 = 30000 * aquabot_view_B.byteIdx.size(1);
  for (aquabot_view_B.i4 = 0; aquabot_view_B.i4 < aquabot_view_B.c_o4;
       aquabot_view_B.i4++) {
    aquabot_view_B.byteIdx_g[aquabot_view_B.i4] =
      aquabot_view_B.byteIdx[aquabot_view_B.i4] + aquabot_view_B.yOff;
  }

  PointCloud2Reader_readFieldFrom(aquabot_view_B.data_g,
    aquabot_view_B.byteIdx_g, aquabot_view_B.pointIdxIsValid,
    aquabot_view_B.varargin_2);
  aquabot_view_B.byteIdx_g.set_size(30000, aquabot_view_B.byteIdx.size(1));
  for (aquabot_view_B.i4 = 0; aquabot_view_B.i4 < aquabot_view_B.c_o4;
       aquabot_view_B.i4++) {
    aquabot_view_B.byteIdx_g[aquabot_view_B.i4] =
      aquabot_view_B.byteIdx[aquabot_view_B.i4] + aquabot_view_B.zOff;
  }

  PointCloud2Reader_readFieldFrom(aquabot_view_B.data_g,
    aquabot_view_B.byteIdx_g, aquabot_view_B.pointIdxIsValid,
    aquabot_view_B.varargin_3);
  xyz.set_size(aquabot_view_B.varargin_1.size(0), 3);
  aquabot_view_B.c_o4 = aquabot_view_B.varargin_1.size(0);
  for (aquabot_view_B.i4 = 0; aquabot_view_B.i4 < aquabot_view_B.c_o4;
       aquabot_view_B.i4++) {
    xyz[aquabot_view_B.i4] = aquabot_view_B.varargin_1[aquabot_view_B.i4];
  }

  aquabot_view_B.c_o4 = aquabot_view_B.varargin_2.size(0);
  for (aquabot_view_B.i4 = 0; aquabot_view_B.i4 < aquabot_view_B.c_o4;
       aquabot_view_B.i4++) {
    xyz[aquabot_view_B.i4 + aquabot_view_B.varargin_1.size(0)] =
      aquabot_view_B.varargin_2[aquabot_view_B.i4];
  }

  aquabot_view_B.c_o4 = aquabot_view_B.varargin_3.size(0);
  for (aquabot_view_B.i4 = 0; aquabot_view_B.i4 < aquabot_view_B.c_o4;
       aquabot_view_B.i4++) {
    xyz[(aquabot_view_B.i4 + aquabot_view_B.varargin_1.size(0)) +
      aquabot_view_B.varargin_2.size(0)] =
      aquabot_view_B.varargin_3[aquabot_view_B.i4];
  }
}

void aquabot_view::aquabot_ReadPointCloud_stepImpl(const
  SL_Bus_sensor_msgs_PointCloud2 *busstruct, real32_T varargout_1[90000])
{
  boolean_T exitg1;
  boolean_T out;
  aquabot_view_B.pc_b.BusStruct = *busstruct;
  aquabot_view_B.pc_b.NumFields = busstruct->fields_SL_Info.CurrentLength;
  aquabot_view_B.xIdx = 0U;
  aquabot_view_B.b_i = 0;
  exitg1 = false;
  while ((!exitg1) && (aquabot_view_B.b_i <= static_cast<int32_T>
                       (aquabot_view_B.pc_b.NumFields) - 1)) {
    out = false;
    if (static_cast<int8_T>(busstruct->fields[aquabot_view_B.b_i].name[0]) ==
        'x') {
      out = true;
    }

    if (out) {
      aquabot_view_B.xIdx = static_cast<uint32_T>(aquabot_view_B.b_i) + 1U;
      exitg1 = true;
    } else {
      aquabot_view_B.b_i++;
    }
  }

  aquabot_view_B.yIdx = 0U;
  aquabot_view_B.b_i = 0;
  exitg1 = false;
  while ((!exitg1) && (aquabot_view_B.b_i <= static_cast<int32_T>
                       (aquabot_view_B.pc_b.NumFields) - 1)) {
    out = false;
    if (static_cast<int8_T>(busstruct->fields[aquabot_view_B.b_i].name[0]) ==
        'y') {
      out = true;
    }

    if (out) {
      aquabot_view_B.yIdx = static_cast<uint32_T>(aquabot_view_B.b_i) + 1U;
      exitg1 = true;
    } else {
      aquabot_view_B.b_i++;
    }
  }

  aquabot_view_B.zIdx = 0U;
  aquabot_view_B.b_i = 0;
  exitg1 = false;
  while ((!exitg1) && (aquabot_view_B.b_i <= static_cast<int32_T>
                       (aquabot_view_B.pc_b.NumFields) - 1)) {
    out = false;
    if (static_cast<int8_T>(busstruct->fields[aquabot_view_B.b_i].name[0]) ==
        'z') {
      out = true;
    }

    if (out) {
      aquabot_view_B.zIdx = static_cast<uint32_T>(aquabot_view_B.b_i) + 1U;
      exitg1 = true;
    } else {
      aquabot_view_B.b_i++;
    }
  }

  PointCloud2BusWrapper_getFieldI(&aquabot_view_B.pc_b);
  PointCloud2BusWrapper_getFiel_m(&aquabot_view_B.pc_b);
  if ((aquabot_view_B.xIdx != 0U) && (aquabot_view_B.yIdx != 0U) &&
      (aquabot_view_B.zIdx != 0U)) {
    aquab_PointCloud2Reader_readXYZ(&aquabot_view_B.pc_b, aquabot_view_B.xIdx,
      aquabot_view_B.yIdx, aquabot_view_B.zIdx, aquabot_view_B.b_xyzTmp);
    aquabot_view_B.xyzTmp.set_size(aquabot_view_B.b_xyzTmp.size(0), 3);
    aquabot_view_B.loop_ub_e = aquabot_view_B.b_xyzTmp.size(0) * 3;
    for (aquabot_view_B.b_i = 0; aquabot_view_B.b_i < aquabot_view_B.loop_ub_e;
         aquabot_view_B.b_i++) {
      aquabot_view_B.xyzTmp[aquabot_view_B.b_i] =
        aquabot_view_B.b_xyzTmp[aquabot_view_B.b_i];
    }
  } else {
    aquabot_view_B.xyzTmp.set_size(30000, 3);
    for (aquabot_view_B.b_i = 0; aquabot_view_B.b_i < 90000; aquabot_view_B.b_i
         ++) {
      aquabot_view_B.xyzTmp[aquabot_view_B.b_i] = (rtNaNF);
    }
  }

  for (aquabot_view_B.b_i = 0; aquabot_view_B.b_i < 90000; aquabot_view_B.b_i++)
  {
    varargout_1[aquabot_view_B.b_i] = aquabot_view_B.xyzTmp[aquabot_view_B.b_i];
  }
}

void aquabot_view::aquabot_view_SystemCore_step_my(boolean_T *varargout_1,
  int32_T *varargout_2_header_stamp_sec, uint32_T
  *varargout_2_header_stamp_nanose, uint8_T varargout_2_header_frame_id[128],
  uint32_T *varargout_2_header_frame_id_SL_, uint32_T
  *varargout_2_header_frame_id_S_0, uint32_T *varargout_2_height, uint32_T
  *varargout_2_width, uint8_T varargout_2_encoding[128], uint32_T
  *varargout_2_encoding_SL_Info_Cu, uint32_T *varargout_2_encoding_SL_Info_Re,
  uint8_T *varargout_2_is_bigendian, uint32_T *varargout_2_step, uint8_T
  varargout_2_data[1244160], uint32_T *varargout_2_data_SL_Info_Curren, uint32_T
  *varargout_2_data_SL_Info_Receiv)
{
  *varargout_1 = Sub_aquabot_view_12.getLatestMessage
    (&aquabot_view_B.b_varargout_2);
  *varargout_2_header_stamp_sec = aquabot_view_B.b_varargout_2.header.stamp.sec;
  *varargout_2_header_stamp_nanose =
    aquabot_view_B.b_varargout_2.header.stamp.nanosec;
  *varargout_2_header_frame_id_SL_ =
    aquabot_view_B.b_varargout_2.header.frame_id_SL_Info.CurrentLength;
  *varargout_2_header_frame_id_S_0 =
    aquabot_view_B.b_varargout_2.header.frame_id_SL_Info.ReceivedLength;
  *varargout_2_height = aquabot_view_B.b_varargout_2.height;
  *varargout_2_width = aquabot_view_B.b_varargout_2.width;
  memcpy(&varargout_2_header_frame_id[0],
         &aquabot_view_B.b_varargout_2.header.frame_id[0], sizeof(uint8_T) << 7U);
  memcpy(&varargout_2_encoding[0], &aquabot_view_B.b_varargout_2.encoding[0],
         sizeof(uint8_T) << 7U);
  *varargout_2_encoding_SL_Info_Cu =
    aquabot_view_B.b_varargout_2.encoding_SL_Info.CurrentLength;
  *varargout_2_encoding_SL_Info_Re =
    aquabot_view_B.b_varargout_2.encoding_SL_Info.ReceivedLength;
  *varargout_2_is_bigendian = aquabot_view_B.b_varargout_2.is_bigendian;
  *varargout_2_step = aquabot_view_B.b_varargout_2.step;
  memcpy(&varargout_2_data[0], &aquabot_view_B.b_varargout_2.data[0], 1244160U *
         sizeof(uint8_T));
  *varargout_2_data_SL_Info_Curren =
    aquabot_view_B.b_varargout_2.data_SL_Info.CurrentLength;
  *varargout_2_data_SL_Info_Receiv =
    aquabot_view_B.b_varargout_2.data_SL_Info.ReceivedLength;
}

void aquabot_view::aquabot_view_char(const uint8_T varargin_1_data[], const
  int32_T varargin_1_size[2], char_T y_data[], int32_T y_size[2])
{
  int32_T loop_ub;
  y_size[0] = 1;
  y_size[1] = varargin_1_size[1];
  loop_ub = varargin_1_size[1];
  for (int32_T y_data_tmp = 0; y_data_tmp < loop_ub; y_data_tmp++) {
    y_data[y_data_tmp] = static_cast<int8_T>(varargin_1_data[y_data_tmp]);
  }
}

boolean_T aquabot_view::aquabot_view_strcmp(const char_T a_data[], const int32_T
  a_size[2])
{
  char_T b[4];
  boolean_T b_bool;
  static const char_T tmp[128] = { '\x00', '\x01', '\x02', '\x03', '\x04',
    '\x05', '\x06', '\a', '\b', '\t', '\n', '\v', '\f', '\r', '\x0e', '\x0f',
    '\x10', '\x11', '\x12', '\x13', '\x14', '\x15', '\x16', '\x17', '\x18',
    '\x19', '\x1a', '\x1b', '\x1c', '\x1d', '\x1e', '\x1f', ' ', '!', '\"', '#',
    '$', '%', '&', '\'', '(', ')', '*', '+', ',', '-', '.', '/', '0', '1', '2',
    '3', '4', '5', '6', '7', '8', '9', ':', ';', '<', '=', '>', '?', '@', 'a',
    'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n', 'o', 'p',
    'q', 'r', 's', 't', 'u', 'v', 'w', 'x', 'y', 'z', '[', '\\', ']', '^', '_',
    '`', 'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n',
    'o', 'p', 'q', 'r', 's', 't', 'u', 'v', 'w', 'x', 'y', 'z', '{', '|', '}',
    '~', '\x7f' };

  b[0] = 'r';
  b[1] = 'g';
  b[2] = 'b';
  b[3] = '8';
  b_bool = false;
  if (a_size[1] != 4) {
  } else {
    int32_T b_kstr;
    b_kstr = 1;
    int32_T exitg1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 4) {
        if (tmp[static_cast<uint8_T>(a_data[b_kstr - 1]) & 127] != tmp[
            static_cast<int32_T>(b[b_kstr - 1])]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  return b_bool;
}

void aquabot_view::aquabot_view_permute(const coder::array<uint8_T, 3U> &a,
  coder::array<uint8_T, 3U> &b)
{
  int32_T b_k_0;
  int32_T d;
  int32_T plast;
  int8_T perm[3];
  boolean_T b_0;
  perm[0] = 2;
  perm[1] = 1;
  perm[2] = 3;
  b_0 = true;
  if ((a.size(0) != 0) && (a.size(1) != 0)) {
    boolean_T exitg1;
    plast = 0;
    b_k_0 = 1;
    exitg1 = false;
    while ((!exitg1) && (b_k_0 - 1 < 3)) {
      d = perm[b_k_0 - 1];
      if (a.size(d - 1) != 1) {
        if (plast > d) {
          b_0 = false;
          exitg1 = true;
        } else {
          plast = d;
          b_k_0++;
        }
      } else {
        b_k_0++;
      }
    }
  }

  if (b_0) {
    int32_T d_0;
    b.set_size(a.size(1), a.size(0), 3);
    d_0 = a.size(0) * a.size(1) * 3;
    for (d = 0; d < d_0; d++) {
      b[d] = a[d];
    }
  } else {
    b.set_size(a.size(1), a.size(0), 3);
    d = a.size(1) - 1;
    for (b_k_0 = 0; b_k_0 < 3; b_k_0++) {
      for (plast = 0; plast <= d; plast++) {
        int32_T d_0;
        d_0 = a.size(0) - 1;
        for (int32_T b_k = 0; b_k <= d_0; b_k++) {
          b[(plast + b.size(0) * b_k) + b.size(0) * b.size(1) * b_k_0] = a
            [(a.size(0) * plast + b_k) + a.size(0) * a.size(1) * b_k_0];
        }
      }
    }
  }
}

void aquabot_view::aquabot_v_ImageReader_readImage(const coder::array<uint8_T,
  1U> &data, uint32_T width, uint32_T height, coder::array<uint8_T, 3U> &img)
{
  int32_T b_i;
  int32_T b_k;
  int32_T loop_ub_tmp;
  int32_T n;
  uint32_T q0;
  if (data.size(0) == 0) {
    aquabot_view_B.data_c.set_size(0);
    memcpy((void *)&(aquabot_view_B.data_c.data())[0], (void *)&(((coder::array<
              uint8_T, 1U> *)&data)->data())[0], (uint32_T)((size_t)0 * sizeof
            (uint8_T)));
    img.set_size(0, 1, 1);
  } else {
    aquabot_view_B.data_c.set_size(data.size(0));
    memcpy((void *)&(aquabot_view_B.data_c.data())[0], (void *)&(((coder::array<
              uint8_T, 1U> *)&data)->data())[0], (uint32_T)((size_t)data.size(0)
            * sizeof(uint8_T)));
    aquabot_view_B.u_a = static_cast<uint64_T>(width) * height;
    if (aquabot_view_B.u_a > 4294967295UL) {
      aquabot_view_B.u_a = 4294967295UL;
    }

    aquabot_view_B.u_a = static_cast<uint32_T>(aquabot_view_B.u_a) * 3UL;
    if (aquabot_view_B.u_a > 4294967295UL) {
      aquabot_view_B.u_a = 4294967295UL;
    }

    if (static_cast<uint32_T>(aquabot_view_B.u_a) < 1U) {
      n = 0;
    } else {
      n = static_cast<int32_T>((static_cast<uint32_T>(aquabot_view_B.u_a) - 1U) /
        3U) + 1;
    }

    aquabot_view_B.y.set_size(1, n);
    for (b_k = 0; b_k < n; b_k++) {
      aquabot_view_B.y[b_k] = static_cast<uint32_T>(b_k) * 3U + 1U;
    }

    aquabot_view_B.indexBase.set_size(aquabot_view_B.y.size(1));
    b_k = aquabot_view_B.y.size(1);
    for (n = 0; n < b_k; n++) {
      aquabot_view_B.indexBase[n] = aquabot_view_B.y[n];
    }

    aquabot_view_B.b_index.set_size(aquabot_view_B.indexBase.size(0), 3);
    loop_ub_tmp = aquabot_view_B.indexBase.size(0) * 3;
    for (n = 0; n < loop_ub_tmp; n++) {
      aquabot_view_B.b_index[n] = 0U;
    }

    b_k = aquabot_view_B.indexBase.size(0);
    for (b_i = 0; b_i < 3; b_i++) {
      for (n = 0; n < b_k; n++) {
        aquabot_view_B.d = (static_cast<real_T>(b_i) + 1.0) + static_cast<real_T>
          (aquabot_view_B.indexBase[n]);
        if (aquabot_view_B.d < 4.294967296E+9) {
          q0 = static_cast<uint32_T>(aquabot_view_B.d);
        } else {
          q0 = MAX_uint32_T;
        }

        aquabot_view_B.b_index[n + aquabot_view_B.b_index.size(0) * b_i] = q0 -
          1U;
        if (q0 - 1U > q0) {
          aquabot_view_B.b_index[n + aquabot_view_B.b_index.size(0) * b_i] = 0U;
        }
      }
    }

    aquabot_view_B.data_f.set_size(aquabot_view_B.b_index.size(0), 3);
    for (n = 0; n < loop_ub_tmp; n++) {
      aquabot_view_B.data_f[n] = aquabot_view_B.data_c[static_cast<int32_T>
        (aquabot_view_B.b_index[n]) - 1];
    }

    n = static_cast<int32_T>(width);
    b_k = static_cast<int32_T>(height);
    aquabot_view_B.data = aquabot_view_B.data_f.reshape(n, b_k, 3);
    aquabot_view_permute(aquabot_view_B.data, aquabot_view_B.r);
    img.set_size(aquabot_view_B.r.size(0), aquabot_view_B.r.size(1), 3);
    b_k = aquabot_view_B.r.size(0) * aquabot_view_B.r.size(1) * 3;
    for (n = 0; n < b_k; n++) {
      img[n] = aquabot_view_B.r[n];
    }
  }
}

uint8_T aquabot_view::aqu_ReadImage_updateBusAndState
  (ros_slros2_internal_block_R_m_T *obj, uint32_T busstruct_height, uint32_T
   busstruct_width, const uint8_T busstruct_encoding[128], uint32_T
   busstruct_encoding_SL_Info_Curr, const uint8_T busstruct_data[1244160],
   uint32_T busstruct_data_SL_Info_CurrentL, uint32_T
   busstruct_data_SL_Info_Received)
{
  int32_T loop_ub;
  uint8_T errorCode;
  if (busstruct_data_SL_Info_CurrentL < busstruct_data_SL_Info_Received) {
    errorCode = 3U;
  } else {
    if (busstruct_encoding_SL_Info_Curr < 1U) {
      aquabot_view_B.c_o = 0;
    } else {
      aquabot_view_B.c_o = static_cast<int32_T>(busstruct_encoding_SL_Info_Curr);
    }

    aquabot_view_B.busstruct_encoding_size[0] = 1;
    aquabot_view_B.busstruct_encoding_size[1] = aquabot_view_B.c_o;
    if (aquabot_view_B.c_o - 1 >= 0) {
      memcpy(&aquabot_view_B.busstruct_encoding_data[0], &busstruct_encoding[0],
             static_cast<uint32_T>(aquabot_view_B.c_o) * sizeof(uint8_T));
    }

    aquabot_view_char(aquabot_view_B.busstruct_encoding_data,
                      aquabot_view_B.busstruct_encoding_size,
                      aquabot_view_B.tmp_data_i, aquabot_view_B.tmp_size);
    if (!aquabot_view_strcmp(aquabot_view_B.tmp_data_i, aquabot_view_B.tmp_size))
    {
      errorCode = 1U;
    } else {
      if (busstruct_data_SL_Info_CurrentL < 1U) {
        aquabot_view_B.c_o = 0;
      } else {
        aquabot_view_B.c_o = static_cast<int32_T>
          (busstruct_data_SL_Info_CurrentL);
      }

      if ((busstruct_height > 576U) || (busstruct_width > 720U)) {
        errorCode = 2U;
        aquabot_view_B.rawImage.set_size(576, 720, 3);
        for (aquabot_view_B.i2 = 0; aquabot_view_B.i2 < 1244160;
             aquabot_view_B.i2++) {
          aquabot_view_B.rawImage[aquabot_view_B.i2] = obj->
            Image[aquabot_view_B.i2];
        }
      } else {
        aquabot_view_B.busstruct_data.set_size(aquabot_view_B.c_o);
        for (aquabot_view_B.i2 = 0; aquabot_view_B.i2 < aquabot_view_B.c_o;
             aquabot_view_B.i2++) {
          aquabot_view_B.busstruct_data[aquabot_view_B.i2] =
            busstruct_data[aquabot_view_B.i2];
        }

        aquabot_v_ImageReader_readImage(aquabot_view_B.busstruct_data,
          busstruct_width, busstruct_height, aquabot_view_B.rawImage);
        errorCode = 0U;
      }

      if (errorCode != 0) {
      } else {
        memset(&obj->Image[0], 0, 1244160U * sizeof(uint8_T));
        aquabot_view_B.loop_ub = aquabot_view_B.rawImage.size(2);
        for (aquabot_view_B.i2 = 0; aquabot_view_B.i2 < aquabot_view_B.loop_ub;
             aquabot_view_B.i2++) {
          aquabot_view_B.loop_ub_l = aquabot_view_B.rawImage.size(1);
          for (aquabot_view_B.c_o = 0; aquabot_view_B.c_o <
               aquabot_view_B.loop_ub_l; aquabot_view_B.c_o++) {
            loop_ub = aquabot_view_B.rawImage.size(0);
            for (aquabot_view_B.i3 = 0; aquabot_view_B.i3 < loop_ub;
                 aquabot_view_B.i3++) {
              obj->Image[(aquabot_view_B.i3 + 576 * aquabot_view_B.c_o) + 414720
                * aquabot_view_B.i2] = aquabot_view_B.rawImage
                [(aquabot_view_B.rawImage.size(0) * aquabot_view_B.c_o +
                  aquabot_view_B.i3) + aquabot_view_B.rawImage.size(0) *
                aquabot_view_B.rawImage.size(1) * aquabot_view_B.i2];
            }
          }
        }

        obj->ImageSize[0] = static_cast<uint32_T>(aquabot_view_B.rawImage.size(0));
        obj->ImageSize[1] = static_cast<uint32_T>(aquabot_view_B.rawImage.size(1));
        errorCode = 0U;
      }
    }
  }

  return errorCode;
}

void aquabot_view::aquabot_view_ReadImage_stepImpl
  (ros_slros2_internal_block_R_m_T *obj, uint32_T busstruct_height, uint32_T
   busstruct_width, const uint8_T busstruct_encoding[128], uint32_T
   busstruct_encoding_SL_Info_Curr, const uint8_T busstruct_data[1244160],
   uint32_T busstruct_data_SL_Info_CurrentL, uint32_T
   busstruct_data_SL_Info_Received, uint8_T varargout_1[1244160], uint8_T
   *varargout_2)
{
  *varargout_2 = aqu_ReadImage_updateBusAndState(obj, busstruct_height,
    busstruct_width, busstruct_encoding, busstruct_encoding_SL_Info_Curr,
    busstruct_data, busstruct_data_SL_Info_CurrentL,
    busstruct_data_SL_Info_Received);
  for (aquabot_view_B.i_c = 0; aquabot_view_B.i_c < 720; aquabot_view_B.i_c++) {
    for (aquabot_view_B.i1 = 0; aquabot_view_B.i1 < 3; aquabot_view_B.i1++) {
      memcpy(&varargout_1[aquabot_view_B.i_c * 576 + aquabot_view_B.i1 * 414720],
             &obj->Image[aquabot_view_B.i_c * 576 + aquabot_view_B.i1 * 414720],
             576U * sizeof(uint8_T));
    }
  }
}

void aquabot_view::aquabot_view_SystemCore_step(ros_slros2_internal_block_R_m_T *
  obj, uint32_T varargin_1_height, uint32_T varargin_1_width, const uint8_T
  varargin_1_encoding[128], uint32_T varargin_1_encoding_SL_Info_Cur, const
  uint8_T varargin_1_data[1244160], uint32_T varargin_1_data_SL_Info_Current,
  uint32_T varargin_1_data_SL_Info_Receive, uint8_T varargout_1[1244160],
  uint8_T *varargout_2)
{
  aquabot_view_ReadImage_stepImpl(obj, varargin_1_height, varargin_1_width,
    varargin_1_encoding, varargin_1_encoding_SL_Info_Cur, varargin_1_data,
    varargin_1_data_SL_Info_Current, varargin_1_data_SL_Info_Receive,
    varargout_1, varargout_2);
}

/* Function for MATLAB Function: '<S4>/MATLAB Function' */
void aquabot_view::aquabot_view_bwconncomp(const boolean_T varargin_1[414720],
  real_T *CC_Connectivity, real_T CC_ImageSize[2], real_T *CC_NumObjects, coder::
  array<real_T, 1U> &CC_RegionIndices, coder::array<int32_T, 1U>
  &CC_RegionLengths, coder::array<cell_wrap_6_aquabot_view_T, 2U>
  &CC_PixelIdxList)
{
  int32_T currentColumn;
  int32_T numRuns;
  int32_T row;
  int32_T runCounter;
  CC_ImageSize[0] = 576.0;
  CC_ImageSize[1] = 720.0;
  numRuns = 0;
  for (runCounter = 0; runCounter < 720; runCounter++) {
    if (varargin_1[576 * runCounter]) {
      numRuns++;
    }

    for (row = 0; row < 575; row++) {
      currentColumn = 576 * runCounter + row;
      if (varargin_1[currentColumn + 1] && (!varargin_1[currentColumn])) {
        numRuns++;
      }
    }
  }

  if (numRuns == 0) {
    *CC_NumObjects = 0.0;
    aquabot_view_B.b.set_size(0);
    CC_RegionLengths.set_size(1);
    CC_RegionLengths[0] = 0;
    aquabot_view_B.c.set_size(1, 0);
  } else {
    int32_T firstRunOnPreviousColumn;
    int32_T firstRunOnThisColumn;
    int32_T lastRunOnPreviousColumn;
    uint32_T numComponents;
    aquabot_view_B.startRow.set_size(numRuns);
    aquabot_view_B.endRow.set_size(numRuns);
    aquabot_view_B.startCol.set_size(numRuns);
    runCounter = 0;
    for (currentColumn = 0; currentColumn < 720; currentColumn++) {
      row = 1;
      while (row <= 576) {
        while ((row <= 576) && (!varargin_1[(576 * currentColumn + row) - 1])) {
          row++;
        }

        if ((row <= 576) && varargin_1[(576 * currentColumn + row) - 1]) {
          aquabot_view_B.startCol[runCounter] = static_cast<int16_T>
            (currentColumn + 1);
          aquabot_view_B.startRow[runCounter] = static_cast<int16_T>(row);
          while ((row <= 576) && varargin_1[(576 * currentColumn + row) - 1]) {
            row++;
          }

          aquabot_view_B.endRow[runCounter] = static_cast<int16_T>(row - 1);
          runCounter++;
        }
      }
    }

    CC_RegionLengths.set_size(numRuns);
    for (currentColumn = 0; currentColumn < numRuns; currentColumn++) {
      CC_RegionLengths[currentColumn] = 0;
    }

    runCounter = 0;
    currentColumn = 1;
    row = 1;
    firstRunOnPreviousColumn = -1;
    lastRunOnPreviousColumn = -1;
    firstRunOnThisColumn = 0;
    while (runCounter + 1 <= numRuns) {
      if (currentColumn + 1 == aquabot_view_B.startCol[runCounter]) {
        firstRunOnPreviousColumn = firstRunOnThisColumn + 1;
        firstRunOnThisColumn = runCounter;
        lastRunOnPreviousColumn = runCounter;
        currentColumn = aquabot_view_B.startCol[runCounter];
      } else if (aquabot_view_B.startCol[runCounter] > currentColumn + 1) {
        firstRunOnPreviousColumn = -1;
        lastRunOnPreviousColumn = -1;
        firstRunOnThisColumn = runCounter;
        currentColumn = aquabot_view_B.startCol[runCounter];
      }

      if (firstRunOnPreviousColumn >= 0) {
        for (int32_T p = firstRunOnPreviousColumn - 1; p <
             lastRunOnPreviousColumn; p++) {
          if ((aquabot_view_B.endRow[runCounter] >= aquabot_view_B.startRow[p] -
               1) && (aquabot_view_B.startRow[runCounter] <=
                      aquabot_view_B.endRow[p] + 1)) {
            if (CC_RegionLengths[runCounter] == 0) {
              CC_RegionLengths[runCounter] = CC_RegionLengths[p];
              row++;
            } else if (CC_RegionLengths[runCounter] != CC_RegionLengths[p]) {
              int32_T root_k;
              int32_T root_p;
              for (root_k = runCounter; root_k + 1 != CC_RegionLengths[root_k];
                   root_k = CC_RegionLengths[root_k] - 1) {
                CC_RegionLengths[root_k] =
                  CC_RegionLengths[CC_RegionLengths[root_k] - 1];
              }

              for (root_p = p; root_p + 1 != CC_RegionLengths[root_p]; root_p =
                   CC_RegionLengths[root_p] - 1) {
                CC_RegionLengths[root_p] =
                  CC_RegionLengths[CC_RegionLengths[root_p] - 1];
              }

              if (root_k + 1 != root_p + 1) {
                if (root_p + 1 < root_k + 1) {
                  CC_RegionLengths[root_k] = root_p + 1;
                  CC_RegionLengths[runCounter] = root_p + 1;
                } else {
                  CC_RegionLengths[root_p] = root_k + 1;
                  CC_RegionLengths[p] = root_k + 1;
                }
              }
            }
          }
        }
      }

      if (CC_RegionLengths[runCounter] == 0) {
        CC_RegionLengths[runCounter] = row;
        row++;
      }

      runCounter++;
    }

    aquabot_view_B.labelsRenumbered.set_size(CC_RegionLengths.size(0));
    numComponents = 0U;
    for (runCounter = 0; runCounter < numRuns; runCounter++) {
      currentColumn = CC_RegionLengths[runCounter];
      if (runCounter + 1 == currentColumn) {
        numComponents++;
        aquabot_view_B.labelsRenumbered[runCounter] = static_cast<int32_T>
          (numComponents);
      }

      aquabot_view_B.labelsRenumbered[runCounter] =
        aquabot_view_B.labelsRenumbered[currentColumn - 1];
    }

    CC_RegionLengths.set_size(static_cast<int32_T>(numComponents));
    runCounter = static_cast<int32_T>(numComponents);
    for (currentColumn = 0; currentColumn < runCounter; currentColumn++) {
      CC_RegionLengths[currentColumn] = 0;
    }

    for (currentColumn = 0; currentColumn < numRuns; currentColumn++) {
      row = aquabot_view_B.labelsRenumbered[currentColumn];
      CC_RegionLengths[row - 1] = ((CC_RegionLengths[row - 1] +
        aquabot_view_B.endRow[currentColumn]) -
        aquabot_view_B.startRow[currentColumn]) + 1;
    }

    *CC_NumObjects = numComponents;
    if (CC_RegionLengths.size(0) == 0) {
      aquabot_view_B.y_a = 0.0;
    } else {
      if (CC_RegionLengths.size(0) <= 1024) {
        firstRunOnPreviousColumn = CC_RegionLengths.size(0);
        row = 0;
        currentColumn = 1;
      } else {
        firstRunOnPreviousColumn = 1024;
        currentColumn = static_cast<int32_T>(static_cast<uint32_T>
          (CC_RegionLengths.size(0)) >> 10);
        row = CC_RegionLengths.size(0) - (currentColumn << 10);
        if (row > 0) {
          currentColumn++;
        } else {
          row = 1024;
        }
      }

      aquabot_view_B.y_a = CC_RegionLengths[0];
      for (lastRunOnPreviousColumn = 2; lastRunOnPreviousColumn <=
           firstRunOnPreviousColumn; lastRunOnPreviousColumn++) {
        aquabot_view_B.y_a += static_cast<real_T>
          (CC_RegionLengths[lastRunOnPreviousColumn - 1]);
      }

      for (firstRunOnPreviousColumn = 2; firstRunOnPreviousColumn <=
           currentColumn; firstRunOnPreviousColumn++) {
        lastRunOnPreviousColumn = (firstRunOnPreviousColumn - 1) << 10;
        aquabot_view_B.bsum_e = CC_RegionLengths[lastRunOnPreviousColumn];
        if (firstRunOnPreviousColumn == currentColumn) {
          firstRunOnThisColumn = row;
        } else {
          firstRunOnThisColumn = 1024;
        }

        for (int32_T p = 2; p <= firstRunOnThisColumn; p++) {
          aquabot_view_B.bsum_e += static_cast<real_T>(CC_RegionLengths
            [(lastRunOnPreviousColumn + p) - 1]);
        }

        aquabot_view_B.y_a += aquabot_view_B.bsum_e;
      }
    }

    aquabot_view_B.b.set_size(static_cast<int32_T>(aquabot_view_B.y_a));
    if (CC_RegionLengths.size(0) != 1) {
      aquabot_view_B.x_n.set_size(CC_RegionLengths.size(0));
      row = CC_RegionLengths.size(0);
      for (currentColumn = 0; currentColumn < row; currentColumn++) {
        aquabot_view_B.x_n[currentColumn] = CC_RegionLengths[currentColumn];
      }

      if ((CC_RegionLengths.size(0) != 0) && (CC_RegionLengths.size(0) != 1)) {
        currentColumn = CC_RegionLengths.size(0);
        for (row = 0; row <= currentColumn - 2; row++) {
          aquabot_view_B.x_n[row + 1] = aquabot_view_B.x_n[row + 1] +
            aquabot_view_B.x_n[row];
        }
      }
    } else {
      aquabot_view_B.x_n.set_size(1);
      aquabot_view_B.x_n[0] = CC_RegionLengths[0];
    }

    aquabot_view_B.idxCount_m.set_size(aquabot_view_B.x_n.size(0) + 1);
    aquabot_view_B.idxCount_m[0] = 0;
    row = aquabot_view_B.x_n.size(0);
    for (currentColumn = 0; currentColumn < row; currentColumn++) {
      aquabot_view_B.idxCount_m[currentColumn + 1] =
        aquabot_view_B.x_n[currentColumn];
    }

    for (currentColumn = 0; currentColumn < numRuns; currentColumn++) {
      row = (aquabot_view_B.startCol[currentColumn] - 1) * 576;
      firstRunOnPreviousColumn = aquabot_view_B.labelsRenumbered[currentColumn]
        - 1;
      lastRunOnPreviousColumn = aquabot_view_B.startRow[currentColumn];
      firstRunOnThisColumn = aquabot_view_B.endRow[currentColumn];
      for (int32_T p = lastRunOnPreviousColumn; p <= firstRunOnThisColumn; p++)
      {
        aquabot_view_B.idxCount_m[firstRunOnPreviousColumn] =
          aquabot_view_B.idxCount_m[firstRunOnPreviousColumn] + 1;
        aquabot_view_B.b[aquabot_view_B.idxCount_m[firstRunOnPreviousColumn] - 1]
          = p + row;
      }
    }

    aquabot_view_B.c.set_size(1, static_cast<int32_T>(numComponents));
    if (static_cast<int32_T>(numComponents) != 0) {
      for (numRuns = 0; numRuns < runCounter; numRuns++) {
        aquabot_view_B.c[aquabot_view_B.c.size(0) * numRuns].f1.set_size(0);
      }
    }

    if (CC_RegionLengths.size(0) != 1) {
      aquabot_view_B.x_n.set_size(CC_RegionLengths.size(0));
      row = CC_RegionLengths.size(0);
      for (currentColumn = 0; currentColumn < row; currentColumn++) {
        aquabot_view_B.x_n[currentColumn] = CC_RegionLengths[currentColumn];
      }

      if ((CC_RegionLengths.size(0) != 0) && (CC_RegionLengths.size(0) != 1)) {
        numRuns = CC_RegionLengths.size(0);
        for (currentColumn = 0; currentColumn <= numRuns - 2; currentColumn++) {
          aquabot_view_B.x_n[currentColumn + 1] =
            aquabot_view_B.x_n[currentColumn + 1] +
            aquabot_view_B.x_n[currentColumn];
        }
      }
    } else {
      aquabot_view_B.x_n.set_size(1);
      aquabot_view_B.x_n[0] = CC_RegionLengths[0];
    }

    aquabot_view_B.idxCount_m.set_size(aquabot_view_B.x_n.size(0) + 1);
    aquabot_view_B.idxCount_m[0] = 0;
    row = aquabot_view_B.x_n.size(0);
    for (currentColumn = 0; currentColumn < row; currentColumn++) {
      aquabot_view_B.idxCount_m[currentColumn + 1] =
        aquabot_view_B.x_n[currentColumn];
    }

    for (numRuns = 0; numRuns < runCounter; numRuns++) {
      firstRunOnPreviousColumn = aquabot_view_B.idxCount_m[numRuns];
      currentColumn = aquabot_view_B.idxCount_m[numRuns + 1];
      if (firstRunOnPreviousColumn + 1 > currentColumn) {
        firstRunOnPreviousColumn = 0;
        currentColumn = 0;
      }

      row = currentColumn - firstRunOnPreviousColumn;
      aquabot_view_B.c[aquabot_view_B.c.size(0) * numRuns].f1.set_size(row);
      for (currentColumn = 0; currentColumn < row; currentColumn++) {
        aquabot_view_B.c[numRuns].f1[currentColumn] =
          aquabot_view_B.b[firstRunOnPreviousColumn + currentColumn];
      }
    }
  }

  CC_RegionIndices.set_size(aquabot_view_B.b.size(0));
  row = aquabot_view_B.b.size(0);
  for (currentColumn = 0; currentColumn < row; currentColumn++) {
    CC_RegionIndices[currentColumn] = aquabot_view_B.b[currentColumn];
  }

  CC_PixelIdxList.set_size(1, aquabot_view_B.c.size(1));
  row = aquabot_view_B.c.size(1);
  for (currentColumn = 0; currentColumn < row; currentColumn++) {
    CC_PixelIdxList[currentColumn] = aquabot_view_B.c[currentColumn];
  }

  *CC_Connectivity = 8.0;
}

/* Function for MATLAB Function: '<S4>/MATLAB Function1' */
void aquabot_view::aqua_cameraIntrinsicsFromOpenCV(const real_T intrinsicMatrix
  [9], const real_T distortionCoefficients_data[], const real32_T imageSize[2],
  cameraParameters_aquabot_view_T *iobj_0, real_T intrinsics_FocalLength[2],
  real_T intrinsics_PrincipalPoint[2], real32_T intrinsics_ImageSize[2], real_T
  intrinsics_RadialDistortion[3], real_T intrinsics_TangentialDistortion[2],
  real_T *intrinsics_Skew, real_T intrinsics_K[9],
  cameraParameters_aquabot_view_T **intrinsics_CameraParameters,
  c_vision_internal_codegen_cam_T intrinsics_cameraIntrinsicsArra[], int32_T
  intrinsics_cameraIntrinsicsAr_0[2])
{
  c_vision_internal_codegen_cam_T b;
  intrinsics_FocalLength[0] = intrinsicMatrix[0];
  intrinsics_FocalLength[1] = intrinsicMatrix[4];
  intrinsics_PrincipalPoint[0] = intrinsicMatrix[6] + 1.0;
  intrinsics_PrincipalPoint[1] = intrinsicMatrix[7] + 1.0;
  intrinsics_ImageSize[0] = imageSize[0];
  intrinsics_ImageSize[1] = imageSize[1];
  intrinsics_RadialDistortion[0] = distortionCoefficients_data[0];
  intrinsics_RadialDistortion[1] = distortionCoefficients_data[1];
  intrinsics_RadialDistortion[2] = distortionCoefficients_data[4];
  intrinsics_TangentialDistortion[0] = distortionCoefficients_data[2];
  intrinsics_TangentialDistortion[1] = distortionCoefficients_data[3];
  *intrinsics_Skew = 0.0;
  intrinsics_K[0] = intrinsicMatrix[0];
  intrinsics_K[3] = 0.0;
  intrinsics_K[6] = intrinsicMatrix[6] + 1.0;
  intrinsics_K[1] = 0.0;
  intrinsics_K[4] = intrinsicMatrix[4];
  intrinsics_K[7] = intrinsicMatrix[7] + 1.0;
  intrinsics_K[2] = 0.0;
  intrinsics_K[5] = 0.0;
  intrinsics_K[8] = 1.0;
  *intrinsics_CameraParameters = iobj_0;
  b.__dummy = 0;
  intrinsics_cameraIntrinsicsAr_0[0] = 1;
  intrinsics_cameraIntrinsicsAr_0[1] = 1;
  intrinsics_cameraIntrinsicsArra[0] = b;
}

/* Function for MATLAB Function: '<S4>/MATLAB Function1' */
void aquabot_view::aquab_rigidtform3d_rigidtform3d(real_T b_this_Translation[3],
  real_T b_this_R[9], c_images_geotrans_internal_ri_T b_this_Data_data[],
  int32_T b_this_Data_size[2])
{
  c_images_geotrans_internal_ri_T b;
  static const real_T c[9] = { 0.0, -0.25881904510252074, 0.96592582628906831,
    -1.0, 0.0, 0.0, 0.0, -0.96592582628906831, -0.25881904510252074 };

  memcpy(&b_this_R[0], &c[0], 9U * sizeof(real_T));
  b_this_Translation[0] = 0.1;
  b_this_Translation[1] = 0.0;
  b_this_Translation[2] = -0.2;
  b.__dummy = 0;
  b_this_Data_size[0] = 1;
  b_this_Data_size[1] = 1;
  b_this_Data_data[0] = b;
}

/* Function for MATLAB Function: '<S4>/MATLAB Function1' */
void aquabot_view::aquabo_pointCloudBase_set_Color(pointCloud_aquabot_view_T
  *b_this, const coder::array<uint8_T, 2U> &b_value)
{
  int32_T k;
  boolean_T exitg1;
  boolean_T isInputScalarRGBTriplet;
  boolean_T p;
  aquabot_view_B.x1[0] = static_cast<uint32_T>(b_value.size(0));
  aquabot_view_B.x1[1] = static_cast<uint32_T>(b_value.size(1));
  isInputScalarRGBTriplet = false;
  p = true;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 2)) {
    if ((k << 1) + 1 != static_cast<int32_T>(aquabot_view_B.x1[k])) {
      p = false;
      exitg1 = true;
    } else {
      k++;
    }
  }

  if (p) {
    isInputScalarRGBTriplet = true;
  }

  if (isInputScalarRGBTriplet) {
    int32_T ncols;
    int32_T nrows;
    b_this->Color.set_size(b_value.size(0) * 30000, b_value.size(1));
    nrows = b_value.size(0);
    ncols = b_value.size(1);
    for (k = 0; k < ncols; k++) {
      int32_T iacol;
      int32_T ibmat;
      iacol = k * nrows;
      ibmat = nrows * 30000 * k - 1;
      for (int32_T loop_ub = 0; loop_ub < 30000; loop_ub++) {
        int32_T ibcol;
        ibcol = (loop_ub * nrows + ibmat) + 1;
        for (int32_T b_this_0 = 0; b_this_0 < nrows; b_this_0++) {
          b_this->Color[ibcol + b_this_0] = b_value[iacol + b_this_0];
        }
      }
    }
  } else {
    int32_T loop_ub;
    b_this->Color.set_size(b_value.size(0), b_value.size(1));
    loop_ub = b_value.size(0) * b_value.size(1);
    for (k = 0; k < loop_ub; k++) {
      b_this->Color[k] = b_value[k];
    }
  }
}

/* Function for MATLAB Function: '<S4>/MATLAB Function1' */
pointCloud_aquabot_view_T *aquabot_view::aquabot_v_pointCloud_pointCloud
  (pointCloud_aquabot_view_T *b_this, const real32_T varargin_1[90000])
{
  c_pointclouds_internal_codege_T b;
  pointCloud_aquabot_view_T *c_this;
  int32_T b_this_0;
  c_this = b_this;
  b_this->HasKdtreeConstructed = false;
  b_this->HasLocationHandleAllocated = false;
  for (b_this_0 = 0; b_this_0 < 90000; b_this_0++) {
    b_this->Location[b_this_0] = varargin_1[b_this_0];
  }

  aquabot_view_B.c_n.set(NULL, 0, 0);
  aquabo_pointCloudBase_set_Color(b_this, aquabot_view_B.c_n);
  b_this->Normal.set_size(0, 0);
  b_this->Intensity.set_size(0, 0);
  b_this->RangeData.set_size(0, 0);
  b.__dummy = 0;
  b_this->PointCloudArrayData.set_size(1, 1);
  b_this->PointCloudArrayData[0] = b;
  b_this->Kdtree = NULL;
  b_this->LocationHandle = NULL;
  b_this->matlabCodegenIsDeleted = false;
  return c_this;
}

/* Function for MATLAB Function: '<S4>/MATLAB Function1' */
real32_T aquabot_view::aquabot_view_xnrm2(int32_T n, const real32_T x[9],
  int32_T ix0)
{
  int32_T kend;
  real32_T scale;
  real32_T y;
  y = 0.0F;
  scale = 1.29246971E-26F;
  kend = (ix0 + n) - 1;
  for (int32_T k = ix0; k <= kend; k++) {
    real32_T absxk;
    absxk = static_cast<real32_T>(fabs(static_cast<real_T>(x[k - 1])));
    if (absxk > scale) {
      real32_T t;
      t = scale / absxk;
      y = y * t * t + 1.0F;
      scale = absxk;
    } else {
      real32_T t;
      t = absxk / scale;
      y += t * t;
    }
  }

  return scale * static_cast<real32_T>(sqrt(static_cast<real_T>(y)));
}

/* Function for MATLAB Function: '<S4>/MATLAB Function1' */
real32_T aquabot_view::aquabot_view_xnrm2_b(const real32_T x[3], int32_T ix0)
{
  real32_T scale;
  real32_T y;
  y = 0.0F;
  scale = 1.29246971E-26F;
  for (int32_T k = ix0; k <= ix0 + 1; k++) {
    real32_T absxk;
    absxk = static_cast<real32_T>(fabs(static_cast<real_T>(x[k - 1])));
    if (absxk > scale) {
      real32_T t;
      t = scale / absxk;
      y = y * t * t + 1.0F;
      scale = absxk;
    } else {
      real32_T t;
      t = absxk / scale;
      y += t * t;
    }
  }

  return scale * static_cast<real32_T>(sqrt(static_cast<real_T>(y)));
}

/* Function for MATLAB Function: '<S4>/MATLAB Function1' */
void aquabot_view::aquabot_view_xaxpy(int32_T n, real32_T a, const real32_T x[3],
  int32_T ix0, real32_T y[9], int32_T iy0)
{
  if (!(a == 0.0F)) {
    for (int32_T k = 0; k < n; k++) {
      int32_T tmp;
      tmp = (iy0 + k) - 1;
      y[tmp] += x[(ix0 + k) - 1] * a;
    }
  }
}

/* Function for MATLAB Function: '<S4>/MATLAB Function1' */
void aquabot_view::aquabot_view_xrotg(real32_T *a, real32_T *b, real32_T *c,
  real32_T *s)
{
  real32_T absa;
  real32_T absb;
  real32_T roe;
  real32_T scale;
  roe = *b;
  absa = static_cast<real32_T>(fabs(static_cast<real_T>(*a)));
  absb = static_cast<real32_T>(fabs(static_cast<real_T>(*b)));
  if (absa > absb) {
    roe = *a;
  }

  scale = absa + absb;
  if (scale == 0.0F) {
    *s = 0.0F;
    *c = 1.0F;
    *a = 0.0F;
    *b = 0.0F;
  } else {
    real32_T ads;
    real32_T bds;
    ads = absa / scale;
    bds = absb / scale;
    scale *= static_cast<real32_T>(sqrt(static_cast<real_T>(ads * ads + bds *
      bds)));
    if (roe < 0.0F) {
      scale = -scale;
    }

    *c = *a / scale;
    *s = *b / scale;
    if (absa > absb) {
      *b = *s;
    } else if (*c != 0.0F) {
      *b = 1.0F / *c;
    } else {
      *b = 1.0F;
    }

    *a = scale;
  }
}

/* Function for MATLAB Function: '<S4>/MATLAB Function1' */
void aquabot_view::aquabot_view_svd(const real32_T A[9], real32_T U[3])
{
  int32_T b_A_tmp;
  int32_T e_k;
  int32_T k_k;
  int32_T qq_tmp;
  real32_T shift;
  boolean_T apply_transform;
  boolean_T exitg1;
  for (aquabot_view_B.qp1 = 0; aquabot_view_B.qp1 < 9; aquabot_view_B.qp1++) {
    aquabot_view_B.b_A[aquabot_view_B.qp1] = A[aquabot_view_B.qp1];
  }

  aquabot_view_B.s_c[0] = 0.0F;
  aquabot_view_B.e[0] = 0.0F;
  aquabot_view_B.work[0] = 0.0F;
  aquabot_view_B.s_c[1] = 0.0F;
  aquabot_view_B.e[1] = 0.0F;
  aquabot_view_B.work[1] = 0.0F;
  aquabot_view_B.s_c[2] = 0.0F;
  aquabot_view_B.e[2] = 0.0F;
  aquabot_view_B.work[2] = 0.0F;
  for (aquabot_view_B.m = 0; aquabot_view_B.m < 2; aquabot_view_B.m++) {
    aquabot_view_B.qp1 = aquabot_view_B.m + 2;
    qq_tmp = 3 * aquabot_view_B.m + aquabot_view_B.m;
    aquabot_view_B.qq = qq_tmp + 1;
    apply_transform = false;
    aquabot_view_B.nrm = aquabot_view_xnrm2(3 - aquabot_view_B.m,
      aquabot_view_B.b_A, qq_tmp + 1);
    if (aquabot_view_B.nrm > 0.0F) {
      apply_transform = true;
      if (aquabot_view_B.b_A[qq_tmp] < 0.0F) {
        aquabot_view_B.nrm = -aquabot_view_B.nrm;
      }

      aquabot_view_B.s_c[aquabot_view_B.m] = aquabot_view_B.nrm;
      if (static_cast<real32_T>(fabs(static_cast<real_T>(aquabot_view_B.nrm))) >=
          9.86076132E-32F) {
        aquabot_view_B.nrm = 1.0F / aquabot_view_B.nrm;
        aquabot_view_B.qjj = (qq_tmp - aquabot_view_B.m) + 3;
        for (e_k = aquabot_view_B.qq; e_k <= aquabot_view_B.qjj; e_k++) {
          aquabot_view_B.b_A[e_k - 1] *= aquabot_view_B.nrm;
        }
      } else {
        aquabot_view_B.qjj = (qq_tmp - aquabot_view_B.m) + 3;
        for (e_k = aquabot_view_B.qq; e_k <= aquabot_view_B.qjj; e_k++) {
          aquabot_view_B.b_A[e_k - 1] /= aquabot_view_B.s_c[aquabot_view_B.m];
        }
      }

      aquabot_view_B.b_A[qq_tmp]++;
      aquabot_view_B.s_c[aquabot_view_B.m] =
        -aquabot_view_B.s_c[aquabot_view_B.m];
    } else {
      aquabot_view_B.s_c[aquabot_view_B.m] = 0.0F;
    }

    for (aquabot_view_B.qq = aquabot_view_B.qp1; aquabot_view_B.qq < 4;
         aquabot_view_B.qq++) {
      aquabot_view_B.qjj = (aquabot_view_B.qq - 1) * 3 + aquabot_view_B.m;
      if (apply_transform) {
        aquabot_view_B.nrm = 0.0F;
        e_k = 3 - aquabot_view_B.m;
        for (k_k = 0; k_k < e_k; k_k++) {
          aquabot_view_B.nrm += aquabot_view_B.b_A[qq_tmp + k_k] *
            aquabot_view_B.b_A[aquabot_view_B.qjj + k_k];
        }

        aquabot_view_B.nrm /= aquabot_view_B.b_A[qq_tmp];
        if (!(-aquabot_view_B.nrm == 0.0F)) {
          for (k_k = 0; k_k < e_k; k_k++) {
            b_A_tmp = aquabot_view_B.qjj + k_k;
            aquabot_view_B.b_A[b_A_tmp] += aquabot_view_B.b_A[qq_tmp + k_k] *
              -aquabot_view_B.nrm;
          }
        }
      }

      aquabot_view_B.e[aquabot_view_B.qq - 1] =
        aquabot_view_B.b_A[aquabot_view_B.qjj];
    }

    if (aquabot_view_B.m + 1 <= 1) {
      aquabot_view_B.nrm = aquabot_view_xnrm2_b(aquabot_view_B.e, 2);
      if (aquabot_view_B.nrm == 0.0F) {
        aquabot_view_B.e[0] = 0.0F;
      } else {
        if (aquabot_view_B.e[1] < 0.0F) {
          aquabot_view_B.e[0] = -aquabot_view_B.nrm;
        } else {
          aquabot_view_B.e[0] = aquabot_view_B.nrm;
        }

        aquabot_view_B.nrm = aquabot_view_B.e[0];
        if (static_cast<real32_T>(fabs(static_cast<real_T>(aquabot_view_B.e[0])))
            >= 9.86076132E-32F) {
          aquabot_view_B.nrm = 1.0F / aquabot_view_B.e[0];
          for (qq_tmp = aquabot_view_B.qp1; qq_tmp < 4; qq_tmp++) {
            aquabot_view_B.e[qq_tmp - 1] *= aquabot_view_B.nrm;
          }
        } else {
          for (qq_tmp = aquabot_view_B.qp1; qq_tmp < 4; qq_tmp++) {
            aquabot_view_B.e[qq_tmp - 1] /= aquabot_view_B.nrm;
          }
        }

        aquabot_view_B.e[1]++;
        aquabot_view_B.e[0] = -aquabot_view_B.e[0];
        for (qq_tmp = aquabot_view_B.qp1; qq_tmp < 4; qq_tmp++) {
          aquabot_view_B.work[qq_tmp - 1] = 0.0F;
        }

        for (qq_tmp = aquabot_view_B.qp1; qq_tmp < 4; qq_tmp++) {
          aquabot_view_B.rt = aquabot_view_B.e[qq_tmp - 1];
          if (!(aquabot_view_B.rt == 0.0F)) {
            aquabot_view_B.qq = (qq_tmp - 1) * 3;
            aquabot_view_B.work[1] += aquabot_view_B.b_A[aquabot_view_B.qq + 1] *
              aquabot_view_B.rt;
            aquabot_view_B.work[2] += aquabot_view_B.b_A[aquabot_view_B.qq + 2] *
              aquabot_view_B.rt;
          }
        }

        for (qq_tmp = aquabot_view_B.qp1; qq_tmp < 4; qq_tmp++) {
          aquabot_view_xaxpy(2, -aquabot_view_B.e[qq_tmp - 1] /
                             aquabot_view_B.e[1], aquabot_view_B.work, 2,
                             aquabot_view_B.b_A, 3 * (qq_tmp - 1) + 2);
        }
      }
    }
  }

  aquabot_view_B.m = 1;
  aquabot_view_B.s_c[2] = aquabot_view_B.b_A[8];
  aquabot_view_B.e[1] = aquabot_view_B.b_A[7];
  aquabot_view_B.e[2] = 0.0F;
  aquabot_view_B.qp1 = 0;
  aquabot_view_B.ztest0 = aquabot_view_B.s_c[0];
  if (aquabot_view_B.s_c[0] != 0.0F) {
    aquabot_view_B.rt = static_cast<real32_T>(fabs(static_cast<real_T>
      (aquabot_view_B.s_c[0])));
    aquabot_view_B.r_g = aquabot_view_B.s_c[0] / aquabot_view_B.rt;
    aquabot_view_B.ztest0 = aquabot_view_B.rt;
    aquabot_view_B.s_c[0] = aquabot_view_B.rt;
    aquabot_view_B.e[0] /= aquabot_view_B.r_g;
  }

  if (aquabot_view_B.e[0] != 0.0F) {
    aquabot_view_B.rt = static_cast<real32_T>(fabs(static_cast<real_T>
      (aquabot_view_B.e[0])));
    aquabot_view_B.r_g = aquabot_view_B.e[0];
    aquabot_view_B.e[0] = aquabot_view_B.rt;
    aquabot_view_B.s_c[1] *= aquabot_view_B.rt / aquabot_view_B.r_g;
  }

  if ((aquabot_view_B.ztest0 >= aquabot_view_B.e[0]) || rtIsNaNF
      (aquabot_view_B.e[0])) {
    aquabot_view_B.nrm = aquabot_view_B.ztest0;
  } else {
    aquabot_view_B.nrm = aquabot_view_B.e[0];
  }

  aquabot_view_B.ztest0 = aquabot_view_B.s_c[1];
  if (aquabot_view_B.s_c[1] != 0.0F) {
    aquabot_view_B.rt = static_cast<real32_T>(fabs(static_cast<real_T>
      (aquabot_view_B.s_c[1])));
    aquabot_view_B.r_g = aquabot_view_B.s_c[1] / aquabot_view_B.rt;
    aquabot_view_B.ztest0 = aquabot_view_B.rt;
    aquabot_view_B.s_c[1] = aquabot_view_B.rt;
    aquabot_view_B.e[1] = aquabot_view_B.b_A[7] / aquabot_view_B.r_g;
  }

  if (aquabot_view_B.e[1] != 0.0F) {
    aquabot_view_B.rt = static_cast<real32_T>(fabs(static_cast<real_T>
      (aquabot_view_B.e[1])));
    aquabot_view_B.r_g = aquabot_view_B.e[1];
    aquabot_view_B.e[1] = aquabot_view_B.rt;
    aquabot_view_B.s_c[2] = aquabot_view_B.rt / aquabot_view_B.r_g *
      aquabot_view_B.b_A[8];
  }

  if ((!(aquabot_view_B.ztest0 >= aquabot_view_B.e[1])) && (!rtIsNaNF
       (aquabot_view_B.e[1]))) {
    aquabot_view_B.ztest0 = aquabot_view_B.e[1];
  }

  if ((!(aquabot_view_B.nrm >= aquabot_view_B.ztest0)) && (!rtIsNaNF
       (aquabot_view_B.ztest0))) {
    aquabot_view_B.nrm = aquabot_view_B.ztest0;
  }

  aquabot_view_B.ztest0 = aquabot_view_B.s_c[2];
  if (aquabot_view_B.s_c[2] != 0.0F) {
    aquabot_view_B.rt = static_cast<real32_T>(fabs(static_cast<real_T>
      (aquabot_view_B.s_c[2])));
    aquabot_view_B.ztest0 = aquabot_view_B.rt;
    aquabot_view_B.s_c[2] = aquabot_view_B.rt;
  }

  if (!(aquabot_view_B.ztest0 >= 0.0F)) {
    aquabot_view_B.ztest0 = 0.0F;
  }

  if (!(aquabot_view_B.nrm >= aquabot_view_B.ztest0)) {
    aquabot_view_B.nrm = aquabot_view_B.ztest0;
  }

  while ((aquabot_view_B.m + 2 > 0) && (aquabot_view_B.qp1 < 75)) {
    qq_tmp = aquabot_view_B.m + 1;
    exitg1 = false;
    while (!(exitg1 || (qq_tmp == 0))) {
      aquabot_view_B.ztest0 = static_cast<real32_T>(fabs(static_cast<real_T>
        (aquabot_view_B.e[qq_tmp - 1])));
      if ((aquabot_view_B.ztest0 <= (static_cast<real32_T>(fabs
             (static_cast<real_T>(aquabot_view_B.s_c[qq_tmp - 1]))) +
            static_cast<real32_T>(fabs(static_cast<real_T>
              (aquabot_view_B.s_c[qq_tmp])))) * 1.1920929E-7F) ||
          ((aquabot_view_B.ztest0 <= 9.86076132E-32F) || ((aquabot_view_B.qp1 >
             20) && (aquabot_view_B.ztest0 <= 1.1920929E-7F * aquabot_view_B.nrm))))
      {
        aquabot_view_B.e[qq_tmp - 1] = 0.0F;
        exitg1 = true;
      } else {
        qq_tmp--;
      }
    }

    if (aquabot_view_B.m + 1 == qq_tmp) {
      aquabot_view_B.qjj = 4;
    } else {
      aquabot_view_B.qq = aquabot_view_B.m + 2;
      aquabot_view_B.qjj = aquabot_view_B.m + 2;
      exitg1 = false;
      while ((!exitg1) && (aquabot_view_B.qjj >= qq_tmp)) {
        aquabot_view_B.qq = aquabot_view_B.qjj;
        if (aquabot_view_B.qjj == qq_tmp) {
          exitg1 = true;
        } else {
          aquabot_view_B.ztest0 = 0.0F;
          if (aquabot_view_B.qjj < aquabot_view_B.m + 2) {
            aquabot_view_B.ztest0 = static_cast<real32_T>(fabs
              (static_cast<real_T>(aquabot_view_B.e[aquabot_view_B.qjj - 1])));
          }

          if (aquabot_view_B.qjj > qq_tmp + 1) {
            aquabot_view_B.ztest0 += static_cast<real32_T>(fabs
              (static_cast<real_T>(aquabot_view_B.e[aquabot_view_B.qjj - 2])));
          }

          aquabot_view_B.rt = static_cast<real32_T>(fabs(static_cast<real_T>
            (aquabot_view_B.s_c[aquabot_view_B.qjj - 1])));
          if ((aquabot_view_B.rt <= 1.1920929E-7F * aquabot_view_B.ztest0) ||
              (aquabot_view_B.rt <= 9.86076132E-32F)) {
            aquabot_view_B.s_c[aquabot_view_B.qjj - 1] = 0.0F;
            exitg1 = true;
          } else {
            aquabot_view_B.qjj--;
          }
        }
      }

      if (aquabot_view_B.qq == qq_tmp) {
        aquabot_view_B.qjj = 3;
      } else if (aquabot_view_B.m + 2 == aquabot_view_B.qq) {
        aquabot_view_B.qjj = 1;
      } else {
        aquabot_view_B.qjj = 2;
        qq_tmp = aquabot_view_B.qq;
      }
    }

    switch (aquabot_view_B.qjj) {
     case 1:
      aquabot_view_B.ztest0 = aquabot_view_B.e[aquabot_view_B.m];
      aquabot_view_B.e[aquabot_view_B.m] = 0.0F;
      for (aquabot_view_B.qq = aquabot_view_B.m + 1; aquabot_view_B.qq >= qq_tmp
           + 1; aquabot_view_B.qq--) {
        aquabot_view_xrotg(&aquabot_view_B.s_c[aquabot_view_B.qq - 1],
                           &aquabot_view_B.ztest0, &aquabot_view_B.rt,
                           &aquabot_view_B.r_g);
        if (aquabot_view_B.qq > qq_tmp + 1) {
          aquabot_view_B.ztest0 = -aquabot_view_B.r_g * aquabot_view_B.e[0];
          aquabot_view_B.e[0] *= aquabot_view_B.rt;
        }
      }
      break;

     case 2:
      aquabot_view_B.ztest0 = aquabot_view_B.e[qq_tmp - 1];
      aquabot_view_B.e[qq_tmp - 1] = 0.0F;
      for (aquabot_view_B.qq = qq_tmp + 1; aquabot_view_B.qq <= aquabot_view_B.m
           + 2; aquabot_view_B.qq++) {
        aquabot_view_xrotg(&aquabot_view_B.s_c[aquabot_view_B.qq - 1],
                           &aquabot_view_B.ztest0, &aquabot_view_B.r_g,
                           &aquabot_view_B.smm1);
        aquabot_view_B.rt = aquabot_view_B.e[aquabot_view_B.qq - 1];
        aquabot_view_B.ztest0 = -aquabot_view_B.smm1 * aquabot_view_B.rt;
        aquabot_view_B.e[aquabot_view_B.qq - 1] = aquabot_view_B.rt *
          aquabot_view_B.r_g;
      }
      break;

     case 3:
      aquabot_view_B.ztest0 = aquabot_view_B.s_c[aquabot_view_B.m + 1];
      aquabot_view_B.rt = static_cast<real32_T>(fabs(static_cast<real_T>
        (aquabot_view_B.ztest0)));
      aquabot_view_B.r_g = static_cast<real32_T>(fabs(static_cast<real_T>
        (aquabot_view_B.s_c[aquabot_view_B.m])));
      if ((aquabot_view_B.rt >= aquabot_view_B.r_g) || rtIsNaNF
          (aquabot_view_B.r_g)) {
        aquabot_view_B.r_g = aquabot_view_B.rt;
      }

      aquabot_view_B.rt = static_cast<real32_T>(fabs(static_cast<real_T>
        (aquabot_view_B.e[aquabot_view_B.m])));
      if ((aquabot_view_B.r_g >= aquabot_view_B.rt) || rtIsNaNF
          (aquabot_view_B.rt)) {
        aquabot_view_B.rt = aquabot_view_B.r_g;
      }

      aquabot_view_B.r_g = static_cast<real32_T>(fabs(static_cast<real_T>
        (aquabot_view_B.s_c[qq_tmp])));
      if ((aquabot_view_B.rt >= aquabot_view_B.r_g) || rtIsNaNF
          (aquabot_view_B.r_g)) {
        aquabot_view_B.r_g = aquabot_view_B.rt;
      }

      aquabot_view_B.rt = static_cast<real32_T>(fabs(static_cast<real_T>
        (aquabot_view_B.e[qq_tmp])));
      if ((aquabot_view_B.r_g >= aquabot_view_B.rt) || rtIsNaNF
          (aquabot_view_B.rt)) {
        aquabot_view_B.rt = aquabot_view_B.r_g;
      }

      aquabot_view_B.ztest0 /= aquabot_view_B.rt;
      aquabot_view_B.smm1 = aquabot_view_B.s_c[aquabot_view_B.m] /
        aquabot_view_B.rt;
      aquabot_view_B.emm1 = aquabot_view_B.e[aquabot_view_B.m] /
        aquabot_view_B.rt;
      aquabot_view_B.r_g = aquabot_view_B.s_c[qq_tmp] / aquabot_view_B.rt;
      aquabot_view_B.smm1 = ((aquabot_view_B.smm1 + aquabot_view_B.ztest0) *
        (aquabot_view_B.smm1 - aquabot_view_B.ztest0) + aquabot_view_B.emm1 *
        aquabot_view_B.emm1) / 2.0F;
      aquabot_view_B.emm1 *= aquabot_view_B.ztest0;
      aquabot_view_B.emm1 *= aquabot_view_B.emm1;
      if ((aquabot_view_B.smm1 != 0.0F) || (aquabot_view_B.emm1 != 0.0F)) {
        shift = static_cast<real32_T>(sqrt(static_cast<real_T>
          (aquabot_view_B.smm1 * aquabot_view_B.smm1 + aquabot_view_B.emm1)));
        if (aquabot_view_B.smm1 < 0.0F) {
          shift = -shift;
        }

        shift = aquabot_view_B.emm1 / (aquabot_view_B.smm1 + shift);
      } else {
        shift = 0.0F;
      }

      aquabot_view_B.ztest0 = (aquabot_view_B.r_g + aquabot_view_B.ztest0) *
        (aquabot_view_B.r_g - aquabot_view_B.ztest0) + shift;
      aquabot_view_B.rt = aquabot_view_B.e[qq_tmp] / aquabot_view_B.rt *
        aquabot_view_B.r_g;
      for (aquabot_view_B.qq = qq_tmp + 1; aquabot_view_B.qq <= aquabot_view_B.m
           + 1; aquabot_view_B.qq++) {
        aquabot_view_xrotg(&aquabot_view_B.ztest0, &aquabot_view_B.rt,
                           &aquabot_view_B.r_g, &aquabot_view_B.smm1);
        if (aquabot_view_B.qq > qq_tmp + 1) {
          aquabot_view_B.e[0] = aquabot_view_B.ztest0;
        }

        aquabot_view_B.emm1 = aquabot_view_B.e[aquabot_view_B.qq - 1];
        aquabot_view_B.rt = aquabot_view_B.s_c[aquabot_view_B.qq - 1];
        aquabot_view_B.e[aquabot_view_B.qq - 1] = aquabot_view_B.emm1 *
          aquabot_view_B.r_g - aquabot_view_B.rt * aquabot_view_B.smm1;
        aquabot_view_B.ztest0 = aquabot_view_B.smm1 *
          aquabot_view_B.s_c[aquabot_view_B.qq];
        aquabot_view_B.s_c[aquabot_view_B.qq] *= aquabot_view_B.r_g;
        aquabot_view_B.s_c[aquabot_view_B.qq - 1] = aquabot_view_B.rt *
          aquabot_view_B.r_g + aquabot_view_B.emm1 * aquabot_view_B.smm1;
        aquabot_view_xrotg(&aquabot_view_B.s_c[aquabot_view_B.qq - 1],
                           &aquabot_view_B.ztest0, &aquabot_view_B.r_g,
                           &aquabot_view_B.smm1);
        aquabot_view_B.rt = aquabot_view_B.e[aquabot_view_B.qq - 1];
        aquabot_view_B.ztest0 = aquabot_view_B.rt * aquabot_view_B.r_g +
          aquabot_view_B.smm1 * aquabot_view_B.s_c[aquabot_view_B.qq];
        aquabot_view_B.s_c[aquabot_view_B.qq] = aquabot_view_B.rt *
          -aquabot_view_B.smm1 + aquabot_view_B.r_g *
          aquabot_view_B.s_c[aquabot_view_B.qq];
        aquabot_view_B.rt = aquabot_view_B.smm1 *
          aquabot_view_B.e[aquabot_view_B.qq];
        aquabot_view_B.e[aquabot_view_B.qq] *= aquabot_view_B.r_g;
      }

      aquabot_view_B.e[aquabot_view_B.m] = aquabot_view_B.ztest0;
      aquabot_view_B.qp1++;
      break;

     default:
      if (aquabot_view_B.s_c[qq_tmp] < 0.0F) {
        aquabot_view_B.s_c[qq_tmp] = -aquabot_view_B.s_c[qq_tmp];
      }

      aquabot_view_B.qp1 = qq_tmp + 1;
      while ((qq_tmp + 1 < 3) && (aquabot_view_B.s_c[qq_tmp] <
              aquabot_view_B.s_c[aquabot_view_B.qp1])) {
        aquabot_view_B.rt = aquabot_view_B.s_c[qq_tmp];
        aquabot_view_B.s_c[qq_tmp] = aquabot_view_B.s_c[aquabot_view_B.qp1];
        aquabot_view_B.s_c[aquabot_view_B.qp1] = aquabot_view_B.rt;
        qq_tmp = aquabot_view_B.qp1;
        aquabot_view_B.qp1++;
      }

      aquabot_view_B.qp1 = 0;
      aquabot_view_B.m--;
      break;
    }
  }

  U[0] = aquabot_view_B.s_c[0];
  U[1] = aquabot_view_B.s_c[1];
  U[2] = aquabot_view_B.s_c[2];
}

/* Function for MATLAB Function: '<S4>/MATLAB Function1' */
real32_T aquabot_view::aquabot_view_maximum(const real32_T x[3])
{
  int32_T idx;
  int32_T k;
  real32_T ex;
  if (!rtIsNaNF(x[0])) {
    idx = 1;
  } else {
    boolean_T exitg1;
    idx = 0;
    k = 2;
    exitg1 = false;
    while ((!exitg1) && (k < 4)) {
      if (!rtIsNaNF(x[k - 1])) {
        idx = k;
        exitg1 = true;
      } else {
        k++;
      }
    }
  }

  if (idx == 0) {
    ex = x[0];
  } else {
    ex = x[idx - 1];
    for (k = idx + 1; k < 4; k++) {
      real32_T x_0;
      x_0 = x[k - 1];
      if (ex < x_0) {
        ex = x_0;
      }
    }
  }

  return ex;
}

/* Function for MATLAB Function: '<S4>/MATLAB Function1' */
void aquabot_view::aq_pointCloud_surfaceNormalImpl(pointCloud_aquabot_view_T
  *b_this, coder::array<real32_T, 2U> &normals)
{
  void* b_locationPtr;
  void* locationPtr;
  boolean_T createIndex;
  for (int32_T i = 0; i < 90000; i++) {
    aquabot_view_B.loc_b[i] = b_this->Location[i];
  }

  if (!b_this->HasLocationHandleAllocated) {
    for (int32_T i = 0; i < 90000; i++) {
      aquabot_view_B.location[i] = b_this->Location[i];
    }

    locationPtr = NULL;
    kdtreeGetLocationDataPointer_single(&aquabot_view_B.location[0], 30000U, 3U,
      &locationPtr);
    b_this->LocationHandle = locationPtr;
    b_this->HasLocationHandleAllocated = true;
  }

  if (!b_this->HasKdtreeConstructed) {
    locationPtr = NULL;
    kdtreeConstruct_single(&locationPtr);
    b_this->Kdtree = locationPtr;
    b_this->HasKdtreeConstructed = true;
    createIndex = true;
  } else {
    locationPtr = b_this->Kdtree;
    b_locationPtr = b_this->LocationHandle;
    createIndex = kdtreeNeedsReindex_single(locationPtr, b_locationPtr);
  }

  if (createIndex) {
    locationPtr = b_this->Kdtree;
    b_locationPtr = b_this->LocationHandle;
    kdtreeIndex_single(locationPtr, b_locationPtr, 30000U, 3U, 4.0, 1.0, 0.0);
  }

  locationPtr = b_this->Kdtree;
  memcpy(&aquabot_view_B.location[0], &aquabot_view_B.loc_b[0], 90000U * sizeof
         (real32_T));
  aquabot_view_B.indices.set_size(6, 30000);
  aquabot_view_B.a__8.set_size(6, 30000);
  aquabot_view_B.valid.set_size(30000);
  kdtreeKNNSearch_single(locationPtr, &aquabot_view_B.location[0], 30000U, 3U,
    6U, 0.0, 0.0F, &aquabot_view_B.indices[0], &aquabot_view_B.a__8[0],
    &(aquabot_view_B.valid.data())[0], 2000, 500U);
  aquabot_view_B.b_normals.set_size(30000, 3);
  PCANormalImpl_single(&aquabot_view_B.loc_b[0], &aquabot_view_B.indices[0],
                       &(aquabot_view_B.valid.data())[0], 30000U, 6U,
                       &aquabot_view_B.b_normals[0]);
  normals.set_size(30000, 3);
  for (int32_T i = 0; i < 90000; i++) {
    normals[i] = aquabot_view_B.b_normals[i];
  }
}

/* Function for MATLAB Function: '<S4>/MATLAB Function1' */
pointCloud_aquabot_view_T *aquabot_view::aquabot_pointCloud_pointCloud_k
  (pointCloud_aquabot_view_T *b_this, const real32_T varargin_1[90000], const
   coder::array<uint8_T, 2U> &varargin_3, const coder::array<real32_T, 2U>
   &varargin_5, const coder::array<real32_T, 2U> &varargin_7)
{
  c_pointclouds_internal_codege_T b;
  pointCloud_aquabot_view_T *c_this;
  int32_T i;
  int32_T loop_ub;
  c_this = b_this;
  b_this->HasKdtreeConstructed = false;
  b_this->HasLocationHandleAllocated = false;
  for (i = 0; i < 90000; i++) {
    b_this->Location[i] = varargin_1[i];
  }

  aquabo_pointCloudBase_set_Color(b_this, varargin_3);
  b_this->Normal.set_size(varargin_5.size(0), varargin_5.size(1));
  loop_ub = varargin_5.size(0) * varargin_5.size(1);
  for (i = 0; i < loop_ub; i++) {
    b_this->Normal[i] = varargin_5[i];
  }

  b_this->Intensity.set_size(varargin_7.size(0), varargin_7.size(1));
  loop_ub = varargin_7.size(0) * varargin_7.size(1);
  for (i = 0; i < loop_ub; i++) {
    b_this->Intensity[i] = varargin_7[i];
  }

  b_this->RangeData.set_size(0, 0);
  b.__dummy = 0;
  b_this->PointCloudArrayData.set_size(1, 1);
  b_this->PointCloudArrayData[0] = b;
  b_this->Kdtree = NULL;
  b_this->LocationHandle = NULL;
  b_this->matlabCodegenIsDeleted = false;
  return c_this;
}

/* Function for MATLAB Function: '<S4>/MATLAB Function1' */
pointCloud_aquabot_view_T *aquabot_view::aquabot_view_pctransform(const
  pointCloud_aquabot_view_T *ptCloudIn, const real_T tform_Translation[3], const
  real_T tform_R[9], pointCloud_aquabot_view_T *iobj_0)
{
  pointCloud_aquabot_view_T *ptCloudOut;
  boolean_T exitg1;
  boolean_T guard1;
  boolean_T isodd;
  for (aquabot_view_B.k_f = 0; aquabot_view_B.k_f < 3; aquabot_view_B.k_f++) {
    aquabot_view_B.idx_c = aquabot_view_B.k_f << 2;
    aquabot_view_B.T[aquabot_view_B.idx_c] = static_cast<real32_T>
      (tform_R[aquabot_view_B.k_f]);
    aquabot_view_B.T[aquabot_view_B.idx_c + 1] = static_cast<real32_T>
      (tform_R[aquabot_view_B.k_f + 3]);
    aquabot_view_B.T[aquabot_view_B.idx_c + 2] = static_cast<real32_T>
      (tform_R[aquabot_view_B.k_f + 6]);
    aquabot_view_B.T[aquabot_view_B.idx_c + 3] = static_cast<real32_T>
      (tform_Translation[aquabot_view_B.k_f]);
  }

  aquabot_view_B.T[12] = 0.0F;
  aquabot_view_B.T[13] = 0.0F;
  aquabot_view_B.T[14] = 0.0F;
  aquabot_view_B.T[15] = 1.0F;
  aquabot_view_B.lobj_0.matlabCodegenIsDeleted = true;
  for (aquabot_view_B.count = 0; aquabot_view_B.count < 30000;
       aquabot_view_B.count++) {
    aquabot_view_B.smax = ptCloudIn->Location[aquabot_view_B.count];
    aquabot_view_B.s_ff = ptCloudIn->Location[aquabot_view_B.count + 30000];
    aquabot_view_B.absx = ptCloudIn->Location[aquabot_view_B.count + 60000];
    for (aquabot_view_B.k_f = 0; aquabot_view_B.k_f < 3; aquabot_view_B.k_f++) {
      aquabot_view_B.idx_c = aquabot_view_B.k_f << 2;
      aquabot_view_B.rotMatOut[aquabot_view_B.k_f] =
        (aquabot_view_B.T[aquabot_view_B.idx_c + 1] * aquabot_view_B.s_ff +
         aquabot_view_B.T[aquabot_view_B.idx_c] * aquabot_view_B.smax) +
        aquabot_view_B.T[aquabot_view_B.idx_c + 2] * aquabot_view_B.absx;
    }

    aquabot_view_B.loc[aquabot_view_B.count] = aquabot_view_B.rotMatOut[0] +
      aquabot_view_B.T[3];
    aquabot_view_B.loc[aquabot_view_B.count + 30000] = aquabot_view_B.rotMatOut
      [1] + aquabot_view_B.T[7];
    aquabot_view_B.loc[aquabot_view_B.count + 60000] = aquabot_view_B.rotMatOut
      [2] + aquabot_view_B.T[11];
  }

  aquabot_view_B.nv.set_size(0, 0);
  if ((ptCloudIn->Normal.size(0) != 0) && (ptCloudIn->Normal.size(1) != 0)) {
    for (aquabot_view_B.k_f = 0; aquabot_view_B.k_f < 3; aquabot_view_B.k_f++) {
      aquabot_view_B.idx_c = aquabot_view_B.k_f << 2;
      aquabot_view_B.T_c[3 * aquabot_view_B.k_f] =
        aquabot_view_B.T[aquabot_view_B.idx_c];
      aquabot_view_B.T_c[3 * aquabot_view_B.k_f + 1] =
        aquabot_view_B.T[aquabot_view_B.idx_c + 1];
      aquabot_view_B.T_c[3 * aquabot_view_B.k_f + 2] =
        aquabot_view_B.T[aquabot_view_B.idx_c + 2];
    }

    aquabot_view_svd(aquabot_view_B.T_c, aquabot_view_B.rotMatOut);
    if (!rtIsNaNF(aquabot_view_B.rotMatOut[0])) {
      aquabot_view_B.idx_c = 1;
    } else {
      aquabot_view_B.idx_c = 0;
      aquabot_view_B.k_f = 2;
      exitg1 = false;
      while ((!exitg1) && (aquabot_view_B.k_f < 4)) {
        if (!rtIsNaNF(aquabot_view_B.rotMatOut[aquabot_view_B.k_f - 1])) {
          aquabot_view_B.idx_c = aquabot_view_B.k_f;
          exitg1 = true;
        } else {
          aquabot_view_B.k_f++;
        }
      }
    }

    if (aquabot_view_B.idx_c == 0) {
      aquabot_view_B.smax = aquabot_view_B.rotMatOut[0];
    } else {
      aquabot_view_B.smax = aquabot_view_B.rotMatOut[aquabot_view_B.idx_c - 1];
      for (aquabot_view_B.k_f = aquabot_view_B.idx_c + 1; aquabot_view_B.k_f < 4;
           aquabot_view_B.k_f++) {
        aquabot_view_B.s_ff = aquabot_view_B.rotMatOut[aquabot_view_B.k_f - 1];
        if (aquabot_view_B.smax > aquabot_view_B.s_ff) {
          aquabot_view_B.smax = aquabot_view_B.s_ff;
        }
      }
    }

    aquabot_view_B.s_ff = aquabot_view_maximum(aquabot_view_B.rotMatOut);
    aquabot_view_B.absx = static_cast<real32_T>(fabs(static_cast<real_T>
      (aquabot_view_B.s_ff)));
    if (rtIsInfF(aquabot_view_B.absx) || rtIsNaNF(aquabot_view_B.absx)) {
      aquabot_view_B.absx = (rtNaNF);
    } else if (aquabot_view_B.absx < 2.3509887E-38F) {
      aquabot_view_B.absx = 1.4013E-45F;
    } else {
      frexp(static_cast<real_T>(aquabot_view_B.absx), &aquabot_view_B.jj);
      aquabot_view_B.absx = static_cast<real32_T>(ldexp(1.0, aquabot_view_B.jj -
        24));
    }

    guard1 = false;
    if (aquabot_view_B.s_ff - aquabot_view_B.smax < 100.0F * aquabot_view_B.absx)
    {
      memcpy(&aquabot_view_B.A_o[0], &aquabot_view_B.T[0], sizeof(real32_T) <<
             4U);
      aquabot_view_B.ipiv[0] = 1;
      aquabot_view_B.ipiv[1] = 2;
      aquabot_view_B.ipiv[2] = 3;
      for (aquabot_view_B.k_f = 0; aquabot_view_B.k_f < 3; aquabot_view_B.k_f++)
      {
        aquabot_view_B.jj = aquabot_view_B.k_f * 5;
        aquabot_view_B.count = 4 - aquabot_view_B.k_f;
        aquabot_view_B.jA = 0;
        aquabot_view_B.smax = static_cast<real32_T>(fabs(static_cast<real_T>
          (aquabot_view_B.A_o[aquabot_view_B.jj])));
        for (aquabot_view_B.idx_c = 2; aquabot_view_B.idx_c <=
             aquabot_view_B.count; aquabot_view_B.idx_c++) {
          aquabot_view_B.s_ff = static_cast<real32_T>(fabs(static_cast<real_T>
            (aquabot_view_B.A_o[(aquabot_view_B.jj + aquabot_view_B.idx_c) - 1])));
          if (aquabot_view_B.s_ff > aquabot_view_B.smax) {
            aquabot_view_B.jA = aquabot_view_B.idx_c - 1;
            aquabot_view_B.smax = aquabot_view_B.s_ff;
          }
        }

        if (aquabot_view_B.A_o[aquabot_view_B.jj + aquabot_view_B.jA] != 0.0F) {
          if (aquabot_view_B.jA != 0) {
            aquabot_view_B.idx_c = aquabot_view_B.k_f + aquabot_view_B.jA;
            aquabot_view_B.ipiv[aquabot_view_B.k_f] = static_cast<int8_T>
              (aquabot_view_B.idx_c + 1);
            aquabot_view_B.smax = aquabot_view_B.A_o[aquabot_view_B.k_f];
            aquabot_view_B.A_o[aquabot_view_B.k_f] =
              aquabot_view_B.A_o[aquabot_view_B.idx_c];
            aquabot_view_B.A_o[aquabot_view_B.idx_c] = aquabot_view_B.smax;
            aquabot_view_B.smax = aquabot_view_B.A_o[aquabot_view_B.k_f + 4];
            aquabot_view_B.A_o[aquabot_view_B.k_f + 4] =
              aquabot_view_B.A_o[aquabot_view_B.idx_c + 4];
            aquabot_view_B.A_o[aquabot_view_B.idx_c + 4] = aquabot_view_B.smax;
            aquabot_view_B.smax = aquabot_view_B.A_o[aquabot_view_B.k_f + 8];
            aquabot_view_B.A_o[aquabot_view_B.k_f + 8] =
              aquabot_view_B.A_o[aquabot_view_B.idx_c + 8];
            aquabot_view_B.A_o[aquabot_view_B.idx_c + 8] = aquabot_view_B.smax;
            aquabot_view_B.smax = aquabot_view_B.A_o[aquabot_view_B.k_f + 12];
            aquabot_view_B.A_o[aquabot_view_B.k_f + 12] =
              aquabot_view_B.A_o[aquabot_view_B.idx_c + 12];
            aquabot_view_B.A_o[aquabot_view_B.idx_c + 12] = aquabot_view_B.smax;
          }

          aquabot_view_B.count = (aquabot_view_B.jj - aquabot_view_B.k_f) + 4;
          for (aquabot_view_B.idx_c = aquabot_view_B.jj + 2;
               aquabot_view_B.idx_c <= aquabot_view_B.count;
               aquabot_view_B.idx_c++) {
            aquabot_view_B.A_o[aquabot_view_B.idx_c - 1] /=
              aquabot_view_B.A_o[aquabot_view_B.jj];
          }
        }

        aquabot_view_B.jA = aquabot_view_B.jj + 6;
        aquabot_view_B.e_p = 2 - aquabot_view_B.k_f;
        for (aquabot_view_B.idx_c = 0; aquabot_view_B.idx_c <=
             aquabot_view_B.e_p; aquabot_view_B.idx_c++) {
          aquabot_view_B.smax = aquabot_view_B.A_o[((aquabot_view_B.idx_c << 2)
            + aquabot_view_B.jj) + 4];
          if (aquabot_view_B.smax != 0.0F) {
            aquabot_view_B.f = (aquabot_view_B.jA - aquabot_view_B.k_f) + 2;
            for (aquabot_view_B.count = aquabot_view_B.jA; aquabot_view_B.count <=
                 aquabot_view_B.f; aquabot_view_B.count++) {
              aquabot_view_B.A_o[aquabot_view_B.count - 1] +=
                aquabot_view_B.A_o[((aquabot_view_B.jj + aquabot_view_B.count) -
                                    aquabot_view_B.jA) + 1] *
                -aquabot_view_B.smax;
            }
          }

          aquabot_view_B.jA += 4;
        }
      }

      isodd = (aquabot_view_B.ipiv[0] > 1);
      if (aquabot_view_B.ipiv[1] > 2) {
        isodd = !isodd;
      }

      aquabot_view_B.smax = aquabot_view_B.A_o[0] * aquabot_view_B.A_o[5] *
        aquabot_view_B.A_o[10] * aquabot_view_B.A_o[15];
      if (aquabot_view_B.ipiv[2] > 3) {
        isodd = !isodd;
      }

      if (isodd) {
        aquabot_view_B.smax = -aquabot_view_B.smax;
      }

      if (static_cast<real32_T>(fabs(static_cast<real_T>(aquabot_view_B.smax -
             1.0F))) < 1.1920929E-5F) {
        aquabot_view_B.count = static_cast<int32_T>(static_cast<real_T>
          (ptCloudIn->Normal.size(0) * ptCloudIn->Normal.size(1)) / 3.0);
        aquabot_view_B.nv.set_size(ptCloudIn->Normal.size(0),
          ptCloudIn->Normal.size(1));
        aquabot_view_B.jA = aquabot_view_B.count << 1;
        for (aquabot_view_B.jj = 0; aquabot_view_B.jj < aquabot_view_B.count;
             aquabot_view_B.jj++) {
          aquabot_view_B.smax = ptCloudIn->Normal[aquabot_view_B.jj];
          aquabot_view_B.e_p = aquabot_view_B.jj + aquabot_view_B.count;
          aquabot_view_B.s_ff = ptCloudIn->Normal[aquabot_view_B.e_p];
          aquabot_view_B.f = aquabot_view_B.jj + aquabot_view_B.jA;
          aquabot_view_B.absx = ptCloudIn->Normal[aquabot_view_B.f];
          for (aquabot_view_B.k_f = 0; aquabot_view_B.k_f < 3;
               aquabot_view_B.k_f++) {
            aquabot_view_B.idx_c = aquabot_view_B.k_f << 2;
            aquabot_view_B.rotMatOut[aquabot_view_B.k_f] =
              (aquabot_view_B.T[aquabot_view_B.idx_c + 1] * aquabot_view_B.s_ff
               + aquabot_view_B.T[aquabot_view_B.idx_c] * aquabot_view_B.smax) +
              aquabot_view_B.T[aquabot_view_B.idx_c + 2] * aquabot_view_B.absx;
          }

          aquabot_view_B.nv[aquabot_view_B.jj] = aquabot_view_B.rotMatOut[0];
          aquabot_view_B.nv[aquabot_view_B.e_p] = aquabot_view_B.rotMatOut[1];
          aquabot_view_B.nv[aquabot_view_B.f] = aquabot_view_B.rotMatOut[2];
        }
      } else {
        guard1 = true;
      }
    } else {
      guard1 = true;
    }

    if (guard1) {
      aq_pointCloud_surfaceNormalImpl(aquabot_v_pointCloud_pointCloud
        (&aquabot_view_B.lobj_0, aquabot_view_B.loc), aquabot_view_B.l);
      aquabot_view_B.nv.set_size(30000, 3);
      for (aquabot_view_B.k_f = 0; aquabot_view_B.k_f < 90000;
           aquabot_view_B.k_f++) {
        aquabot_view_B.nv[aquabot_view_B.k_f] =
          aquabot_view_B.l[aquabot_view_B.k_f];
      }
    }
  }

  ptCloudOut = aquabot_pointCloud_pointCloud_k(iobj_0, aquabot_view_B.loc,
    ptCloudIn->Color, aquabot_view_B.nv, ptCloudIn->Intensity);
  if (!aquabot_view_B.lobj_0.matlabCodegenIsDeleted) {
    if (aquabot_view_B.lobj_0.HasLocationHandleAllocated) {
      aquabot_view_B.locationPtr_p5 = aquabot_view_B.lobj_0.LocationHandle;
      kdtreeDeleteLocationDataPointer_single(aquabot_view_B.locationPtr_p5);
    }

    if (aquabot_view_B.lobj_0.HasKdtreeConstructed) {
      aquabot_view_B.locationPtr_p5 = aquabot_view_B.lobj_0.Kdtree;
      kdtreeDeleteObj_single(aquabot_view_B.locationPtr_p5);
    }
  }

  return ptCloudOut;
}

/* Function for MATLAB Function: '<S4>/MATLAB Function1' */
void aquabot_view::aquabot_view_eml_find(const boolean_T x[30000], coder::array<
  int32_T, 1U> &i)
{
  int32_T idx;
  int32_T ii;
  boolean_T exitg1;
  idx = 0;
  i.set_size(30000);
  ii = 0;
  exitg1 = false;
  while ((!exitg1) && (ii < 30000)) {
    if (x[ii]) {
      idx++;
      i[idx - 1] = ii + 1;
      if (idx >= 30000) {
        exitg1 = true;
      } else {
        ii++;
      }
    } else {
      ii++;
    }
  }

  if (idx < 1) {
    i.set_size(0);
  } else {
    i.set_size(idx);
  }
}

/* Function for MATLAB Function: '<S4>/MATLAB Function1' */
void aquabot_view::aquabot_view_repmat(const real_T varargin_1[2], coder::array<
  real32_T, 2U> &b)
{
  int32_T outsize_idx_0_tmp_tmp_tmp;
  int32_T outsize_idx_1_tmp_tmp_tmp;
  outsize_idx_0_tmp_tmp_tmp = static_cast<int32_T>(varargin_1[0]);
  outsize_idx_1_tmp_tmp_tmp = static_cast<int32_T>(varargin_1[1]);
  b.set_size(outsize_idx_0_tmp_tmp_tmp, outsize_idx_1_tmp_tmp_tmp);
  outsize_idx_1_tmp_tmp_tmp = static_cast<int32_T>(varargin_1[0]) *
    static_cast<int32_T>(varargin_1[1]);
  for (outsize_idx_0_tmp_tmp_tmp = 0; outsize_idx_0_tmp_tmp_tmp <
       outsize_idx_1_tmp_tmp_tmp; outsize_idx_0_tmp_tmp_tmp++) {
    b[outsize_idx_0_tmp_tmp_tmp] = (rtNaNF);
  }
}

/* Function for MATLAB Function: '<S4>/MATLAB Function1' */
void aquabot_view::aquab_pointCloudBase_subsetImpl(const
  pointCloud_aquabot_view_T *b_this, const coder::array<real_T, 1U> &indices,
  real32_T loc[90000], coder::array<uint8_T, 2U> &c, coder::array<real32_T, 2U>
  &nv, coder::array<real32_T, 2U> &intensity, coder::array<real32_T, 2U> &r)
{
  real_T b_this_0[2];
  real_T indices_0;
  int32_T i;
  int32_T i_0;
  int32_T loc_tmp;
  int32_T loop_ub;
  for (i = 0; i < 90000; i++) {
    loc[i] = (rtNaNF);
  }

  loop_ub = indices.size(0);
  for (i = 0; i < 3; i++) {
    for (i_0 = 0; i_0 < loop_ub; i_0++) {
      loc_tmp = (30000 * i + static_cast<int32_T>(indices[i_0])) - 1;
      loc[loc_tmp] = b_this->Location[loc_tmp];
    }
  }

  if ((b_this->Color.size(0) != 0) && (b_this->Color.size(1) != 0)) {
    c.set_size(b_this->Color.size(0), b_this->Color.size(1));
    loop_ub = b_this->Color.size(0) * b_this->Color.size(1);
    for (i = 0; i < loop_ub; i++) {
      c[i] = 0U;
    }

    loop_ub = b_this->Color.size(1);
    for (i = 0; i < loop_ub; i++) {
      loc_tmp = indices.size(0);
      for (i_0 = 0; i_0 < loc_tmp; i_0++) {
        indices_0 = indices[i_0];
        c[(static_cast<int32_T>(indices_0) + c.size(0) * i) - 1] = b_this->
          Color[(b_this->Color.size(0) * i + static_cast<int32_T>(indices_0)) -
          1];
      }
    }
  } else {
    c.set_size(0, 0);
  }

  if ((b_this->Normal.size(0) != 0) && (b_this->Normal.size(1) != 0)) {
    nv.set_size(b_this->Normal.size(0), b_this->Normal.size(1));
    loop_ub = b_this->Normal.size(0) * b_this->Normal.size(1);
    for (i = 0; i < loop_ub; i++) {
      nv[i] = (rtNaNF);
    }

    loop_ub = b_this->Normal.size(1);
    for (i = 0; i < loop_ub; i++) {
      loc_tmp = indices.size(0);
      for (i_0 = 0; i_0 < loc_tmp; i_0++) {
        indices_0 = indices[i_0];
        nv[(static_cast<int32_T>(indices_0) + nv.size(0) * i) - 1] =
          b_this->Normal[(b_this->Normal.size(0) * i + static_cast<int32_T>
                          (indices_0)) - 1];
      }
    }
  } else {
    nv.set_size(0, 0);
  }

  if ((b_this->Intensity.size(0) != 0) && (b_this->Intensity.size(1) != 0)) {
    b_this_0[0] = b_this->Intensity.size(0);
    b_this_0[1] = b_this->Intensity.size(1);
    aquabot_view_repmat(b_this_0, intensity);
    loop_ub = indices.size(0);
    for (i = 0; i < loop_ub; i++) {
      indices_0 = indices[i];
      intensity[static_cast<int32_T>(indices_0) - 1] = b_this->Intensity[
        static_cast<int32_T>(indices_0) - 1];
    }
  } else {
    intensity.set_size(0, 0);
  }

  if ((b_this->RangeData.size(0) != 0) && (b_this->RangeData.size(1) != 0)) {
    r.set_size(b_this->RangeData.size(0), b_this->RangeData.size(1));
    loop_ub = b_this->RangeData.size(0) * b_this->RangeData.size(1);
    for (i = 0; i < loop_ub; i++) {
      r[i] = (rtNaNF);
    }

    loop_ub = b_this->RangeData.size(1);
    for (i = 0; i < loop_ub; i++) {
      loc_tmp = indices.size(0);
      for (i_0 = 0; i_0 < loc_tmp; i_0++) {
        indices_0 = indices[i_0];
        r[(static_cast<int32_T>(indices_0) + r.size(0) * i) - 1] =
          b_this->RangeData[(b_this->RangeData.size(0) * i + static_cast<int32_T>
                             (indices_0)) - 1];
      }
    }
  } else {
    r.set_size(0, 0);
  }
}

/* Function for MATLAB Function: '<S4>/MATLAB Function1' */
pointCloud_aquabot_view_T *aquabot_view::aquabot_view_pointCloud_select(const
  pointCloud_aquabot_view_T *b_this, const boolean_T varargin_1[30000],
  pointCloud_aquabot_view_T *iobj_0)
{
  pointCloud_aquabot_view_T *ptCloudOut;
  aquabot_view_eml_find(varargin_1, aquabot_view_B.r4);
  aquabot_view_B.r5.set_size(aquabot_view_B.r4.size(0));
  aquabot_view_B.loop_ub_h = aquabot_view_B.r4.size(0);
  for (aquabot_view_B.i5 = 0; aquabot_view_B.i5 < aquabot_view_B.loop_ub_h;
       aquabot_view_B.i5++) {
    aquabot_view_B.r5[aquabot_view_B.i5] = aquabot_view_B.r4[aquabot_view_B.i5];
  }

  aquab_pointCloudBase_subsetImpl(b_this, aquabot_view_B.r5,
    aquabot_view_B.loc_d, aquabot_view_B.c_lx, aquabot_view_B.nv_o,
    aquabot_view_B.intensity, aquabot_view_B.range);
  ptCloudOut = aquabot_pointCloud_pointCloud_k(iobj_0, aquabot_view_B.loc_d,
    aquabot_view_B.c_lx, aquabot_view_B.nv_o, aquabot_view_B.intensity);
  ptCloudOut->RangeData.set_size(aquabot_view_B.range.size(0),
    aquabot_view_B.range.size(1));
  aquabot_view_B.loop_ub_h = aquabot_view_B.range.size(0) *
    aquabot_view_B.range.size(1);
  for (aquabot_view_B.i5 = 0; aquabot_view_B.i5 < aquabot_view_B.loop_ub_h;
       aquabot_view_B.i5++) {
    ptCloudOut->RangeData[aquabot_view_B.i5] =
      aquabot_view_B.range[aquabot_view_B.i5];
  }

  return ptCloudOut;
}

/* Function for MATLAB Function: '<S4>/MATLAB Function1' */
void aquabot_view::aquabot_vi_projectLidarToCamera(const
  pointCloud_aquabot_view_T *ptCloudIn, const real_T intrinsic_K[9], const
  real_T tform_Translation[3], const real_T tform_R[9], real32_T projectPoints
  [60000])
{
  aquabot_view_B.frontPts.matlabCodegenIsDeleted = true;
  aquabot_view_B.ptCloudTransformed.matlabCodegenIsDeleted = true;
  aquabot_view_pctransform(ptCloudIn, tform_Translation, tform_R,
    &aquabot_view_B.ptCloudTransformed);
  for (aquabot_view_B.j = 0; aquabot_view_B.j < 60000; aquabot_view_B.j++) {
    projectPoints[aquabot_view_B.j] = 0.0F;
  }

  for (aquabot_view_B.j = 0; aquabot_view_B.j < 30000; aquabot_view_B.j++) {
    aquabot_view_B.ptCloudTransformed_j[aquabot_view_B.j] =
      (aquabot_view_B.ptCloudTransformed.Location[aquabot_view_B.j + 60000] >
       0.0F);
  }

  aquabot_view_pointCloud_select(&aquabot_view_B.ptCloudTransformed,
    aquabot_view_B.ptCloudTransformed_j, &aquabot_view_B.frontPts);
  for (aquabot_view_B.j = 0; aquabot_view_B.j < 30000; aquabot_view_B.j++) {
    aquabot_view_B.s_f = aquabot_view_B.frontPts.Location[aquabot_view_B.j];
    aquabot_view_B.y_i = aquabot_view_B.frontPts.Location[aquabot_view_B.j +
      30000];
    aquabot_view_B.z = aquabot_view_B.frontPts.Location[aquabot_view_B.j + 60000];
    aquabot_view_B.ptCloudTransformed.Location[aquabot_view_B.j] =
      aquabot_view_B.s_f;
    aquabot_view_B.ptCloudTransformed.Location[aquabot_view_B.j + 30000] =
      aquabot_view_B.y_i;
    aquabot_view_B.ptCloudTransformed.Location[aquabot_view_B.j + 60000] =
      aquabot_view_B.z;
  }

  for (aquabot_view_B.j = 0; aquabot_view_B.j < 3; aquabot_view_B.j++) {
    aquabot_view_B.A[3 * aquabot_view_B.j] = intrinsic_K[aquabot_view_B.j];
    aquabot_view_B.A[3 * aquabot_view_B.j + 1] = intrinsic_K[aquabot_view_B.j +
      3];
    aquabot_view_B.A[3 * aquabot_view_B.j + 2] = intrinsic_K[aquabot_view_B.j +
      6];
  }

  for (aquabot_view_B.j = 0; aquabot_view_B.j < 30000; aquabot_view_B.j++) {
    aquabot_view_B.coffset = aquabot_view_B.j * 3;
    for (aquabot_view_B.i_mj = 0; aquabot_view_B.i_mj < 3; aquabot_view_B.i_mj++)
    {
      aquabot_view_B.aoffset = aquabot_view_B.i_mj * 3;
      aquabot_view_B.s_f = static_cast<real32_T>
        (aquabot_view_B.A[aquabot_view_B.aoffset]) *
        aquabot_view_B.ptCloudTransformed.Location[aquabot_view_B.j];
      aquabot_view_B.s_f += static_cast<real32_T>
        (aquabot_view_B.A[aquabot_view_B.aoffset + 1]) *
        aquabot_view_B.ptCloudTransformed.Location[aquabot_view_B.j + 30000];
      aquabot_view_B.projectedPts[aquabot_view_B.coffset + aquabot_view_B.i_mj] =
        static_cast<real32_T>(aquabot_view_B.A[aquabot_view_B.aoffset + 2]) *
        aquabot_view_B.ptCloudTransformed.Location[aquabot_view_B.j + 60000] +
        aquabot_view_B.s_f;
    }
  }

  for (aquabot_view_B.j = 0; aquabot_view_B.j < 30000; aquabot_view_B.j++) {
    aquabot_view_B.s_f = aquabot_view_B.projectedPts[3 * aquabot_view_B.j + 2];
    projectPoints[aquabot_view_B.j] = aquabot_view_B.projectedPts[3 *
      aquabot_view_B.j] / aquabot_view_B.s_f;
    projectPoints[aquabot_view_B.j + 30000] = aquabot_view_B.projectedPts[3 *
      aquabot_view_B.j + 1] / aquabot_view_B.s_f;
  }

  if (!aquabot_view_B.ptCloudTransformed.matlabCodegenIsDeleted) {
    if (aquabot_view_B.ptCloudTransformed.HasLocationHandleAllocated) {
      aquabot_view_B.locationPtr_p =
        aquabot_view_B.ptCloudTransformed.LocationHandle;
      kdtreeDeleteLocationDataPointer_single(aquabot_view_B.locationPtr_p);
    }

    if (aquabot_view_B.ptCloudTransformed.HasKdtreeConstructed) {
      aquabot_view_B.locationPtr_p = aquabot_view_B.ptCloudTransformed.Kdtree;
      kdtreeDeleteObj_single(aquabot_view_B.locationPtr_p);
    }
  }

  if (!aquabot_view_B.frontPts.matlabCodegenIsDeleted) {
    if (aquabot_view_B.frontPts.HasLocationHandleAllocated) {
      aquabot_view_B.locationPtr_p = aquabot_view_B.frontPts.LocationHandle;
      kdtreeDeleteLocationDataPointer_single(aquabot_view_B.locationPtr_p);
    }

    if (aquabot_view_B.frontPts.HasKdtreeConstructed) {
      aquabot_view_B.locationPtr_p = aquabot_view_B.frontPts.Kdtree;
      kdtreeDeleteObj_single(aquabot_view_B.locationPtr_p);
    }
  }
}

/* Function for MATLAB Function: '<S4>/MATLAB Function1' */
void aquabot_view::aquabot_vi_projectPointsToImage(const
  pointCloud_aquabot_view_T *pc, const real_T intrinsic_K[9], const real_T
  tform_Translation[3], const real_T tform_R[9], const real32_T imageSize[2],
  coder::array<real32_T, 2U> &imPts, coder::array<real_T, 1U> &indices)
{
  boolean_T b_indices;
  aquabot_vi_projectLidarToCamera((pointCloud_aquabot_view_T *)pc, intrinsic_K,
    tform_Translation, tform_R, aquabot_view_B.b_imPts);
  aquabot_view_B.imageSize = imageSize[0];
  aquabot_view_B.imageSize_o = imageSize[1];
  aquabot_view_B.trueCount = 0;
  for (aquabot_view_B.i_m = 0; aquabot_view_B.i_m < 30000; aquabot_view_B.i_m++)
  {
    aquabot_view_B.b_imPts_o = aquabot_view_B.b_imPts[aquabot_view_B.i_m];
    aquabot_view_B.b_imPts_i = aquabot_view_B.b_imPts[aquabot_view_B.i_m + 30000];
    b_indices = ((aquabot_view_B.b_imPts_o < aquabot_view_B.imageSize) &&
                 (aquabot_view_B.b_imPts_o > 0.0F) && (!rtIsNaNF
      (aquabot_view_B.b_imPts_o)) && ((aquabot_view_B.b_imPts_i <
      aquabot_view_B.imageSize_o) && (aquabot_view_B.b_imPts_i > 0.0F) &&
      (!rtIsNaNF(aquabot_view_B.b_imPts_i))));
    aquabot_view_B.b_indices[aquabot_view_B.i_m] = b_indices;
    if (b_indices) {
      aquabot_view_B.trueCount++;
    }
  }

  aquabot_view_B.tmp_size_idx_0 = aquabot_view_B.trueCount;
  aquabot_view_B.trueCount = 0;
  for (aquabot_view_B.i_m = 0; aquabot_view_B.i_m < 30000; aquabot_view_B.i_m++)
  {
    if (aquabot_view_B.b_indices[aquabot_view_B.i_m]) {
      aquabot_view_B.tmp_data_e[aquabot_view_B.trueCount] = static_cast<int16_T>
        (aquabot_view_B.i_m);
      aquabot_view_B.trueCount++;
    }
  }

  imPts.set_size(aquabot_view_B.tmp_size_idx_0, 2);
  for (aquabot_view_B.i_m = 0; aquabot_view_B.i_m < 2; aquabot_view_B.i_m++) {
    for (aquabot_view_B.trueCount = 0; aquabot_view_B.trueCount <
         aquabot_view_B.tmp_size_idx_0; aquabot_view_B.trueCount++) {
      imPts[aquabot_view_B.trueCount + imPts.size(0) * aquabot_view_B.i_m] =
        aquabot_view_B.b_imPts[30000 * aquabot_view_B.i_m +
        aquabot_view_B.tmp_data_e[aquabot_view_B.trueCount]];
    }
  }

  aquabot_view_eml_find(aquabot_view_B.b_indices, aquabot_view_B.r2);
  indices.set_size(aquabot_view_B.r2.size(0));
  aquabot_view_B.trueCount = aquabot_view_B.r2.size(0);
  for (aquabot_view_B.i_m = 0; aquabot_view_B.i_m < aquabot_view_B.trueCount;
       aquabot_view_B.i_m++) {
    indices[aquabot_view_B.i_m] = aquabot_view_B.r2[aquabot_view_B.i_m];
  }
}

/* Function for MATLAB Function: '<S4>/MATLAB Function1' */
void aquabot_view::aquab_projectLidarPointsOnImage(const real32_T ptCloudIn
  [90000], const real32_T intrinsics_ImageSize[2], const real_T intrinsics_K[9],
  const real_T tform_Translation[3], const real_T tform_R[9], coder::array<
  real32_T, 2U> &imPts, coder::array<real_T, 1U> &indices)
{
  aquabot_view_B.ptCloud.matlabCodegenIsDeleted = true;
  aquabot_v_pointCloud_pointCloud(&aquabot_view_B.ptCloud, ptCloudIn);
  aquabot_view_B.intrinsics_ImageSize_c[0] = intrinsics_ImageSize[1];
  aquabot_view_B.intrinsics_ImageSize_c[1] = intrinsics_ImageSize[0];
  aquabot_vi_projectPointsToImage(&aquabot_view_B.ptCloud, intrinsics_K,
    tform_Translation, tform_R, aquabot_view_B.intrinsics_ImageSize_c, imPts,
    indices);
  if (!aquabot_view_B.ptCloud.matlabCodegenIsDeleted) {
    if (aquabot_view_B.ptCloud.HasLocationHandleAllocated) {
      aquabot_view_B.locationPtr = aquabot_view_B.ptCloud.LocationHandle;
      kdtreeDeleteLocationDataPointer_single(aquabot_view_B.locationPtr);
    }

    if (aquabot_view_B.ptCloud.HasKdtreeConstructed) {
      aquabot_view_B.locationPtr = aquabot_view_B.ptCloud.Kdtree;
      kdtreeDeleteObj_single(aquabot_view_B.locationPtr);
    }
  }
}

real32_T rt_roundf_snf(real32_T u)
{
  real32_T y;
  if (static_cast<real32_T>(fabs(static_cast<real_T>(u))) < 8.388608E+6F) {
    if (u >= 0.5F) {
      y = static_cast<real32_T>(floor(static_cast<real_T>(u + 0.5F)));
    } else if (u > -0.5F) {
      y = u * 0.0F;
    } else {
      y = static_cast<real32_T>(ceil(static_cast<real_T>(u - 0.5F)));
    }
  } else {
    y = u;
  }

  return y;
}

/* Function for MATLAB Function: '<S4>/MATLAB Function1' */
void aquabot_view::aquabot_view_sub2ind(const coder::array<real32_T, 1U>
  &varargin_1, const coder::array<real32_T, 1U> &varargin_2, coder::array<
  int32_T, 1U> &idx)
{
  int32_T loop_ub;
  idx.set_size(varargin_1.size(0));
  loop_ub = varargin_1.size(0);
  for (int32_T i = 0; i < loop_ub; i++) {
    idx[i] = (static_cast<int32_T>(varargin_2[i]) - 1) * 576 + static_cast<
      int32_T>(varargin_1[i]);
  }
}

/* Model step function */
void aquabot_view::step()
{
  SL_Bus_std_msgs_Bool rtb_BusAssignment;
  uint8_T b_varargout_2_data;
  uint8_T b_varargout_2_is_bigendian_0;
  boolean_T b_varargout_1;
  boolean_T b_varargout_2_is_bigendian;
  boolean_T b_varargout_2_is_dense;
  static const sBCDbgMZyfjRVWzdpNZ1R9_aquabo_T tmp = { 0.0/* Area */
  };

  cameraParameters_aquabot_view_T *tmp_0;
  boolean_T guard1;

  /* MATLABSystem: '<S8>/SourceBlock' */
  aquabot_view_SystemCore_step_m(&b_varargout_1,
    &aquabot_view_B.b_varargout_2_header_stamp_sec,
    &aquabot_view_B.b_varargout_2_header_stamp_nano,
    aquabot_view_B.b_varargout_2_header_frame_id,
    &aquabot_view_B.b_varargout_2_header_frame_id_S,
    &aquabot_view_B.b_varargout_2_header_frame_id_m,
    &aquabot_view_B.b_varargout_2_height, &aquabot_view_B.b_varargout_2_width,
    aquabot_view_B.b_varargout_2_fields,
    &aquabot_view_B.b_varargout_2_fields_SL_Info_Cu,
    &aquabot_view_B.b_varargout_2_fields_SL_Info_Re, &b_varargout_2_is_bigendian,
    &aquabot_view_B.b_varargout_2_point_step,
    &aquabot_view_B.b_varargout_2_row_step, aquabot_view_B.b_varargout_2_data_h,
    &aquabot_view_B.b_varargout_2_data_SL_Info_Curr,
    &aquabot_view_B.b_varargout_2_data_SL_Info_Rece, &b_varargout_2_is_dense);

  /* Outputs for Enabled SubSystem: '<Root>/Subsystem2' incorporates:
   *  EnablePort: '<S10>/Enable'
   */
  /* Outputs for Enabled SubSystem: '<S8>/Enabled Subsystem' incorporates:
   *  EnablePort: '<S18>/Enable'
   */
  if (b_varargout_1) {
    /* SignalConversion generated from: '<S18>/In1' */
    aquabot_view_B.In1_p.header.stamp.sec =
      aquabot_view_B.b_varargout_2_header_stamp_sec;
    aquabot_view_B.In1_p.header.stamp.nanosec =
      aquabot_view_B.b_varargout_2_header_stamp_nano;
    memcpy(&aquabot_view_B.In1_p.header.frame_id[0],
           &aquabot_view_B.b_varargout_2_header_frame_id[0], sizeof(uint8_T) <<
           7U);
    aquabot_view_B.In1_p.header.frame_id_SL_Info.CurrentLength =
      aquabot_view_B.b_varargout_2_header_frame_id_S;
    aquabot_view_B.In1_p.header.frame_id_SL_Info.ReceivedLength =
      aquabot_view_B.b_varargout_2_header_frame_id_m;
    aquabot_view_B.In1_p.height = aquabot_view_B.b_varargout_2_height;
    aquabot_view_B.In1_p.width = aquabot_view_B.b_varargout_2_width;
    memcpy(&aquabot_view_B.In1_p.fields[0],
           &aquabot_view_B.b_varargout_2_fields[0], sizeof
           (SL_Bus_sensor_msgs_PointField) << 4U);
    aquabot_view_B.In1_p.fields_SL_Info.CurrentLength =
      aquabot_view_B.b_varargout_2_fields_SL_Info_Cu;
    aquabot_view_B.In1_p.fields_SL_Info.ReceivedLength =
      aquabot_view_B.b_varargout_2_fields_SL_Info_Re;
    aquabot_view_B.In1_p.is_bigendian = b_varargout_2_is_bigendian;
    aquabot_view_B.In1_p.point_step = aquabot_view_B.b_varargout_2_point_step;
    aquabot_view_B.In1_p.row_step = aquabot_view_B.b_varargout_2_row_step;
    memcpy(&aquabot_view_B.In1_p.data[0], &aquabot_view_B.b_varargout_2_data_h[0],
           960000U * sizeof(uint8_T));
    aquabot_view_B.In1_p.data_SL_Info.CurrentLength =
      aquabot_view_B.b_varargout_2_data_SL_Info_Curr;
    aquabot_view_B.In1_p.data_SL_Info.ReceivedLength =
      aquabot_view_B.b_varargout_2_data_SL_Info_Rece;
    aquabot_view_B.In1_p.is_dense = b_varargout_2_is_dense;

    /* MATLABSystem: '<S10>/Read Point Cloud' */
    aquabot_ReadPointCloud_stepImpl(&aquabot_view_B.In1_p,
      aquabot_view_B.ReadPointCloud);
  }

  /* End of MATLABSystem: '<S8>/SourceBlock' */
  /* End of Outputs for SubSystem: '<S8>/Enabled Subsystem' */
  /* End of Outputs for SubSystem: '<Root>/Subsystem2' */

  /* BusAssignment: '<Root>/Bus Assignment2' incorporates:
   *  Constant: '<S3>/Constant'
   */
  aquabot_view_B.BusAssignment2 = aquabot_view_P.Constant_Value_l;

  /* MATLABSystem: '<S9>/SourceBlock' incorporates:
   *  MATLAB Function: '<S4>/MATLAB Function'
   *  MATLAB Function: '<S4>/MATLAB Function2'
   *  MATLABSystem: '<S16>/SourceBlock'
   *  UnitDelay: '<S4>/Unit Delay'
   */
  aquabot_view_SystemCore_step_my(&b_varargout_1,
    &aquabot_view_B.b_varargout_2_header_stamp_sec,
    &aquabot_view_B.b_varargout_2_header_stamp_nano,
    aquabot_view_B.b_varargout_2_header_frame_id,
    &aquabot_view_B.b_varargout_2_header_frame_id_S,
    &aquabot_view_B.b_varargout_2_header_frame_id_m,
    &aquabot_view_B.b_varargout_2_height, &aquabot_view_B.b_varargout_2_width,
    aquabot_view_B.b_varargout_2_encoding,
    &aquabot_view_B.b_varargout_2_fields_SL_Info_Cu,
    &aquabot_view_B.b_varargout_2_fields_SL_Info_Re,
    &b_varargout_2_is_bigendian_0, &aquabot_view_B.b_varargout_2_point_step,
    aquabot_view_B.b_varargout_2_data,
    &aquabot_view_B.b_varargout_2_data_SL_Info_Curr,
    &aquabot_view_B.b_varargout_2_data_SL_Info_Rece);

  /* Outputs for Enabled SubSystem: '<Root>/Detect boat' incorporates:
   *  EnablePort: '<S4>/Enable'
   */
  /* Outputs for Enabled SubSystem: '<S9>/Enabled Subsystem' incorporates:
   *  EnablePort: '<S19>/Enable'
   */
  if (b_varargout_1) {
    /* SignalConversion generated from: '<S19>/In1' */
    aquabot_view_B.In1.header.stamp.sec =
      aquabot_view_B.b_varargout_2_header_stamp_sec;
    aquabot_view_B.In1.header.stamp.nanosec =
      aquabot_view_B.b_varargout_2_header_stamp_nano;
    aquabot_view_B.In1.header.frame_id_SL_Info.CurrentLength =
      aquabot_view_B.b_varargout_2_header_frame_id_S;
    aquabot_view_B.In1.header.frame_id_SL_Info.ReceivedLength =
      aquabot_view_B.b_varargout_2_header_frame_id_m;
    aquabot_view_B.In1.height = aquabot_view_B.b_varargout_2_height;
    aquabot_view_B.In1.width = aquabot_view_B.b_varargout_2_width;
    memcpy(&aquabot_view_B.In1.header.frame_id[0],
           &aquabot_view_B.b_varargout_2_header_frame_id[0], sizeof(uint8_T) <<
           7U);
    memcpy(&aquabot_view_B.In1.encoding[0],
           &aquabot_view_B.b_varargout_2_encoding[0], sizeof(uint8_T) << 7U);
    aquabot_view_B.In1.encoding_SL_Info.CurrentLength =
      aquabot_view_B.b_varargout_2_fields_SL_Info_Cu;
    aquabot_view_B.In1.encoding_SL_Info.ReceivedLength =
      aquabot_view_B.b_varargout_2_fields_SL_Info_Re;
    aquabot_view_B.In1.is_bigendian = b_varargout_2_is_bigendian_0;
    aquabot_view_B.In1.step = aquabot_view_B.b_varargout_2_point_step;
    memcpy(&aquabot_view_B.In1.data[0], &aquabot_view_B.b_varargout_2_data[0],
           1244160U * sizeof(uint8_T));
    aquabot_view_B.In1.data_SL_Info.CurrentLength =
      aquabot_view_B.b_varargout_2_data_SL_Info_Curr;
    aquabot_view_B.In1.data_SL_Info.ReceivedLength =
      aquabot_view_B.b_varargout_2_data_SL_Info_Rece;

    /* MATLABSystem: '<S4>/Read Image' */
    aquabot_view_SystemCore_step(&aquabot_view_DW.obj, aquabot_view_B.In1.height,
      aquabot_view_B.In1.width, aquabot_view_B.In1.encoding,
      aquabot_view_B.In1.encoding_SL_Info.CurrentLength, aquabot_view_B.In1.data,
      aquabot_view_B.In1.data_SL_Info.CurrentLength,
      aquabot_view_B.In1.data_SL_Info.ReceivedLength,
      aquabot_view_B.b_varargout_2_data, &b_varargout_2_is_bigendian_0);
    aquabot_view_DW.ImageSize[0] = aquabot_view_DW.obj.ImageSize[0];
    aquabot_view_DW.ImageSize[1] = aquabot_view_DW.obj.ImageSize[1];
    memcpy(&aquabot_view_DW.Image[0], &aquabot_view_DW.obj.Image[0], 1244160U *
           sizeof(uint8_T));

    /* MATLAB Function: '<S4>/MATLAB Function3' incorporates:
     *  MATLABSystem: '<S4>/Read Image'
     */
    for (aquabot_view_B.d_k = 0; aquabot_view_B.d_k < 414720; aquabot_view_B.d_k
         ++) {
      b_varargout_2_is_bigendian_0 =
        aquabot_view_B.b_varargout_2_data[aquabot_view_B.d_k];
      b_varargout_2_data = aquabot_view_B.b_varargout_2_data[aquabot_view_B.d_k
        + 414720];
      aquabot_view_B.BW[aquabot_view_B.d_k] = ((b_varargout_2_is_bigendian_0 >=
        14) && (b_varargout_2_is_bigendian_0 <= 223) && (b_varargout_2_data >= 7)
        && (b_varargout_2_data <= 19) &&
        (aquabot_view_B.b_varargout_2_data[aquabot_view_B.d_k + 829440] <= 39));
    }

    /* End of MATLAB Function: '<S4>/MATLAB Function3' */

    /* MATLAB Function: '<S4>/MATLAB Function2' */
    aquabot_view_bwconncomp(aquabot_view_B.BW, &aquabot_view_B.bsum,
      aquabot_view_B.expl_temp_m3, &aquabot_view_B.CC_NumObjects,
      aquabot_view_B.CC_RegionIndices, aquabot_view_B.CC_RegionLengths,
      aquabot_view_B.expl_temp);
    aquabot_view_B.firstBlockLength = static_cast<int32_T>
      (aquabot_view_B.CC_NumObjects);
    aquabot_view_B.outstats.set_size(aquabot_view_B.firstBlockLength);

    /* MATLAB Function: '<S4>/MATLAB Function2' */
    for (aquabot_view_B.d_k = 0; aquabot_view_B.d_k <
         aquabot_view_B.firstBlockLength; aquabot_view_B.d_k++) {
      aquabot_view_B.outstats[aquabot_view_B.d_k] = tmp;
    }

    aquabot_view_B.a_m.Area = 0.0;
    aquabot_view_B.a_m.Centroid[0] = 0.0;
    aquabot_view_B.a_m.Centroid[1] = 0.0;
    aquabot_view_B.a_m.BoundingBox[0] = 0.0;
    aquabot_view_B.a_m.BoundingBox[1] = 0.0;
    aquabot_view_B.a_m.BoundingBox[2] = 0.0;
    aquabot_view_B.a_m.BoundingBox[3] = 0.0;
    aquabot_view_B.a_m.MajorAxisLength = 0.0;
    aquabot_view_B.a_m.MinorAxisLength = 0.0;
    aquabot_view_B.a_m.Eccentricity = 0.0;
    aquabot_view_B.a_m.Orientation = 0.0;
    aquabot_view_B.a_m.Image.size[0] = 0;
    aquabot_view_B.a_m.Image.size[1] = 0;
    aquabot_view_B.a_m.FilledImage.size[0] = 0;
    aquabot_view_B.a_m.FilledImage.size[1] = 0;
    aquabot_view_B.a_m.FilledArea = 0.0;
    aquabot_view_B.a_m.EulerNumber = 0.0;
    for (aquabot_view_B.d_k = 0; aquabot_view_B.d_k < 16; aquabot_view_B.d_k++)
    {
      aquabot_view_B.a_m.Extrema[aquabot_view_B.d_k] = 0.0;
    }

    aquabot_view_B.a_m.EquivDiameter = 0.0;
    aquabot_view_B.a_m.Extent = 0.0;
    aquabot_view_B.a_m.PixelIdxList.set_size(0);

    /* MATLAB Function: '<S4>/MATLAB Function2' */
    aquabot_view_B.a_m.PixelList.size[0] = 0;
    aquabot_view_B.a_m.PixelList.size[1] = 2;
    aquabot_view_B.a_m.Perimeter = 0.0;
    aquabot_view_B.a_m.Circularity = 0.0;
    aquabot_view_B.a_m.PixelValues.size[0] = 0;
    aquabot_view_B.a_m.WeightedCentroid[0] = 0.0;
    aquabot_view_B.a_m.WeightedCentroid[1] = 0.0;
    aquabot_view_B.a_m.MeanIntensity = 0.0;
    aquabot_view_B.a_m.MinIntensity = 0.0;
    aquabot_view_B.a_m.MaxIntensity = 0.0;
    aquabot_view_B.a_m.SubarrayIdx.size[0] = 1;
    aquabot_view_B.a_m.SubarrayIdx.size[1] = 0;
    aquabot_view_B.a_m.SubarrayIdxLengths[0] = 0.0;
    aquabot_view_B.a_m.SubarrayIdxLengths[1] = 0.0;
    aquabot_view_B.stats.set_size(aquabot_view_B.firstBlockLength);

    /* MATLAB Function: '<S4>/MATLAB Function2' */
    for (aquabot_view_B.d_k = 0; aquabot_view_B.d_k <
         aquabot_view_B.firstBlockLength; aquabot_view_B.d_k++) {
      aquabot_view_B.stats[aquabot_view_B.d_k] = aquabot_view_B.a_m;
    }

    if (aquabot_view_B.CC_NumObjects != 0.0) {
      if (aquabot_view_B.CC_RegionLengths.size(0) != 1) {
        aquabot_view_B.x.set_size(aquabot_view_B.CC_RegionLengths.size(0));
        aquabot_view_B.hi = aquabot_view_B.CC_RegionLengths.size(0);
        for (aquabot_view_B.d_k = 0; aquabot_view_B.d_k < aquabot_view_B.hi;
             aquabot_view_B.d_k++) {
          aquabot_view_B.x[aquabot_view_B.d_k] =
            aquabot_view_B.CC_RegionLengths[aquabot_view_B.d_k];
        }

        if ((aquabot_view_B.CC_RegionLengths.size(0) != 0) &&
            (aquabot_view_B.CC_RegionLengths.size(0) != 1)) {
          aquabot_view_B.b_varargout_2_header_stamp_sec =
            aquabot_view_B.CC_RegionLengths.size(0);
          for (aquabot_view_B.nblocks = 0; aquabot_view_B.nblocks <=
               aquabot_view_B.b_varargout_2_header_stamp_sec - 2;
               aquabot_view_B.nblocks++) {
            aquabot_view_B.x[aquabot_view_B.nblocks + 1] =
              aquabot_view_B.x[aquabot_view_B.nblocks + 1] +
              aquabot_view_B.x[aquabot_view_B.nblocks];
          }
        }
      } else {
        aquabot_view_B.x.set_size(1);
        aquabot_view_B.x[0] = aquabot_view_B.CC_RegionLengths[0];
      }

      aquabot_view_B.idxCount.set_size(aquabot_view_B.x.size(0) + 1);
      aquabot_view_B.idxCount[0] = 0;
      aquabot_view_B.hi = aquabot_view_B.x.size(0);
      for (aquabot_view_B.d_k = 0; aquabot_view_B.d_k < aquabot_view_B.hi;
           aquabot_view_B.d_k++) {
        aquabot_view_B.idxCount[aquabot_view_B.d_k + 1] =
          aquabot_view_B.x[aquabot_view_B.d_k];
      }

      for (aquabot_view_B.i = 0; aquabot_view_B.i <
           aquabot_view_B.firstBlockLength; aquabot_view_B.i++) {
        aquabot_view_B.nblocks = aquabot_view_B.idxCount[aquabot_view_B.i];
        aquabot_view_B.lastBlockLength =
          aquabot_view_B.idxCount[aquabot_view_B.i + 1];
        if (aquabot_view_B.nblocks + 1 > aquabot_view_B.lastBlockLength) {
          aquabot_view_B.nblocks = 0;
          aquabot_view_B.lastBlockLength = 0;
        }

        aquabot_view_B.hi = aquabot_view_B.lastBlockLength -
          aquabot_view_B.nblocks;
        aquabot_view_B.stats[aquabot_view_B.i].PixelIdxList.set_size
          (aquabot_view_B.hi);
        for (aquabot_view_B.d_k = 0; aquabot_view_B.d_k < aquabot_view_B.hi;
             aquabot_view_B.d_k++) {
          aquabot_view_B.stats[aquabot_view_B.i].PixelIdxList[aquabot_view_B.d_k]
            = aquabot_view_B.CC_RegionIndices[aquabot_view_B.nblocks +
            aquabot_view_B.d_k];
        }
      }
    }

    aquabot_view_B.b_varargout_2_header_stamp_sec = aquabot_view_B.stats.size(0);
    for (aquabot_view_B.d_k = 0; aquabot_view_B.d_k <
         aquabot_view_B.b_varargout_2_header_stamp_sec; aquabot_view_B.d_k++) {
      aquabot_view_B.stats[aquabot_view_B.d_k].Area =
        aquabot_view_B.stats[aquabot_view_B.d_k].PixelIdxList.size(0);
    }

    aquabot_view_B.b_varargout_2_header_stamp_sec = aquabot_view_B.stats.size(0);
    for (aquabot_view_B.nblocks = 0; aquabot_view_B.nblocks <
         aquabot_view_B.b_varargout_2_header_stamp_sec; aquabot_view_B.nblocks++)
    {
      aquabot_view_B.outstats[aquabot_view_B.nblocks].Area =
        aquabot_view_B.stats[aquabot_view_B.nblocks].Area;
    }

    if (aquabot_view_B.outstats.size(0) != 0) {
      aquabot_view_B.tf.set_size(1, aquabot_view_B.outstats.size(0));
      aquabot_view_B.hi = aquabot_view_B.outstats.size(0);
      for (aquabot_view_B.b_varargout_2_header_stamp_sec = 0;
           aquabot_view_B.b_varargout_2_header_stamp_sec < aquabot_view_B.hi;
           aquabot_view_B.b_varargout_2_header_stamp_sec++) {
        aquabot_view_B.tf[aquabot_view_B.b_varargout_2_header_stamp_sec] =
          static_cast<uint32_T>
          (aquabot_view_B.outstats[aquabot_view_B.b_varargout_2_header_stamp_sec]
           .Area);
      }

      if (aquabot_view_B.CC_RegionLengths.size(0) != 1) {
        aquabot_view_B.x.set_size(aquabot_view_B.CC_RegionLengths.size(0));
        aquabot_view_B.hi = aquabot_view_B.CC_RegionLengths.size(0);
        for (aquabot_view_B.d_k = 0; aquabot_view_B.d_k < aquabot_view_B.hi;
             aquabot_view_B.d_k++) {
          aquabot_view_B.x[aquabot_view_B.d_k] =
            aquabot_view_B.CC_RegionLengths[aquabot_view_B.d_k];
        }

        if ((aquabot_view_B.CC_RegionLengths.size(0) != 0) &&
            (aquabot_view_B.CC_RegionLengths.size(0) != 1)) {
          aquabot_view_B.nblocks = aquabot_view_B.CC_RegionLengths.size(0);
          for (aquabot_view_B.d_k = 0; aquabot_view_B.d_k <=
               aquabot_view_B.nblocks - 2; aquabot_view_B.d_k++) {
            aquabot_view_B.x[aquabot_view_B.d_k + 1] =
              aquabot_view_B.x[aquabot_view_B.d_k + 1] +
              aquabot_view_B.x[aquabot_view_B.d_k];
          }
        }
      } else {
        aquabot_view_B.x.set_size(1);
        aquabot_view_B.x[0] = aquabot_view_B.CC_RegionLengths[0];
      }

      aquabot_view_B.idxCount.set_size(aquabot_view_B.x.size(0) + 1);
      aquabot_view_B.idxCount[0] = 0;
      aquabot_view_B.hi = aquabot_view_B.x.size(0);
      for (aquabot_view_B.d_k = 0; aquabot_view_B.d_k < aquabot_view_B.hi;
           aquabot_view_B.d_k++) {
        aquabot_view_B.idxCount[aquabot_view_B.d_k + 1] =
          aquabot_view_B.x[aquabot_view_B.d_k];
      }

      aquabot_view_B.pixelsToKeep.set_size(0);
      aquabot_view_B.regionsToKeep.set_size(1, aquabot_view_B.tf.size(1));
      aquabot_view_B.hi = aquabot_view_B.tf.size(1);
      for (aquabot_view_B.d_k = 0; aquabot_view_B.d_k < aquabot_view_B.hi;
           aquabot_view_B.d_k++) {
        aquabot_view_B.regionsToKeep[aquabot_view_B.d_k] = (static_cast<int32_T>
          (aquabot_view_B.tf[aquabot_view_B.d_k]) <= 10000);
      }

      for (aquabot_view_B.k = 0; aquabot_view_B.k <
           aquabot_view_B.firstBlockLength; aquabot_view_B.k++) {
        if (aquabot_view_B.regionsToKeep[aquabot_view_B.k]) {
          aquabot_view_B.nblocks = aquabot_view_B.idxCount[aquabot_view_B.k];
          aquabot_view_B.lastBlockLength =
            aquabot_view_B.idxCount[aquabot_view_B.k + 1];
          if (aquabot_view_B.nblocks + 1 > aquabot_view_B.lastBlockLength) {
            aquabot_view_B.nblocks = 0;
            aquabot_view_B.lastBlockLength = 0;
          }

          aquabot_view_B.b_varargout_2_header_stamp_sec =
            aquabot_view_B.pixelsToKeep.size(0);
          aquabot_view_B.lastBlockLength -= aquabot_view_B.nblocks;
          aquabot_view_B.pixelsToKeep.set_size(aquabot_view_B.lastBlockLength +
            aquabot_view_B.pixelsToKeep.size(0));
          for (aquabot_view_B.d_k = 0; aquabot_view_B.d_k <
               aquabot_view_B.lastBlockLength; aquabot_view_B.d_k++) {
            aquabot_view_B.pixelsToKeep[aquabot_view_B.b_varargout_2_header_stamp_sec
              + aquabot_view_B.d_k] =
              aquabot_view_B.CC_RegionIndices[aquabot_view_B.nblocks +
              aquabot_view_B.d_k];
          }
        }
      }

      memset(&aquabot_view_B.BW[0], 0, 414720U * sizeof(boolean_T));
      aquabot_view_B.x.set_size(aquabot_view_B.pixelsToKeep.size(0));
      aquabot_view_B.hi = aquabot_view_B.pixelsToKeep.size(0);
      for (aquabot_view_B.d_k = 0; aquabot_view_B.d_k < aquabot_view_B.hi;
           aquabot_view_B.d_k++) {
        aquabot_view_B.x[aquabot_view_B.d_k] = static_cast<int32_T>
          (aquabot_view_B.pixelsToKeep[aquabot_view_B.d_k]);
      }

      aquabot_view_B.hi = aquabot_view_B.x.size(0);
      for (aquabot_view_B.d_k = 0; aquabot_view_B.d_k < aquabot_view_B.hi;
           aquabot_view_B.d_k++) {
        aquabot_view_B.BW[aquabot_view_B.x[aquabot_view_B.d_k] - 1] = true;
      }
    }

    /* Outputs for Enabled SubSystem: '<S4>/Subsystem' incorporates:
     *  EnablePort: '<S15>/Enable'
     */
    if (aquabot_view_DW.UnitDelay_DSTATE > 0.0) {
      /* MATLABSystem: '<S16>/SourceBlock' */
      b_varargout_1 = Sub_aquabot_view_26.getLatestMessage
        (&aquabot_view_B.b_varargout_2_o);

      /* Outputs for Enabled SubSystem: '<S16>/Enabled Subsystem' incorporates:
       *  EnablePort: '<S17>/Enable'
       */
      if (b_varargout_1) {
        /* SignalConversion generated from: '<S17>/In1' */
        aquabot_view_B.In1_pk = aquabot_view_B.b_varargout_2_o;
      }

      /* End of Outputs for SubSystem: '<S16>/Enabled Subsystem' */
    }

    /* End of Outputs for SubSystem: '<S4>/Subsystem' */

    /* MATLAB Function: '<S4>/MATLAB Function1' incorporates:
     *  MATLABSystem: '<S10>/Read Point Cloud'
     *  MATLABSystem: '<S16>/SourceBlock'
     *  UnitDelay: '<S4>/Unit Delay'
     */
    if (aquabot_view_B.In1_pk.d_SL_Info.ReceivedLength <= 0U) {
      aquabot_view_B.b_varargout_2_header_stamp_sec = 1;
      aquabot_view_B.pose_boat[0] = 0.0;
      aquabot_view_B.pose_boat[1] = 0.0;
      aquabot_view_B.pose_boat[2] = 0.0;
    } else {
      aquabot_view_B.b_varargout_2_header_stamp_sec = 0;
      for (aquabot_view_B.d_k = 0; aquabot_view_B.d_k < 3; aquabot_view_B.d_k++)
      {
        aquabot_view_B.assign_temp_R[3 * aquabot_view_B.d_k] =
          aquabot_view_B.In1_pk.k[aquabot_view_B.d_k];
        aquabot_view_B.assign_temp_R[3 * aquabot_view_B.d_k + 1] =
          aquabot_view_B.In1_pk.k[aquabot_view_B.d_k + 3];
        aquabot_view_B.assign_temp_R[3 * aquabot_view_B.d_k + 2] =
          aquabot_view_B.In1_pk.k[aquabot_view_B.d_k + 6];
      }

      aquabot_view_B.d_k = static_cast<int32_T>
        (aquabot_view_B.In1_pk.d_SL_Info.ReceivedLength);
      if (aquabot_view_B.d_k - 1 >= 0) {
        memcpy(&aquabot_view_B.tmp_data_n[0], &aquabot_view_B.In1_pk.d[0],
               static_cast<uint32_T>(aquabot_view_B.d_k) * sizeof(real_T));
      }

      aquabot_view_B.fv[0] = static_cast<real32_T>(aquabot_view_B.In1_pk.height);
      aquabot_view_B.fv[1] = static_cast<real32_T>(aquabot_view_B.In1_pk.width);
      aqua_cameraIntrinsicsFromOpenCV(aquabot_view_B.assign_temp_R,
        aquabot_view_B.tmp_data_n, aquabot_view_B.fv, &aquabot_view_B.lobj_0_a,
        aquabot_view_B.expl_temp_m3, aquabot_view_B.expl_temp_j,
        aquabot_view_B.intrinsics_ImageSize, aquabot_view_B.expl_temp_m,
        aquabot_view_B.expl_temp_h, &aquabot_view_B.bsum,
        aquabot_view_B.intrinsics_K, &tmp_0, &aquabot_view_B.expl_temp_data,
        aquabot_view_B.expl_temp_size);
      aquab_rigidtform3d_rigidtform3d(aquabot_view_B.expl_temp_m,
        aquabot_view_B.assign_temp_R, &aquabot_view_B.expl_temp_data_i,
        aquabot_view_B.expl_temp_size);
      aquab_projectLidarPointsOnImage(aquabot_view_B.ReadPointCloud,
        aquabot_view_B.intrinsics_ImageSize, aquabot_view_B.intrinsics_K,
        aquabot_view_B.expl_temp_m, aquabot_view_B.assign_temp_R,
        aquabot_view_B.r_pt, aquabot_view_B.idx);
      aquabot_view_B.firstBlockLength = aquabot_view_B.r_pt.size(0) << 1;
      for (aquabot_view_B.k = 0; aquabot_view_B.k <
           aquabot_view_B.firstBlockLength; aquabot_view_B.k++) {
        aquabot_view_B.r_pt[aquabot_view_B.k] = rt_roundf_snf
          (aquabot_view_B.r_pt[aquabot_view_B.k]);
      }

      aquabot_view_B.r_pt.set_size(aquabot_view_B.r_pt.size(0), 2);
      for (aquabot_view_B.d_k = 0; aquabot_view_B.d_k <
           aquabot_view_B.firstBlockLength; aquabot_view_B.d_k++) {
        aquabot_view_B.bsum_l = aquabot_view_B.r_pt[aquabot_view_B.d_k];
        if (aquabot_view_B.bsum_l >= 1.0F) {
          aquabot_view_B.r_pt[aquabot_view_B.d_k] = aquabot_view_B.bsum_l;
        } else {
          aquabot_view_B.r_pt[aquabot_view_B.d_k] = 1.0F;
        }
      }

      aquabot_view_B.r_pt_b.set_size(aquabot_view_B.r_pt.size(0));
      aquabot_view_B.hi = aquabot_view_B.r_pt.size(0);
      aquabot_view_B.r_pt_p.set_size(aquabot_view_B.r_pt.size(0));
      for (aquabot_view_B.d_k = 0; aquabot_view_B.d_k < aquabot_view_B.hi;
           aquabot_view_B.d_k++) {
        aquabot_view_B.r_pt_b[aquabot_view_B.d_k] =
          aquabot_view_B.r_pt[aquabot_view_B.d_k + aquabot_view_B.r_pt.size(0)];
        aquabot_view_B.r_pt_p[aquabot_view_B.d_k] =
          aquabot_view_B.r_pt[aquabot_view_B.d_k];
      }

      aquabot_view_sub2ind(aquabot_view_B.r_pt_b, aquabot_view_B.r_pt_p,
                           aquabot_view_B.r1);
      aquabot_view_B.i = aquabot_view_B.r1.size(0);
      aquabot_view_B.hi = aquabot_view_B.r1.size(0);
      for (aquabot_view_B.d_k = 0; aquabot_view_B.d_k < aquabot_view_B.hi;
           aquabot_view_B.d_k++) {
        aquabot_view_B.tmp_data_a[aquabot_view_B.d_k] =
          aquabot_view_B.BW[aquabot_view_B.r1[aquabot_view_B.d_k] - 1];
      }

      aquabot_view_B.k = aquabot_view_B.i - 1;
      aquabot_view_B.hi = 0;
      for (aquabot_view_B.i = 0; aquabot_view_B.i <= aquabot_view_B.k;
           aquabot_view_B.i++) {
        if (aquabot_view_B.tmp_data_a[aquabot_view_B.i]) {
          aquabot_view_B.hi++;
        }
      }

      aquabot_view_B.firstBlockLength = 0;
      for (aquabot_view_B.i = 0; aquabot_view_B.i <= aquabot_view_B.k;
           aquabot_view_B.i++) {
        if (aquabot_view_B.tmp_data_a[aquabot_view_B.i]) {
          aquabot_view_B.tmp_data[aquabot_view_B.firstBlockLength] =
            static_cast<int16_T>(aquabot_view_B.i);
          aquabot_view_B.firstBlockLength++;
        }
      }

      aquabot_view_B.x_c.set_size(aquabot_view_B.hi, 3);
      for (aquabot_view_B.d_k = 0; aquabot_view_B.d_k < 3; aquabot_view_B.d_k++)
      {
        for (aquabot_view_B.firstBlockLength = 0;
             aquabot_view_B.firstBlockLength < aquabot_view_B.hi;
             aquabot_view_B.firstBlockLength++) {
          aquabot_view_B.x_c[aquabot_view_B.firstBlockLength +
            aquabot_view_B.x_c.size(0) * aquabot_view_B.d_k] =
            aquabot_view_B.ReadPointCloud[(30000 * aquabot_view_B.d_k +
            static_cast<int32_T>(aquabot_view_B.idx[static_cast<int32_T>
            (aquabot_view_B.tmp_data[aquabot_view_B.firstBlockLength])])) - 1];
        }
      }

      aquabot_view_B.r_pt_b.set_size(aquabot_view_B.r_pt.size(0));
      aquabot_view_B.hi = aquabot_view_B.r_pt.size(0);
      aquabot_view_B.r_pt_p.set_size(aquabot_view_B.r_pt.size(0));
      for (aquabot_view_B.d_k = 0; aquabot_view_B.d_k < aquabot_view_B.hi;
           aquabot_view_B.d_k++) {
        aquabot_view_B.r_pt_b[aquabot_view_B.d_k] =
          aquabot_view_B.r_pt[aquabot_view_B.d_k + aquabot_view_B.r_pt.size(0)];
        aquabot_view_B.r_pt_p[aquabot_view_B.d_k] =
          aquabot_view_B.r_pt[aquabot_view_B.d_k];
      }

      aquabot_view_sub2ind(aquabot_view_B.r_pt_b, aquabot_view_B.r_pt_p,
                           aquabot_view_B.r1);
      aquabot_view_B.i = aquabot_view_B.r1.size(0);
      aquabot_view_B.hi = aquabot_view_B.r1.size(0);
      for (aquabot_view_B.d_k = 0; aquabot_view_B.d_k < aquabot_view_B.hi;
           aquabot_view_B.d_k++) {
        aquabot_view_B.tmp_data_a[aquabot_view_B.d_k] =
          aquabot_view_B.BW[aquabot_view_B.r1[aquabot_view_B.d_k] - 1];
      }

      aquabot_view_B.k = aquabot_view_B.i - 1;
      aquabot_view_B.hi = 0;
      for (aquabot_view_B.i = 0; aquabot_view_B.i <= aquabot_view_B.k;
           aquabot_view_B.i++) {
        if (aquabot_view_B.tmp_data_a[aquabot_view_B.i]) {
          aquabot_view_B.hi++;
        }
      }

      guard1 = false;
      if (aquabot_view_B.hi == 0) {
        guard1 = true;
      } else {
        aquabot_view_B.r_pt_b.set_size(aquabot_view_B.r_pt.size(0));
        aquabot_view_B.hi = aquabot_view_B.r_pt.size(0);
        aquabot_view_B.r_pt_p.set_size(aquabot_view_B.r_pt.size(0));
        for (aquabot_view_B.d_k = 0; aquabot_view_B.d_k < aquabot_view_B.hi;
             aquabot_view_B.d_k++) {
          aquabot_view_B.r_pt_b[aquabot_view_B.d_k] =
            aquabot_view_B.r_pt[aquabot_view_B.d_k + aquabot_view_B.r_pt.size(0)];
          aquabot_view_B.r_pt_p[aquabot_view_B.d_k] =
            aquabot_view_B.r_pt[aquabot_view_B.d_k];
        }

        aquabot_view_sub2ind(aquabot_view_B.r_pt_b, aquabot_view_B.r_pt_p,
                             aquabot_view_B.r1);
        aquabot_view_B.i = aquabot_view_B.r1.size(0);
        aquabot_view_B.hi = aquabot_view_B.r1.size(0);
        for (aquabot_view_B.d_k = 0; aquabot_view_B.d_k < aquabot_view_B.hi;
             aquabot_view_B.d_k++) {
          aquabot_view_B.tmp_data_a[aquabot_view_B.d_k] =
            aquabot_view_B.BW[aquabot_view_B.r1[aquabot_view_B.d_k] - 1];
        }

        aquabot_view_B.k = aquabot_view_B.i - 1;
        aquabot_view_B.hi = 0;
        for (aquabot_view_B.i = 0; aquabot_view_B.i <= aquabot_view_B.k;
             aquabot_view_B.i++) {
          if (aquabot_view_B.tmp_data_a[aquabot_view_B.i]) {
            aquabot_view_B.hi++;
          }
        }

        if (aquabot_view_B.hi == 0) {
          guard1 = true;
        } else {
          aquabot_view_B.r_pt_b.set_size(aquabot_view_B.r_pt.size(0));
          aquabot_view_B.hi = aquabot_view_B.r_pt.size(0);
          aquabot_view_B.r_pt_p.set_size(aquabot_view_B.r_pt.size(0));
          for (aquabot_view_B.d_k = 0; aquabot_view_B.d_k < aquabot_view_B.hi;
               aquabot_view_B.d_k++) {
            aquabot_view_B.r_pt_b[aquabot_view_B.d_k] =
              aquabot_view_B.r_pt[aquabot_view_B.d_k + aquabot_view_B.r_pt.size
              (0)];
            aquabot_view_B.r_pt_p[aquabot_view_B.d_k] =
              aquabot_view_B.r_pt[aquabot_view_B.d_k];
          }

          aquabot_view_sub2ind(aquabot_view_B.r_pt_b, aquabot_view_B.r_pt_p,
                               aquabot_view_B.r1);
          aquabot_view_B.i = aquabot_view_B.r1.size(0);
          aquabot_view_B.hi = aquabot_view_B.r1.size(0);
          for (aquabot_view_B.d_k = 0; aquabot_view_B.d_k < aquabot_view_B.hi;
               aquabot_view_B.d_k++) {
            aquabot_view_B.tmp_data_a[aquabot_view_B.d_k] =
              aquabot_view_B.BW[aquabot_view_B.r1[aquabot_view_B.d_k] - 1];
          }

          aquabot_view_B.k = aquabot_view_B.i - 1;
          aquabot_view_B.hi = 0;
          for (aquabot_view_B.i = 0; aquabot_view_B.i <= aquabot_view_B.k;
               aquabot_view_B.i++) {
            if (aquabot_view_B.tmp_data_a[aquabot_view_B.i]) {
              aquabot_view_B.hi++;
            }
          }

          if (aquabot_view_B.hi <= 1024) {
            aquabot_view_B.r_pt_b.set_size(aquabot_view_B.r_pt.size(0));
            aquabot_view_B.hi = aquabot_view_B.r_pt.size(0);
            aquabot_view_B.r_pt_p.set_size(aquabot_view_B.r_pt.size(0));
            for (aquabot_view_B.d_k = 0; aquabot_view_B.d_k < aquabot_view_B.hi;
                 aquabot_view_B.d_k++) {
              aquabot_view_B.r_pt_b[aquabot_view_B.d_k] =
                aquabot_view_B.r_pt[aquabot_view_B.d_k +
                aquabot_view_B.r_pt.size(0)];
              aquabot_view_B.r_pt_p[aquabot_view_B.d_k] =
                aquabot_view_B.r_pt[aquabot_view_B.d_k];
            }

            aquabot_view_sub2ind(aquabot_view_B.r_pt_b, aquabot_view_B.r_pt_p,
                                 aquabot_view_B.r1);
            aquabot_view_B.i = aquabot_view_B.r1.size(0);
            aquabot_view_B.hi = aquabot_view_B.r1.size(0);
            for (aquabot_view_B.d_k = 0; aquabot_view_B.d_k < aquabot_view_B.hi;
                 aquabot_view_B.d_k++) {
              aquabot_view_B.tmp_data_a[aquabot_view_B.d_k] =
                aquabot_view_B.BW[aquabot_view_B.r1[aquabot_view_B.d_k] - 1];
            }

            aquabot_view_B.k = aquabot_view_B.i - 1;
            aquabot_view_B.firstBlockLength = 0;
            for (aquabot_view_B.i = 0; aquabot_view_B.i <= aquabot_view_B.k;
                 aquabot_view_B.i++) {
              if (aquabot_view_B.tmp_data_a[aquabot_view_B.i]) {
                aquabot_view_B.firstBlockLength++;
              }
            }

            aquabot_view_B.lastBlockLength = 0;
            aquabot_view_B.nblocks = 1;
          } else {
            aquabot_view_B.firstBlockLength = 1024;
            aquabot_view_B.r_pt_b.set_size(aquabot_view_B.r_pt.size(0));
            aquabot_view_B.hi = aquabot_view_B.r_pt.size(0);
            aquabot_view_B.r_pt_p.set_size(aquabot_view_B.r_pt.size(0));
            for (aquabot_view_B.d_k = 0; aquabot_view_B.d_k < aquabot_view_B.hi;
                 aquabot_view_B.d_k++) {
              aquabot_view_B.r_pt_b[aquabot_view_B.d_k] =
                aquabot_view_B.r_pt[aquabot_view_B.d_k +
                aquabot_view_B.r_pt.size(0)];
              aquabot_view_B.r_pt_p[aquabot_view_B.d_k] =
                aquabot_view_B.r_pt[aquabot_view_B.d_k];
            }

            aquabot_view_sub2ind(aquabot_view_B.r_pt_b, aquabot_view_B.r_pt_p,
                                 aquabot_view_B.r1);
            aquabot_view_B.i = aquabot_view_B.r1.size(0);
            aquabot_view_B.hi = aquabot_view_B.r1.size(0);
            for (aquabot_view_B.d_k = 0; aquabot_view_B.d_k < aquabot_view_B.hi;
                 aquabot_view_B.d_k++) {
              aquabot_view_B.tmp_data_a[aquabot_view_B.d_k] =
                aquabot_view_B.BW[aquabot_view_B.r1[aquabot_view_B.d_k] - 1];
            }

            aquabot_view_B.k = aquabot_view_B.i - 1;
            aquabot_view_B.hi = 0;
            for (aquabot_view_B.i = 0; aquabot_view_B.i <= aquabot_view_B.k;
                 aquabot_view_B.i++) {
              if (aquabot_view_B.tmp_data_a[aquabot_view_B.i]) {
                aquabot_view_B.hi++;
              }
            }

            aquabot_view_B.nblocks = static_cast<int32_T>(static_cast<uint32_T>
              (aquabot_view_B.hi) >> 10);
            aquabot_view_B.r_pt_b.set_size(aquabot_view_B.r_pt.size(0));
            aquabot_view_B.hi = aquabot_view_B.r_pt.size(0);
            aquabot_view_B.r_pt_p.set_size(aquabot_view_B.r_pt.size(0));
            for (aquabot_view_B.d_k = 0; aquabot_view_B.d_k < aquabot_view_B.hi;
                 aquabot_view_B.d_k++) {
              aquabot_view_B.r_pt_b[aquabot_view_B.d_k] =
                aquabot_view_B.r_pt[aquabot_view_B.d_k +
                aquabot_view_B.r_pt.size(0)];
              aquabot_view_B.r_pt_p[aquabot_view_B.d_k] =
                aquabot_view_B.r_pt[aquabot_view_B.d_k];
            }

            aquabot_view_sub2ind(aquabot_view_B.r_pt_b, aquabot_view_B.r_pt_p,
                                 aquabot_view_B.r1);
            aquabot_view_B.i = aquabot_view_B.r1.size(0);
            aquabot_view_B.hi = aquabot_view_B.r1.size(0);
            for (aquabot_view_B.d_k = 0; aquabot_view_B.d_k < aquabot_view_B.hi;
                 aquabot_view_B.d_k++) {
              aquabot_view_B.tmp_data_a[aquabot_view_B.d_k] =
                aquabot_view_B.BW[aquabot_view_B.r1[aquabot_view_B.d_k] - 1];
            }

            aquabot_view_B.k = aquabot_view_B.i - 1;
            aquabot_view_B.hi = 0;
            for (aquabot_view_B.i = 0; aquabot_view_B.i <= aquabot_view_B.k;
                 aquabot_view_B.i++) {
              if (aquabot_view_B.tmp_data_a[aquabot_view_B.i]) {
                aquabot_view_B.hi++;
              }
            }

            aquabot_view_B.lastBlockLength = aquabot_view_B.hi -
              (aquabot_view_B.nblocks << 10);
            if (aquabot_view_B.lastBlockLength > 0) {
              aquabot_view_B.nblocks++;
            } else {
              aquabot_view_B.lastBlockLength = 1024;
            }
          }

          aquabot_view_B.xpageoffset = aquabot_view_B.r_pt.size(0);
          for (aquabot_view_B.k = 0; aquabot_view_B.k < 3; aquabot_view_B.k++) {
            aquabot_view_B.r_pt_b.set_size(aquabot_view_B.r_pt.size(0));
            aquabot_view_B.r_pt_p.set_size(aquabot_view_B.r_pt.size(0));
            for (aquabot_view_B.d_k = 0; aquabot_view_B.d_k <
                 aquabot_view_B.xpageoffset; aquabot_view_B.d_k++) {
              aquabot_view_B.r_pt_b[aquabot_view_B.d_k] =
                aquabot_view_B.r_pt[aquabot_view_B.d_k +
                aquabot_view_B.r_pt.size(0)];
              aquabot_view_B.r_pt_p[aquabot_view_B.d_k] =
                aquabot_view_B.r_pt[aquabot_view_B.d_k];
            }

            aquabot_view_sub2ind(aquabot_view_B.r_pt_b, aquabot_view_B.r_pt_p,
                                 aquabot_view_B.r1);
            aquabot_view_B.i = aquabot_view_B.r1.size(0);
            aquabot_view_B.hi = aquabot_view_B.r1.size(0);
            for (aquabot_view_B.d_k = 0; aquabot_view_B.d_k < aquabot_view_B.hi;
                 aquabot_view_B.d_k++) {
              aquabot_view_B.tmp_data_a[aquabot_view_B.d_k] =
                aquabot_view_B.BW[aquabot_view_B.r1[aquabot_view_B.d_k] - 1];
            }

            aquabot_view_B.end_tmp = aquabot_view_B.i - 1;
            aquabot_view_B.hi = 0;
            for (aquabot_view_B.i = 0; aquabot_view_B.i <=
                 aquabot_view_B.end_tmp; aquabot_view_B.i++) {
              if (aquabot_view_B.tmp_data_a[aquabot_view_B.i]) {
                aquabot_view_B.hi++;
              }
            }

            aquabot_view_B.b_x[aquabot_view_B.k] =
              aquabot_view_B.x_c[aquabot_view_B.k * aquabot_view_B.hi];
            for (aquabot_view_B.d_k = 2; aquabot_view_B.d_k <=
                 aquabot_view_B.firstBlockLength; aquabot_view_B.d_k++) {
              aquabot_view_B.hi = 0;
              for (aquabot_view_B.i = 0; aquabot_view_B.i <=
                   aquabot_view_B.end_tmp; aquabot_view_B.i++) {
                if (aquabot_view_B.tmp_data_a[aquabot_view_B.i]) {
                  aquabot_view_B.hi++;
                }
              }

              aquabot_view_B.b_x[aquabot_view_B.k] += aquabot_view_B.x_c
                [(aquabot_view_B.k * aquabot_view_B.hi + aquabot_view_B.d_k) - 1];
            }

            for (aquabot_view_B.ib = 2; aquabot_view_B.ib <=
                 aquabot_view_B.nblocks; aquabot_view_B.ib++) {
              aquabot_view_B.hi = 0;
              for (aquabot_view_B.i = 0; aquabot_view_B.i <=
                   aquabot_view_B.end_tmp; aquabot_view_B.i++) {
                if (aquabot_view_B.tmp_data_a[aquabot_view_B.i]) {
                  aquabot_view_B.hi++;
                }
              }

              aquabot_view_B.xblockoffset = ((aquabot_view_B.ib - 1) << 10) +
                aquabot_view_B.k * aquabot_view_B.hi;
              aquabot_view_B.bsum_l =
                aquabot_view_B.x_c[aquabot_view_B.xblockoffset];
              if (aquabot_view_B.ib == aquabot_view_B.nblocks) {
                aquabot_view_B.hi = aquabot_view_B.lastBlockLength;
              } else {
                aquabot_view_B.hi = 1024;
              }

              for (aquabot_view_B.i = 2; aquabot_view_B.i <= aquabot_view_B.hi;
                   aquabot_view_B.i++) {
                aquabot_view_B.bsum_l += aquabot_view_B.x_c
                  [(aquabot_view_B.xblockoffset + aquabot_view_B.i) - 1];
              }

              aquabot_view_B.b_x[aquabot_view_B.k] += aquabot_view_B.bsum_l;
            }
          }
        }
      }

      if (guard1) {
        aquabot_view_B.b_x[0] = 0.0F;
        aquabot_view_B.b_x[1] = 0.0F;
        aquabot_view_B.b_x[2] = 0.0F;
      }

      aquabot_view_B.r_pt_b.set_size(aquabot_view_B.r_pt.size(0));
      aquabot_view_B.hi = aquabot_view_B.r_pt.size(0);
      aquabot_view_B.r_pt_p.set_size(aquabot_view_B.r_pt.size(0));
      for (aquabot_view_B.d_k = 0; aquabot_view_B.d_k < aquabot_view_B.hi;
           aquabot_view_B.d_k++) {
        aquabot_view_B.r_pt_b[aquabot_view_B.d_k] =
          aquabot_view_B.r_pt[aquabot_view_B.d_k + aquabot_view_B.r_pt.size(0)];
        aquabot_view_B.r_pt_p[aquabot_view_B.d_k] =
          aquabot_view_B.r_pt[aquabot_view_B.d_k];
      }

      aquabot_view_sub2ind(aquabot_view_B.r_pt_b, aquabot_view_B.r_pt_p,
                           aquabot_view_B.r1);
      aquabot_view_B.i = aquabot_view_B.r1.size(0);
      aquabot_view_B.hi = aquabot_view_B.r1.size(0);
      for (aquabot_view_B.d_k = 0; aquabot_view_B.d_k < aquabot_view_B.hi;
           aquabot_view_B.d_k++) {
        aquabot_view_B.tmp_data_a[aquabot_view_B.d_k] =
          aquabot_view_B.BW[aquabot_view_B.r1[aquabot_view_B.d_k] - 1];
      }

      aquabot_view_B.k = aquabot_view_B.i - 1;
      aquabot_view_B.hi = 0;
      for (aquabot_view_B.i = 0; aquabot_view_B.i <= aquabot_view_B.k;
           aquabot_view_B.i++) {
        if (aquabot_view_B.tmp_data_a[aquabot_view_B.i]) {
          aquabot_view_B.hi++;
        }
      }

      aquabot_view_B.pose_boat[0] = aquabot_view_B.b_x[0] / static_cast<real32_T>
        (aquabot_view_B.hi);
      aquabot_view_B.pose_boat[1] = aquabot_view_B.b_x[1] / static_cast<real32_T>
        (aquabot_view_B.hi);
      aquabot_view_B.pose_boat[2] = aquabot_view_B.b_x[2] / static_cast<real32_T>
        (aquabot_view_B.hi);
      for (aquabot_view_B.i = 0; aquabot_view_B.i < 3; aquabot_view_B.i++) {
        if (rtIsNaN(aquabot_view_B.pose_boat[aquabot_view_B.i])) {
          aquabot_view_B.pose_boat[aquabot_view_B.i] = 0.0;
        }
      }
    }

    /* MATLAB Function: '<S4>/MATLAB Function' */
    aquabot_view_B.nz = aquabot_view_B.BW[0];
    for (aquabot_view_B.k = 0; aquabot_view_B.k < 1023; aquabot_view_B.k++) {
      aquabot_view_B.nz += static_cast<real_T>
        (aquabot_view_B.BW[aquabot_view_B.k + 1]);
    }

    for (aquabot_view_B.ib = 0; aquabot_view_B.ib < 404; aquabot_view_B.ib++) {
      aquabot_view_B.xblockoffset = (aquabot_view_B.ib + 1) << 10;
      aquabot_view_B.bsum = aquabot_view_B.BW[aquabot_view_B.xblockoffset];
      for (aquabot_view_B.d_k = 0; aquabot_view_B.d_k < 1023; aquabot_view_B.d_k
           ++) {
        aquabot_view_B.bsum += static_cast<real_T>(aquabot_view_B.BW
          [(aquabot_view_B.xblockoffset + aquabot_view_B.d_k) + 1]);
      }

      aquabot_view_B.nz += aquabot_view_B.bsum;
    }

    aquabot_view_bwconncomp(aquabot_view_B.BW, &aquabot_view_B.bsum,
      aquabot_view_B.expl_temp_m3, &aquabot_view_B.CC_NumObjects,
      aquabot_view_B.CC_RegionIndices, aquabot_view_B.CC_RegionLengths,
      aquabot_view_B.expl_temp);
    aquabot_view_B.firstBlockLength = static_cast<int32_T>
      (aquabot_view_B.CC_NumObjects);
    aquabot_view_B.s.Centroid[0] = 0.0;
    aquabot_view_B.s.Centroid[1] = 0.0;
    aquabot_view_B.outstats_k.set_size(aquabot_view_B.firstBlockLength);

    /* MATLAB Function: '<S4>/MATLAB Function' */
    for (aquabot_view_B.d_k = 0; aquabot_view_B.d_k <
         aquabot_view_B.firstBlockLength; aquabot_view_B.d_k++) {
      aquabot_view_B.outstats_k[aquabot_view_B.d_k] = aquabot_view_B.s;
    }

    aquabot_view_B.a.Area = 0.0;
    aquabot_view_B.a.Centroid[0] = 0.0;
    aquabot_view_B.a.Centroid[1] = 0.0;
    aquabot_view_B.a.BoundingBox[0] = 0.0;
    aquabot_view_B.a.BoundingBox[1] = 0.0;
    aquabot_view_B.a.BoundingBox[2] = 0.0;
    aquabot_view_B.a.BoundingBox[3] = 0.0;
    aquabot_view_B.a.MajorAxisLength = 0.0;
    aquabot_view_B.a.MinorAxisLength = 0.0;
    aquabot_view_B.a.Eccentricity = 0.0;
    aquabot_view_B.a.Orientation = 0.0;
    aquabot_view_B.a.Image.size[0] = 0;
    aquabot_view_B.a.Image.size[1] = 0;
    aquabot_view_B.a.FilledImage.size[0] = 0;
    aquabot_view_B.a.FilledImage.size[1] = 0;
    aquabot_view_B.a.FilledArea = 0.0;
    aquabot_view_B.a.EulerNumber = 0.0;
    for (aquabot_view_B.d_k = 0; aquabot_view_B.d_k < 16; aquabot_view_B.d_k++)
    {
      aquabot_view_B.a.Extrema[aquabot_view_B.d_k] = 0.0;
    }

    aquabot_view_B.a.EquivDiameter = 0.0;
    aquabot_view_B.a.Extent = 0.0;
    aquabot_view_B.a.PixelIdxList.set_size(0);
    aquabot_view_B.a.PixelList.set_size(0, 2);

    /* MATLAB Function: '<S4>/MATLAB Function' */
    aquabot_view_B.a.Perimeter = 0.0;
    aquabot_view_B.a.Circularity = 0.0;
    aquabot_view_B.a.PixelValues.size[0] = 0;
    aquabot_view_B.a.WeightedCentroid[0] = 0.0;
    aquabot_view_B.a.WeightedCentroid[1] = 0.0;
    aquabot_view_B.a.MeanIntensity = 0.0;
    aquabot_view_B.a.MinIntensity = 0.0;
    aquabot_view_B.a.MaxIntensity = 0.0;
    aquabot_view_B.a.SubarrayIdx.size[0] = 1;
    aquabot_view_B.a.SubarrayIdx.size[1] = 0;
    aquabot_view_B.a.SubarrayIdxLengths[0] = 0.0;
    aquabot_view_B.a.SubarrayIdxLengths[1] = 0.0;
    aquabot_view_B.stats_c.set_size(aquabot_view_B.firstBlockLength);

    /* MATLAB Function: '<S4>/MATLAB Function' */
    for (aquabot_view_B.d_k = 0; aquabot_view_B.d_k <
         aquabot_view_B.firstBlockLength; aquabot_view_B.d_k++) {
      aquabot_view_B.stats_c[aquabot_view_B.d_k] = aquabot_view_B.a;
    }

    if (aquabot_view_B.CC_NumObjects != 0.0) {
      if ((aquabot_view_B.CC_RegionLengths.size(0) != 1) &&
          (aquabot_view_B.CC_RegionLengths.size(0) != 0) &&
          (aquabot_view_B.CC_RegionLengths.size(0) != 1)) {
        aquabot_view_B.k = aquabot_view_B.CC_RegionLengths.size(0);
        for (aquabot_view_B.nblocks = 0; aquabot_view_B.nblocks <=
             aquabot_view_B.k - 2; aquabot_view_B.nblocks++) {
          aquabot_view_B.CC_RegionLengths[aquabot_view_B.nblocks + 1] =
            aquabot_view_B.CC_RegionLengths[aquabot_view_B.nblocks + 1] +
            aquabot_view_B.CC_RegionLengths[aquabot_view_B.nblocks];
        }
      }

      aquabot_view_B.idxCount.set_size(aquabot_view_B.CC_RegionLengths.size(0) +
        1);
      aquabot_view_B.idxCount[0] = 0;
      aquabot_view_B.hi = aquabot_view_B.CC_RegionLengths.size(0);
      for (aquabot_view_B.d_k = 0; aquabot_view_B.d_k < aquabot_view_B.hi;
           aquabot_view_B.d_k++) {
        aquabot_view_B.idxCount[aquabot_view_B.d_k + 1] =
          aquabot_view_B.CC_RegionLengths[aquabot_view_B.d_k];
      }

      for (aquabot_view_B.i = 0; aquabot_view_B.i <
           aquabot_view_B.firstBlockLength; aquabot_view_B.i++) {
        aquabot_view_B.nblocks = aquabot_view_B.idxCount[aquabot_view_B.i];
        aquabot_view_B.lastBlockLength =
          aquabot_view_B.idxCount[aquabot_view_B.i + 1];
        if (aquabot_view_B.nblocks + 1 > aquabot_view_B.lastBlockLength) {
          aquabot_view_B.nblocks = 0;
          aquabot_view_B.lastBlockLength = 0;
        }

        aquabot_view_B.hi = aquabot_view_B.lastBlockLength -
          aquabot_view_B.nblocks;
        aquabot_view_B.stats_c[aquabot_view_B.i].PixelIdxList.set_size
          (aquabot_view_B.hi);
        for (aquabot_view_B.d_k = 0; aquabot_view_B.d_k < aquabot_view_B.hi;
             aquabot_view_B.d_k++) {
          aquabot_view_B.stats_c[aquabot_view_B.i]
            .PixelIdxList[aquabot_view_B.d_k] =
            aquabot_view_B.CC_RegionIndices[aquabot_view_B.nblocks +
            aquabot_view_B.d_k];
        }
      }
    }

    aquabot_view_B.firstBlockLength = aquabot_view_B.stats_c.size(0);
    for (aquabot_view_B.nblocks = 0; aquabot_view_B.nblocks <
         aquabot_view_B.firstBlockLength; aquabot_view_B.nblocks++) {
      aquabot_view_B.k = aquabot_view_B.stats_c[aquabot_view_B.nblocks].
        PixelIdxList.size(0);
      if (aquabot_view_B.k != 0) {
        aquabot_view_B.CC_RegionLengths.set_size(aquabot_view_B.k);
        for (aquabot_view_B.d_k = 0; aquabot_view_B.d_k < aquabot_view_B.k;
             aquabot_view_B.d_k++) {
          aquabot_view_B.CC_RegionLengths[aquabot_view_B.d_k] = static_cast<
            int32_T>(aquabot_view_B.stats_c[aquabot_view_B.nblocks]
                     .PixelIdxList[aquabot_view_B.d_k]) - 1;
        }

        aquabot_view_B.idxCount.set_size(aquabot_view_B.CC_RegionLengths.size(0));
        aquabot_view_B.hi = aquabot_view_B.CC_RegionLengths.size(0);
        for (aquabot_view_B.d_k = 0; aquabot_view_B.d_k < aquabot_view_B.hi;
             aquabot_view_B.d_k++) {
          aquabot_view_B.idxCount[aquabot_view_B.d_k] =
            aquabot_view_B.CC_RegionLengths[aquabot_view_B.d_k] / 576;
        }

        aquabot_view_B.stats_c[aquabot_view_B.nblocks].PixelList.set_size
          (aquabot_view_B.idxCount.size(0), 2);
        aquabot_view_B.hi = aquabot_view_B.idxCount.size(0);
        for (aquabot_view_B.d_k = 0; aquabot_view_B.d_k < aquabot_view_B.hi;
             aquabot_view_B.d_k++) {
          aquabot_view_B.stats_c[aquabot_view_B.nblocks]
            .PixelList[aquabot_view_B.d_k] = static_cast<real_T>
            (aquabot_view_B.idxCount[aquabot_view_B.d_k]) + 1.0;
        }

        aquabot_view_B.hi = aquabot_view_B.CC_RegionLengths.size(0);
        for (aquabot_view_B.d_k = 0; aquabot_view_B.d_k < aquabot_view_B.hi;
             aquabot_view_B.d_k++) {
          aquabot_view_B.stats_c[aquabot_view_B.nblocks]
            .PixelList[aquabot_view_B.d_k + aquabot_view_B.idxCount.size(0)] =
            (aquabot_view_B.CC_RegionLengths[aquabot_view_B.d_k] -
             aquabot_view_B.idxCount[aquabot_view_B.d_k] * 576) + 1;
        }
      } else {
        aquabot_view_B.stats_c[aquabot_view_B.nblocks].PixelList.set_size(0, 2);
      }
    }

    aquabot_view_B.i = aquabot_view_B.stats_c.size(0);
    for (aquabot_view_B.d_k = 0; aquabot_view_B.d_k < aquabot_view_B.i;
         aquabot_view_B.d_k++) {
      aquabot_view_B.k = aquabot_view_B.stats_c[aquabot_view_B.d_k].
        PixelList.size(0);
      if (aquabot_view_B.k == 0) {
        aquabot_view_B.expl_temp_m3[0] = 0.0;
        aquabot_view_B.expl_temp_m3[1] = 0.0;
      } else {
        if (aquabot_view_B.k <= 1024) {
          aquabot_view_B.firstBlockLength = aquabot_view_B.k;
          aquabot_view_B.lastBlockLength = 0;
          aquabot_view_B.nblocks = 1;
        } else {
          aquabot_view_B.firstBlockLength = 1024;
          aquabot_view_B.nblocks = static_cast<int32_T>(static_cast<uint32_T>
            (aquabot_view_B.k) >> 10);
          aquabot_view_B.lastBlockLength = aquabot_view_B.k -
            (aquabot_view_B.nblocks << 10);
          if (aquabot_view_B.lastBlockLength > 0) {
            aquabot_view_B.nblocks++;
          } else {
            aquabot_view_B.lastBlockLength = 1024;
          }
        }

        for (aquabot_view_B.k = 0; aquabot_view_B.k < 2; aquabot_view_B.k++) {
          aquabot_view_B.xpageoffset = aquabot_view_B.k *
            aquabot_view_B.stats_c[aquabot_view_B.d_k].PixelList.size(0);
          aquabot_view_B.expl_temp_m3[aquabot_view_B.k] = static_cast<int32_T>
            (aquabot_view_B.stats_c[aquabot_view_B.d_k]
             .PixelList[aquabot_view_B.xpageoffset]);
          for (aquabot_view_B.hi = 2; aquabot_view_B.hi <=
               aquabot_view_B.firstBlockLength; aquabot_view_B.hi++) {
            aquabot_view_B.expl_temp_m3[aquabot_view_B.k] += static_cast<real_T>
              (static_cast<int32_T>(aquabot_view_B.stats_c[aquabot_view_B.d_k].
                PixelList[(aquabot_view_B.xpageoffset + aquabot_view_B.hi) - 1]));
          }

          for (aquabot_view_B.ib = 2; aquabot_view_B.ib <=
               aquabot_view_B.nblocks; aquabot_view_B.ib++) {
            aquabot_view_B.xblockoffset = ((aquabot_view_B.ib - 1) << 10) +
              aquabot_view_B.xpageoffset;
            aquabot_view_B.bsum = static_cast<int32_T>
              (aquabot_view_B.stats_c[aquabot_view_B.d_k]
               .PixelList[aquabot_view_B.xblockoffset]);
            if (aquabot_view_B.ib == aquabot_view_B.nblocks) {
              aquabot_view_B.hi = aquabot_view_B.lastBlockLength;
            } else {
              aquabot_view_B.hi = 1024;
            }

            for (aquabot_view_B.end_tmp = 2; aquabot_view_B.end_tmp <=
                 aquabot_view_B.hi; aquabot_view_B.end_tmp++) {
              aquabot_view_B.bsum += static_cast<real_T>(static_cast<int32_T>
                (aquabot_view_B.stats_c[aquabot_view_B.d_k].PixelList
                 [(aquabot_view_B.xblockoffset + aquabot_view_B.end_tmp) - 1]));
            }

            aquabot_view_B.expl_temp_m3[aquabot_view_B.k] += aquabot_view_B.bsum;
          }
        }
      }

      aquabot_view_B.k = aquabot_view_B.stats_c[aquabot_view_B.d_k].
        PixelList.size(0);
      aquabot_view_B.stats_c[aquabot_view_B.d_k].Centroid[0] =
        aquabot_view_B.expl_temp_m3[0] / static_cast<real_T>(aquabot_view_B.k);
      aquabot_view_B.stats_c[aquabot_view_B.d_k].Centroid[1] =
        aquabot_view_B.expl_temp_m3[1] / static_cast<real_T>(aquabot_view_B.k);
    }

    aquabot_view_B.nblocks = aquabot_view_B.stats_c.size(0);
    for (aquabot_view_B.firstBlockLength = 0; aquabot_view_B.firstBlockLength <
         aquabot_view_B.nblocks; aquabot_view_B.firstBlockLength++) {
      aquabot_view_B.outstats_k[aquabot_view_B.firstBlockLength].Centroid[0] =
        aquabot_view_B.stats_c[aquabot_view_B.firstBlockLength].Centroid[0];
      aquabot_view_B.outstats_k[aquabot_view_B.firstBlockLength].Centroid[1] =
        aquabot_view_B.stats_c[aquabot_view_B.firstBlockLength].Centroid[1];
    }

    if (aquabot_view_B.outstats_k.size(0) >= 1) {
      aquabot_view_B.rel_yaw = (aquabot_view_B.outstats_k[0].Centroid[0] / 720.0
        - 0.5) * 3.1415926535897931;
    } else {
      aquabot_view_B.rel_yaw = 0.0;
    }

    /* RelationalOperator: '<S4>/GreaterThan' incorporates:
     *  Constant: '<S4>/Sight Threshold'
     *  MATLAB Function: '<S4>/MATLAB Function'
     */
    aquabot_view_B.GreaterThan = (aquabot_view_B.nz >
      aquabot_view_P.SightThreshold_Value);

    /* Update for UnitDelay: '<S4>/Unit Delay' incorporates:
     *  MATLAB Function: '<S4>/MATLAB Function1'
     */
    aquabot_view_DW.UnitDelay_DSTATE =
      aquabot_view_B.b_varargout_2_header_stamp_sec;
  }

  /* End of MATLABSystem: '<S9>/SourceBlock' */
  /* End of Outputs for SubSystem: '<S9>/Enabled Subsystem' */
  /* End of Outputs for SubSystem: '<Root>/Detect boat' */

  /* BusAssignment: '<Root>/Bus Assignment' */
  rtb_BusAssignment.data = aquabot_view_B.GreaterThan;

  /* MATLABSystem: '<S5>/SinkBlock' */
  Pub_aquabot_view_38.publish(&rtb_BusAssignment);

  /* BusAssignment: '<Root>/Bus Assignment1' */
  aquabot_view_B.BusAssignment1.data = aquabot_view_B.rel_yaw;

  /* MATLABSystem: '<S6>/SinkBlock' */
  Pub_aquabot_view_45.publish(&aquabot_view_B.BusAssignment1);

  /* BusAssignment: '<Root>/Bus Assignment2' */
  aquabot_view_B.BusAssignment2.pose.position.x = aquabot_view_B.pose_boat[0];
  aquabot_view_B.BusAssignment2.pose.position.y = aquabot_view_B.pose_boat[1];
  aquabot_view_B.BusAssignment2.pose.position.z = aquabot_view_B.pose_boat[2];

  /* MATLABSystem: '<S7>/SinkBlock' */
  Pub_aquabot_view_48.publish(&aquabot_view_B.BusAssignment2);
}

/* Model initialize function */
void aquabot_view::initialize()
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* Start for MATLABSystem: '<S8>/SourceBlock' */
  aquabot_view_DW.objisempty_h = true;
  aquabot_v_SystemCore_setup_myfs(&aquabot_view_DW.obj_o);

  /* Start for Enabled SubSystem: '<Root>/Subsystem2' */
  /* Start for MATLABSystem: '<S10>/Read Point Cloud' */
  aquabot_view_DW.objisempty = true;
  aquabot_view_DW.obj_k.isInitialized = 1;

  /* End of Start for SubSystem: '<Root>/Subsystem2' */

  /* Start for MATLABSystem: '<S9>/SourceBlock' */
  aquabot_view_DW.objisempty_k = true;
  aquabot__SystemCore_setup_myfsq(&aquabot_view_DW.obj_e);

  /* Start for Enabled SubSystem: '<Root>/Detect boat' */
  /* Start for MATLABSystem: '<S4>/Read Image' */
  aquabot_view_DW.objisempty_h0 = true;
  aquabot_view_DW.obj.isInitialized = 1;
  aquabot_view_DW.ImageSize[0] = aquabot_view_DW.obj.ImageSize[0];
  aquabot_view_DW.ImageSize[1] = aquabot_view_DW.obj.ImageSize[1];
  memcpy(&aquabot_view_DW.Image[0], &aquabot_view_DW.obj.Image[0], 1244160U *
         sizeof(uint8_T));

  /* Start for Enabled SubSystem: '<S4>/Subsystem' */
  /* Start for MATLABSystem: '<S16>/SourceBlock' */
  aquabot_view_DW.objisempty_m = true;
  aquabot_view_SystemCore_setup(&aquabot_view_DW.obj_h);

  /* End of Start for SubSystem: '<S4>/Subsystem' */
  /* End of Start for SubSystem: '<Root>/Detect boat' */

  /* Start for MATLABSystem: '<S5>/SinkBlock' */
  aquabot_view_DW.objisempty_ke = true;
  aquabot_view_SystemCore_setup_m(&aquabot_view_DW.obj_g);

  /* Start for MATLABSystem: '<S6>/SinkBlock' */
  aquabot_view_DW.objisempty_e = true;
  aquabot_vie_SystemCore_setup_my(&aquabot_view_DW.obj_hm);

  /* Start for MATLABSystem: '<S7>/SinkBlock' */
  aquabot_view_DW.objisempty_j = true;
  aquabot_vi_SystemCore_setup_myf(&aquabot_view_DW.obj_b);

  {
    int32_T i;

    /* SystemInitialize for Enabled SubSystem: '<S8>/Enabled Subsystem' */
    /* SystemInitialize for SignalConversion generated from: '<S18>/In1' incorporates:
     *  Outport: '<S18>/Out1'
     */
    aquabot_view_B.In1_p = aquabot_view_P.Out1_Y0_p;

    /* End of SystemInitialize for SubSystem: '<S8>/Enabled Subsystem' */

    /* SystemInitialize for Enabled SubSystem: '<Root>/Subsystem2' */
    for (i = 0; i < 90000; i++) {
      /* SystemInitialize for MATLABSystem: '<S10>/Read Point Cloud' incorporates:
       *  Outport: '<S10>/raw_3D'
       */
      aquabot_view_B.ReadPointCloud[i] = aquabot_view_P.raw_3D_Y0;
    }

    /* End of SystemInitialize for SubSystem: '<Root>/Subsystem2' */

    /* SystemInitialize for Enabled SubSystem: '<S9>/Enabled Subsystem' */
    /* SystemInitialize for SignalConversion generated from: '<S19>/In1' incorporates:
     *  Outport: '<S19>/Out1'
     */
    aquabot_view_B.In1 = aquabot_view_P.Out1_Y0;

    /* End of SystemInitialize for SubSystem: '<S9>/Enabled Subsystem' */

    /* SystemInitialize for Enabled SubSystem: '<Root>/Detect boat' */
    /* InitializeConditions for UnitDelay: '<S4>/Unit Delay' */
    aquabot_view_DW.UnitDelay_DSTATE = aquabot_view_P.UnitDelay_InitialCondition;

    /* SystemInitialize for Enabled SubSystem: '<S4>/Subsystem' */
    /* SystemInitialize for Enabled SubSystem: '<S16>/Enabled Subsystem' */
    /* SystemInitialize for SignalConversion generated from: '<S17>/In1' incorporates:
     *  Outport: '<S17>/Out1'
     */
    aquabot_view_B.In1_pk = aquabot_view_P.Out1_Y0_k;

    /* End of SystemInitialize for SubSystem: '<S16>/Enabled Subsystem' */
    /* End of SystemInitialize for SubSystem: '<S4>/Subsystem' */

    /* InitializeConditions for MATLABSystem: '<S4>/Read Image' */
    aquabot_vie_ReadImage_resetImpl(&aquabot_view_DW.obj);
    aquabot_view_DW.ImageSize[0] = aquabot_view_DW.obj.ImageSize[0];
    aquabot_view_DW.ImageSize[1] = aquabot_view_DW.obj.ImageSize[1];
    memcpy(&aquabot_view_DW.Image[0], &aquabot_view_DW.obj.Image[0], 1244160U *
           sizeof(uint8_T));

    /* SystemInitialize for RelationalOperator: '<S4>/GreaterThan' incorporates:
     *  Outport: '<S4>/Output'
     */
    aquabot_view_B.GreaterThan = aquabot_view_P.Output_Y0;

    /* SystemInitialize for Outport: '<S4>/Output1' */
    aquabot_view_B.rel_yaw = aquabot_view_P.Output1_Y0;

    /* SystemInitialize for Outport: '<S4>/Output2' */
    aquabot_view_B.pose_boat[0] = aquabot_view_P.Output2_Y0;
    aquabot_view_B.pose_boat[1] = aquabot_view_P.Output2_Y0;
    aquabot_view_B.pose_boat[2] = aquabot_view_P.Output2_Y0;

    /* End of SystemInitialize for SubSystem: '<Root>/Detect boat' */
  }
}

/* Model terminate function */
void aquabot_view::terminate()
{
  /* Terminate for MATLABSystem: '<S8>/SourceBlock' */
  if (!aquabot_view_DW.obj_o.matlabCodegenIsDeleted) {
    aquabot_view_DW.obj_o.matlabCodegenIsDeleted = true;
  }

  /* End of Terminate for MATLABSystem: '<S8>/SourceBlock' */

  /* Terminate for MATLABSystem: '<S9>/SourceBlock' */
  if (!aquabot_view_DW.obj_e.matlabCodegenIsDeleted) {
    aquabot_view_DW.obj_e.matlabCodegenIsDeleted = true;
  }

  /* End of Terminate for MATLABSystem: '<S9>/SourceBlock' */

  /* Terminate for Enabled SubSystem: '<Root>/Detect boat' */
  /* Terminate for MATLABSystem: '<S4>/Read Image' */
  aquabot_view_DW.ImageSize[0] = aquabot_view_DW.obj.ImageSize[0];
  aquabot_view_DW.ImageSize[1] = aquabot_view_DW.obj.ImageSize[1];
  memcpy(&aquabot_view_DW.Image[0], &aquabot_view_DW.obj.Image[0], 1244160U *
         sizeof(uint8_T));

  /* Terminate for Enabled SubSystem: '<S4>/Subsystem' */
  /* Terminate for MATLABSystem: '<S16>/SourceBlock' */
  if (!aquabot_view_DW.obj_h.matlabCodegenIsDeleted) {
    aquabot_view_DW.obj_h.matlabCodegenIsDeleted = true;
  }

  /* End of Terminate for MATLABSystem: '<S16>/SourceBlock' */
  /* End of Terminate for SubSystem: '<S4>/Subsystem' */
  /* End of Terminate for SubSystem: '<Root>/Detect boat' */

  /* Terminate for MATLABSystem: '<S5>/SinkBlock' */
  if (!aquabot_view_DW.obj_g.matlabCodegenIsDeleted) {
    aquabot_view_DW.obj_g.matlabCodegenIsDeleted = true;
  }

  /* End of Terminate for MATLABSystem: '<S5>/SinkBlock' */

  /* Terminate for MATLABSystem: '<S6>/SinkBlock' */
  if (!aquabot_view_DW.obj_hm.matlabCodegenIsDeleted) {
    aquabot_view_DW.obj_hm.matlabCodegenIsDeleted = true;
  }

  /* End of Terminate for MATLABSystem: '<S6>/SinkBlock' */

  /* Terminate for MATLABSystem: '<S7>/SinkBlock' */
  if (!aquabot_view_DW.obj_b.matlabCodegenIsDeleted) {
    aquabot_view_DW.obj_b.matlabCodegenIsDeleted = true;
  }

  /* End of Terminate for MATLABSystem: '<S7>/SinkBlock' */
}

/* Constructor */
aquabot_view::aquabot_view() :
  aquabot_view_B(),
  aquabot_view_DW(),
  aquabot_view_M()
{
  /* Currently there is no constructor body generated.*/
}

/* Destructor */
aquabot_view::~aquabot_view()
{
  /* Currently there is no destructor body generated.*/
}

/* Real-Time Model get method */
RT_MODEL_aquabot_view_T * aquabot_view::getRTM()
{
  return (&aquabot_view_M);
}
