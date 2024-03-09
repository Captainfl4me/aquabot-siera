/*
 * aquabot_view.h
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

#ifndef RTW_HEADER_aquabot_view_h_
#define RTW_HEADER_aquabot_view_h_
#include <cstring>
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "PCANormalCore_api.hpp"
#include "cvstCG_kdtree.h"
#include "slros2_initialize.h"
#include "coder_array.h"
#include "aquabot_view_types.h"

extern "C"
{

#include "rt_nonfinite.h"

}

extern "C"
{

#include "rtGetNaN.h"

}

#include <stddef.h>

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

/* Block signals (default storage) */
struct B_aquabot_view_T {
  s_48rvm6kxzsa25RQQXhDnzF_aqua_T a;
  s_K3X5z0rBQ6KcDg7d33fBWG_aqua_T a_m;
  coder::array<uint32_T,2> tf;
  coder::array<boolean_T,2> regionsToKeep;
  coder::array<sBCDbgMZyfjRVWzdpNZ1R9_aquabo_T,1> outstats;
  coder::array<s_K3X5z0rBQ6KcDg7d33fBWG_aqua_T,1> stats;
  coder::array<s8812E1qfIkP21aQTfYt8iB_aquab_T,1> outstats_k;
  coder::array<s_48rvm6kxzsa25RQQXhDnzF_aqua_T,1> stats_c;
  coder::array<uint32_T,1> indexBase;
  coder::array<uint32_T,2> b_index;
  coder::array<uint32_T,2> y;
  coder::array<int16_T,1> startRow;
  coder::array<int16_T,1> endRow;
  coder::array<int16_T,1> startCol;
  coder::array<int16_T,1> startRow_g;
  coder::array<int16_T,1> endRow_l;
  coder::array<int16_T,1> startCol_d;
  coder::array<uint32_T,2> indices;
  coder::array<uint32_T,1> valid;
  coder::array<real_T,1> CC_RegionIndices;
  coder::array<real_T,1> pixelsToKeep;
  coder::array<real_T,1> idx;
  coder::array<real_T,1> pc;
  coder::array<real_T,1> r5;
  coder::array<real_T,2> byteIdx;
  coder::array<real_T,2> byteIdx_g;
  coder::array<real_T,2> r3;
  coder::array<real_T,2> b_b;
  coder::array<uint8_T,1> busstruct_data;
  coder::array<uint8_T,1> data_c;
  coder::array<uint8_T,1> data_g;
  coder::array<uint8_T,1> rawData;
  coder::array<real32_T,1> r_pt_b;
  coder::array<real32_T,1> r_pt_p;
  coder::array<real32_T,1> varargin_1;
  coder::array<real32_T,1> varargin_2;
  coder::array<real32_T,1> varargin_3;
  coder::array<real32_T,2> r_pt;
  coder::array<real32_T,2> x_c;
  coder::array<real32_T,2> nv;
  coder::array<real32_T,2> l;
  coder::array<real32_T,2> xyzTmp;
  coder::array<real32_T,2> b_xyzTmp;
  coder::array<real32_T,2> nv_o;
  coder::array<real32_T,2> intensity;
  coder::array<real32_T,2> range;
  coder::array<real32_T,2> a__8;
  coder::array<real32_T,2> b_normals;
  coder::array<uint8_T,3> rawImage;
  coder::array<uint8_T,3> r;
  coder::array<uint8_T,3> data;
  coder::array<int32_T,1> CC_RegionLengths;
  coder::array<int32_T,1> idxCount;
  coder::array<int32_T,1> x;
  coder::array<int32_T,1> r1;
  coder::array<int32_T,1> r2;
  coder::array<int32_T,1> b;
  coder::array<int32_T,1> labelsRenumbered;
  coder::array<int32_T,1> idxCount_m;
  coder::array<int32_T,1> x_n;
  coder::array<int32_T,1> b_p;
  coder::array<int32_T,1> labelsRenumbered_j;
  coder::array<int32_T,1> idxCount_d;
  coder::array<int32_T,1> x_d;
  coder::array<int32_T,1> r4;
  coder::array<cell_wrap_6_aquabot_view_T,2> expl_temp;
  coder::array<cell_wrap_6_aquabot_view_T,2> c;
  coder::array<cell_wrap_6_aquabot_view_T,2> c_l;
  coder::array<uint8_T,2> data_f;
  coder::array<uint8_T,2> c_lx;
  coder::array<uint8_T,2> c_n;
  pointCloud_aquabot_view_T ptCloud;
  pointCloud_aquabot_view_T frontPts;
  pointCloud_aquabot_view_T ptCloudTransformed;
  pointCloud_aquabot_view_T lobj_0;
  SL_Bus_sensor_msgs_Image In1;        /* '<S19>/In1' */
  SL_Bus_sensor_msgs_Image b_varargout_2;
  uint8_T b_varargout_2_data[1244160];
  SL_Bus_sensor_msgs_PointCloud2 In1_p;/* '<S18>/In1' */
  c_ros_slros2_internal_block_P_T pc_b;
  SL_Bus_sensor_msgs_PointCloud2 b_varargout_2_l;
  uint8_T b_varargout_2_data_h[960000];
  boolean_T BW[414720];                /* '<S4>/MATLAB Function3' */
  real32_T projectedPts[90000];
  real32_T loc[90000];
  real32_T loc_b[90000];
  real32_T location[90000];
  real32_T loc_d[90000];
  real32_T b_imPts[60000];
  int16_T tmp_data[30000];
  int16_T tmp_data_e[30000];
  int16_T tmp_data_b[30000];
  int16_T tmp_data_j[30000];
  int16_T tmp_data_f[30000];
  boolean_T tmp_data_a[30000];
  boolean_T b_indices[30000];
  boolean_T ptCloudTransformed_j[30000];
  boolean_T pointIdxIsValid[30000];
  boolean_T x_j[30000];
  SL_Bus_sensor_msgs_PointField b_varargout_2_fields[16];
  SL_Bus_sensor_msgs_CameraInfo In1_pk;/* '<S17>/In1' */
  SL_Bus_sensor_msgs_CameraInfo b_varargout_2_o;
  real_T tmp_data_n[128];
  SL_Bus_geometry_msgs_PoseStamped BusAssignment2;/* '<Root>/Bus Assignment2' */
  char_T tmp_data_i[128];
  uint8_T b_varargout_2_header_frame_id[128];
  uint8_T b_varargout_2_encoding[128];
  uint8_T busstruct_encoding_data[128];
  real_T intrinsics_K[9];
  real_T assign_temp_R[9];
  real_T A[9];
  real32_T T[16];
  real32_T A_o[16];
  real_T y_data[8];
  char_T b_zeroDelimTopic[61];
  char_T b_zeroDelimTopic_n[59];
  char_T b_zeroDelimTopic_m[46];
  real32_T T_c[9];
  real32_T b_A[9];
  real_T pose_boat[3];                 /* '<S4>/MATLAB Function1' */
  real_T expl_temp_m[3];
  s8812E1qfIkP21aQTfYt8iB_aquab_T s;
  real_T expl_temp_m3[2];
  real_T expl_temp_j[2];
  real_T expl_temp_h[2];
  real32_T b_x[3];
  real32_T rotMatOut[3];
  real32_T s_c[3];
  real32_T e[3];
  real32_T work[3];
  real_T rel_yaw;                      /* '<S4>/MATLAB Function' */
  real32_T ReadPointCloud[90000];      /* '<S10>/Read Point Cloud' */
  real_T CC_NumObjects;
  real_T nz;
  real_T bsum;
  real_T d;
  real_T yOff;
  real_T zOff;
  real_T u;
  real_T y_a;
  real_T bsum_e;
  SL_Bus_std_msgs_Float64 BusAssignment1;/* '<Root>/Bus Assignment1' */
  uint64_T u_a;
  uint64_T u1;
  uint32_T x1[2];
  int32_T expl_temp_size[2];
  int32_T busstruct_encoding_size[2];
  int32_T tmp_size[2];
  int32_T y_size[2];
  real32_T intrinsics_ImageSize[2];
  real32_T fv[2];
  real32_T intrinsics_ImageSize_c[2];
  void* locationPtr;
  void* locationPtr_p;
  void* locationPtr_p5;
  int8_T ipiv[4];
  real32_T bsum_l;
  real32_T imageSize;
  real32_T imageSize_o;
  real32_T b_imPts_o;
  real32_T b_imPts_i;
  real32_T s_f;
  real32_T y_i;
  real32_T z;
  real32_T absx;
  real32_T smax;
  real32_T s_ff;
  real32_T nrm;
  real32_T rt;
  real32_T r_g;
  real32_T ztest0;
  real32_T smm1;
  real32_T emm1;
  int32_T b_varargout_2_header_stamp_sec;
  int32_T k;
  int32_T d_k;
  int32_T firstBlockLength;
  int32_T lastBlockLength;
  int32_T nblocks;
  int32_T xblockoffset;
  int32_T hi;
  int32_T ib;
  int32_T xpageoffset;
  int32_T i;
  int32_T end_tmp;
  int32_T i_c;
  int32_T i1;
  int32_T c_o;
  int32_T i2;
  int32_T i3;
  int32_T loop_ub;
  int32_T loop_ub_l;
  int32_T trueCount;
  int32_T i_m;
  int32_T tmp_size_idx_0;
  int32_T coffset;
  int32_T aoffset;
  int32_T j;
  int32_T i_mj;
  int32_T count;
  int32_T idx_c;
  int32_T k_f;
  int32_T jj;
  int32_T jA;
  int32_T e_p;
  int32_T f;
  int32_T qp1;
  int32_T qq;
  int32_T qjj;
  int32_T m;
  int32_T b_i;
  int32_T loop_ub_e;
  int32_T c_o4;
  int32_T i4;
  int32_T i5;
  int32_T loop_ub_h;
  uint32_T b_varargout_2_header_stamp_nano;
  uint32_T b_varargout_2_header_frame_id_S;
  uint32_T b_varargout_2_header_frame_id_m;
  uint32_T b_varargout_2_height;
  uint32_T b_varargout_2_width;
  uint32_T b_varargout_2_fields_SL_Info_Cu;
  uint32_T b_varargout_2_fields_SL_Info_Re;
  uint32_T b_varargout_2_point_step;
  uint32_T b_varargout_2_row_step;
  uint32_T b_varargout_2_data_SL_Info_Curr;
  uint32_T b_varargout_2_data_SL_Info_Rece;
  uint32_T xIdx;
  uint32_T yIdx;
  uint32_T zIdx;
  uint32_T numPointsActual;
  cameraParameters_aquabot_view_T lobj_0_a;
  c_vision_internal_codegen_cam_T expl_temp_data;
  c_images_geotrans_internal_ri_T expl_temp_data_i;
  boolean_T GreaterThan;               /* '<S4>/GreaterThan' */
};

/* Block states (default storage) for system '<Root>' */
struct DW_aquabot_view_T {
  ros_slros2_internal_block_R_m_T obj; /* '<S4>/Read Image' */
  ros_slros2_internal_block_Sub_T obj_e;/* '<S9>/SourceBlock' */
  ros_slros2_internal_block_Sub_T obj_o;/* '<S8>/SourceBlock' */
  ros_slros2_internal_block_Sub_T obj_h;/* '<S16>/SourceBlock' */
  ros_slros2_internal_block_Pub_T obj_b;/* '<S7>/SinkBlock' */
  ros_slros2_internal_block_Pub_T obj_hm;/* '<S6>/SinkBlock' */
  ros_slros2_internal_block_Pub_T obj_g;/* '<S5>/SinkBlock' */
  real_T UnitDelay_DSTATE;             /* '<S4>/Unit Delay' */
  uint32_T ImageSize[2];               /* '<S4>/Read Image' */
  ros_slros2_internal_block_Rea_T obj_k;/* '<S10>/Read Point Cloud' */
  uint8_T Image[1244160];              /* '<S4>/Read Image' */
  boolean_T objisempty;                /* '<S10>/Read Point Cloud' */
  boolean_T objisempty_k;              /* '<S9>/SourceBlock' */
  boolean_T objisempty_h;              /* '<S8>/SourceBlock' */
  boolean_T objisempty_j;              /* '<S7>/SinkBlock' */
  boolean_T objisempty_e;              /* '<S6>/SinkBlock' */
  boolean_T objisempty_ke;             /* '<S5>/SinkBlock' */
  boolean_T objisempty_m;              /* '<S16>/SourceBlock' */
  boolean_T objisempty_h0;             /* '<S4>/Read Image' */
};

/* Parameters (default storage) */
struct P_aquabot_view_T_ {
  SL_Bus_sensor_msgs_Image Out1_Y0;    /* Computed Parameter: Out1_Y0
                                        * Referenced by: '<S19>/Out1'
                                        */
  SL_Bus_sensor_msgs_Image Constant_Value;/* Computed Parameter: Constant_Value
                                           * Referenced by: '<S9>/Constant'
                                           */
  SL_Bus_sensor_msgs_PointCloud2 Out1_Y0_p;/* Computed Parameter: Out1_Y0_p
                                            * Referenced by: '<S18>/Out1'
                                            */
  SL_Bus_sensor_msgs_PointCloud2 Constant_Value_m;/* Computed Parameter: Constant_Value_m
                                                   * Referenced by: '<S8>/Constant'
                                                   */
  SL_Bus_sensor_msgs_CameraInfo Out1_Y0_k;/* Computed Parameter: Out1_Y0_k
                                           * Referenced by: '<S17>/Out1'
                                           */
  SL_Bus_sensor_msgs_CameraInfo Constant_Value_f;/* Computed Parameter: Constant_Value_f
                                                  * Referenced by: '<S16>/Constant'
                                                  */
  SL_Bus_geometry_msgs_PoseStamped Constant_Value_l;/* Computed Parameter: Constant_Value_l
                                                     * Referenced by: '<S3>/Constant'
                                                     */
  SL_Bus_std_msgs_Bool Constant_Value_j;/* Computed Parameter: Constant_Value_j
                                         * Referenced by: '<S1>/Constant'
                                         */
  SL_Bus_std_msgs_Float64 Constant_Value_mw;/* Computed Parameter: Constant_Value_mw
                                             * Referenced by: '<S2>/Constant'
                                             */
  real_T Output1_Y0;                   /* Computed Parameter: Output1_Y0
                                        * Referenced by: '<S4>/Output1'
                                        */
  real_T Output2_Y0;                   /* Computed Parameter: Output2_Y0
                                        * Referenced by: '<S4>/Output2'
                                        */
  real_T UnitDelay_InitialCondition;   /* Expression: 1
                                        * Referenced by: '<S4>/Unit Delay'
                                        */
  real_T SightThreshold_Value;         /* Expression: 20
                                        * Referenced by: '<S4>/Sight Threshold'
                                        */
  real32_T raw_3D_Y0;                  /* Computed Parameter: raw_3D_Y0
                                        * Referenced by: '<S10>/raw_3D'
                                        */
  boolean_T Output_Y0;                 /* Computed Parameter: Output_Y0
                                        * Referenced by: '<S4>/Output'
                                        */
};

/* Real-time Model Data Structure */
struct tag_RTM_aquabot_view_T {
  const char_T *errorStatus;
};

/* Class declaration for model aquabot_view */
class aquabot_view
{
  /* public data and function members */
 public:
  /* Real-Time Model get method */
  RT_MODEL_aquabot_view_T * getRTM();

  /* model start function */
  void start();

  /* Initial conditions function */
  void initialize();

  /* model step function */
  void step();

  /* model terminate function */
  void terminate();

  /* Constructor */
  aquabot_view();

  /* Destructor */
  ~aquabot_view();

  /* private data and function members */
 private:
  /* Block signals */
  B_aquabot_view_T aquabot_view_B;

  /* Block states */
  DW_aquabot_view_T aquabot_view_DW;

  /* Tunable parameters */
  static P_aquabot_view_T aquabot_view_P;

  /* private member function(s) for subsystem '<Root>'*/
  void aquabot_v_SystemCore_setup_myfs(ros_slros2_internal_block_Sub_T *obj);
  void aquabot__SystemCore_setup_myfsq(ros_slros2_internal_block_Sub_T *obj);
  void aquabot_view_SystemCore_setup(ros_slros2_internal_block_Sub_T *obj);
  void aquabot_view_SystemCore_setup_m(ros_slros2_internal_block_Pub_T *obj);
  void aquabot_vie_SystemCore_setup_my(ros_slros2_internal_block_Pub_T *obj);
  void aquabot_vi_SystemCore_setup_myf(ros_slros2_internal_block_Pub_T *obj);
  void aquabot_vie_ReadImage_resetImpl(ros_slros2_internal_block_R_m_T *obj);
  void aquabot_view_SystemCore_step_m(boolean_T *varargout_1, int32_T
    *varargout_2_header_stamp_sec, uint32_T *varargout_2_header_stamp_nanose,
    uint8_T varargout_2_header_frame_id[128], uint32_T
    *varargout_2_header_frame_id_SL_, uint32_T *varargout_2_header_frame_id_S_0,
    uint32_T *varargout_2_height, uint32_T *varargout_2_width,
    SL_Bus_sensor_msgs_PointField varargout_2_fields[16], uint32_T
    *varargout_2_fields_SL_Info_Curr, uint32_T *varargout_2_fields_SL_Info_Rece,
    boolean_T *varargout_2_is_bigendian, uint32_T *varargout_2_point_step,
    uint32_T *varargout_2_row_step, uint8_T varargout_2_data[960000], uint32_T
    *varargout_2_data_SL_Info_Curren, uint32_T *varargout_2_data_SL_Info_Receiv,
    boolean_T *varargout_2_is_dense);
  uint32_T PointCloud2BusWrapper_getFieldI(const c_ros_slros2_internal_block_P_T
    *obj);
  uint32_T PointCloud2BusWrapper_getFiel_m(const c_ros_slros2_internal_block_P_T
    *obj);
  void aquabot_view_bsxfun(const coder::array<real_T, 1U> &a, const real_T
    b_data[], const int32_T b_size[2], coder::array<real_T, 2U> &c);
  void PointCloud2Reader_getByteIndexF(const c_ros_slros2_internal_block_P_T *pc,
    uint32_T fieldIdx, coder::array<real_T, 2U> &byteIdx, boolean_T
    pointIdxIsValid[30000]);
  void PointCloud2Reader_readFieldFrom(const coder::array<uint8_T, 1U> &data,
    const coder::array<real_T, 2U> &byteIdx, const boolean_T pointIdxIsValid
    [30000], coder::array<real32_T, 1U> &fieldPoints);
  void aquab_PointCloud2Reader_readXYZ(const c_ros_slros2_internal_block_P_T *pc,
    uint32_T xIdx, uint32_T yIdx, uint32_T zIdx, coder::array<real32_T, 2U> &xyz);
  void aquabot_ReadPointCloud_stepImpl(const SL_Bus_sensor_msgs_PointCloud2
    *busstruct, real32_T varargout_1[90000]);
  void aquabot_view_SystemCore_step_my(boolean_T *varargout_1, int32_T
    *varargout_2_header_stamp_sec, uint32_T *varargout_2_header_stamp_nanose,
    uint8_T varargout_2_header_frame_id[128], uint32_T
    *varargout_2_header_frame_id_SL_, uint32_T *varargout_2_header_frame_id_S_0,
    uint32_T *varargout_2_height, uint32_T *varargout_2_width, uint8_T
    varargout_2_encoding[128], uint32_T *varargout_2_encoding_SL_Info_Cu,
    uint32_T *varargout_2_encoding_SL_Info_Re, uint8_T *varargout_2_is_bigendian,
    uint32_T *varargout_2_step, uint8_T varargout_2_data[1244160], uint32_T
    *varargout_2_data_SL_Info_Curren, uint32_T *varargout_2_data_SL_Info_Receiv);
  void aquabot_view_char(const uint8_T varargin_1_data[], const int32_T
    varargin_1_size[2], char_T y_data[], int32_T y_size[2]);
  boolean_T aquabot_view_strcmp(const char_T a_data[], const int32_T a_size[2]);
  void aquabot_view_permute(const coder::array<uint8_T, 3U> &a, coder::array<
    uint8_T, 3U> &b);
  void aquabot_v_ImageReader_readImage(const coder::array<uint8_T, 1U> &data,
    uint32_T width, uint32_T height, coder::array<uint8_T, 3U> &img);
  uint8_T aqu_ReadImage_updateBusAndState(ros_slros2_internal_block_R_m_T *obj,
    uint32_T busstruct_height, uint32_T busstruct_width, const uint8_T
    busstruct_encoding[128], uint32_T busstruct_encoding_SL_Info_Curr, const
    uint8_T busstruct_data[1244160], uint32_T busstruct_data_SL_Info_CurrentL,
    uint32_T busstruct_data_SL_Info_Received);
  void aquabot_view_ReadImage_stepImpl(ros_slros2_internal_block_R_m_T *obj,
    uint32_T busstruct_height, uint32_T busstruct_width, const uint8_T
    busstruct_encoding[128], uint32_T busstruct_encoding_SL_Info_Curr, const
    uint8_T busstruct_data[1244160], uint32_T busstruct_data_SL_Info_CurrentL,
    uint32_T busstruct_data_SL_Info_Received, uint8_T varargout_1[1244160],
    uint8_T *varargout_2);
  void aquabot_view_SystemCore_step(ros_slros2_internal_block_R_m_T *obj,
    uint32_T varargin_1_height, uint32_T varargin_1_width, const uint8_T
    varargin_1_encoding[128], uint32_T varargin_1_encoding_SL_Info_Cur, const
    uint8_T varargin_1_data[1244160], uint32_T varargin_1_data_SL_Info_Current,
    uint32_T varargin_1_data_SL_Info_Receive, uint8_T varargout_1[1244160],
    uint8_T *varargout_2);
  void aquabot_view_bwconncomp(const boolean_T varargin_1[414720], real_T
    *CC_Connectivity, real_T CC_ImageSize[2], real_T *CC_NumObjects, coder::
    array<real_T, 1U> &CC_RegionIndices, coder::array<int32_T, 1U>
    &CC_RegionLengths, coder::array<cell_wrap_6_aquabot_view_T, 2U>
    &CC_PixelIdxList);
  void aqua_cameraIntrinsicsFromOpenCV(const real_T intrinsicMatrix[9], const
    real_T distortionCoefficients_data[], const real32_T imageSize[2],
    cameraParameters_aquabot_view_T *iobj_0, real_T intrinsics_FocalLength[2],
    real_T intrinsics_PrincipalPoint[2], real32_T intrinsics_ImageSize[2],
    real_T intrinsics_RadialDistortion[3], real_T
    intrinsics_TangentialDistortion[2], real_T *intrinsics_Skew, real_T
    intrinsics_K[9], cameraParameters_aquabot_view_T
    **intrinsics_CameraParameters, c_vision_internal_codegen_cam_T
    intrinsics_cameraIntrinsicsArra[], int32_T intrinsics_cameraIntrinsicsAr_0[2]);
  void aquab_rigidtform3d_rigidtform3d(real_T b_this_Translation[3], real_T
    b_this_R[9], c_images_geotrans_internal_ri_T b_this_Data_data[], int32_T
    b_this_Data_size[2]);
  void aquabo_pointCloudBase_set_Color(pointCloud_aquabot_view_T *b_this, const
    coder::array<uint8_T, 2U> &b_value);
  pointCloud_aquabot_view_T *aquabot_v_pointCloud_pointCloud
    (pointCloud_aquabot_view_T *b_this, const real32_T varargin_1[90000]);
  real32_T aquabot_view_xnrm2(int32_T n, const real32_T x[9], int32_T ix0);
  real32_T aquabot_view_xnrm2_b(const real32_T x[3], int32_T ix0);
  void aquabot_view_xaxpy(int32_T n, real32_T a, const real32_T x[3], int32_T
    ix0, real32_T y[9], int32_T iy0);
  void aquabot_view_xrotg(real32_T *a, real32_T *b, real32_T *c, real32_T *s);
  void aquabot_view_svd(const real32_T A[9], real32_T U[3]);
  real32_T aquabot_view_maximum(const real32_T x[3]);
  void aq_pointCloud_surfaceNormalImpl(pointCloud_aquabot_view_T *b_this, coder::
    array<real32_T, 2U> &normals);
  pointCloud_aquabot_view_T *aquabot_pointCloud_pointCloud_k
    (pointCloud_aquabot_view_T *b_this, const real32_T varargin_1[90000], const
     coder::array<uint8_T, 2U> &varargin_3, const coder::array<real32_T, 2U>
     &varargin_5, const coder::array<real32_T, 2U> &varargin_7);
  pointCloud_aquabot_view_T *aquabot_view_pctransform(const
    pointCloud_aquabot_view_T *ptCloudIn, const real_T tform_Translation[3],
    const real_T tform_R[9], pointCloud_aquabot_view_T *iobj_0);
  void aquabot_view_eml_find(const boolean_T x[30000], coder::array<int32_T, 1U>
    &i);
  void aquabot_view_repmat(const real_T varargin_1[2], coder::array<real32_T, 2U>
    &b);
  void aquab_pointCloudBase_subsetImpl(const pointCloud_aquabot_view_T *b_this,
    const coder::array<real_T, 1U> &indices, real32_T loc[90000], coder::array<
    uint8_T, 2U> &c, coder::array<real32_T, 2U> &nv, coder::array<real32_T, 2U>
    &intensity, coder::array<real32_T, 2U> &r);
  pointCloud_aquabot_view_T *aquabot_view_pointCloud_select(const
    pointCloud_aquabot_view_T *b_this, const boolean_T varargin_1[30000],
    pointCloud_aquabot_view_T *iobj_0);
  void aquabot_vi_projectLidarToCamera(const pointCloud_aquabot_view_T
    *ptCloudIn, const real_T intrinsic_K[9], const real_T tform_Translation[3],
    const real_T tform_R[9], real32_T projectPoints[60000]);
  void aquabot_vi_projectPointsToImage(const pointCloud_aquabot_view_T *pc,
    const real_T intrinsic_K[9], const real_T tform_Translation[3], const real_T
    tform_R[9], const real32_T imageSize[2], coder::array<real32_T, 2U> &imPts,
    coder::array<real_T, 1U> &indices);
  void aquab_projectLidarPointsOnImage(const real32_T ptCloudIn[90000], const
    real32_T intrinsics_ImageSize[2], const real_T intrinsics_K[9], const real_T
    tform_Translation[3], const real_T tform_R[9], coder::array<real32_T, 2U>
    &imPts, coder::array<real_T, 1U> &indices);
  void aquabot_view_sub2ind(const coder::array<real32_T, 1U> &varargin_1, const
    coder::array<real32_T, 1U> &varargin_2, coder::array<int32_T, 1U> &idx);

  /* Real-Time Model */
  RT_MODEL_aquabot_view_T aquabot_view_M;
};

extern volatile boolean_T stopRequested;
extern volatile boolean_T runModel;

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'aquabot_view'
 * '<S1>'   : 'aquabot_view/Blank Message'
 * '<S2>'   : 'aquabot_view/Blank Message1'
 * '<S3>'   : 'aquabot_view/Blank Message2'
 * '<S4>'   : 'aquabot_view/Detect boat'
 * '<S5>'   : 'aquabot_view/Publish'
 * '<S6>'   : 'aquabot_view/Publish1'
 * '<S7>'   : 'aquabot_view/Publish2'
 * '<S8>'   : 'aquabot_view/SubLidar3D'
 * '<S9>'   : 'aquabot_view/Subscribe'
 * '<S10>'  : 'aquabot_view/Subsystem2'
 * '<S11>'  : 'aquabot_view/Detect boat/MATLAB Function'
 * '<S12>'  : 'aquabot_view/Detect boat/MATLAB Function1'
 * '<S13>'  : 'aquabot_view/Detect boat/MATLAB Function2'
 * '<S14>'  : 'aquabot_view/Detect boat/MATLAB Function3'
 * '<S15>'  : 'aquabot_view/Detect boat/Subsystem'
 * '<S16>'  : 'aquabot_view/Detect boat/Subsystem/Subscribe'
 * '<S17>'  : 'aquabot_view/Detect boat/Subsystem/Subscribe/Enabled Subsystem'
 * '<S18>'  : 'aquabot_view/SubLidar3D/Enabled Subsystem'
 * '<S19>'  : 'aquabot_view/Subscribe/Enabled Subsystem'
 */
#endif                                 /* RTW_HEADER_aquabot_view_h_ */
