/*
 * MATLAB0_sf.c
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "MATLAB0_sf".
 *
 * Model version              : 1.780
 * Simulink Coder version : 23.2 (R2023b) 01-Aug-2023
 * C source code generated on : Thu Nov 30 15:19:47 2023
 *
 * Target selection: rtwsfcn.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Linux 64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "MATLAB0_sf.h"
#include "MATLAB0_sf_types.h"
#include "rtwtypes.h"
#include <string.h>
#include <math.h>
#include "rt_nonfinite.h"
#include "rt_defines.h"
#include <float.h>
#include <stdlib.h>
#include <stddef.h>
#include "MATLAB0_sf_private.h"
#include "simstruc.h"
#include "fixedpoint.h"
#if defined(RT_MALLOC) || defined(MATLAB_MEX_FILE)

extern void *MATLAB0_malloc(SimStruct *S);

#endif

#ifndef __RTW_UTFREE__
#if defined (MATLAB_MEX_FILE)

extern void * utMalloc(size_t);
extern void utFree(void *);

#endif
#endif                                 /* #ifndef __RTW_UTFREE__ */

/* Forward declaration for local functions */
static void MATLAB0_emxInit_real_T(emxArray_real_T_MATLAB0_T **pEmxArray,
  int32_T numDimensions);
static void MATLAB0_emxInitStruct_navPath(navPath_MATLAB0_T *pStruct);
static binaryOccupancyMap_MATLAB0_T *binaryOccupancyMap_binaryOccupa
  (binaryOccupancyMap_MATLAB0_T *obj, const boolean_T varargin_1[422500]);
static stateSpaceSE2_MATLAB0_T *MAT_stateSpaceSE2_stateSpaceSE2
  (stateSpaceSE2_MATLAB0_T *obj);
static void MATLAB_MapLayer_getValueAllImpl(binaryOccupancyMap_MATLAB0_T *obj,
  boolean_T val[422500]);
static validatorOccupancyMap_MATLAB0_T *validatorOccupancyMap_validator
  (validatorOccupancyMap_MATLAB0_T *obj, stateSpaceSE2_MATLAB0_T *varargin_1,
   binaryOccupancyMap_MATLAB0_T *varargin_3);
static plannerHybridAStar_MATLAB0_T *plannerHybridAStar_plannerHybri
  (plannerHybridAStar_MATLAB0_T *obj, validatorOccupancyMap_MATLAB0_T *validator,
   real_T varargin_2, real_T varargin_4, real_T varargin_6);
static real_T MATLAB0_rt_atan2d_snf(real_T u0, real_T u1);
static real_T MATLAB0_maximum(const real_T x[4]);
static boolean_T MATLAB0_vectorAny(const boolean_T x_data[], const int32_T
  *x_size);
static void MAT_MapInterface_world2gridImpl(const binaryOccupancyMap_MATLAB0_T
  *obj, const real_T worldXY[2], real_T gridInd[2]);
static void MATLAB0_emxInit_boolean_T(emxArray_boolean_T_MATLAB0_T **pEmxArray,
  int32_T numDimensions);
static void MATLAB_emxEnsureCapacity_real_T(emxArray_real_T_MATLAB0_T *emxArray,
  int32_T oldNumel);
static real_T MATLAB0_mod(real_T x, real_T y);
static void MATLAB0_expand_mod(const emxArray_real_T_MATLAB0_T *a,
  emxArray_real_T_MATLAB0_T *c);
static void MATLAB0_emxFree_real_T(emxArray_real_T_MATLAB0_T **pEmxArray);
static void MAT_emxEnsureCapacity_boolean_T(emxArray_boolean_T_MATLAB0_T
  *emxArray, int32_T oldNumel);
static void MATLAB0_emxFree_boolean_T(emxArray_boolean_T_MATLAB0_T **pEmxArray);
static void binaryOccupancyMap_checkOccupan(binaryOccupancyMap_MATLAB0_T *obj,
  const real_T varargin_1[2], real_T *occupied, boolean_T *validIds);
static boolean_T validatorOccupancyMap_isStateVa(validatorOccupancyMap_MATLAB0_T
  *obj, const real_T state[3]);
static void MATLAB0_emxInit_cell_wrap_53(emxArray_cell_wrap_53_MATLAB0_T
  **pEmxArray, int32_T numDimensions);
static void emxInitStruct_reedsSheppConnect(reedsSheppConnection_MATLAB0_T
  *pStruct);
static void plannerHybridAStar_validateStar(plannerHybridAStar_MATLAB0_T *obj,
  const real_T start[3], const real_T goal[3]);
static c_robotics_core_internal_Name_T *NameValueParser_NameValueParser
  (c_robotics_core_internal_Name_T *obj);
static void MATLAB0_validatestring(const char_T str[6], char_T out_data[],
  int32_T out_size[2]);
static boolean_T MATLAB0_ifWhileCond(const boolean_T x[3]);
static void MATLAB0_repelem(const real_T x[6], real_T varargin_1,
  emxArray_real_T_MATLAB0_T *y);
static boolean_T MATLAB0_any(const emxArray_boolean_T_MATLAB0_T *x);
static void MATLAB0_binary_expand_op_23(emxArray_boolean_T_MATLAB0_T *in1, const
  emxArray_real_T_MATLAB0_T *in2, const emxArray_real_T_MATLAB0_T *in3);
static void MATLAB0_wrapToPi_o(emxArray_real_T_MATLAB0_T *theta);
static void MATLAB0_expand_max(const emxArray_real_T_MATLAB0_T *a, const
  emxArray_real_T_MATLAB0_T *b, emxArray_real_T_MATLAB0_T *c);
static void MATLAB0_maximum2(const emxArray_real_T_MATLAB0_T *x, const
  emxArray_real_T_MATLAB0_T *y, emxArray_real_T_MATLAB0_T *ex);
static void MATLAB0_expand_min(const emxArray_real_T_MATLAB0_T *a, const
  emxArray_real_T_MATLAB0_T *b, emxArray_real_T_MATLAB0_T *c);
static void MATLAB0_minimum2(const emxArray_real_T_MATLAB0_T *x, const
  emxArray_real_T_MATLAB0_T *y, emxArray_real_T_MATLAB0_T *ex);
static void stateSpaceSE2_enforceStateBound(const stateSpaceSE2_MATLAB0_T *obj,
  const emxArray_real_T_MATLAB0_T *state, emxArray_real_T_MATLAB0_T
  *boundedState);
static navPath_MATLAB0_T *MATLAB0_navPath_navPath(navPath_MATLAB0_T *obj,
  stateSpaceSE2_MATLAB0_T *stateSpace, const emxArray_real_T_MATLAB0_T *states);
static void MATLAB0_ind2sub(const real_T ndx[422500], int32_T varargout_1[422500],
  int32_T varargout_2[422500], int32_T varargout_3[422500]);
static void MATLAB_plannerAStarGrid_set_Map(plannerAStarGrid_MATLAB0_T *obj,
  binaryOccupancyMap_MATLAB0_T *input);
static void validateAStarBuiltinCostFunctio(char_T strVal_data[], int32_T
  strVal_size[2], real_T *idx);
static plannerAStarGrid_MATLAB0_T *plannerAStarGrid_plannerAStarGr
  (plannerAStarGrid_MATLAB0_T *obj, binaryOccupancyMap_MATLAB0_T *varargin_1);
static void MATLAB0_emxFree_char_T(emxArray_char_T_MATLAB0_T **pEmxArray);
static void MATL_emxFreeStruct_cell_wrap_53(cell_wrap_53_MATLAB0_T *pStruct);
static void MATLAB0_emxFree_cell_wrap_53(emxArray_cell_wrap_53_MATLAB0_T
  **pEmxArray);
static void emxFreeStruct_reedsSheppConnect(reedsSheppConnection_MATLAB0_T
  *pStruct);
static void MATLAB0_ind2sub_a(const real_T ndx[422500], int32_T varargout_1
  [422500], int32_T varargout_2[422500]);
static c_nav_algs_internal_plannerAS_T *plannerAStarGrid_plannerAStar_e
  (c_nav_algs_internal_plannerAS_T *obj, const real_T map[422500], real_T
   obstacleThreshold);
static c_nav_algs_internal_plannerAS_T *plannerAStarGrid_initializeInte
  (plannerAStarGrid_MATLAB0_T *obj, c_nav_algs_internal_plannerAS_T *iobj_0);
static void MATLAB0_sum(const real_T x_data[], const int32_T x_size[2], real_T
  y_data[], int32_T *y_size);
static real_T MATLAB0_minimum(const real_T x_data[], const int32_T *x_size);
static real_T MATLAB0_binary_expand_op(const real_T in1_data[], const int32_T
  *in1_size, const real_T in2_data[], const int32_T *in2_size);
static void MATL_plannerAStarGrid_Chebyshev(const real_T pose1[845000], const
  real_T pose2[845000], real_T dist[422500]);
static void M_plannerAStarGrid_getNeighbors(const
  c_nav_algs_internal_plannerAS_T *obj, real_T Neighbors_data[], int32_T
  Neighbors_size[2], real_T *NumNeighbors);
static void MATLAB0_emxInit_int32_T(emxArray_int32_T_MATLAB0_T **pEmxArray,
  int32_T numDimensions);
static void plannerAStarGrid_reconstructPat(c_nav_algs_internal_plannerAS_T *obj,
  real_T CurrentRow, real_T CurrentCol, const real_T startIn[2]);
static void MATLA_emxEnsureCapacity_int32_T(emxArray_int32_T_MATLAB0_T *emxArray,
  int32_T oldNumel);
static void MATLAB0_emxFree_int32_T(emxArray_int32_T_MATLAB0_T **pEmxArray);
static real_T MAT_plannerAStarGrid_gcostValue(const
  c_nav_algs_internal_plannerAS_T *obj, real_T CurrentRow, real_T CurrentCol,
  real_T i, real_T j);
static real_T MATLAB0_rt_roundd_snf(real_T u);
static void MATLAB_plannerAStarGrid_runPlan(c_nav_algs_internal_plannerAS_T
  *astarInternal, const real_T start[2], const real_T goal[2]);
static void plannerAStarGrid_getGCostMatrix(const
  c_nav_algs_internal_plannerAS_T *astarInternal, real_T GCostMatrix[422500]);
static real_T plannerHybridAStar_get2DHeurist(plannerHybridAStar_MATLAB0_T *obj,
  c_nav_algs_internal_plannerAS_T *heuristic2DObjInternal, const real_T point[2]);
static real_T plannerHybridAStar_get3DHeurist(const plannerHybridAStar_MATLAB0_T
  *obj, const real_T start[3], const real_T goal[3]);
static real_T MATLAB0_maximum_b(const real_T x[2]);
static void MATLAB0_linspace(real_T d1, real_T d2, real_T y[5]);
static void MATLAB0_repmat(const real_T a_data[], real_T b_data[], int32_T
  b_size[2]);
static reedsSheppConnection_MATLAB0_T *reedsSheppConnection_reedsShepp
  (reedsSheppConnection_MATLAB0_T *obj, real_T varargin_2, real_T varargin_4,
   real_T varargin_6);
static void MATLAB0_PriorityQueue_top(nav_algs_internal_PriorityQue_T *obj,
  real_T nodeData_data[], int32_T nodeData_size[2], real_T *nodeId);
static boolean_T MATLAB0_strcmp(const char_T a_data[], const int32_T a_size[2]);
static real_T MATLAB0_rt_remd_snf(real_T u0, real_T u1);
static void MA_plannerHybridAStar_closeCell(plannerHybridAStar_MATLAB0_T *obj,
  real_T direction, real_T Indice);
static void plannerHybridAStar_getCircularP(real_T length, const real_T
  curvature[8], const real_T initialNodePose[3], const real_T direction[8],
  real_T newNodesPoses[24], real_T ICRsData[24]);
static void MATLA_emxInit_cell_wrap_53_5x44(emxArray_cell_wrap_53_5x44_MA_T
  *pEmxArray);
static void MATLAB0_strcmp_c(const emxArray_char_T_MATLAB0_T *a, boolean_T
  b_bool[44]);
static void ReedsSheppBuiltins_autonomousRe(const real_T startPose[3], const
  real_T goalPose[3], real_T turningRadius, real_T forwardCost, real_T
  reverseCost, const char_T pathSegments[3], const
  emxArray_cell_wrap_53_MATLAB0_T *disabledTypes, real_T cost_data[], int32_T
  *cost_size, real_T motionLengths_data[], int32_T motionLengths_size[2], real_T
  motionTypes_data[], int32_T motionTypes_size[2]);
static void MATLA_emxTrim_cell_wrap_53_5x44(cell_wrap_53_MATLAB0_T data[220],
  int32_T fromIndex, int32_T toIndex);
static void MATLAB0_emxInit_char_T(emxArray_char_T_MATLAB0_T **pEmxArray,
  int32_T numDimensions);
static void MATL_emxInitStruct_cell_wrap_53(cell_wrap_53_MATLAB0_T *pStruct);
static void MAT_emxExpand_cell_wrap_53_5x44(cell_wrap_53_MATLAB0_T data[220],
  int32_T fromIndex, int32_T toIndex);
static void emxEnsureCapacity_cell_wrap_53(cell_wrap_53_MATLAB0_T data[220],
  const int32_T size[2], int32_T oldNumel);
static void MATLAB0_emxInit_uint64_T(emxArray_uint64_T_MATLAB0_T **pEmxArray,
  int32_T numDimensions);
static void MATLAB0_sortIdx(const emxArray_real_T_MATLAB0_T *x,
  emxArray_int32_T_MATLAB0_T *idx);
static void MATLAB0_merge(int32_T idx_data[], int32_T x_data[], int32_T offset,
  int32_T np, int32_T nq, int32_T iwork_data[], int32_T xwork_data[]);
static void MATLAB0_sort(int32_T x_data[], const int32_T *x_size, int32_T
  idx_data[], int32_T *idx_size);
static void MATLAB0_do_vectors(const real_T a[44], const
  emxArray_real_T_MATLAB0_T *b, real_T c_data[], int32_T c_size[2], int32_T
  ia_data[], int32_T *ia_size, int32_T *ib_size);
static void MATLAB0_emxFree_uint64_T(emxArray_uint64_T_MATLAB0_T **pEmxArray);
static void MATLAB0_merge_n(int32_T idx_data[], real_T x_data[], int32_T offset,
  int32_T np, int32_T nq, int32_T iwork_data[], real_T xwork_data[]);
static void MATLAB0_sort_o(real_T x_data[], const int32_T *x_size);
static void MATLAB_emxEnsureCapacity_char_T(emxArray_char_T_MATLAB0_T *emxArray,
  int32_T oldNumel);
static void MATL_emxEnsureCapacity_uint64_T(emxArray_uint64_T_MATLAB0_T
  *emxArray, int32_T oldNumel);
static void ReedsSheppConnection_connectInt(const reedsSheppConnection_MATLAB0_T
  *this, const real_T startPose[3], const real_T goalPose[3], real_T
  motionLengths_data[], int32_T motionLengths_size[2], cell_wrap_53_MATLAB0_T
  motionTypes_data[], int32_T motionTypes_size[2], real_T motionCosts_data[],
  int32_T *motionCosts_size, real_T motionDirections_data[], int32_T
  motionDirections_size[2]);
static void emxInit_reedsSheppPathSegment_4(emxArray_reedsSheppPathSegmen_T
  *pEmxArray);
static void MATL_emxFreeMatrix_cell_wrap_53(cell_wrap_53_MATLAB0_T pMatrix[5]);
static void emxFreeStruct_reedsSheppPathSeg(reedsSheppPathSegment_MATLAB0_T
  *pStruct);
static void emxTrim_reedsSheppPathSegment_4(reedsSheppPathSegment_MATLAB0_T
  data[44], int32_T fromIndex, int32_T toIndex);
static void MATL_emxInitMatrix_cell_wrap_53(cell_wrap_53_MATLAB0_T pMatrix[5]);
static void emxInitStruct_reedsSheppPathSeg(reedsSheppPathSegment_MATLAB0_T
  *pStruct);
static void emxExpand_reedsSheppPathSegment(reedsSheppPathSegment_MATLAB0_T
  data[44], int32_T fromIndex, int32_T toIndex);
static void emxEnsureCapacity_reedsSheppPat(reedsSheppPathSegment_MATLAB0_T
  data[44], const int32_T *size, int32_T oldNumel);
static void MATLA_emxFree_cell_wrap_53_5x44(emxArray_cell_wrap_53_5x44_MA_T
  *pEmxArray);
static void MATLAB0_emxCopy_char_T(emxArray_char_T_MATLAB0_T **dst,
  emxArray_char_T_MATLAB0_T * const *src);
static void MATL_emxCopyStruct_cell_wrap_53(cell_wrap_53_MATLAB0_T *dst, const
  cell_wrap_53_MATLAB0_T *src);
static real_T MATLAB0_mod_p(real_T x);
static void MATLAB0_linspace_j(real_T d1, real_T d2, real_T n,
  emxArray_real_T_MATLAB0_T *y);
static void emxFree_reedsSheppPathSegment_4(emxArray_reedsSheppPathSegmen_T
  *pEmxArray);
static void M_MapInterface_world2gridImpl_d(const binaryOccupancyMap_MATLAB0_T
  *obj, const emxArray_real_T_MATLAB0_T *worldXY, emxArray_real_T_MATLAB0_T
  *gridInd);
static void binaryOccupancyMap_checkOccup_f(binaryOccupancyMap_MATLAB0_T *obj,
  const emxArray_real_T_MATLAB0_T *varargin_1, emxArray_real_T_MATLAB0_T
  *occupied, emxArray_boolean_T_MATLAB0_T *validIds);
static void validatorOccupancyMap_checkMapO(const
  validatorOccupancyMap_MATLAB0_T *obj, const emxArray_real_T_MATLAB0_T *stateXY,
  emxArray_boolean_T_MATLAB0_T *validPos);
static void MATLAB0_all(const emxArray_boolean_T_MATLAB0_T *x,
  emxArray_boolean_T_MATLAB0_T *y);
static void MATLAB0_binary_expand_op_1(emxArray_boolean_T_MATLAB0_T *in1, const
  emxArray_real_T_MATLAB0_T *in2, const real_T in3[2], const
  emxArray_real_T_MATLAB0_T *in4, const emxArray_real_T_MATLAB0_T *in5);
static void MATLAB0_binary_expand_op_2(emxArray_boolean_T_MATLAB0_T *in1, const
  emxArray_real_T_MATLAB0_T *in2, const real_T in3[2], const
  emxArray_real_T_MATLAB0_T *in4, const emxArray_real_T_MATLAB0_T *in5, const
  emxArray_boolean_T_MATLAB0_T *in6);
static void MATLAB0_binary_expand_op_5(emxArray_boolean_T_MATLAB0_T *in1, const
  emxArray_real_T_MATLAB0_T *in2, const real_T in3[2], const
  emxArray_real_T_MATLAB0_T *in4, const emxArray_real_T_MATLAB0_T *in5, const
  emxArray_boolean_T_MATLAB0_T *in6, const emxArray_real_T_MATLAB0_T *in7);
static boolean_T plannerHybridAStar_checkAnalyti(plannerHybridAStar_MATLAB0_T
  *obj, const real_T initialPose[3], const real_T finalPose[3], real_T stepSize,
  const reedsSheppConnection_MATLAB0_T *rsPathObj);
static void plannerHybridAStar_getStraightP(real_T length, const real_T
  initialNodePose[3], const real_T direction[2], real_T newNodePose[6]);
static void MATLAB0_MapInterface_world2grid(const binaryOccupancyMap_MATLAB0_T
  *obj, const real_T pos[20], real_T idx[20]);
static void plannerHybridAStar_checkNodeVal(const plannerHybridAStar_MATLAB0_T
  *obj, const real_T PointsGrid[20], const real_T direction[10], real_T
  nodeValidity[10]);
static void MATLAB0_linspace_jf(real_T n, emxArray_real_T_MATLAB0_T *y);
static void MATLAB0_binary_expand_op_11(emxArray_real_T_MATLAB0_T *in1, const
  real_T in2_data[], const int32_T in2_size[2], const real_T in3_data[], const
  int32_T in3_size[2], real_T in4, real_T in5);
static void MATLAB0_repmat_k(const real_T a_data[], const int32_T *a_size,
  real_T varargin_2, emxArray_real_T_MATLAB0_T *b);
static void MATLAB0_binary_expand_op_10(emxArray_real_T_MATLAB0_T *in1, const
  real_T in2_data[], const int32_T in2_size[2], const emxArray_real_T_MATLAB0_T *
  in3);
static void MATLAB0_binary_expand_op_8(emxArray_real_T_MATLAB0_T *in1, const
  real_T in2_data[], const int32_T *in2_size, const emxArray_real_T_MATLAB0_T
  *in3);
static void MATLAB0_binary_expand_op_7(emxArray_real_T_MATLAB0_T *in1, const
  real_T in2_data[], const int32_T in2_size[2], const emxArray_real_T_MATLAB0_T *
  in3);
static void MATLAB0_nullAssignment(emxArray_real_T_MATLAB0_T *x);
static void MATLAB0_binary_expand_op_6(emxArray_real_T_MATLAB0_T *in1, const
  real_T in2_data[], const int32_T in2_size[2], const emxArray_real_T_MATLAB0_T *
  in3);
static void plannerHybridAStar_getPosesCirc(const real_T initialPose_data[],
  const int32_T initialPose_size[2], const real_T finalPoses_data[], const
  int32_T finalPoses_size[2], const real_T ICRData_data[], const int32_T
  ICRData_size[2], const real_T radius_data[], const int32_T radius_size[2],
  real_T length, real_T stepSize, emxArray_real_T_MATLAB0_T *poses);
static void MATLAB0_wrapToPi(real_T *theta);
static void MATLAB0_binary_expand_op_12(emxArray_real_T_MATLAB0_T *in1, const
  emxArray_real_T_MATLAB0_T *in2, const emxArray_real_T_MATLAB0_T *in3, const
  real_T in4[3]);
static void MATLA_stateSpaceSE2_interpolate(const stateSpaceSE2_MATLAB0_T *obj,
  const real_T state1[3], const real_T state2[3], emxArray_real_T_MATLAB0_T
  *ratios, emxArray_real_T_MATLAB0_T *interpState);
static void validatorOccupancyMap_isState_c(validatorOccupancyMap_MATLAB0_T *obj,
  const emxArray_real_T_MATLAB0_T *state, emxArray_boolean_T_MATLAB0_T *isValid);
static void plannerHybridAStar_isPrimitiveV(plannerHybridAStar_MATLAB0_T *obj,
  const real_T initialPose[3], const real_T finalPoses[30], const real_T
  ICRsData[24], const real_T radius_data[], real_T length, real_T stepSize,
  boolean_T result[10], real_T finalPosesGridIndices_data[], int32_T
  finalPosesGridIndices_size[2]);
static int32_T MATLAB0_intnnz(const boolean_T s[10]);
static void MATLAB0_NodeMap_traceBack(nav_algs_internal_NodeMap_MAT_T *obj,
  real_T idx, emxArray_real_T_MATLAB0_T *nodeDataVec);
static void MATLAB0_flipud(emxArray_real_T_MATLAB0_T *x);
static boolean_T MATLAB0_strcmp_c3(const char_T a_data[], const int32_T a_size[2]);
static void plannerHybridAStar_getFinalPath(const plannerHybridAStar_MATLAB0_T
  *obj, const emxArray_real_T_MATLAB0_T *pathData, emxArray_real_T_MATLAB0_T
  *finalPathData);
static void MATLAB0_nonzeros(const real_T s[5], real_T v_data[], int32_T *v_size);
static void MATLAB0_diff(const real_T x_data[], const int32_T *x_size, real_T
  y_data[], int32_T *y_size);
static void ReedsSheppBuiltins_autonomous_n(const real_T startPose[3], const
  real_T goalPose[3], const emxArray_real_T_MATLAB0_T *samples, real_T
  turningRadius, const real_T segmentsLengths[5], const int32_T
  segmentsDirections[5], const uint32_T segmentsTypes[5],
  emxArray_real_T_MATLAB0_T *poses, emxArray_real_T_MATLAB0_T *directions);
static void MATLAB0_unique_vector(const emxArray_real_T_MATLAB0_T *a,
  emxArray_real_T_MATLAB0_T *b);
static void plannerHybridAStar_getInterpola(const plannerHybridAStar_MATLAB0_T
  *obj, const emxArray_real_T_MATLAB0_T *pathData, emxArray_real_T_MATLAB0_T
  *path, emxArray_real_T_MATLAB0_T *dir);
static void plannerHybridAStar_closeCell_i(plannerHybridAStar_MATLAB0_T *obj,
  const real_T direction_data[], const int32_T *direction_size, const real_T
  Indice_data[]);
static void MATLAB0_binary_expand_op_21(real_T in1_data[], int32_T *in1_size,
  const real_T in2_data[], const int8_T in3[2], const
  plannerHybridAStar_MATLAB0_T *in4, const emxArray_real_T_MATLAB0_T *in5, const
  real_T in6_data[], const int32_T *in6_size);
static void plannerHybridAStar_get2DHeuri_o(plannerHybridAStar_MATLAB0_T *obj,
  c_nav_algs_internal_plannerAS_T *heuristic2DObjInternal, const real_T
  point_data[], const int32_T point_size[2], real_T cost_data[], int32_T
  *cost_size);
static void MATLAB0_plus(real_T in1_data[], int32_T *in1_size, const real_T
  in2_data[], const int32_T *in2_size, const real_T in3_data[], const int32_T
  *in3_size);
static void plannerHybridAStar_calculateCos(plannerHybridAStar_MATLAB0_T *obj,
  c_nav_algs_internal_plannerAS_T *heuristic2DObjInternal, const real_T
  newNodeData_data[], const int32_T newNodeData_size[2], const real_T
  currentNode_data[], const real_T curvature_data[], const int32_T
  *curvature_size, const real_T direction_data[], const int32_T *direction_size,
  real_T fScore_data[], int32_T *fScore_size, real_T gScore_data[], int32_T
  *gScore_size, real_T hScore_data[], int32_T *hScore_size);
static void MATLAB0_eml_find(const boolean_T x[10], int32_T i_data[], int32_T
  *i_size);
static int32_T MATLAB0_intnnz_l(const boolean_T s[8]);
static int32_T MATLAB0_intnnz_li(const boolean_T s[2]);
static void MATLAB0_repmat_kk(const real_T a[3], real_T varargin_1,
  emxArray_real_T_MATLAB0_T *b);
static navPath_MATLAB0_T *MATLAB0_plannerHybridAStar_plan
  (plannerHybridAStar_MATLAB0_T *obj, const real_T start[3], const real_T goal[3],
   plannerAStarGrid_MATLAB0_T *iobj_0, navPath_MATLAB0_T *iobj_1);
static boolean_T validatorOccupancyMap_isMotionV(validatorOccupancyMap_MATLAB0_T
  *obj, const real_T state1[3], real_T state2[3]);
static void MATLAB0_emxFreeStruct_navPath(navPath_MATLAB0_T *pStruct);
static void MATLAB0_sf_gateway_c6_MATLAB0(SimStruct *S);

#if defined(MATLAB_MEX_FILE)
#include "rt_nonfinite.c"
#endif

static const char_T *RT_MEMORY_ALLOCATION_ERROR =
  "memory allocation error in generated S-Function";
int32_T MATLAB0_div_s32(int32_T numerator, int32_T denominator)
{
  int32_T quotient;
  uint32_T tempAbsQuotient;
  if (denominator == 0) {
    quotient = numerator >= 0 ? MAX_int32_T : MIN_int32_T;

    /* Divide by zero handler */
  } else {
    tempAbsQuotient = (numerator < 0 ? ~(uint32_T)numerator + 1U : (uint32_T)
                       numerator) / (denominator < 0 ? ~(uint32_T)denominator +
      1U : (uint32_T)denominator);
    quotient = (numerator < 0) != (denominator < 0) ? -(int32_T)tempAbsQuotient :
      (int32_T)tempAbsQuotient;
  }

  return quotient;
}

static void MATLAB0_emxInit_real_T(emxArray_real_T_MATLAB0_T **pEmxArray,
  int32_T numDimensions)
{
  static emxArray_real_T_MATLAB0_T *emxArray;
  int32_T i;
  *pEmxArray = (emxArray_real_T_MATLAB0_T *)malloc(sizeof
    (emxArray_real_T_MATLAB0_T));
  emxArray = *pEmxArray;
  emxArray->data = (real_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int32_T *)malloc(sizeof(int32_T) * (uint32_T)numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

static void MATLAB0_emxInitStruct_navPath(navPath_MATLAB0_T *pStruct)
{
  MATLAB0_emxInit_real_T(&pStruct->StateInternal, 2);
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static binaryOccupancyMap_MATLAB0_T *binaryOccupancyMap_binaryOccupa
  (binaryOccupancyMap_MATLAB0_T *obj, const boolean_T varargin_1[422500])
{
  binaryOccupancyMap_MATLAB0_T *b_obj;
  int32_T i;
  boolean_T a;
  b_obj = obj;
  obj->HasParent = false;
  obj->SharedProperties.GridOriginInLocal[0] = 0.0;
  obj->SharedProperties.GridOriginInLocal[1] = 0.0;
  obj->SharedProperties.LocalOriginInWorld[0] = 0.0;
  obj->SharedProperties.LocalOriginInWorld[1] = 0.0;
  obj->Index.Head[0] = 1.0;
  obj->Index.Head[1] = 1.0;
  obj->DefaultValueInternal = false;
  a = obj->DefaultValueInternal;
  obj->Buffer.Index = &obj->Index;
  for (i = 0; i < 422500; i++) {
    obj->Buffer.Buffer[i] = a;
  }

  for (i = 0; i < 422500; i++) {
    obj->Buffer.Buffer[i] = varargin_1[i];
  }

  obj->Index.Head[0] = 1.0;
  obj->Index.Head[1] = 1.0;
  return b_obj;
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static stateSpaceSE2_MATLAB0_T *MAT_stateSpaceSE2_stateSpaceSE2
  (stateSpaceSE2_MATLAB0_T *obj)
{
  stateSpaceSE2_MATLAB0_T *b_obj;
  int32_T i;
  static const real_T c[6] = { -100.0, -100.0, -3.1415926535897931, 100.0, 100.0,
    3.1415926535897931 };

  b_obj = obj;
  obj->WeightXY = 1.0;
  obj->WeightTheta = 0.1;
  obj->Name[0] = 'S';
  obj->Name[1] = 'E';
  obj->Name[2] = '2';
  for (i = 0; i < 6; i++) {
    obj->StateBoundsInternal[i] = c[i];
  }

  obj->SkipStateValidation = false;
  return b_obj;
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static void MATLAB_MapLayer_getValueAllImpl(binaryOccupancyMap_MATLAB0_T *obj,
  boolean_T val[422500])
{
  static int32_T b;
  static int32_T b_k;
  static int32_T c_k;
  static int32_T d_k;
  static int32_T f;
  static int32_T i;
  static int32_T i1;
  static int32_T j;
  static int32_T k;
  static int32_T pageroot;
  static int32_T pagesize;
  static int32_T stride;
  static boolean_T buffer[325];
  real_T p;
  int32_T b_x;
  int32_T e_k;
  int32_T g_k;
  boolean_T x[2];
  boolean_T exitg1;
  boolean_T y;
  x[0] = (obj->Index.Head[0] == 1.0);
  x[1] = (obj->Index.Head[1] == 1.0);
  y = true;
  g_k = 0;
  exitg1 = false;
  while ((!exitg1) && (g_k <= 1)) {
    if (!x[g_k]) {
      y = false;
      exitg1 = true;
    } else {
      g_k++;
    }
  }

  if (y) {
    for (g_k = 0; g_k < 422500; g_k++) {
      val[g_k] = obj->Buffer.Buffer[g_k];
    }
  } else {
    for (g_k = 0; g_k < 422500; g_k++) {
      val[g_k] = obj->Buffer.Buffer[g_k];
    }

    stride = 1;
    for (g_k = 0; g_k < 2; g_k++) {
      p = -(obj->Index.Head[g_k] - 1.0);
      if (p < 0.0) {
        b_x = -(int32_T)p;
        y = false;
      } else {
        b_x = (int32_T)p;
        y = true;
      }

      if (b_x > 650) {
        b_x -= 650 * MATLAB0_div_s32(b_x, 650);
      }

      if (b_x > 325) {
        b_x = 650 - b_x;
        y = !y;
      }

      x[g_k] = y;
      pagesize = stride * 650;
      if (b_x > 0) {
        b = -649 * g_k + 649;
        for (i = 0; i <= b; i++) {
          pageroot = i * pagesize;
          for (j = 0; j < stride; j++) {
            i1 = pageroot + j;
            if (x[g_k]) {
              for (k = 0; k < b_x; k++) {
                buffer[k] = val[((k - b_x) + 650) * stride + i1];
              }

              for (b_k = 650; b_k >= b_x + 1; b_k--) {
                val[i1 + (b_k - 1) * stride] = val[((b_k - b_x) - 1) * stride +
                  i1];
              }

              for (c_k = 0; c_k < b_x; c_k++) {
                val[i1 + c_k * stride] = buffer[c_k];
              }
            } else {
              for (d_k = 0; d_k < b_x; d_k++) {
                buffer[d_k] = val[d_k * stride + i1];
              }

              f = 650 - b_x;
              for (e_k = 0; e_k < f; e_k++) {
                val[i1 + e_k * stride] = val[(e_k + b_x) * stride + i1];
              }

              for (e_k = 0; e_k < b_x; e_k++) {
                val[i1 + ((e_k - b_x) + 650) * stride] = buffer[e_k];
              }
            }
          }
        }
      }

      stride = pagesize;
    }

    if (!obj->HasParent) {
      for (g_k = 0; g_k < 422500; g_k++) {
        obj->Buffer.Buffer[g_k] = val[g_k];
      }

      obj->Index.Head[0] = 1.0;
      obj->Index.Head[1] = 1.0;
    }
  }
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static validatorOccupancyMap_MATLAB0_T *validatorOccupancyMap_validator
  (validatorOccupancyMap_MATLAB0_T *obj, stateSpaceSE2_MATLAB0_T *varargin_1,
   binaryOccupancyMap_MATLAB0_T *varargin_3)
{
  static boolean_T unusedExpr[422500];
  validatorOccupancyMap_MATLAB0_T *b_obj;
  b_obj = obj;
  obj->StateSpace = varargin_1;
  obj->Map = varargin_3;
  obj->XYIndices[0] = 1.0;
  obj->XYIndices[1] = 2.0;
  obj->ValidationDistance = (rtInf);
  obj->SkipStateValidation = false;
  MATLAB_MapLayer_getValueAllImpl(varargin_3, unusedExpr);
  return b_obj;
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static plannerHybridAStar_MATLAB0_T *plannerHybridAStar_plannerHybri
  (plannerHybridAStar_MATLAB0_T *obj, validatorOccupancyMap_MATLAB0_T *validator,
   real_T varargin_2, real_T varargin_4, real_T varargin_6)
{
  static boolean_T unusedExpr[422500];
  binaryOccupancyMap_MATLAB0_T *c_obj;
  plannerHybridAStar_MATLAB0_T *b_obj;
  int32_T j;
  int32_T neighborX_tmp;
  int8_T neighborX[9];
  int8_T neighborY[9];
  b_obj = obj;
  obj->CustomTransitionCostFlag = 0.0;
  obj->CustomAECostFlag = 0.0;
  obj->MotionPrimitiveLength = 0.0;
  obj->StateValidator = validator;
  obj->Map = obj->StateValidator->Map;
  obj->Dimensions[0] = 650.0;
  obj->Dimensions[1] = 650.0;
  obj->PathFound = false;
  obj->StartPose[0] = (rtNaN);
  obj->StartPose[1] = (rtNaN);
  obj->StartPose[2] = (rtNaN);
  obj->GoalPose[0] = (rtNaN);
  obj->GoalPose[1] = (rtNaN);
  obj->GoalPose[2] = (rtNaN);
  obj->MinTurningRadius = varargin_2;
  obj->MotionPrimitiveLength = varargin_4;
  obj->ForwardCost = 1.0;
  obj->ReverseCost = 3.0;
  obj->DirectionSwitchingCost = 0.0;
  obj->AnalyticExpansionInterval = 5.0;
  obj->InterpolationDistance = varargin_6;
  c_obj = obj->Map;
  MATLAB_MapLayer_getValueAllImpl(c_obj, unusedExpr);
  for (j = 0; j < 3; j++) {
    neighborX[3 * j] = (int8_T)(j - 1);
    neighborY[3 * j] = -1;
    neighborX_tmp = 3 * j + 1;
    neighborX[neighborX_tmp] = (int8_T)(j - 1);
    neighborY[neighborX_tmp] = 0;
    neighborX_tmp = 3 * j + 2;
    neighborX[neighborX_tmp] = (int8_T)(j - 1);
    neighborY[neighborX_tmp] = 1;
  }

  for (j = 0; j < 9; j++) {
    obj->Neighbors[j] = neighborX[j];
  }

  for (j = 0; j < 9; j++) {
    obj->Neighbors[j + 9] = neighborY[j];
  }

  return b_obj;
}

static real_T MATLAB0_rt_atan2d_snf(real_T u0, real_T u1)
{
  real_T y;
  int32_T tmp;
  int32_T tmp_0;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    if (u0 > 0.0) {
      tmp = 1;
    } else {
      tmp = -1;
    }

    if (u1 > 0.0) {
      tmp_0 = 1;
    } else {
      tmp_0 = -1;
    }

    y = atan2(tmp, tmp_0);
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = atan2(u0, u1);
  }

  return y;
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static real_T MATLAB0_maximum(const real_T x[4])
{
  static real_T b_ex;
  static real_T x_0;
  real_T ex;
  int32_T idx;
  int32_T k;
  boolean_T exitg1;
  if (!rtIsNaN(x[0])) {
    idx = 1;
  } else {
    idx = 0;
    k = 2;
    exitg1 = false;
    while ((!exitg1) && (k <= 4)) {
      if (!rtIsNaN(x[k - 1])) {
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
    b_ex = x[idx - 1];
    for (k = idx + 1; k < 5; k++) {
      x_0 = x[k - 1];
      if (b_ex < x_0) {
        b_ex = x_0;
      }
    }

    ex = b_ex;
  }

  return ex;
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static boolean_T MATLAB0_vectorAny(const boolean_T x_data[], const int32_T
  *x_size)
{
  int32_T k;
  boolean_T exitg1;
  boolean_T y;
  y = false;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k <= *x_size - 1)) {
    if (x_data[k]) {
      y = true;
      exitg1 = true;
    } else {
      k++;
    }
  }

  return y;
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static void MAT_MapInterface_world2gridImpl(const binaryOccupancyMap_MATLAB0_T
  *obj, const real_T worldXY[2], real_T gridInd[2])
{
  static real_T a[4];
  static real_T x[4];
  static real_T gOrig[2];
  static real_T gridXY[2];
  static real_T absx;
  static real_T r;
  static real_T r_0;
  static const int32_T tmp = 2;
  static int32_T b_k;
  static int32_T c_k;
  static int32_T exponent;
  int32_T i;
  boolean_T originIdx[2];
  gridXY[0] = (worldXY[1] - obj->SharedProperties.LocalOriginInWorld[1]) -
    obj->SharedProperties.GridOriginInLocal[1];
  gridXY[1] = (worldXY[0] - obj->SharedProperties.LocalOriginInWorld[0]) -
    obj->SharedProperties.GridOriginInLocal[0];
  gridInd[0] = ceil(gridXY[0]);
  x[0] = obj->SharedProperties.GridOriginInLocal[0];
  x[2] = obj->SharedProperties.GridOriginInLocal[1];
  gridInd[1] = ceil(gridXY[1]);
  x[1] = obj->SharedProperties.GridOriginInLocal[0] + 650.0;
  x[3] = obj->SharedProperties.GridOriginInLocal[1] + 650.0;
  for (b_k = 0; b_k < 4; b_k++) {
    a[b_k] = fabs(x[b_k]);
  }

  for (c_k = 0; c_k < 2; c_k++) {
    gOrig[c_k] = fabs(gridXY[c_k]);
  }

  absx = fabs(MATLAB0_maximum(a));
  if (rtIsInf(absx) || rtIsNaN(absx)) {
    r = (rtNaN);
  } else if (absx < 4.4501477170144028E-308) {
    r = 4.94065645841247E-324;
  } else {
    frexp(absx, &exponent);
    r = ldexp(1.0, exponent - 53);
  }

  r_0 = r * 2.0;
  originIdx[0] = (gOrig[0] < r_0);
  originIdx[1] = (gOrig[1] < r_0);
  if (MATLAB0_vectorAny(originIdx, &tmp)) {
    for (i = 0; i < 2; i++) {
      if (originIdx[i]) {
        gridInd[i] = 1.0;
      }
    }
  }

  gridInd[0] = 651.0 - gridInd[0];
}

static void MATLAB0_emxInit_boolean_T(emxArray_boolean_T_MATLAB0_T **pEmxArray,
  int32_T numDimensions)
{
  emxArray_boolean_T_MATLAB0_T *emxArray;
  int32_T i;
  *pEmxArray = (emxArray_boolean_T_MATLAB0_T *)malloc(sizeof
    (emxArray_boolean_T_MATLAB0_T));
  emxArray = *pEmxArray;
  emxArray->data = (boolean_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int32_T *)malloc(sizeof(int32_T) * (uint32_T)numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

static void MATLAB_emxEnsureCapacity_real_T(emxArray_real_T_MATLAB0_T *emxArray,
  int32_T oldNumel)
{
  static void *newData;
  int32_T i;
  int32_T newNumel;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  newNumel = 1;
  for (i = 0; i < emxArray->numDimensions; i++) {
    newNumel *= emxArray->size[i];
  }

  if (newNumel > emxArray->allocatedSize) {
    i = emxArray->allocatedSize;
    if (i < 16) {
      i = 16;
    }

    while (i < newNumel) {
      if (i > 1073741823) {
        i = MAX_int32_T;
      } else {
        i <<= 1;
      }
    }

    newData = malloc((uint32_T)i * sizeof(real_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(real_T) * (uint32_T)oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = (real_T *)newData;
    emxArray->allocatedSize = i;
    emxArray->canFreeData = true;
  }
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static real_T MATLAB0_mod(real_T x, real_T y)
{
  static real_T q;
  real_T r;
  boolean_T rEQ0;
  r = x;
  if (rtIsNaN(x) || rtIsNaN(y) || rtIsInf(x)) {
    r = (rtNaN);
  } else if (x == 0.0) {
    r = 0.0;
  } else if (rtIsInf(y)) {
    if (x < 0.0) {
      r = y;
    }
  } else {
    r = fmod(x, y);
    rEQ0 = (r == 0.0);
    if ((!rEQ0) && (y > floor(y))) {
      q = fabs(x / y);
      rEQ0 = !(fabs(q - floor(q + 0.5)) > 2.2204460492503131E-16 * q);
    }

    if (rEQ0) {
      r = 0.0;
    } else if (x < 0.0) {
      r += y;
    }
  }

  return r;
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static void MATLAB0_expand_mod(const emxArray_real_T_MATLAB0_T *a,
  emxArray_real_T_MATLAB0_T *c)
{
  int32_T b;
  int32_T b_k;
  int32_T k;
  k = c->size[0] * c->size[1];
  c->size[0] = a->size[0];
  c->size[1] = 2;
  MATLAB_emxEnsureCapacity_real_T(c, k);
  if (a->size[0] != 0) {
    for (k = 0; k < 2; k++) {
      b = c->size[0] - 1;
      for (b_k = 0; b_k <= b; b_k++) {
        c->data[b_k + c->size[0] * k] = MATLAB0_mod(a->data[a->size[0] * k + b_k],
          650.0);
      }
    }
  }
}

static void MATLAB0_emxFree_real_T(emxArray_real_T_MATLAB0_T **pEmxArray)
{
  if (*pEmxArray != (emxArray_real_T_MATLAB0_T *)NULL) {
    if (((*pEmxArray)->data != (real_T *)NULL) && (*pEmxArray)->canFreeData) {
      free((*pEmxArray)->data);
    }

    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = (emxArray_real_T_MATLAB0_T *)NULL;
  }
}

static void MAT_emxEnsureCapacity_boolean_T(emxArray_boolean_T_MATLAB0_T
  *emxArray, int32_T oldNumel)
{
  static void *newData;
  int32_T i;
  int32_T newNumel;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  newNumel = 1;
  for (i = 0; i < emxArray->numDimensions; i++) {
    newNumel *= emxArray->size[i];
  }

  if (newNumel > emxArray->allocatedSize) {
    i = emxArray->allocatedSize;
    if (i < 16) {
      i = 16;
    }

    while (i < newNumel) {
      if (i > 1073741823) {
        i = MAX_int32_T;
      } else {
        i <<= 1;
      }
    }

    newData = malloc((uint32_T)i * sizeof(boolean_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(boolean_T) * (uint32_T)oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = (boolean_T *)newData;
    emxArray->allocatedSize = i;
    emxArray->canFreeData = true;
  }
}

static void MATLAB0_emxFree_boolean_T(emxArray_boolean_T_MATLAB0_T **pEmxArray)
{
  if (*pEmxArray != (emxArray_boolean_T_MATLAB0_T *)NULL) {
    if (((*pEmxArray)->data != (boolean_T *)NULL) && (*pEmxArray)->canFreeData)
    {
      free((*pEmxArray)->data);
    }

    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = (emxArray_boolean_T_MATLAB0_T *)NULL;
  }
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static void binaryOccupancyMap_checkOccupan(binaryOccupancyMap_MATLAB0_T *obj,
  const real_T varargin_1[2], real_T *occupied, boolean_T *validIds)
{
  static d_matlabshared_autonomous_int_T *b_obj;
  static emxArray_boolean_T_MATLAB0_T *values;
  static emxArray_real_T_MATLAB0_T *mapStart;
  static emxArray_real_T_MATLAB0_T *r;
  static real_T xlimits[2];
  static real_T obj_0;
  static real_T obj_1;
  int32_T i;
  int32_T loop_ub;
  int32_T trueCount;
  int8_T b_idx_0;
  obj_0 = obj->SharedProperties.LocalOriginInWorld[0] +
    obj->SharedProperties.GridOriginInLocal[0];
  obj_1 = obj->SharedProperties.LocalOriginInWorld[1] +
    obj->SharedProperties.GridOriginInLocal[1];
  *validIds = ((varargin_1[0] >= obj_0) && (varargin_1[0] <= obj_0 + 650.0) &&
               (varargin_1[1] >= obj_1) && (varargin_1[1] <= obj_1 + 650.0));
  MAT_MapInterface_world2gridImpl(obj, varargin_1, xlimits);
  *occupied = -1.0;
  if ((xlimits[0] > 0.0) && (xlimits[0] < 651.0) && (xlimits[1] > 0.0) &&
      (xlimits[1] < 651.0)) {
    MATLAB0_emxInit_boolean_T(&values, 1);
    b_obj = obj->Buffer.Index;
    trueCount = 0;
    for (i = 0; i < 1; i++) {
      trueCount++;
    }

    MATLAB0_emxInit_real_T(&mapStart, 2);
    i = mapStart->size[0] * mapStart->size[1];
    mapStart->size[0] = trueCount;
    mapStart->size[1] = 2;
    MATLAB_emxEnsureCapacity_real_T(mapStart, i);
    for (i = 0; i < 2; i++) {
      for (loop_ub = 0; loop_ub < trueCount; loop_ub++) {
        mapStart->data[mapStart->size[0] * i] = (b_obj->Head[i] + xlimits[i]) -
          1.0;
      }
    }

    MATLAB0_emxInit_real_T(&r, 2);
    loop_ub = mapStart->size[0] << 1;
    i = mapStart->size[0] * mapStart->size[1];
    mapStart->size[1] = 2;
    MATLAB_emxEnsureCapacity_real_T(mapStart, i);
    for (i = 0; i < loop_ub; i++) {
      mapStart->data[i]--;
    }

    if (mapStart->size[0] == 1) {
      i = r->size[0] * r->size[1];
      r->size[0] = 1;
      r->size[1] = 2;
      MATLAB_emxEnsureCapacity_real_T(r, i);
      r->data[0] = MATLAB0_mod(mapStart->data[0], 650.0);
      r->data[1] = MATLAB0_mod(mapStart->data[1], 650.0);
    } else {
      MATLAB0_expand_mod(mapStart, r);
    }

    MATLAB0_emxFree_real_T(&mapStart);
    loop_ub = r->size[0] << 1;
    i = r->size[0] * r->size[1];
    r->size[1] = 2;
    MATLAB_emxEnsureCapacity_real_T(r, i);
    for (i = 0; i < loop_ub; i++) {
      r->data[i]++;
    }

    i = values->size[0];
    values->size[0] = r->size[0];
    MAT_emxEnsureCapacity_boolean_T(values, i);
    loop_ub = r->size[0];
    for (i = 0; i < loop_ub; i++) {
      values->data[i] = obj->Buffer.Buffer[(int32_T)((r->data[i + r->size[0]] -
        1.0) * 650.0 + r->data[i]) - 1];
    }

    MATLAB0_emxFree_real_T(&r);
    b_idx_0 = -1;
    loop_ub = values->size[0];
    for (i = 0; i < loop_ub; i++) {
      b_idx_0 = (int8_T)values->data[i];
    }

    MATLAB0_emxFree_boolean_T(&values);
    *occupied = b_idx_0;
  }
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static boolean_T validatorOccupancyMap_isStateVa(validatorOccupancyMap_MATLAB0_T
  *obj, const real_T state[3])
{
  static stateSpaceSE2_MATLAB0_T *b_obj;
  static real_T ssBounds[6];
  static real_T ssUpperBounds[2];
  static real_T state_0[2];
  static real_T xyInd[2];
  static real_T f_tmp;
  static real_T isOccupied;
  static real_T originIdx_tmp;
  static const int32_T tmp = 2;
  static int32_T b_k;
  static int32_T jcol;
  static int32_T k;
  static int32_T trueCount;
  int32_T i;
  boolean_T originIdx[2];
  boolean_T e;
  boolean_T exitg1;
  boolean_T f;
  boolean_T h;
  boolean_T idxInBounds;
  boolean_T isInBounds;
  boolean_T isValid;
  xyInd[0] = obj->XYIndices[0];
  xyInd[1] = obj->XYIndices[1];
  b_obj = obj->StateSpace;
  for (jcol = 0; jcol < 6; jcol++) {
    ssBounds[jcol] = b_obj->StateBoundsInternal[jcol];
  }

  for (jcol = 0; jcol < 2; jcol++) {
    originIdx_tmp = state[(int32_T)xyInd[jcol] - 1];
    originIdx[jcol] = ((originIdx_tmp >= ssBounds[jcol]) && (originIdx_tmp <=
      ssBounds[jcol + 3]));
  }

  isInBounds = true;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k <= 1)) {
    if (!originIdx[k]) {
      isInBounds = false;
      exitg1 = true;
    } else {
      k++;
    }
  }

  if (obj->SkipStateValidation) {
    originIdx_tmp = state[(int32_T)xyInd[0] - 1];
    idxInBounds = (originIdx_tmp >= obj->MapBounds[0]);
    h = (originIdx_tmp <= obj->MapBounds[2]);
    f_tmp = state[(int32_T)xyInd[1] - 1];
    f = (f_tmp >= obj->MapBounds[1]);
    e = (f_tmp <= obj->MapBounds[3]);
    ssUpperBounds[0] = f_tmp - obj->MapBounds[1];
    ssUpperBounds[1] = originIdx_tmp - obj->MapBounds[0];
    for (b_k = 0; b_k < 2; b_k++) {
      originIdx_tmp = ceil(ssUpperBounds[b_k]);
      ssUpperBounds[b_k] = originIdx_tmp;
      originIdx[b_k] = (fabs(originIdx_tmp) < 2.2204460492503131E-16);
    }

    if (MATLAB0_vectorAny(originIdx, &tmp)) {
      trueCount = 0;
      for (i = 0; i < 2; i++) {
        if (originIdx[i]) {
          trueCount++;
          ssUpperBounds[i] = 1.0;
        }
      }
    }

    ssUpperBounds[0] = 651.0 - ssUpperBounds[0];
    if (idxInBounds && h && f && e) {
      idxInBounds = !obj->ValidMatrix[(int32_T)((ssUpperBounds[1] - 1.0) * 650.0
        + ssUpperBounds[0]) - 1];
    } else {
      idxInBounds = false;
    }

    isValid = (isInBounds && idxInBounds);
  } else {
    state_0[0] = state[(int32_T)xyInd[0] - 1];
    state_0[1] = state[(int32_T)xyInd[1] - 1];
    binaryOccupancyMap_checkOccupan(obj->Map, state_0, &isOccupied, &idxInBounds);
    isValid = (isInBounds && idxInBounds && (!(isOccupied != 0.0)));
  }

  return isValid;
}

static void MATLAB0_emxInit_cell_wrap_53(emxArray_cell_wrap_53_MATLAB0_T
  **pEmxArray, int32_T numDimensions)
{
  emxArray_cell_wrap_53_MATLAB0_T *emxArray;
  int32_T i;
  *pEmxArray = (emxArray_cell_wrap_53_MATLAB0_T *)malloc(sizeof
    (emxArray_cell_wrap_53_MATLAB0_T));
  emxArray = *pEmxArray;
  emxArray->data = (cell_wrap_53_MATLAB0_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int32_T *)malloc(sizeof(int32_T) * (uint32_T)numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

static void emxInitStruct_reedsSheppConnect(reedsSheppConnection_MATLAB0_T
  *pStruct)
{
  MATLAB0_emxInit_cell_wrap_53(&pStruct->DisabledPathTypesInternal, 2);
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static void plannerHybridAStar_validateStar(plannerHybridAStar_MATLAB0_T *obj,
  const real_T start[3], const real_T goal[3])
{
  static binaryOccupancyMap_MATLAB0_T *b_map;
  static binaryOccupancyMap_MATLAB0_T *map;
  static validatorOccupancyMap_MATLAB0_T *b_validator;
  static validatorOccupancyMap_MATLAB0_T *validator;
  static real_T b_isOccupied;
  static real_T isOccupied;
  boolean_T isValid;
  validator = obj->StateValidator;
  isValid = validatorOccupancyMap_isStateVa(validator, start);
  if (!isValid) {
    map = validator->Map;
    binaryOccupancyMap_checkOccupan(map, &start[0], &isOccupied, &isValid);
  }

  b_validator = obj->StateValidator;
  isValid = validatorOccupancyMap_isStateVa(b_validator, goal);
  if (!isValid) {
    b_map = b_validator->Map;
    binaryOccupancyMap_checkOccupan(b_map, &goal[0], &b_isOccupied, &isValid);
  }

  obj->StartPose[0] = start[0];
  obj->StartPose[1] = start[1];
  obj->StartPose[2] = start[2];
  obj->GoalPose[0] = goal[0];
  obj->GoalPose[1] = goal[1];
  obj->GoalPose[2] = goal[2];
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static c_robotics_core_internal_Name_T *NameValueParser_NameValueParser
  (c_robotics_core_internal_Name_T *obj)
{
  b_cell_wrap_19_MATLAB0_T b;
  c_robotics_core_internal_Name_T *b_obj;
  int32_T i;
  static const char_T c[6] = { 'g', 'r', 'e', 'e', 'd', 'y' };

  b_obj = obj;
  for (i = 0; i < 6; i++) {
    b.f1[i] = c[i];
  }

  obj->Defaults = b;
  return b_obj;
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static void MATLAB0_validatestring(const char_T str[6], char_T out_data[],
  int32_T out_size[2])
{
  int32_T kstr;
  int32_T nmatched;
  boolean_T b_bool;
  static const char_T b[128] = { '\x00', '\x01', '\x02', '\x03', '\x04', '\x05',
    '\x06', '\a', '\b', '\t', '\n', '\v', '\f', '\r', '\x0e', '\x0f', '\x10',
    '\x11', '\x12', '\x13', '\x14', '\x15', '\x16', '\x17', '\x18', '\x19',
    '\x1a', '\x1b', '\x1c', '\x1d', '\x1e', '\x1f', ' ', '!', '\"', '#', '$',
    '%', '&', '\'', '(', ')', '*', '+', ',', '-', '.', '/', '0', '1', '2', '3',
    '4', '5', '6', '7', '8', '9', ':', ';', '<', '=', '>', '?', '@', 'a', 'b',
    'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n', 'o', 'p', 'q',
    'r', 's', 't', 'u', 'v', 'w', 'x', 'y', 'z', '[', '\\', ']', '^', '_', '`',
    'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n', 'o',
    'p', 'q', 'r', 's', 't', 'u', 'v', 'w', 'x', 'y', 'z', '{', '|', '}', '~',
    '\x7f' };

  static const char_T c[6] = { 'g', 'r', 'e', 'e', 'd', 'y' };

  static const char_T d[10] = { 'e', 'x', 'h', 'a', 'u', 's', 't', 'i', 'v', 'e'
  };

  int32_T exitg1;
  nmatched = 0;
  b_bool = false;
  kstr = 0;
  do {
    exitg1 = 0;
    if (kstr < 6) {
      if (b[(uint8_T)str[kstr] & 127] != b[(int32_T)c[kstr]]) {
        exitg1 = 1;
      } else {
        kstr++;
      }
    } else {
      b_bool = true;
      exitg1 = 1;
    }
  } while (exitg1 == 0);

  if (b_bool) {
    nmatched = 1;
    out_size[0] = 1;
    out_size[1] = 6;
    for (kstr = 0; kstr < 6; kstr++) {
      out_data[kstr] = c[kstr];
    }
  } else {
    b_bool = false;
    kstr = 0;
    do {
      exitg1 = 0;
      if (kstr < 6) {
        if (b[(uint8_T)str[kstr] & 127] != b[(int32_T)d[kstr]]) {
          exitg1 = 1;
        } else {
          kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);

    if (b_bool) {
      out_size[0] = 1;
      out_size[1] = 10;
      for (kstr = 0; kstr < 10; kstr++) {
        out_data[kstr] = d[kstr];
      }

      nmatched = 1;
    } else {
      out_size[0] = 1;
      out_size[1] = 6;
      for (kstr = 0; kstr < 6; kstr++) {
        out_data[kstr] = ' ';
      }
    }
  }

  if (nmatched == 0) {
    out_size[0] = 1;
    out_size[1] = 6;
    for (kstr = 0; kstr < 6; kstr++) {
      out_data[kstr] = ' ';
    }
  }
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static boolean_T MATLAB0_ifWhileCond(const boolean_T x[3])
{
  int32_T k;
  boolean_T exitg1;
  boolean_T y;
  y = true;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 3)) {
    if (!x[k]) {
      y = false;
      exitg1 = true;
    } else {
      k++;
    }
  }

  return y;
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static void MATLAB0_repelem(const real_T x[6], real_T varargin_1,
  emxArray_real_T_MATLAB0_T *y)
{
  int32_T colIdx;
  int32_T j;
  int32_T k;
  int32_T tmp;
  tmp = (int32_T)varargin_1;
  j = y->size[0] * y->size[1];
  y->size[0] = (int32_T)varargin_1;
  y->size[1] = 6;
  MATLAB_emxEnsureCapacity_real_T(y, j);
  if ((int32_T)varargin_1 != 0) {
    colIdx = -1;
    for (j = 0; j < 6; j++) {
      colIdx++;
      for (k = 0; k < tmp; k++) {
        y->data[k + y->size[0] * colIdx] = x[j];
      }
    }
  }
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static boolean_T MATLAB0_any(const emxArray_boolean_T_MATLAB0_T *x)
{
  int32_T ix;
  boolean_T exitg1;
  boolean_T y;
  y = false;
  ix = 1;
  exitg1 = false;
  while ((!exitg1) && (ix <= x->size[0])) {
    if (x->data[ix - 1]) {
      y = true;
      exitg1 = true;
    } else {
      ix++;
    }
  }

  return y;
}

static void MATLAB0_binary_expand_op_23(emxArray_boolean_T_MATLAB0_T *in1, const
  emxArray_real_T_MATLAB0_T *in2, const emxArray_real_T_MATLAB0_T *in3)
{
  int32_T i;
  int32_T loop_ub;
  int32_T stride_0_0;
  int32_T stride_1_0;
  loop_ub = in3->size[0] == 1 ? in2->size[0] : in3->size[0];
  i = in1->size[0];
  in1->size[0] = loop_ub;
  MAT_emxEnsureCapacity_boolean_T(in1, i);
  stride_0_0 = (in2->size[0] != 1);
  stride_1_0 = (in3->size[0] != 1);
  for (i = 0; i < loop_ub; i++) {
    in1->data[i] = ((in2->data[i * stride_0_0] == 0.0) && (in3->data[i *
      stride_1_0] > 0.0));
  }
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static void MATLAB0_wrapToPi_o(emxArray_real_T_MATLAB0_T *theta)
{
  static emxArray_boolean_T_MATLAB0_T *tmp;
  static emxArray_boolean_T_MATLAB0_T *y_0;
  static emxArray_real_T_MATLAB0_T *y;
  int32_T k;
  int32_T nx;
  nx = theta->size[0];
  MATLAB0_emxInit_real_T(&y, 1);
  k = y->size[0];
  y->size[0] = theta->size[0];
  MATLAB_emxEnsureCapacity_real_T(y, k);
  for (k = 0; k < nx; k++) {
    y->data[k] = fabs(theta->data[k]);
  }

  MATLAB0_emxInit_boolean_T(&y_0, 1);
  k = y_0->size[0];
  y_0->size[0] = y->size[0];
  MAT_emxEnsureCapacity_boolean_T(y_0, k);
  nx = y->size[0];
  for (k = 0; k < nx; k++) {
    y_0->data[k] = (y->data[k] > 3.1415926535897931);
  }

  if (MATLAB0_any(y_0)) {
    k = y->size[0];
    y->size[0] = theta->size[0];
    MATLAB_emxEnsureCapacity_real_T(y, k);
    nx = theta->size[0];
    for (k = 0; k < nx; k++) {
      y->data[k] = theta->data[k] + 3.1415926535897931;
    }

    k = theta->size[0];
    theta->size[0] = y->size[0];
    MATLAB_emxEnsureCapacity_real_T(theta, k);
    nx = y->size[0];
    for (k = 0; k < nx; k++) {
      theta->data[k] = MATLAB0_mod(y->data[k], 6.2831853071795862);
    }

    MATLAB0_emxInit_boolean_T(&tmp, 1);
    if (theta->size[0] == y->size[0]) {
      k = tmp->size[0];
      tmp->size[0] = theta->size[0];
      MAT_emxEnsureCapacity_boolean_T(tmp, k);
      nx = theta->size[0];
      for (k = 0; k < nx; k++) {
        tmp->data[k] = ((theta->data[k] == 0.0) && (y->data[k] > 0.0));
      }
    } else {
      MATLAB0_binary_expand_op_23(tmp, theta, y);
    }

    nx = tmp->size[0] - 1;
    for (k = 0; k <= nx; k++) {
      if (tmp->data[k]) {
        theta->data[k] = 6.2831853071795862;
      }
    }

    MATLAB0_emxFree_boolean_T(&tmp);
    nx = theta->size[0];
    for (k = 0; k < nx; k++) {
      theta->data[k] -= 3.1415926535897931;
    }
  }

  MATLAB0_emxFree_boolean_T(&y_0);
  MATLAB0_emxFree_real_T(&y);
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static void MATLAB0_expand_max(const emxArray_real_T_MATLAB0_T *a, const
  emxArray_real_T_MATLAB0_T *b, emxArray_real_T_MATLAB0_T *c)
{
  static real_T u0;
  static real_T u1;
  int32_T b_k;
  int32_T csz_idx_0;
  int32_T f;
  boolean_T d;
  boolean_T e;
  if (b->size[0] == 1) {
    csz_idx_0 = a->size[0];
  } else if (a->size[0] == 1) {
    csz_idx_0 = b->size[0];
  } else if (a->size[0] <= b->size[0]) {
    csz_idx_0 = a->size[0];
  } else {
    csz_idx_0 = b->size[0];
  }

  b_k = c->size[0] * c->size[1];
  c->size[0] = csz_idx_0;
  c->size[1] = 3;
  MATLAB_emxEnsureCapacity_real_T(c, b_k);
  if (csz_idx_0 != 0) {
    d = (a->size[0] != 1);
    e = (b->size[0] != 1);
    for (csz_idx_0 = 0; csz_idx_0 < 3; csz_idx_0++) {
      f = c->size[0] - 1;
      for (b_k = 0; b_k <= f; b_k++) {
        u0 = a->data[d * b_k + a->size[0] * csz_idx_0];
        u1 = b->data[e * b_k + b->size[0] * csz_idx_0];
        if ((u0 >= u1) || rtIsNaN(u1)) {
          c->data[b_k + c->size[0] * csz_idx_0] = u0;
        } else {
          c->data[b_k + c->size[0] * csz_idx_0] = u1;
        }
      }
    }
  }
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static void MATLAB0_maximum2(const emxArray_real_T_MATLAB0_T *x, const
  emxArray_real_T_MATLAB0_T *y, emxArray_real_T_MATLAB0_T *ex)
{
  static real_T varargin_1;
  static real_T varargin_2;
  int32_T i;
  int32_T loop_ub;
  if (x->size[0] == y->size[0]) {
    i = ex->size[0] * ex->size[1];
    ex->size[0] = x->size[0];
    ex->size[1] = 3;
    MATLAB_emxEnsureCapacity_real_T(ex, i);
    loop_ub = x->size[0] * 3;
    for (i = 0; i < loop_ub; i++) {
      varargin_1 = x->data[i];
      varargin_2 = y->data[i];
      if ((varargin_1 >= varargin_2) || rtIsNaN(varargin_2)) {
        ex->data[i] = varargin_1;
      } else {
        ex->data[i] = varargin_2;
      }
    }
  } else {
    MATLAB0_expand_max(x, y, ex);
  }
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static void MATLAB0_expand_min(const emxArray_real_T_MATLAB0_T *a, const
  emxArray_real_T_MATLAB0_T *b, emxArray_real_T_MATLAB0_T *c)
{
  static real_T u0;
  static real_T u1;
  int32_T b_k;
  int32_T csz_idx_0;
  int32_T f;
  boolean_T d;
  boolean_T e;
  if (b->size[0] == 1) {
    csz_idx_0 = a->size[0];
  } else if (a->size[0] == 1) {
    csz_idx_0 = b->size[0];
  } else if (a->size[0] <= b->size[0]) {
    csz_idx_0 = a->size[0];
  } else {
    csz_idx_0 = b->size[0];
  }

  b_k = c->size[0] * c->size[1];
  c->size[0] = csz_idx_0;
  c->size[1] = 3;
  MATLAB_emxEnsureCapacity_real_T(c, b_k);
  if (csz_idx_0 != 0) {
    d = (a->size[0] != 1);
    e = (b->size[0] != 1);
    for (csz_idx_0 = 0; csz_idx_0 < 3; csz_idx_0++) {
      f = c->size[0] - 1;
      for (b_k = 0; b_k <= f; b_k++) {
        u0 = a->data[d * b_k + a->size[0] * csz_idx_0];
        u1 = b->data[e * b_k + b->size[0] * csz_idx_0];
        if ((u0 <= u1) || rtIsNaN(u1)) {
          c->data[b_k + c->size[0] * csz_idx_0] = u0;
        } else {
          c->data[b_k + c->size[0] * csz_idx_0] = u1;
        }
      }
    }
  }
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static void MATLAB0_minimum2(const emxArray_real_T_MATLAB0_T *x, const
  emxArray_real_T_MATLAB0_T *y, emxArray_real_T_MATLAB0_T *ex)
{
  static real_T varargin_1;
  static real_T varargin_2;
  int32_T i;
  int32_T loop_ub;
  if (x->size[0] == y->size[0]) {
    i = ex->size[0] * ex->size[1];
    ex->size[0] = x->size[0];
    ex->size[1] = 3;
    MATLAB_emxEnsureCapacity_real_T(ex, i);
    loop_ub = x->size[0] * 3;
    for (i = 0; i < loop_ub; i++) {
      varargin_1 = x->data[i];
      varargin_2 = y->data[i];
      if ((varargin_1 <= varargin_2) || rtIsNaN(varargin_2)) {
        ex->data[i] = varargin_1;
      } else {
        ex->data[i] = varargin_2;
      }
    }
  } else {
    MATLAB0_expand_min(x, y, ex);
  }
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static void stateSpaceSE2_enforceStateBound(const stateSpaceSE2_MATLAB0_T *obj,
  const emxArray_real_T_MATLAB0_T *state, emxArray_real_T_MATLAB0_T
  *boundedState)
{
  static emxArray_real_T_MATLAB0_T *b;
  static emxArray_real_T_MATLAB0_T *bounds;
  static emxArray_real_T_MATLAB0_T *bounds_0;
  static emxArray_real_T_MATLAB0_T *bounds_1;
  static emxArray_real_T_MATLAB0_T *tmp;
  static emxArray_real_T_MATLAB0_T *x;
  int32_T i;
  int32_T loop_ub;
  int32_T x_0;
  MATLAB0_emxInit_real_T(&x, 2);
  i = x->size[0] * x->size[1];
  x->size[0] = state->size[0];
  x->size[1] = 3;
  MATLAB_emxEnsureCapacity_real_T(x, i);
  loop_ub = state->size[0] * 3;
  for (i = 0; i < loop_ub; i++) {
    x->data[i] = state->data[i];
  }

  MATLAB0_emxInit_real_T(&bounds, 2);
  MATLAB0_repelem(obj->StateBoundsInternal, (real_T)state->size[0], bounds);
  MATLAB0_emxInit_real_T(&b, 1);
  i = b->size[0];
  b->size[0] = state->size[0];
  MATLAB_emxEnsureCapacity_real_T(b, i);
  loop_ub = state->size[0];
  for (i = 0; i < loop_ub; i++) {
    b->data[i] = state->data[(state->size[0] << 1) + i];
  }

  MATLAB0_wrapToPi_o(b);
  loop_ub = b->size[0];
  for (i = 0; i < loop_ub; i++) {
    x->data[i + (x->size[0] << 1)] = b->data[i];
  }

  MATLAB0_emxFree_real_T(&b);
  MATLAB0_emxInit_real_T(&bounds_0, 2);
  i = bounds_0->size[0] * bounds_0->size[1];
  bounds_0->size[0] = bounds->size[0];
  bounds_0->size[1] = 3;
  MATLAB_emxEnsureCapacity_real_T(bounds_0, i);
  MATLAB0_emxInit_real_T(&bounds_1, 2);
  i = bounds_1->size[0] * bounds_1->size[1];
  bounds_1->size[0] = bounds->size[0];
  bounds_1->size[1] = 3;
  MATLAB_emxEnsureCapacity_real_T(bounds_1, i);
  loop_ub = bounds->size[0];
  for (i = 0; i < 3; i++) {
    for (x_0 = 0; x_0 < loop_ub; x_0++) {
      bounds_0->data[x_0 + bounds_0->size[0] * i] = bounds->data[bounds->size[0]
        * i + x_0];
      bounds_1->data[x_0 + bounds_1->size[0] * i] = bounds->data[(i + 3) *
        bounds->size[0] + x_0];
    }
  }

  MATLAB0_emxFree_real_T(&bounds);
  MATLAB0_emxInit_real_T(&tmp, 2);
  MATLAB0_maximum2(x, bounds_0, tmp);
  MATLAB0_emxFree_real_T(&bounds_0);
  MATLAB0_emxFree_real_T(&x);
  MATLAB0_minimum2(tmp, bounds_1, boundedState);
  MATLAB0_emxFree_real_T(&tmp);
  MATLAB0_emxFree_real_T(&bounds_1);
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static navPath_MATLAB0_T *MATLAB0_navPath_navPath(navPath_MATLAB0_T *obj,
  stateSpaceSE2_MATLAB0_T *stateSpace, const emxArray_real_T_MATLAB0_T *states)
{
  static emxArray_real_T_MATLAB0_T *statesInternal;
  static int32_T loop_ub;
  navPath_MATLAB0_T *b_obj;
  int32_T i;
  b_obj = obj;
  obj->StateSpace = stateSpace;
  MATLAB0_emxInit_real_T(&statesInternal, 2);
  stateSpaceSE2_enforceStateBound(obj->StateSpace, states, statesInternal);
  i = obj->StateInternal->size[0] * obj->StateInternal->size[1];
  obj->StateInternal->size[0] = statesInternal->size[0];
  obj->StateInternal->size[1] = 3;
  MATLAB_emxEnsureCapacity_real_T(obj->StateInternal, i);
  loop_ub = statesInternal->size[0] * 3;
  for (i = 0; i < loop_ub; i++) {
    obj->StateInternal->data[i] = statesInternal->data[i];
  }

  obj->NumStates = statesInternal->size[0];
  MATLAB0_emxFree_real_T(&statesInternal);
  return b_obj;
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static void MATLAB0_ind2sub(const real_T ndx[422500], int32_T varargout_1[422500],
  int32_T varargout_2[422500], int32_T varargout_3[422500])
{
  int32_T i;
  int32_T varargout_1_0;
  int32_T vk;
  for (i = 0; i < 422500; i++) {
    varargout_1_0 = (int32_T)ndx[i] - 1;
    vk = varargout_1_0 / 422500;
    varargout_3[i] = vk + 1;
    varargout_1_0 -= vk * 422500;
    vk = varargout_1_0 / 650;
    varargout_2[i] = vk + 1;
    varargout_1[i] = (varargout_1_0 - vk * 650) + 1;
  }
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static void MATLAB_plannerAStarGrid_set_Map(plannerAStarGrid_MATLAB0_T *obj,
  binaryOccupancyMap_MATLAB0_T *input)
{
  static real_T tmp[422500];
  static int32_T varargout_4[422500];
  static int32_T varargout_5[422500];
  static int32_T varargout_6[422500];
  int32_T i;
  obj->Map = input;
  for (i = 0; i < 1267500; i++) {
    obj->IdPose[i] = 0.0;
  }

  for (i = 0; i < 422500; i++) {
    tmp[i] = (real_T)i + 1.0;
  }

  MATLAB0_ind2sub(tmp, varargout_4, varargout_5, varargout_6);
  for (i = 0; i < 422500; i++) {
    obj->IdPose[i] = varargout_4[i];
  }

  for (i = 0; i < 422500; i++) {
    obj->IdPose[i + 422500] = varargout_5[i];
  }

  for (i = 0; i < 422500; i++) {
    obj->IdPose[i + 845000] = varargout_6[i];
  }
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static void validateAStarBuiltinCostFunctio(char_T strVal_data[], int32_T
  strVal_size[2], real_T *idx)
{
  static int8_T ii_data[4];
  int32_T b_idx;
  int32_T b_ii;
  boolean_T x[4];
  static const char_T a[9] = { 'E', 'u', 'c', 'l', 'i', 'd', 'e', 'a', 'n' };

  static const char_T d[9] = { 'M', 'a', 'n', 'h', 'a', 't', 't', 'a', 'n' };

  static const char_T e[9] = { 'C', 'h', 'e', 'b', 'y', 's', 'h', 'e', 'v' };

  int32_T exitg1;
  boolean_T exitg2;
  strVal_size[0] = 1;
  strVal_size[1] = 9;
  for (b_idx = 0; b_idx < 9; b_idx++) {
    strVal_data[b_idx] = a[b_idx];
  }

  x[0] = true;
  x[1] = false;
  b_idx = 0;
  do {
    exitg1 = 0;
    if (b_idx < 9) {
      if (d[b_idx] != a[b_idx]) {
        exitg1 = 1;
      } else {
        b_idx++;
      }
    } else {
      x[1] = true;
      exitg1 = 1;
    }
  } while (exitg1 == 0);

  x[2] = false;
  b_idx = 0;
  do {
    exitg1 = 0;
    if (b_idx < 9) {
      if (e[b_idx] != a[b_idx]) {
        exitg1 = 1;
      } else {
        b_idx++;
      }
    } else {
      x[2] = true;
      exitg1 = 1;
    }
  } while (exitg1 == 0);

  x[3] = false;
  b_idx = 0;
  b_ii = 0;
  exitg2 = false;
  while ((!exitg2) && (b_ii < 4)) {
    if (x[b_ii]) {
      b_idx++;
      ii_data[b_idx - 1] = (int8_T)(b_ii + 1);
      if (b_idx >= 4) {
        exitg2 = true;
      } else {
        b_ii++;
      }
    } else {
      b_ii++;
    }
  }

  *idx = ii_data[0];
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static plannerAStarGrid_MATLAB0_T *plannerAStarGrid_plannerAStarGr
  (plannerAStarGrid_MATLAB0_T *obj, binaryOccupancyMap_MATLAB0_T *varargin_1)
{
  static real_T b;
  static real_T c;
  static int32_T a__2_size[2];
  static char_T a__2_data[9];
  plannerAStarGrid_MATLAB0_T *b_obj;
  b_obj = obj;
  obj->isFirstRun = 1.0;
  MATLAB_plannerAStarGrid_set_Map(obj, varargin_1);
  obj->UseCustomG = 1.0;
  obj->UseCustomG = 0.0;
  validateAStarBuiltinCostFunctio(a__2_data, a__2_size, &b);
  obj->GCost = b;
  obj->UseCustomG = 0.0;
  obj->UseCustomH = 1.0;
  obj->UseCustomH = 0.0;
  validateAStarBuiltinCostFunctio(a__2_data, a__2_size, &c);
  obj->HCost = c;
  obj->UseCustomH = 0.0;
  obj->OccupiedThreshold = 0.65;
  obj->TieBreaker = 0.0;
  obj->DiagonalSearch = 1.0;
  obj->isFirstRun = 0.0;
  return b_obj;
}

static void MATLAB0_emxFree_char_T(emxArray_char_T_MATLAB0_T **pEmxArray)
{
  if (*pEmxArray != (emxArray_char_T_MATLAB0_T *)NULL) {
    if (((*pEmxArray)->data != (char_T *)NULL) && (*pEmxArray)->canFreeData) {
      free((*pEmxArray)->data);
    }

    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = (emxArray_char_T_MATLAB0_T *)NULL;
  }
}

static void MATL_emxFreeStruct_cell_wrap_53(cell_wrap_53_MATLAB0_T *pStruct)
{
  MATLAB0_emxFree_char_T(&pStruct->f1);
}

static void MATLAB0_emxFree_cell_wrap_53(emxArray_cell_wrap_53_MATLAB0_T
  **pEmxArray)
{
  int32_T i;
  int32_T numEl;
  if (*pEmxArray != (emxArray_cell_wrap_53_MATLAB0_T *)NULL) {
    if ((*pEmxArray)->data != (cell_wrap_53_MATLAB0_T *)NULL) {
      numEl = 1;
      for (i = 0; i < (*pEmxArray)->numDimensions; i++) {
        numEl *= (*pEmxArray)->size[i];
      }

      for (i = 0; i < numEl; i++) {
        MATL_emxFreeStruct_cell_wrap_53(&(*pEmxArray)->data[i]);
      }

      if ((*pEmxArray)->canFreeData) {
        free((*pEmxArray)->data);
      }
    }

    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = (emxArray_cell_wrap_53_MATLAB0_T *)NULL;
  }
}

static void emxFreeStruct_reedsSheppConnect(reedsSheppConnection_MATLAB0_T
  *pStruct)
{
  MATLAB0_emxFree_cell_wrap_53(&pStruct->DisabledPathTypesInternal);
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static void MATLAB0_ind2sub_a(const real_T ndx[422500], int32_T varargout_1
  [422500], int32_T varargout_2[422500])
{
  int32_T i;
  int32_T varargout_1_0;
  int32_T vk;
  for (i = 0; i < 422500; i++) {
    varargout_1_0 = (int32_T)ndx[i] - 1;
    vk = varargout_1_0 / 650;
    varargout_2[i] = vk + 1;
    varargout_1[i] = (varargout_1_0 - vk * 650) + 1;
  }
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static c_nav_algs_internal_plannerAS_T *plannerAStarGrid_plannerAStar_e
  (c_nav_algs_internal_plannerAS_T *obj, const real_T map[422500], real_T
   obstacleThreshold)
{
  static real_T tmp[422500];
  static int32_T varargout_3[422500];
  static int32_T varargout_4[422500];
  c_nav_algs_internal_plannerAS_T *b_obj;
  int32_T i;
  b_obj = obj;
  obj->NumPathPoints = 0.0;
  obj->UseCustomG = 0.0;
  obj->UseCustomH = 0.0;
  obj->GCostMethod = 1.0;
  obj->HCostMethod = 1.0;
  for (i = 0; i < 422500; i++) {
    obj->Map[i] = map[i];
  }

  for (i = 0; i < 845000; i++) {
    obj->PathInternal[i] = 0.0;
  }

  obj->ObstacleThreshold = obstacleThreshold;
  obj->MapResolution = 1.0;
  for (i = 0; i < 422500; i++) {
    obj->NodesExploredIndicesInternal[i] = 0.0;
    obj->ParentCol[i] = 0.0;
    obj->ParentRow[i] = 0.0;
    obj->MapIndex[i] = (real_T)i + 1.0;
    tmp[i] = (real_T)i + 1.0;
  }

  MATLAB0_ind2sub_a(tmp, varargout_3, varargout_4);
  for (i = 0; i < 422500; i++) {
    obj->AllNodes[i] = varargout_3[i];
    obj->AllNodes[i + 422500] = varargout_4[i];
  }

  return b_obj;
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static c_nav_algs_internal_plannerAS_T *plannerAStarGrid_initializeInte
  (plannerAStarGrid_MATLAB0_T *obj, c_nav_algs_internal_plannerAS_T *iobj_0)
{
  static binaryOccupancyMap_MATLAB0_T *val;
  static real_T mat_0[422500];
  static real_T b_distMethodVal;
  static real_T d_val;
  static real_T distMethodVal;
  static real_T e_val;
  static real_T th;
  static boolean_T mat[422500];
  c_nav_algs_internal_plannerAS_T *astarInternal;
  int32_T i;
  obj->OccupiedThreshold = 0.65;
  val = obj->Map;
  MATLAB_MapLayer_getValueAllImpl(val, mat);
  th = obj->OccupiedThreshold;
  for (i = 0; i < 422500; i++) {
    mat_0[i] = (real_T)mat[i] * 10000.0 / 10000.0;
  }

  astarInternal = plannerAStarGrid_plannerAStar_e(iobj_0, mat_0, th);
  if (obj->UseCustomH == 0.0) {
    distMethodVal = obj->HCost;
    astarInternal->HCostMethod = distMethodVal;
    astarInternal->UseCustomH = 0.0;
  }

  if (obj->UseCustomG == 0.0) {
    b_distMethodVal = obj->GCost;
    astarInternal->GCostMethod = b_distMethodVal;
    astarInternal->UseCustomG = 0.0;
  }

  if (obj->TieBreaker != 0.0) {
    d_val = 1.07;
  } else {
    d_val = 1.0;
  }

  astarInternal->TieBreaker = d_val;
  e_val = obj->DiagonalSearch;
  astarInternal->DiagonalSearchFlag = e_val;
  return astarInternal;
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static void MATLAB0_sum(const real_T x_data[], const int32_T x_size[2], real_T
  y_data[], int32_T *y_size)
{
  int32_T b;
  int32_T xj;
  if (x_size[0] == 0) {
    *y_size = 0;
  } else {
    *y_size = x_size[0];
    b = x_size[0];
    for (xj = 0; xj < b; xj++) {
      y_data[xj] = x_data[x_size[0] + xj] + x_data[xj];
    }
  }
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static real_T MATLAB0_minimum(const real_T x_data[], const int32_T *x_size)
{
  static real_T b_ex;
  static real_T x;
  real_T ex;
  int32_T idx;
  int32_T k;
  int32_T last;
  boolean_T exitg1;
  last = *x_size;
  if ((uint8_T)(*x_size - 1) + 1 <= 2) {
    if ((uint8_T)(*x_size - 1) + 1 == 1) {
      ex = x_data[0];
    } else {
      ex = x_data[*x_size - 1];
      if ((x_data[0] > ex) || (rtIsNaN(x_data[0]) && (!rtIsNaN(ex)))) {
      } else {
        ex = x_data[0];
      }
    }
  } else {
    if (!rtIsNaN(x_data[0])) {
      idx = 1;
    } else {
      idx = 0;
      k = 2;
      exitg1 = false;
      while ((!exitg1) && (k <= *x_size)) {
        if (!rtIsNaN(x_data[k - 1])) {
          idx = k;
          exitg1 = true;
        } else {
          k++;
        }
      }
    }

    if (idx == 0) {
      ex = x_data[0];
    } else {
      b_ex = x_data[idx - 1];
      for (k = idx + 1; k <= last; k++) {
        x = x_data[k - 1];
        if (b_ex > x) {
          b_ex = x;
        }
      }

      ex = b_ex;
    }
  }

  return ex;
}

static real_T MATLAB0_binary_expand_op(const real_T in1_data[], const int32_T
  *in1_size, const real_T in2_data[], const int32_T *in2_size)
{
  static real_T in1_data_0[9];
  real_T out1;
  int32_T i;
  int32_T loop_ub;
  int32_T stride_0_0;
  int32_T stride_1_0;
  loop_ub = *in2_size == 1 ? *in1_size : *in2_size;
  stride_0_0 = (*in1_size != 1);
  stride_1_0 = (*in2_size != 1);
  for (i = 0; i < loop_ub; i++) {
    in1_data_0[i] = in1_data[i * stride_0_0] + in2_data[i * stride_1_0];
  }

  out1 = MATLAB0_minimum(in1_data_0, &loop_ub);
  return out1;
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static void MATL_plannerAStarGrid_Chebyshev(const real_T pose1[845000], const
  real_T pose2[845000], real_T dist[422500])
{
  static real_T difference[845000];
  real_T difference_0;
  real_T difference_1;
  real_T minval;
  int32_T c_k;
  for (c_k = 0; c_k < 845000; c_k++) {
    difference[c_k] = fabs(pose1[c_k] - pose2[c_k]);
  }

  for (c_k = 0; c_k < 422500; c_k++) {
    difference_0 = difference[c_k];
    difference_1 = difference[c_k + 422500];
    if ((difference_0 <= difference_1) || rtIsNaN(difference_1)) {
      minval = difference_0;
    } else {
      minval = difference_1;
    }

    dist[c_k] = (difference_1 + difference_0) - minval;
  }
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static void M_plannerAStarGrid_getNeighbors(const
  c_nav_algs_internal_plannerAS_T *obj, real_T Neighbors_data[], int32_T
  Neighbors_size[2], real_T *NumNeighbors)
{
  int32_T i;
  static const int8_T b_Neighbors[16] = { -1, 0, 1, -1, 1, -1, 0, 1, -1, -1, -1,
    0, 0, 1, 1, 1 };

  static const int8_T c_Neighbors[8] = { 0, -1, 1, 0, -1, 0, 0, 1 };

  if ((int32_T)obj->DiagonalSearchFlag == 0) {
    Neighbors_size[0] = 4;
    Neighbors_size[1] = 2;
    for (i = 0; i < 8; i++) {
      Neighbors_data[i] = c_Neighbors[i];
    }

    *NumNeighbors = 4.0;
  } else {
    Neighbors_size[0] = 8;
    Neighbors_size[1] = 2;
    for (i = 0; i < 16; i++) {
      Neighbors_data[i] = b_Neighbors[i];
    }

    *NumNeighbors = 8.0;
  }
}

static void MATLAB0_emxInit_int32_T(emxArray_int32_T_MATLAB0_T **pEmxArray,
  int32_T numDimensions)
{
  emxArray_int32_T_MATLAB0_T *emxArray;
  int32_T i;
  *pEmxArray = (emxArray_int32_T_MATLAB0_T *)malloc(sizeof
    (emxArray_int32_T_MATLAB0_T));
  emxArray = *pEmxArray;
  emxArray->data = (int32_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int32_T *)malloc(sizeof(int32_T) * (uint32_T)numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static void plannerAStarGrid_reconstructPat(c_nav_algs_internal_plannerAS_T *obj,
  real_T CurrentRow, real_T CurrentCol, const real_T startIn[2])
{
  static emxArray_real_T_MATLAB0_T *b;
  static real_T OptimalPath[845000];
  static real_T k;
  static real_T tmp;
  int32_T CurrentColDummy_tmp;
  int32_T exitg1;
  int32_T j;
  int32_T loop_ub_tmp;
  int32_T nd2;
  int32_T offset;
  int32_T tmp_tmp;
  k = 1.0;
  OptimalPath[0] = CurrentRow;
  OptimalPath[422500] = CurrentCol;
  do {
    exitg1 = 0;
    k++;
    CurrentColDummy_tmp = (((int32_T)CurrentCol - 1) * 650 + (int32_T)CurrentRow)
      - 1;
    CurrentCol = obj->ParentCol[CurrentColDummy_tmp];
    CurrentRow = obj->ParentRow[CurrentColDummy_tmp];
    if ((CurrentRow == 0.0) || (CurrentCol == 0.0)) {
      k--;
      exitg1 = 1;
    } else {
      OptimalPath[(int32_T)k - 1] = CurrentRow;
      OptimalPath[(int32_T)k + 422499] = CurrentCol;
      if ((CurrentCol == startIn[1]) && (CurrentRow == startIn[0])) {
        exitg1 = 1;
      }
    }
  } while (exitg1 == 0);

  obj->NumPathPoints = k;
  MATLAB0_emxInit_real_T(&b, 2);
  loop_ub_tmp = (int32_T)k;
  CurrentColDummy_tmp = b->size[0] * b->size[1];
  b->size[0] = (int32_T)k;
  b->size[1] = 2;
  MATLAB_emxEnsureCapacity_real_T(b, CurrentColDummy_tmp);
  for (CurrentColDummy_tmp = 0; CurrentColDummy_tmp < 2; CurrentColDummy_tmp++)
  {
    for (j = 0; j < loop_ub_tmp; j++) {
      b->data[j + b->size[0] * CurrentColDummy_tmp] = OptimalPath[422500 *
        CurrentColDummy_tmp + j];
    }
  }

  if ((int32_T)k > 1) {
    nd2 = (int32_T)k >> 1;
    for (j = 0; j < 2; j++) {
      offset = j * b->size[0];
      for (loop_ub_tmp = 0; loop_ub_tmp < nd2; loop_ub_tmp++) {
        tmp_tmp = offset + loop_ub_tmp;
        tmp = b->data[tmp_tmp];
        CurrentColDummy_tmp = ((offset + (int32_T)k) - loop_ub_tmp) - 1;
        b->data[tmp_tmp] = b->data[CurrentColDummy_tmp];
        b->data[CurrentColDummy_tmp] = tmp;
      }
    }
  }

  loop_ub_tmp = b->size[0];
  for (CurrentColDummy_tmp = 0; CurrentColDummy_tmp < 2; CurrentColDummy_tmp++)
  {
    for (j = 0; j < loop_ub_tmp; j++) {
      obj->PathInternal[j + 422500 * CurrentColDummy_tmp] = b->data[b->size[0] *
        CurrentColDummy_tmp + j];
    }
  }

  MATLAB0_emxFree_real_T(&b);
}

static void MATLA_emxEnsureCapacity_int32_T(emxArray_int32_T_MATLAB0_T *emxArray,
  int32_T oldNumel)
{
  static void *newData;
  int32_T i;
  int32_T newNumel;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  newNumel = 1;
  for (i = 0; i < emxArray->numDimensions; i++) {
    newNumel *= emxArray->size[i];
  }

  if (newNumel > emxArray->allocatedSize) {
    i = emxArray->allocatedSize;
    if (i < 16) {
      i = 16;
    }

    while (i < newNumel) {
      if (i > 1073741823) {
        i = MAX_int32_T;
      } else {
        i <<= 1;
      }
    }

    newData = malloc((uint32_T)i * sizeof(int32_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(int32_T) * (uint32_T)oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = (int32_T *)newData;
    emxArray->allocatedSize = i;
    emxArray->canFreeData = true;
  }
}

static void MATLAB0_emxFree_int32_T(emxArray_int32_T_MATLAB0_T **pEmxArray)
{
  if (*pEmxArray != (emxArray_int32_T_MATLAB0_T *)NULL) {
    if (((*pEmxArray)->data != (int32_T *)NULL) && (*pEmxArray)->canFreeData) {
      free((*pEmxArray)->data);
    }

    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = (emxArray_int32_T_MATLAB0_T *)NULL;
  }
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static real_T MAT_plannerAStarGrid_gcostValue(const
  c_nav_algs_internal_plannerAS_T *obj, real_T CurrentRow, real_T CurrentCol,
  real_T i, real_T j)
{
  static real_T difference[2];
  static real_T y[2];
  static int32_T c_k;
  real_T difference_0;
  real_T gcostNeighbor;
  if (obj->UseCustomG == 0.0) {
    difference[0] = i;
    difference[1] = j;
    switch ((int32_T)obj->GCostMethod) {
     case 2:
      for (c_k = 0; c_k < 2; c_k++) {
        y[c_k] = fabs(0.0 - difference[c_k]);
      }

      gcostNeighbor = y[0] + y[1];
      break;

     case 3:
      difference[0] = fabs(0.0 - i);
      difference[1] = fabs(0.0 - j);
      if ((difference[0] <= difference[1]) || rtIsNaN(difference[1])) {
        difference_0 = difference[0];
      } else {
        difference_0 = difference[1];
      }

      gcostNeighbor = (difference[0] + difference[1]) - difference_0;
      break;

     case 4:
      gcostNeighbor = -i * -i + -j * -j;
      break;

     default:
      gcostNeighbor = sqrt(-i * -i + -j * -j);
      break;
    }

    gcostNeighbor /= obj->MapResolution;
  } else {
    difference_0 = CurrentRow - (CurrentRow + i);
    y[0] = difference_0 * difference_0;
    difference_0 = CurrentCol - (CurrentCol + j);
    gcostNeighbor = sqrt(difference_0 * difference_0 + y[0]) /
      obj->MapResolution;
  }

  return gcostNeighbor;
}

static real_T MATLAB0_rt_roundd_snf(real_T u)
{
  real_T y;
  if (fabs(u) < 4.503599627370496E+15) {
    if (u >= 0.5) {
      y = floor(u + 0.5);
    } else if (u > -0.5) {
      y = u * 0.0;
    } else {
      y = ceil(u - 0.5);
    }
  } else {
    y = u;
  }

  return y;
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static void MATLAB_plannerAStarGrid_runPlan(c_nav_algs_internal_plannerAS_T
  *astarInternal, const real_T start[2], const real_T goal[2])
{
  static void *OpenList_PQInternal;
  static emxArray_int32_T_MATLAB0_T *ii;
  static real_T c_b[845000];
  static real_T FScore[422500];
  static real_T GScore[422500];
  static real_T ParentCol1[422500];
  static real_T ParentRow2[422500];
  static real_T map[422500];
  static real_T openToPush_data[40];
  static real_T Neighbors_data[16];
  static real_T MinScoreNode_data[5];
  static real_T nodeData[5];
  static real_T CurrentCol_tmp;
  static real_T CurrentRow;
  static real_T CurrentRow_tmp_tmp;
  static real_T JumpCells;
  static real_T dataDim;
  static real_T flag;
  static real_T gcost;
  static real_T goal_0;
  static real_T i;
  static real_T j;
  static real_T nodeId;
  static real_T openToPushK;
  static real_T tentative_gScore;
  static real_T y_idx_0;
  static int32_T Neighbors_size[2];
  static int32_T K;
  static int32_T MinScoreNode_size_idx_0;
  static int32_T b_i;
  static int32_T f;
  static int32_T i_0;
  static int32_T ibmat;
  static int32_T idx;
  static int32_T itilerow;
  static int32_T jcol;
  static int32_T openToPush_size_idx_0;
  static int8_T ClosedMAT[422500];
  static int8_T ExploredNodes[422500];
  static boolean_T x[422500];
  int32_T CurrentRow_tmp_tmp_0;
  int32_T exitg1;
  int32_T tentative_gScore_tmp;
  boolean_T exitg2;
  for (i_0 = 0; i_0 < 422500; i_0++) {
    GScore[i_0] = (rtInf);
    FScore[i_0] = (rtInf);
    ExploredNodes[i_0] = 0;
    ParentCol1[i_0] = 0.0;
    ParentRow2[i_0] = 0.0;
    ClosedMAT[i_0] = 0;
    if (astarInternal->Map[i_0] == 1.0) {
      ClosedMAT[i_0] = 1;
    }
  }

  OpenList_PQInternal = priorityqueuecodegen_constructPQ(5.0, 2.0);
  if (astarInternal->UseCustomH == 0.0) {
    for (jcol = 0; jcol < 2; jcol++) {
      ibmat = jcol * 422500;
      for (itilerow = 0; itilerow < 422500; itilerow++) {
        c_b[ibmat + itilerow] = goal[jcol];
      }
    }

    switch ((int32_T)astarInternal->HCostMethod) {
     case 2:
      for (i_0 = 0; i_0 < 845000; i_0++) {
        c_b[i_0] = fabs(astarInternal->AllNodes[i_0] - c_b[i_0]);
      }

      for (i_0 = 0; i_0 < 422500; i_0++) {
        map[i_0] = c_b[i_0 + 422500] + c_b[i_0];
      }
      break;

     case 3:
      MATL_plannerAStarGrid_Chebyshev(astarInternal->AllNodes, c_b, map);
      break;

     case 4:
      for (i_0 = 0; i_0 < 845000; i_0++) {
        goal_0 = astarInternal->AllNodes[i_0] - c_b[i_0];
        c_b[i_0] = goal_0 * goal_0;
      }

      for (i_0 = 0; i_0 < 422500; i_0++) {
        map[i_0] = c_b[i_0 + 422500] + c_b[i_0];
      }
      break;

     default:
      for (i_0 = 0; i_0 < 845000; i_0++) {
        goal_0 = astarInternal->AllNodes[i_0] - c_b[i_0];
        c_b[i_0] = goal_0 * goal_0;
      }

      for (i_0 = 0; i_0 < 422500; i_0++) {
        map[i_0] = sqrt(c_b[i_0 + 422500] + c_b[i_0]);
      }
      break;
    }

    for (i_0 = 0; i_0 < 422500; i_0++) {
      map[i_0] = map[i_0] / astarInternal->MapResolution *
        astarInternal->TieBreaker;
    }
  } else {
    goal_0 = goal[0];
    dataDim = goal[1];
    for (b_i = 0; b_i < 422500; b_i++) {
      nodeId = astarInternal->AllNodes[b_i] - goal_0;
      y_idx_0 = nodeId * nodeId;
      nodeId = astarInternal->AllNodes[b_i + 422500] - dataDim;
      map[b_i] = sqrt(nodeId * nodeId + y_idx_0) * astarInternal->TieBreaker /
        astarInternal->MapResolution;
    }
  }

  idx = (((int32_T)start[1] - 1) * 650 + (int32_T)start[0]) - 1;
  dataDim = map[idx];
  FScore[idx] = dataDim;
  nodeData[0] = astarInternal->MapIndex[idx];
  nodeData[1] = astarInternal->MapIndex[idx];
  nodeData[2] = FScore[idx];
  nodeData[3] = 0.0;
  nodeData[4] = dataDim;
  priorityqueuecodegen_push(OpenList_PQInternal, &nodeData[0]);
  ExploredNodes[idx] = 1;
  GScore[idx] = 0.0;
  M_plannerAStarGrid_getNeighbors(astarInternal, Neighbors_data, Neighbors_size,
    &goal_0);
  do {
    exitg1 = 0;
    priorityqueuecodegen_getDataDim(OpenList_PQInternal);
    priorityqueuecodegen_top(OpenList_PQInternal, &MinScoreNode_data[0], &nodeId);
    if (MinScoreNode_data[2] == (rtInf)) {
      exitg1 = 1;
    } else {
      flag = priorityqueuecodegen_isEmpty(OpenList_PQInternal);
      if (flag != 0.0) {
        astarInternal->ParentCol[idx] = start[1];
        astarInternal->ParentRow[idx] = start[0];
        exitg1 = 1;
      } else {
        CurrentRow_tmp_tmp = MinScoreNode_data[0];
        CurrentRow = astarInternal->AllNodes[(int32_T)CurrentRow_tmp_tmp - 1];
        CurrentCol_tmp = astarInternal->AllNodes[(int32_T)CurrentRow_tmp_tmp +
          422499];
        if (astarInternal->MapIndex[(((int32_T)goal[1] - 1) * 650 + (int32_T)
             goal[0]) - 1] == CurrentRow_tmp_tmp) {
          for (i_0 = 0; i_0 < 422500; i_0++) {
            astarInternal->ParentCol[i_0] = ParentCol1[i_0];
            astarInternal->ParentRow[i_0] = ParentRow2[i_0];
          }

          plannerAStarGrid_reconstructPat(astarInternal, astarInternal->
            AllNodes[(int32_T)CurrentRow_tmp_tmp - 1], astarInternal->AllNodes
            [(int32_T)CurrentRow_tmp_tmp + 422499], start);
          exitg1 = 1;
        } else {
          priorityqueuecodegen_pop(OpenList_PQInternal);
          ClosedMAT[((int32_T)CurrentRow + 650 * ((int32_T)CurrentCol_tmp - 1))
            - 1] = 1;
          CurrentRow_tmp_tmp_0 = (int32_T)goal_0;
          openToPush_size_idx_0 = (int32_T)goal_0;
          MinScoreNode_size_idx_0 = (int32_T)goal_0 * 5;
          if (MinScoreNode_size_idx_0 - 1 >= 0) {
            memset(&openToPush_data[0], 0, (uint32_T)MinScoreNode_size_idx_0 *
                   sizeof(real_T));
          }

          openToPushK = 1.0;
          for (MinScoreNode_size_idx_0 = 0; MinScoreNode_size_idx_0 <
               CurrentRow_tmp_tmp_0; MinScoreNode_size_idx_0++) {
            i = Neighbors_data[MinScoreNode_size_idx_0];
            j = Neighbors_data[MinScoreNode_size_idx_0 + Neighbors_size[0]];
            dataDim = CurrentRow + i;
            nodeId = CurrentCol_tmp + j;
            if ((dataDim < 1.0) || (dataDim > 650.0) || (nodeId < 1.0) ||
                (nodeId > 650.0)) {
            } else {
              i_0 = (((int32_T)nodeId - 1) * 650 + (int32_T)dataDim) - 1;
              if ((astarInternal->Map[i_0] >= astarInternal->ObstacleThreshold) ||
                  (!(ClosedMAT[i_0] == 0))) {
              } else {
                f = 0;
                dataDim = fabs(i);
                if ((dataDim > 1.0) || (fabs(j) > 1.0)) {
                  JumpCells = fabs(j);
                  if ((dataDim >= JumpCells) || rtIsNaN(JumpCells)) {
                    JumpCells = dataDim;
                  }

                  JumpCells = 2.0 * JumpCells - 1.0;
                  K = 0;
                  exitg2 = false;
                  while ((!exitg2) && (K <= (int32_T)JumpCells - 1)) {
                    if (astarInternal->Map[(((int32_T)(MATLAB0_rt_roundd_snf(j /
                            JumpCells) + CurrentCol_tmp) - 1) * 650 + (int32_T)
                                            (MATLAB0_rt_roundd_snf(i / JumpCells)
                          + CurrentRow)) - 1] == 1.0) {
                      f = 1;
                      exitg2 = true;
                    } else {
                      K++;
                    }
                  }
                }

                if (f == 0) {
                  gcost = MAT_plannerAStarGrid_gcostValue(astarInternal,
                    CurrentRow, CurrentCol_tmp, i, j);
                  tentative_gScore_tmp = (((int32_T)CurrentCol_tmp - 1) * 650 +
                    (int32_T)CurrentRow) - 1;
                  tentative_gScore = GScore[tentative_gScore_tmp] + gcost;
                  if (FScore[i_0] == (rtInf)) {
                    ExploredNodes[i_0] = 1;
                    ParentCol1[i_0] = CurrentCol_tmp;
                    ParentRow2[i_0] = CurrentRow;
                    GScore[i_0] = tentative_gScore;
                    dataDim = map[i_0];
                    FScore[i_0] = dataDim + tentative_gScore;
                    openToPush_data[(int32_T)openToPushK - 1] =
                      astarInternal->MapIndex[i_0];
                    openToPush_data[((int32_T)openToPushK + (int32_T)goal_0) - 1]
                      = astarInternal->MapIndex[tentative_gScore_tmp];
                    openToPush_data[((int32_T)openToPushK + ((int32_T)goal_0 <<
                      1)) - 1] = FScore[i_0];
                    openToPush_data[((int32_T)openToPushK + (int32_T)goal_0 * 3)
                      - 1] = GScore[i_0];
                    openToPush_data[((int32_T)openToPushK + ((int32_T)goal_0 <<
                      2)) - 1] = dataDim;
                    for (i_0 = 0; i_0 < 5; i_0++) {
                      nodeData[i_0] = openToPush_data[((int32_T)goal_0 * i_0 +
                        (int32_T)openToPushK) - 1];
                    }

                    priorityqueuecodegen_push(OpenList_PQInternal, &nodeData[0]);
                    openToPushK++;
                  } else if (!(tentative_gScore >= GScore[i_0])) {
                    ParentCol1[i_0] = CurrentCol_tmp;
                    ParentRow2[i_0] = CurrentRow;
                    GScore[i_0] = tentative_gScore;
                    FScore[i_0] = map[i_0] + tentative_gScore;
                  }
                }
              }
            }
          }
        }
      }
    }
  } while (exitg1 == 0);

  for (i_0 = 0; i_0 < 422500; i_0++) {
    astarInternal->GCostMatrix[i_0] = GScore[i_0];
    x[i_0] = (ExploredNodes[i_0] == 1);
  }

  idx = 0;
  MATLAB0_emxInit_int32_T(&ii, 1);
  i_0 = ii->size[0];
  ii->size[0] = 422500;
  MATLA_emxEnsureCapacity_int32_T(ii, i_0);
  i_0 = 0;
  exitg2 = false;
  while ((!exitg2) && (i_0 < 422500)) {
    if (x[i_0]) {
      idx++;
      ii->data[idx - 1] = i_0 + 1;
      if (idx >= 422500) {
        exitg2 = true;
      } else {
        i_0++;
      }
    } else {
      i_0++;
    }
  }

  if (idx < 1) {
    tentative_gScore_tmp = 0;
  } else {
    tentative_gScore_tmp = idx;
  }

  i_0 = ii->size[0];
  ii->size[0] = tentative_gScore_tmp;
  MATLA_emxEnsureCapacity_int32_T(ii, i_0);
  astarInternal->NumNodesExplored = tentative_gScore_tmp;
  for (i_0 = 0; i_0 < tentative_gScore_tmp; i_0++) {
    astarInternal->NodesExploredIndicesInternal[i_0] = ii->data[i_0];
  }

  MATLAB0_emxFree_int32_T(&ii);
  priorityqueuecodegen_destructPQ(OpenList_PQInternal);
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static void plannerAStarGrid_getGCostMatrix(const
  c_nav_algs_internal_plannerAS_T *astarInternal, real_T GCostMatrix[422500])
{
  real_T GCostMatrix_0;
  int32_T i;
  for (i = 0; i < 422500; i++) {
    GCostMatrix_0 = astarInternal->GCostMatrix[i];
    GCostMatrix[i] = GCostMatrix_0;
    if (GCostMatrix_0 == -1.0) {
      GCostMatrix[i] = (rtInf);
    }
  }
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static real_T plannerHybridAStar_get2DHeurist(plannerHybridAStar_MATLAB0_T *obj,
  c_nav_algs_internal_plannerAS_T *heuristic2DObjInternal, const real_T point[2])
{
  static binaryOccupancyMap_MATLAB0_T *b_obj;
  static real_T b_x[422500];
  static real_T minval[422500];
  static real_T y[422500];
  static real_T matPoint_data[18];
  static real_T neighborPoints[18];
  static real_T tmp_data_0[18];
  static real_T b_data[9];
  static real_T b_data_0[9];
  static real_T c_data[9];
  static real_T matPoint[2];
  static real_T indices;
  static real_T minCost;
  static real_T neighborPoints_0;
  static real_T u1;
  static real_T x;
  static int32_T tmp_size[2];
  static int32_T c_size;
  static int32_T i;
  static int32_T neighborPoints_tmp;
  static int8_T tmp_data[9];
  static boolean_T tmp[9];
  real_T cost;
  int32_T tmp_size_idx_0;
  boolean_T d_tmp;
  b_obj = obj->Map;
  MAT_MapInterface_world2gridImpl(b_obj, point, matPoint);
  indices = (matPoint[1] - 1.0) * 650.0 + matPoint[0];
  x = obj->Heuristic2DMat[(int32_T)indices - 1];
  if (rtIsInf(x)) {
    for (i = 0; i < 2; i++) {
      for (c_size = 0; c_size < 9; c_size++) {
        neighborPoints_tmp = 9 * i + c_size;
        neighborPoints[neighborPoints_tmp] = obj->Neighbors[neighborPoints_tmp]
          + matPoint[i];
      }
    }

    c_size = 0;
    for (i = 0; i < 9; i++) {
      minCost = neighborPoints[i];
      neighborPoints_0 = neighborPoints[i + 9];
      d_tmp = ((minCost > 0.0) && (neighborPoints_0 > 0.0) && (minCost <= 650.0));
      d_tmp = (d_tmp && (neighborPoints_0 <= 650.0));
      tmp[i] = d_tmp;
      if (d_tmp) {
        c_size++;
      }
    }

    tmp_size_idx_0 = c_size;
    c_size = 0;
    for (i = 0; i < 9; i++) {
      if (tmp[i]) {
        tmp_data[c_size] = (int8_T)i;
        c_size++;
      }
    }

    for (i = 0; i < tmp_size_idx_0; i++) {
      c_size = tmp_data[i];
      b_data[i] = obj->Heuristic2DMat[(int32_T)((neighborPoints[c_size + 9] -
        1.0) * 650.0 + neighborPoints[c_size]) - 1];
    }

    for (i = 0; i < 2; i++) {
      for (c_size = 0; c_size < tmp_size_idx_0; c_size++) {
        matPoint_data[c_size + tmp_size_idx_0 * i] = matPoint[i] -
          neighborPoints[9 * i + tmp_data[c_size]];
      }
    }

    tmp_size[0] = tmp_size_idx_0;
    tmp_size[1] = 2;
    neighborPoints_tmp = tmp_size_idx_0 << 1;
    for (c_size = 0; c_size < neighborPoints_tmp; c_size++) {
      minCost = matPoint_data[c_size];
      tmp_data_0[c_size] = minCost * minCost;
    }

    MATLAB0_sum(tmp_data_0, tmp_size, c_data, &c_size);
    for (i = 0; i < c_size; i++) {
      c_data[i] = sqrt(c_data[i]);
    }

    if (tmp_size_idx_0 == c_size) {
      for (i = 0; i < tmp_size_idx_0; i++) {
        b_data_0[i] = b_data[i] + c_data[i];
      }

      minCost = MATLAB0_minimum(b_data_0, &tmp_size_idx_0);
    } else {
      minCost = MATLAB0_binary_expand_op(b_data, &tmp_size_idx_0, c_data,
        &c_size);
    }

    if (!rtIsInf(minCost)) {
      obj->Heuristic2DMat[(int32_T)indices - 1] = minCost;
    } else {
      MATLAB_plannerAStarGrid_runPlan(heuristic2DObjInternal, obj->GoalPoint,
        matPoint);
      plannerAStarGrid_getGCostMatrix(heuristic2DObjInternal, y);
      for (i = 0; i < 422500; i++) {
        b_x[i] = obj->Heuristic2DMat[i];
      }

      for (i = 0; i < 422500; i++) {
        neighborPoints_0 = b_x[i];
        u1 = y[i];
        if ((neighborPoints_0 <= u1) || rtIsNaN(u1)) {
          minval[i] = neighborPoints_0;
        } else {
          minval[i] = u1;
        }
      }

      for (i = 0; i < 422500; i++) {
        obj->Heuristic2DMat[i] = minval[i];
      }
    }
  }

  cost = obj->Heuristic2DMat[(int32_T)indices - 1];
  return cost;
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static real_T plannerHybridAStar_get3DHeurist(const plannerHybridAStar_MATLAB0_T
  *obj, const real_T start[3], const real_T goal[3])
{
  static real_T a__2[5];
  static real_T x[5];
  static real_T a__1;
  static real_T pathLength;
  static real_T pathLength_0;
  static boolean_T allPathTypes[44];
  int32_T k;
  for (k = 0; k < 44; k++) {
    allPathTypes[k] = true;
  }

  autonomousReedsSheppSegmentsCodegen_real64(&start[0], 1U, &goal[0], 1U,
    obj->MinTurningRadius, obj->ForwardCost, obj->ReverseCost, &allPathTypes[0],
    0U, 1U, true, 3U, &a__1, &x[0], &a__2[0]);
  for (k = 0; k < 5; k++) {
    a__2[k] = fabs(x[k]);
  }

  pathLength = a__2[0] + a__2[1];
  pathLength_0 = (pathLength + a__2[2]) + a__2[3];
  return pathLength_0 + a__2[4];
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static real_T MATLAB0_maximum_b(const real_T x[2])
{
  real_T ex;
  if ((x[0] < x[1]) || (rtIsNaN(x[0]) && (!rtIsNaN(x[1])))) {
    ex = x[1];
  } else {
    ex = x[0];
  }

  return ex;
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static void MATLAB0_linspace(real_T d1, real_T d2, real_T y[5])
{
  static real_T d2scaled;
  real_T delta1;
  real_T delta2;
  y[4] = d2;
  y[0] = d1;
  if (d1 == -d2) {
    d2scaled = d2 / 4.0;
    y[1] = -2.0 * d2scaled;
    y[3] = 2.0 * d2scaled;
    y[2] = 0.0;
  } else if (((d1 < 0.0) != (d2 < 0.0)) && ((fabs(d1) > 8.9884656743115785E+307)
              || (fabs(d2) > 8.9884656743115785E+307))) {
    delta1 = d1 / 4.0;
    delta2 = d2 / 4.0;
    y[1] = (d1 + delta2) - delta1;
    y[2] = (delta2 * 2.0 + d1) - delta1 * 2.0;
    y[3] = (delta2 * 3.0 + d1) - delta1 * 3.0;
  } else {
    delta1 = (d2 - d1) / 4.0;
    y[1] = d1 + delta1;
    y[2] = 2.0 * delta1 + d1;
    y[3] = 3.0 * delta1 + d1;
  }
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static void MATLAB0_repmat(const real_T a_data[], real_T b_data[], int32_T
  b_size[2])
{
  real_T a;
  real_T a_0;
  real_T a_1;
  real_T a_2;
  int32_T ibtile;
  int32_T jtilecol;
  b_size[0] = 1;
  b_size[1] = 8;
  a = a_data[0];
  a_0 = a_data[1];
  a_1 = a_data[2];
  a_2 = a_data[3];
  for (jtilecol = 0; jtilecol < 2; jtilecol++) {
    ibtile = jtilecol << 2;
    b_data[ibtile] = a;
    b_data[ibtile + 1] = a_0;
    b_data[ibtile + 2] = a_1;
    b_data[ibtile + 3] = a_2;
  }
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static reedsSheppConnection_MATLAB0_T *reedsSheppConnection_reedsShepp
  (reedsSheppConnection_MATLAB0_T *obj, real_T varargin_2, real_T varargin_4,
   real_T varargin_6)
{
  reedsSheppConnection_MATLAB0_T *b_obj;
  b_obj = obj;
  obj->MinTurningRadius = varargin_2;
  obj->DisabledPathTypesInternal->size[0] = 0;
  obj->DisabledPathTypesInternal->size[1] = 0;
  obj->ForwardCostInternal = varargin_4;
  obj->ReverseCostInternal = varargin_6;
  return b_obj;
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static void MATLAB0_PriorityQueue_top(nav_algs_internal_PriorityQue_T *obj,
  real_T nodeData_data[], int32_T nodeData_size[2], real_T *nodeId)
{
  real_T dataDim;
  dataDim = priorityqueuecodegen_getDataDim(obj->PQInternal);
  nodeData_size[0] = 1;
  nodeData_size[1] = (int32_T)dataDim;
  priorityqueuecodegen_top(obj->PQInternal, &nodeData_data[0], nodeId);
  (*nodeId)++;
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static boolean_T MATLAB0_strcmp(const char_T a_data[], const int32_T a_size[2])
{
  int32_T kstr;
  boolean_T b_bool;
  static const char_T c[10] = { 'e', 'x', 'h', 'a', 'u', 's', 't', 'i', 'v', 'e'
  };

  int32_T exitg1;
  b_bool = false;
  if (a_size[1] == 10) {
    kstr = 0;
    do {
      exitg1 = 0;
      if (kstr < 10) {
        if (a_data[kstr] != c[kstr]) {
          exitg1 = 1;
        } else {
          kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  return b_bool;
}

static real_T MATLAB0_rt_remd_snf(real_T u0, real_T u1)
{
  real_T q;
  real_T y;
  if (rtIsNaN(u0) || rtIsNaN(u1) || rtIsInf(u0)) {
    y = (rtNaN);
  } else if (rtIsInf(u1)) {
    y = u0;
  } else {
    if (u1 < 0.0) {
      q = ceil(u1);
    } else {
      q = floor(u1);
    }

    if ((u1 != 0.0) && (u1 != q)) {
      q = fabs(u0 / u1);
      if (!(fabs(q - floor(q + 0.5)) > DBL_EPSILON * q)) {
        y = 0.0 * u0;
      } else {
        y = fmod(u0, u1);
      }
    } else {
      y = fmod(u0, u1);
    }
  }

  return y;
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static void MA_plannerHybridAStar_closeCell(plannerHybridAStar_MATLAB0_T *obj,
  real_T direction, real_T Indice)
{
  int32_T i;
  int32_T trueCount;
  trueCount = 0;
  if (direction == 1.0) {
    for (i = 0; i < 1; i++) {
      trueCount++;
    }
  }

  for (i = 0; i < trueCount; i++) {
    obj->visitedCellsFront[(int32_T)Indice - 1] = true;
  }

  trueCount = 0;
  if (!(direction == 1.0)) {
    for (i = 0; i < 1; i++) {
      trueCount++;
    }
  }

  for (i = 0; i < trueCount; i++) {
    obj->visitedCellsBack[(int32_T)Indice - 1] = true;
  }
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static void plannerHybridAStar_getCircularP(real_T length, const real_T
  curvature[8], const real_T initialNodePose[3], const real_T direction[8],
  real_T newNodesPoses[24], real_T ICRsData[24])
{
  static real_T centerX[8];
  static real_T b;
  static real_T b_b;
  static real_T initialNodePose_0;
  static real_T initialNodePose_1;
  static real_T initialNodePose_2;
  real_T centerX_0;
  real_T centerY;
  real_T curvature_0;
  real_T turningRadius;
  real_T x_tmp;
  int32_T k;
  b = sin(initialNodePose[2]);
  initialNodePose_0 = initialNodePose[0];
  b_b = cos(initialNodePose[2]);
  initialNodePose_1 = initialNodePose[1];
  initialNodePose_2 = initialNodePose[2];
  for (k = 0; k < 8; k++) {
    curvature_0 = curvature[k];
    turningRadius = 1.0 / curvature_0;
    curvature_0 *= length;
    centerX_0 = initialNodePose_0 - turningRadius * b;
    centerX[k] = centerX_0;
    centerY = turningRadius * b_b + initialNodePose_1;
    x_tmp = direction[k] * curvature_0 + initialNodePose_2;
    newNodesPoses[k] = turningRadius * sin(x_tmp) + centerX_0;
    newNodesPoses[k + 8] = centerY - turningRadius * cos(x_tmp);
    newNodesPoses[k + 16] = x_tmp;
    ICRsData[k] = centerX_0;
    ICRsData[k + 8] = centerY;
    ICRsData[k + 16] = curvature_0;
  }
}

static void MATLA_emxInit_cell_wrap_53_5x44(emxArray_cell_wrap_53_5x44_MA_T
  *pEmxArray)
{
  pEmxArray->size[0] = 0;
  pEmxArray->size[1] = 0;
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static void MATLAB0_strcmp_c(const emxArray_char_T_MATLAB0_T *a, boolean_T
  b_bool[44])
{
  static int32_T ab_kstr;
  static int32_T b_kstr;
  static int32_T bb_kstr;
  static int32_T c_kstr;
  static int32_T cb_kstr;
  static int32_T d_kstr;
  static int32_T db_kstr;
  static int32_T e_kstr;
  static int32_T eb_kstr;
  static int32_T f_kstr;
  static int32_T fb_kstr;
  static int32_T g_kstr;
  static int32_T gb_kstr;
  static int32_T h_kstr;
  static int32_T hb_kstr;
  static int32_T i_kstr;
  static int32_T ib_kstr;
  static int32_T j_kstr;
  static int32_T jb_kstr;
  static int32_T k_kstr;
  static int32_T kb_kstr;
  static int32_T kstr;
  static int32_T l_kstr;
  static int32_T lb_kstr;
  static int32_T m_kstr;
  static int32_T mb_kstr;
  static int32_T n_kstr;
  static int32_T nb_kstr;
  static int32_T o_kstr;
  static int32_T p_kstr;
  static int32_T q_kstr;
  static int32_T r_kstr;
  static int32_T s_kstr;
  static int32_T t_kstr;
  static int32_T u_kstr;
  static int32_T v_kstr;
  static int32_T w_kstr;
  static int32_T x_kstr;
  static int32_T y_kstr;
  int32_T ob_kstr;
  static const char_T c[6] = { 'L', 'p', 'S', 'p', 'L', 'p' };

  static const char_T d[6] = { 'L', 'n', 'S', 'n', 'L', 'n' };

  static const char_T e[6] = { 'R', 'p', 'S', 'p', 'R', 'p' };

  static const char_T f[6] = { 'R', 'n', 'S', 'n', 'R', 'n' };

  static const char_T g[6] = { 'L', 'p', 'S', 'p', 'R', 'p' };

  static const char_T h[6] = { 'L', 'n', 'S', 'n', 'R', 'n' };

  static const char_T i[6] = { 'R', 'p', 'S', 'p', 'L', 'p' };

  static const char_T j[6] = { 'R', 'n', 'S', 'n', 'L', 'n' };

  static const char_T k[6] = { 'L', 'p', 'R', 'n', 'L', 'p' };

  static const char_T l[6] = { 'L', 'n', 'R', 'p', 'L', 'n' };

  static const char_T m[6] = { 'R', 'p', 'L', 'n', 'R', 'p' };

  static const char_T n[6] = { 'R', 'n', 'L', 'p', 'R', 'n' };

  static const char_T o[6] = { 'L', 'p', 'R', 'p', 'L', 'n' };

  static const char_T p[6] = { 'L', 'n', 'R', 'n', 'L', 'p' };

  static const char_T q[6] = { 'R', 'p', 'L', 'p', 'R', 'n' };

  static const char_T r[6] = { 'R', 'n', 'L', 'n', 'R', 'p' };

  static const char_T s[8] = { 'L', 'p', 'R', 'p', 'L', 'n', 'R', 'n' };

  static const char_T t[8] = { 'L', 'n', 'R', 'n', 'L', 'p', 'R', 'p' };

  static const char_T u[8] = { 'R', 'p', 'L', 'p', 'R', 'n', 'L', 'n' };

  static const char_T v[8] = { 'R', 'n', 'L', 'n', 'R', 'p', 'L', 'p' };

  static const char_T w[8] = { 'L', 'p', 'R', 'n', 'L', 'n', 'R', 'p' };

  static const char_T x[8] = { 'L', 'n', 'R', 'p', 'L', 'p', 'R', 'n' };

  static const char_T y[8] = { 'R', 'p', 'L', 'n', 'R', 'n', 'L', 'p' };

  static const char_T ab[8] = { 'R', 'n', 'L', 'p', 'R', 'p', 'L', 'n' };

  static const char_T bb[8] = { 'L', 'p', 'R', 'n', 'S', 'n', 'L', 'n' };

  static const char_T cb[8] = { 'L', 'n', 'R', 'p', 'S', 'p', 'L', 'p' };

  static const char_T db[8] = { 'R', 'p', 'L', 'n', 'S', 'n', 'R', 'n' };

  static const char_T eb[8] = { 'R', 'n', 'L', 'p', 'S', 'p', 'R', 'p' };

  static const char_T fb[8] = { 'L', 'p', 'R', 'n', 'S', 'n', 'R', 'n' };

  static const char_T gb[8] = { 'L', 'n', 'R', 'p', 'S', 'p', 'R', 'p' };

  static const char_T hb[8] = { 'R', 'p', 'L', 'n', 'S', 'n', 'L', 'n' };

  static const char_T ib[8] = { 'R', 'n', 'L', 'p', 'S', 'p', 'L', 'p' };

  static const char_T jb[8] = { 'L', 'p', 'S', 'p', 'R', 'p', 'L', 'n' };

  static const char_T kb[8] = { 'L', 'n', 'S', 'n', 'R', 'n', 'L', 'p' };

  static const char_T lb[8] = { 'R', 'p', 'S', 'p', 'L', 'p', 'R', 'n' };

  static const char_T mb[8] = { 'R', 'n', 'S', 'n', 'L', 'n', 'R', 'p' };

  static const char_T nb[8] = { 'R', 'n', 'S', 'n', 'R', 'n', 'L', 'p' };

  static const char_T ob[8] = { 'R', 'p', 'S', 'p', 'R', 'p', 'L', 'n' };

  static const char_T pb[8] = { 'L', 'n', 'S', 'n', 'L', 'n', 'R', 'p' };

  static const char_T qb[8] = { 'L', 'p', 'S', 'p', 'L', 'p', 'R', 'n' };

  static const char_T rb[10] = { 'L', 'p', 'R', 'n', 'S', 'n', 'L', 'n', 'R',
    'p' };

  static const char_T sb[10] = { 'L', 'n', 'R', 'p', 'S', 'p', 'L', 'p', 'R',
    'n' };

  static const char_T tb[10] = { 'R', 'p', 'L', 'n', 'S', 'n', 'R', 'n', 'L',
    'p' };

  static const char_T ub[10] = { 'R', 'n', 'L', 'p', 'S', 'p', 'R', 'p', 'L',
    'n' };

  int32_T exitg1;
  b_bool[0] = false;
  if (a->size[1] == 6) {
    kstr = 0;
    do {
      exitg1 = 0;
      if (kstr < 6) {
        if (a->data[kstr] != c[kstr]) {
          exitg1 = 1;
        } else {
          kstr++;
        }
      } else {
        b_bool[0] = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  b_bool[1] = false;
  if (a->size[1] == 6) {
    b_kstr = 0;
    do {
      exitg1 = 0;
      if (b_kstr < 6) {
        if (a->data[b_kstr] != d[b_kstr]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool[1] = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  b_bool[2] = false;
  if (a->size[1] == 6) {
    c_kstr = 0;
    do {
      exitg1 = 0;
      if (c_kstr < 6) {
        if (a->data[c_kstr] != e[c_kstr]) {
          exitg1 = 1;
        } else {
          c_kstr++;
        }
      } else {
        b_bool[2] = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  b_bool[3] = false;
  if (a->size[1] == 6) {
    d_kstr = 0;
    do {
      exitg1 = 0;
      if (d_kstr < 6) {
        if (a->data[d_kstr] != f[d_kstr]) {
          exitg1 = 1;
        } else {
          d_kstr++;
        }
      } else {
        b_bool[3] = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  b_bool[4] = false;
  if (a->size[1] == 6) {
    e_kstr = 0;
    do {
      exitg1 = 0;
      if (e_kstr < 6) {
        if (a->data[e_kstr] != g[e_kstr]) {
          exitg1 = 1;
        } else {
          e_kstr++;
        }
      } else {
        b_bool[4] = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  b_bool[5] = false;
  if (a->size[1] == 6) {
    f_kstr = 0;
    do {
      exitg1 = 0;
      if (f_kstr < 6) {
        if (a->data[f_kstr] != h[f_kstr]) {
          exitg1 = 1;
        } else {
          f_kstr++;
        }
      } else {
        b_bool[5] = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  b_bool[6] = false;
  if (a->size[1] == 6) {
    g_kstr = 0;
    do {
      exitg1 = 0;
      if (g_kstr < 6) {
        if (a->data[g_kstr] != i[g_kstr]) {
          exitg1 = 1;
        } else {
          g_kstr++;
        }
      } else {
        b_bool[6] = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  b_bool[7] = false;
  if (a->size[1] == 6) {
    h_kstr = 0;
    do {
      exitg1 = 0;
      if (h_kstr < 6) {
        if (a->data[h_kstr] != j[h_kstr]) {
          exitg1 = 1;
        } else {
          h_kstr++;
        }
      } else {
        b_bool[7] = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  b_bool[8] = false;
  if (a->size[1] == 6) {
    i_kstr = 0;
    do {
      exitg1 = 0;
      if (i_kstr < 6) {
        if (a->data[i_kstr] != k[i_kstr]) {
          exitg1 = 1;
        } else {
          i_kstr++;
        }
      } else {
        b_bool[8] = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  b_bool[9] = false;
  if (a->size[1] == 6) {
    j_kstr = 0;
    do {
      exitg1 = 0;
      if (j_kstr < 6) {
        if (a->data[j_kstr] != l[j_kstr]) {
          exitg1 = 1;
        } else {
          j_kstr++;
        }
      } else {
        b_bool[9] = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  b_bool[10] = false;
  if (a->size[1] == 6) {
    k_kstr = 0;
    do {
      exitg1 = 0;
      if (k_kstr < 6) {
        if (a->data[k_kstr] != m[k_kstr]) {
          exitg1 = 1;
        } else {
          k_kstr++;
        }
      } else {
        b_bool[10] = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  b_bool[11] = false;
  if (a->size[1] == 6) {
    l_kstr = 0;
    do {
      exitg1 = 0;
      if (l_kstr < 6) {
        if (a->data[l_kstr] != n[l_kstr]) {
          exitg1 = 1;
        } else {
          l_kstr++;
        }
      } else {
        b_bool[11] = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  b_bool[12] = false;
  if (a->size[1] == 6) {
    m_kstr = 0;
    do {
      exitg1 = 0;
      if (m_kstr < 6) {
        if (a->data[m_kstr] != o[m_kstr]) {
          exitg1 = 1;
        } else {
          m_kstr++;
        }
      } else {
        b_bool[12] = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  b_bool[13] = false;
  if (a->size[1] == 6) {
    n_kstr = 0;
    do {
      exitg1 = 0;
      if (n_kstr < 6) {
        if (a->data[n_kstr] != p[n_kstr]) {
          exitg1 = 1;
        } else {
          n_kstr++;
        }
      } else {
        b_bool[13] = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  b_bool[14] = false;
  if (a->size[1] == 6) {
    o_kstr = 0;
    do {
      exitg1 = 0;
      if (o_kstr < 6) {
        if (a->data[o_kstr] != q[o_kstr]) {
          exitg1 = 1;
        } else {
          o_kstr++;
        }
      } else {
        b_bool[14] = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  b_bool[15] = false;
  if (a->size[1] == 6) {
    p_kstr = 0;
    do {
      exitg1 = 0;
      if (p_kstr < 6) {
        if (a->data[p_kstr] != r[p_kstr]) {
          exitg1 = 1;
        } else {
          p_kstr++;
        }
      } else {
        b_bool[15] = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  b_bool[16] = false;
  if (a->size[1] == 8) {
    q_kstr = 0;
    do {
      exitg1 = 0;
      if (q_kstr < 8) {
        if (a->data[q_kstr] != s[q_kstr]) {
          exitg1 = 1;
        } else {
          q_kstr++;
        }
      } else {
        b_bool[16] = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  b_bool[17] = false;
  if (a->size[1] == 8) {
    r_kstr = 0;
    do {
      exitg1 = 0;
      if (r_kstr < 8) {
        if (a->data[r_kstr] != t[r_kstr]) {
          exitg1 = 1;
        } else {
          r_kstr++;
        }
      } else {
        b_bool[17] = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  b_bool[18] = false;
  if (a->size[1] == 8) {
    s_kstr = 0;
    do {
      exitg1 = 0;
      if (s_kstr < 8) {
        if (a->data[s_kstr] != u[s_kstr]) {
          exitg1 = 1;
        } else {
          s_kstr++;
        }
      } else {
        b_bool[18] = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  b_bool[19] = false;
  if (a->size[1] == 8) {
    t_kstr = 0;
    do {
      exitg1 = 0;
      if (t_kstr < 8) {
        if (a->data[t_kstr] != v[t_kstr]) {
          exitg1 = 1;
        } else {
          t_kstr++;
        }
      } else {
        b_bool[19] = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  b_bool[20] = false;
  if (a->size[1] == 8) {
    u_kstr = 0;
    do {
      exitg1 = 0;
      if (u_kstr < 8) {
        if (a->data[u_kstr] != w[u_kstr]) {
          exitg1 = 1;
        } else {
          u_kstr++;
        }
      } else {
        b_bool[20] = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  b_bool[21] = false;
  if (a->size[1] == 8) {
    v_kstr = 0;
    do {
      exitg1 = 0;
      if (v_kstr < 8) {
        if (a->data[v_kstr] != x[v_kstr]) {
          exitg1 = 1;
        } else {
          v_kstr++;
        }
      } else {
        b_bool[21] = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  b_bool[22] = false;
  if (a->size[1] == 8) {
    w_kstr = 0;
    do {
      exitg1 = 0;
      if (w_kstr < 8) {
        if (a->data[w_kstr] != y[w_kstr]) {
          exitg1 = 1;
        } else {
          w_kstr++;
        }
      } else {
        b_bool[22] = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  b_bool[23] = false;
  if (a->size[1] == 8) {
    x_kstr = 0;
    do {
      exitg1 = 0;
      if (x_kstr < 8) {
        if (a->data[x_kstr] != ab[x_kstr]) {
          exitg1 = 1;
        } else {
          x_kstr++;
        }
      } else {
        b_bool[23] = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  b_bool[24] = false;
  if (a->size[1] == 8) {
    y_kstr = 0;
    do {
      exitg1 = 0;
      if (y_kstr < 8) {
        if (a->data[y_kstr] != bb[y_kstr]) {
          exitg1 = 1;
        } else {
          y_kstr++;
        }
      } else {
        b_bool[24] = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  b_bool[25] = false;
  if (a->size[1] == 8) {
    ab_kstr = 0;
    do {
      exitg1 = 0;
      if (ab_kstr < 8) {
        if (a->data[ab_kstr] != cb[ab_kstr]) {
          exitg1 = 1;
        } else {
          ab_kstr++;
        }
      } else {
        b_bool[25] = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  b_bool[26] = false;
  if (a->size[1] == 8) {
    bb_kstr = 0;
    do {
      exitg1 = 0;
      if (bb_kstr < 8) {
        if (a->data[bb_kstr] != db[bb_kstr]) {
          exitg1 = 1;
        } else {
          bb_kstr++;
        }
      } else {
        b_bool[26] = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  b_bool[27] = false;
  if (a->size[1] == 8) {
    cb_kstr = 0;
    do {
      exitg1 = 0;
      if (cb_kstr < 8) {
        if (a->data[cb_kstr] != eb[cb_kstr]) {
          exitg1 = 1;
        } else {
          cb_kstr++;
        }
      } else {
        b_bool[27] = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  b_bool[28] = false;
  if (a->size[1] == 8) {
    db_kstr = 0;
    do {
      exitg1 = 0;
      if (db_kstr < 8) {
        if (a->data[db_kstr] != fb[db_kstr]) {
          exitg1 = 1;
        } else {
          db_kstr++;
        }
      } else {
        b_bool[28] = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  b_bool[29] = false;
  if (a->size[1] == 8) {
    eb_kstr = 0;
    do {
      exitg1 = 0;
      if (eb_kstr < 8) {
        if (a->data[eb_kstr] != gb[eb_kstr]) {
          exitg1 = 1;
        } else {
          eb_kstr++;
        }
      } else {
        b_bool[29] = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  b_bool[30] = false;
  if (a->size[1] == 8) {
    fb_kstr = 0;
    do {
      exitg1 = 0;
      if (fb_kstr < 8) {
        if (a->data[fb_kstr] != hb[fb_kstr]) {
          exitg1 = 1;
        } else {
          fb_kstr++;
        }
      } else {
        b_bool[30] = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  b_bool[31] = false;
  if (a->size[1] == 8) {
    gb_kstr = 0;
    do {
      exitg1 = 0;
      if (gb_kstr < 8) {
        if (a->data[gb_kstr] != ib[gb_kstr]) {
          exitg1 = 1;
        } else {
          gb_kstr++;
        }
      } else {
        b_bool[31] = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  b_bool[32] = false;
  if (a->size[1] == 8) {
    hb_kstr = 0;
    do {
      exitg1 = 0;
      if (hb_kstr < 8) {
        if (a->data[hb_kstr] != jb[hb_kstr]) {
          exitg1 = 1;
        } else {
          hb_kstr++;
        }
      } else {
        b_bool[32] = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  b_bool[33] = false;
  if (a->size[1] == 8) {
    ib_kstr = 0;
    do {
      exitg1 = 0;
      if (ib_kstr < 8) {
        if (a->data[ib_kstr] != kb[ib_kstr]) {
          exitg1 = 1;
        } else {
          ib_kstr++;
        }
      } else {
        b_bool[33] = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  b_bool[34] = false;
  if (a->size[1] == 8) {
    jb_kstr = 0;
    do {
      exitg1 = 0;
      if (jb_kstr < 8) {
        if (a->data[jb_kstr] != lb[jb_kstr]) {
          exitg1 = 1;
        } else {
          jb_kstr++;
        }
      } else {
        b_bool[34] = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  b_bool[35] = false;
  if (a->size[1] == 8) {
    kb_kstr = 0;
    do {
      exitg1 = 0;
      if (kb_kstr < 8) {
        if (a->data[kb_kstr] != mb[kb_kstr]) {
          exitg1 = 1;
        } else {
          kb_kstr++;
        }
      } else {
        b_bool[35] = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  b_bool[36] = false;
  if (a->size[1] == 8) {
    lb_kstr = 0;
    do {
      exitg1 = 0;
      if (lb_kstr < 8) {
        if (a->data[lb_kstr] != nb[lb_kstr]) {
          exitg1 = 1;
        } else {
          lb_kstr++;
        }
      } else {
        b_bool[36] = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  b_bool[37] = false;
  if (a->size[1] == 8) {
    mb_kstr = 0;
    do {
      exitg1 = 0;
      if (mb_kstr < 8) {
        if (a->data[mb_kstr] != ob[mb_kstr]) {
          exitg1 = 1;
        } else {
          mb_kstr++;
        }
      } else {
        b_bool[37] = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  b_bool[38] = false;
  if (a->size[1] == 8) {
    nb_kstr = 0;
    do {
      exitg1 = 0;
      if (nb_kstr < 8) {
        if (a->data[nb_kstr] != pb[nb_kstr]) {
          exitg1 = 1;
        } else {
          nb_kstr++;
        }
      } else {
        b_bool[38] = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  b_bool[39] = false;
  if (a->size[1] == 8) {
    ob_kstr = 0;
    do {
      exitg1 = 0;
      if (ob_kstr < 8) {
        if (a->data[ob_kstr] != qb[ob_kstr]) {
          exitg1 = 1;
        } else {
          ob_kstr++;
        }
      } else {
        b_bool[39] = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  b_bool[40] = false;
  if (a->size[1] == 10) {
    ob_kstr = 0;
    do {
      exitg1 = 0;
      if (ob_kstr < 10) {
        if (a->data[ob_kstr] != rb[ob_kstr]) {
          exitg1 = 1;
        } else {
          ob_kstr++;
        }
      } else {
        b_bool[40] = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  b_bool[41] = false;
  if (a->size[1] == 10) {
    ob_kstr = 0;
    do {
      exitg1 = 0;
      if (ob_kstr < 10) {
        if (a->data[ob_kstr] != sb[ob_kstr]) {
          exitg1 = 1;
        } else {
          ob_kstr++;
        }
      } else {
        b_bool[41] = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  b_bool[42] = false;
  if (a->size[1] == 10) {
    ob_kstr = 0;
    do {
      exitg1 = 0;
      if (ob_kstr < 10) {
        if (a->data[ob_kstr] != tb[ob_kstr]) {
          exitg1 = 1;
        } else {
          ob_kstr++;
        }
      } else {
        b_bool[42] = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  b_bool[43] = false;
  if (a->size[1] == 10) {
    ob_kstr = 0;
    do {
      exitg1 = 0;
      if (ob_kstr < 10) {
        if (a->data[ob_kstr] != ub[ob_kstr]) {
          exitg1 = 1;
        } else {
          ob_kstr++;
        }
      } else {
        b_bool[43] = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static void ReedsSheppBuiltins_autonomousRe(const real_T startPose[3], const
  real_T goalPose[3], real_T turningRadius, real_T forwardCost, real_T
  reverseCost, const char_T pathSegments[3], const
  emxArray_cell_wrap_53_MATLAB0_T *disabledTypes, real_T cost_data[], int32_T
  *cost_size, real_T motionLengths_data[], int32_T motionLengths_size[2], real_T
  motionTypes_data[], int32_T motionTypes_size[2])
{
  static b_cell_62_MATLAB0_T d;
  static real_T numDisabledPathTypes;
  static real_T numPaths;
  static int32_T i;
  static int32_T n;
  static int32_T ret;
  static boolean_T allPathTypes[44];
  static boolean_T match[44];
  boolean_T isOptimal;
  static const char_T b_b[3] = { 'a', 'l', 'l' };

  static int32_T k;
  int32_T i_0;
  uint32_T tmp;
  static const char_T g[6] = { 'L', 'p', 'S', 'p', 'L', 'p' };

  static const char_T h[6] = { 'L', 'n', 'S', 'n', 'L', 'n' };

  static const char_T bb[8] = { 'L', 'p', 'R', 'p', 'L', 'n', 'R', 'n' };

  static const char_T j[6] = { 'R', 'p', 'S', 'p', 'R', 'p' };

  static const char_T cb[8] = { 'L', 'n', 'R', 'n', 'L', 'p', 'R', 'p' };

  static const char_T l[6] = { 'R', 'n', 'S', 'n', 'R', 'n' };

  static const char_T ac[10] = { 'L', 'p', 'R', 'n', 'S', 'n', 'L', 'n', 'R',
    'p' };

  static const char_T db[8] = { 'R', 'p', 'L', 'p', 'R', 'n', 'L', 'n' };

  static const char_T m[6] = { 'L', 'p', 'S', 'p', 'R', 'p' };

  static const char_T bc[10] = { 'L', 'n', 'R', 'p', 'S', 'p', 'L', 'p', 'R',
    'n' };

  static const char_T eb[8] = { 'R', 'n', 'L', 'n', 'R', 'p', 'L', 'p' };

  static const char_T o[6] = { 'L', 'n', 'S', 'n', 'R', 'n' };

  static const char_T cc[10] = { 'R', 'p', 'L', 'n', 'S', 'n', 'R', 'n', 'L',
    'p' };

  static const char_T fb[8] = { 'L', 'p', 'R', 'n', 'L', 'n', 'R', 'p' };

  static const char_T p[6] = { 'R', 'p', 'S', 'p', 'L', 'p' };

  static const char_T dc[10] = { 'R', 'n', 'L', 'p', 'S', 'p', 'R', 'p', 'L',
    'n' };

  static const char_T gb[8] = { 'L', 'n', 'R', 'p', 'L', 'p', 'R', 'n' };

  static const char_T q[6] = { 'R', 'n', 'S', 'n', 'L', 'n' };

  static const char_T hb[8] = { 'R', 'p', 'L', 'n', 'R', 'n', 'L', 'p' };

  static const char_T r[6] = { 'L', 'p', 'R', 'n', 'L', 'p' };

  static const char_T ib[8] = { 'R', 'n', 'L', 'p', 'R', 'p', 'L', 'n' };

  static const char_T s[6] = { 'L', 'n', 'R', 'p', 'L', 'n' };

  static const char_T jb[8] = { 'L', 'p', 'R', 'n', 'S', 'n', 'L', 'n' };

  static const char_T t[6] = { 'R', 'p', 'L', 'n', 'R', 'p' };

  static const char_T kb[8] = { 'L', 'n', 'R', 'p', 'S', 'p', 'L', 'p' };

  static const char_T u[6] = { 'R', 'n', 'L', 'p', 'R', 'n' };

  static const char_T lb[8] = { 'R', 'p', 'L', 'n', 'S', 'n', 'R', 'n' };

  static const char_T v[6] = { 'L', 'p', 'R', 'p', 'L', 'n' };

  static const char_T mb[8] = { 'R', 'n', 'L', 'p', 'S', 'p', 'R', 'p' };

  static const char_T w[6] = { 'L', 'n', 'R', 'n', 'L', 'p' };

  static const char_T nb[8] = { 'L', 'p', 'R', 'n', 'S', 'n', 'R', 'n' };

  static const char_T x[6] = { 'R', 'p', 'L', 'p', 'R', 'n' };

  static const char_T ob[8] = { 'L', 'n', 'R', 'p', 'S', 'p', 'R', 'p' };

  static const char_T ab[6] = { 'R', 'n', 'L', 'n', 'R', 'p' };

  static const char_T pb[8] = { 'R', 'p', 'L', 'n', 'S', 'n', 'L', 'n' };

  static const char_T qb[8] = { 'R', 'n', 'L', 'p', 'S', 'p', 'L', 'p' };

  static const char_T rb[8] = { 'L', 'p', 'S', 'p', 'R', 'p', 'L', 'n' };

  static const char_T sb[8] = { 'L', 'n', 'S', 'n', 'R', 'n', 'L', 'p' };

  static const char_T tb[8] = { 'R', 'p', 'S', 'p', 'L', 'p', 'R', 'n' };

  static const char_T ub[8] = { 'R', 'n', 'S', 'n', 'L', 'n', 'R', 'p' };

  static const char_T vb[8] = { 'R', 'n', 'S', 'n', 'R', 'n', 'L', 'p' };

  static const char_T wb[8] = { 'R', 'p', 'S', 'p', 'R', 'p', 'L', 'n' };

  static const char_T xb[8] = { 'L', 'n', 'S', 'n', 'L', 'n', 'R', 'p' };

  static const char_T yb[8] = { 'L', 'p', 'S', 'p', 'L', 'p', 'R', 'n' };

  boolean_T exitg1;
  for (i_0 = 0; i_0 < 44; i_0++) {
    allPathTypes[i_0] = true;
  }

  numDisabledPathTypes = 0.0;
  if ((disabledTypes->size[0] == 0) || (disabledTypes->size[1] == 0)) {
    n = -1;
  } else {
    if (disabledTypes->size[0] >= disabledTypes->size[1]) {
      i_0 = disabledTypes->size[0];
    } else {
      i_0 = disabledTypes->size[1];
    }

    n = i_0 - 1;
  }

  for (i = 0; i <= n; i++) {
    for (i_0 = 0; i_0 < 6; i_0++) {
      d.f1[i_0] = g[i_0];
      d.f2[i_0] = h[i_0];
      d.f3[i_0] = j[i_0];
      d.f4[i_0] = l[i_0];
      d.f5[i_0] = m[i_0];
      d.f6[i_0] = o[i_0];
      d.f7[i_0] = p[i_0];
      d.f8[i_0] = q[i_0];
      d.f9[i_0] = r[i_0];
      d.f10[i_0] = s[i_0];
      d.f11[i_0] = t[i_0];
      d.f12[i_0] = u[i_0];
      d.f13[i_0] = v[i_0];
      d.f14[i_0] = w[i_0];
      d.f15[i_0] = x[i_0];
      d.f16[i_0] = ab[i_0];
    }

    for (i_0 = 0; i_0 < 8; i_0++) {
      d.f17[i_0] = bb[i_0];
      d.f18[i_0] = cb[i_0];
      d.f19[i_0] = db[i_0];
      d.f20[i_0] = eb[i_0];
      d.f21[i_0] = fb[i_0];
      d.f22[i_0] = gb[i_0];
      d.f23[i_0] = hb[i_0];
      d.f24[i_0] = ib[i_0];
      d.f25[i_0] = jb[i_0];
      d.f26[i_0] = kb[i_0];
      d.f27[i_0] = lb[i_0];
      d.f28[i_0] = mb[i_0];
      d.f29[i_0] = nb[i_0];
      d.f30[i_0] = ob[i_0];
      d.f31[i_0] = pb[i_0];
      d.f32[i_0] = qb[i_0];
      d.f33[i_0] = rb[i_0];
      d.f34[i_0] = sb[i_0];
      d.f35[i_0] = tb[i_0];
      d.f36[i_0] = ub[i_0];
      d.f37[i_0] = vb[i_0];
      d.f38[i_0] = wb[i_0];
      d.f39[i_0] = xb[i_0];
      d.f40[i_0] = yb[i_0];
    }

    for (i_0 = 0; i_0 < 10; i_0++) {
      d.f41[i_0] = ac[i_0];
      d.f42[i_0] = bc[i_0];
      d.f43[i_0] = cc[i_0];
      d.f44[i_0] = dc[i_0];
    }

    MATLAB0_strcmp_c(disabledTypes->data[i].f1, match);
    isOptimal = false;
    k = 0;
    exitg1 = false;
    while ((!exitg1) && (k <= 43)) {
      if (match[k]) {
        isOptimal = true;
        exitg1 = true;
      } else {
        k++;
      }
    }

    if (isOptimal) {
      numDisabledPathTypes++;
      for (i_0 = 0; i_0 < 44; i_0++) {
        if (match[i_0]) {
          allPathTypes[i_0] = false;
        }
      }
    }
  }

  numPaths = 1.0;
  isOptimal = true;
  ret = memcmp(&pathSegments[0], &b_b[0], 3);
  if (ret == 0) {
    isOptimal = false;
    numPaths = 44.0 - numDisabledPathTypes;
  }

  *cost_size = (int32_T)numPaths;
  motionLengths_size[0] = 5;
  motionLengths_size[1] = (int32_T)numPaths;
  motionTypes_size[0] = 5;
  motionTypes_size[1] = (int32_T)numPaths;
  if (numDisabledPathTypes < 4.294967296E+9) {
    tmp = (uint32_T)numDisabledPathTypes;
  } else {
    tmp = MAX_uint32_T;
  }

  if (numPaths >= 0.0) {
    i_0 = (int32_T)numPaths;
  } else {
    i_0 = 0;
  }

  autonomousReedsSheppSegmentsCodegen_real64(&startPose[0], 1U, &goalPose[0], 1U,
    turningRadius, forwardCost, reverseCost, &allPathTypes[0], tmp, (uint32_T)
    i_0, isOptimal, 3U, &cost_data[0], &motionLengths_data[0],
    &motionTypes_data[0]);
}

static void MATLA_emxTrim_cell_wrap_53_5x44(cell_wrap_53_MATLAB0_T data[220],
  int32_T fromIndex, int32_T toIndex)
{
  int32_T i;
  for (i = fromIndex; i < toIndex; i++) {
    MATL_emxFreeStruct_cell_wrap_53(&data[i]);
  }
}

static void MATLAB0_emxInit_char_T(emxArray_char_T_MATLAB0_T **pEmxArray,
  int32_T numDimensions)
{
  emxArray_char_T_MATLAB0_T *emxArray;
  int32_T i;
  *pEmxArray = (emxArray_char_T_MATLAB0_T *)malloc(sizeof
    (emxArray_char_T_MATLAB0_T));
  emxArray = *pEmxArray;
  emxArray->data = (char_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int32_T *)malloc(sizeof(int32_T) * (uint32_T)numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

static void MATL_emxInitStruct_cell_wrap_53(cell_wrap_53_MATLAB0_T *pStruct)
{
  MATLAB0_emxInit_char_T(&pStruct->f1, 2);
}

static void MAT_emxExpand_cell_wrap_53_5x44(cell_wrap_53_MATLAB0_T data[220],
  int32_T fromIndex, int32_T toIndex)
{
  int32_T i;
  for (i = fromIndex; i < toIndex; i++) {
    MATL_emxInitStruct_cell_wrap_53(&data[i]);
  }
}

static void emxEnsureCapacity_cell_wrap_53(cell_wrap_53_MATLAB0_T data[220],
  const int32_T size[2], int32_T oldNumel)
{
  int32_T newNumel;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  newNumel = size[0] * size[1];
  if (oldNumel > newNumel) {
    MATLA_emxTrim_cell_wrap_53_5x44(data, newNumel, oldNumel);
  } else if (oldNumel < newNumel) {
    MAT_emxExpand_cell_wrap_53_5x44(data, oldNumel, newNumel);
  }
}

static void MATLAB0_emxInit_uint64_T(emxArray_uint64_T_MATLAB0_T **pEmxArray,
  int32_T numDimensions)
{
  emxArray_uint64_T_MATLAB0_T *emxArray;
  int32_T i;
  *pEmxArray = (emxArray_uint64_T_MATLAB0_T *)malloc(sizeof
    (emxArray_uint64_T_MATLAB0_T));
  emxArray = *pEmxArray;
  emxArray->data = (uint64_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int32_T *)malloc(sizeof(int32_T) * (uint32_T)numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static void MATLAB0_sortIdx(const emxArray_real_T_MATLAB0_T *x,
  emxArray_int32_T_MATLAB0_T *idx)
{
  static emxArray_int32_T_MATLAB0_T *iwork;
  static real_T x_0;
  static int32_T i;
  static int32_T i2;
  static int32_T j;
  static int32_T k;
  static int32_T n;
  static int32_T p;
  static int32_T pEnd;
  static int32_T q;
  static int32_T qEnd;
  int32_T b_k;
  int32_T kEnd;
  n = x->size[1] + 1;
  kEnd = idx->size[0] * idx->size[1];
  idx->size[0] = 1;
  idx->size[1] = x->size[1];
  MATLA_emxEnsureCapacity_int32_T(idx, kEnd);
  kEnd = x->size[1];
  if (kEnd - 1 >= 0) {
    memset(&idx->data[0], 0, (uint32_T)kEnd * sizeof(int32_T));
  }

  if (x->size[1] != 0) {
    MATLAB0_emxInit_int32_T(&iwork, 1);
    kEnd = iwork->size[0];
    iwork->size[0] = x->size[1];
    MATLA_emxEnsureCapacity_int32_T(iwork, kEnd);
    i = x->size[1] - 1;
    for (b_k = 1; b_k <= i; b_k += 2) {
      x_0 = x->data[b_k];
      if ((x->data[b_k - 1] <= x_0) || rtIsNaN(x_0)) {
        idx->data[b_k - 1] = b_k;
        idx->data[b_k] = b_k + 1;
      } else {
        idx->data[b_k - 1] = b_k + 1;
        idx->data[b_k] = b_k;
      }
    }

    if (((uint32_T)x->size[1] & 1U) != 0U) {
      idx->data[x->size[1] - 1] = x->size[1];
    }

    i = 2;
    while (i < n - 1) {
      i2 = i << 1;
      j = 1;
      pEnd = i + 1;
      while (pEnd < n) {
        p = j;
        q = pEnd - 1;
        qEnd = j + i2;
        if (qEnd > n) {
          qEnd = n;
        }

        k = 0;
        kEnd = qEnd - j;
        while (k + 1 <= kEnd) {
          x_0 = x->data[idx->data[q] - 1];
          b_k = idx->data[p - 1];
          if ((x->data[b_k - 1] <= x_0) || rtIsNaN(x_0)) {
            iwork->data[k] = b_k;
            p++;
            if (p == pEnd) {
              while (q + 1 < qEnd) {
                k++;
                iwork->data[k] = idx->data[q];
                q++;
              }
            }
          } else {
            iwork->data[k] = idx->data[q];
            q++;
            if (q + 1 == qEnd) {
              while (p < pEnd) {
                k++;
                iwork->data[k] = idx->data[p - 1];
                p++;
              }
            }
          }

          k++;
        }

        for (pEnd = 0; pEnd < kEnd; pEnd++) {
          idx->data[(j + pEnd) - 1] = iwork->data[pEnd];
        }

        j = qEnd;
        pEnd = qEnd + i;
      }

      i = i2;
    }

    MATLAB0_emxFree_int32_T(&iwork);
  }
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static void MATLAB0_merge(int32_T idx_data[], int32_T x_data[], int32_T offset,
  int32_T np, int32_T nq, int32_T iwork_data[], int32_T xwork_data[])
{
  static int32_T n;
  static int32_T p;
  static int32_T q;
  int32_T exitg1;
  int32_T iout;
  int32_T offset1;
  int32_T tmp;
  if (nq != 0) {
    n = np + nq;
    for (iout = 0; iout < n; iout++) {
      tmp = offset + iout;
      iwork_data[iout] = idx_data[tmp];
      xwork_data[iout] = x_data[tmp];
    }

    p = 0;
    q = np;
    iout = offset - 1;
    do {
      exitg1 = 0;
      iout++;
      if (xwork_data[p] <= xwork_data[q]) {
        idx_data[iout] = iwork_data[p];
        x_data[iout] = xwork_data[p];
        if (p + 1 < np) {
          p++;
        } else {
          exitg1 = 1;
        }
      } else {
        idx_data[iout] = iwork_data[q];
        x_data[iout] = xwork_data[q];
        if (q + 1 < n) {
          q++;
        } else {
          offset1 = iout - p;
          for (iout = p + 1; iout <= np; iout++) {
            tmp = offset1 + iout;
            idx_data[tmp] = iwork_data[iout - 1];
            x_data[tmp] = xwork_data[iout - 1];
          }

          exitg1 = 1;
        }
      }
    } while (exitg1 == 0);
  }
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static void MATLAB0_sort(int32_T x_data[], const int32_T *x_size, int32_T
  idx_data[], int32_T *idx_size)
{
  static int32_T b_idx_data[44];
  static int32_T b_x_data[44];
  static int32_T iwork_data[44];
  static int32_T vwork_data[44];
  static int32_T xwork_data[44];
  static int32_T perm[4];
  static int32_T x4[4];
  static int32_T b;
  static int32_T bLen;
  static int32_T bLen2;
  static int32_T b_i;
  static int32_T b_j;
  static int32_T b_x_size;
  static int32_T c_k;
  static int32_T d_k;
  static int32_T dim;
  static int32_T g_k;
  static int32_T i;
  static int32_T i1;
  static int32_T i2;
  static int32_T i3;
  static int32_T i4;
  static int32_T j;
  static int32_T k;
  static int32_T nBlocks;
  static int32_T nDone;
  static int32_T nLeft;
  static int32_T nPairs;
  static int32_T nTail;
  static int32_T tailOffset;
  static int32_T vstride;
  static int32_T vwork_size_idx_0;
  static int8_T idx4[4];
  int32_T idx4_tmp;
  int32_T nQuartets_tmp;
  int32_T perm_0;
  int32_T vwork;
  dim = 0;
  if (*x_size != 1) {
    dim = -1;
    perm_0 = *x_size;
  } else {
    perm_0 = 1;
  }

  b = perm_0 - 1;
  vwork_size_idx_0 = perm_0;
  *idx_size = *x_size;
  vstride = 1;
  for (c_k = 0; c_k <= dim; c_k++) {
    vstride *= *x_size;
  }

  for (i = 0; i < 1; i++) {
    for (j = 0; j < vstride; j++) {
      for (k = 0; k <= b; k++) {
        vwork_data[k] = x_data[k * vstride + j];
      }

      b_x_size = vwork_size_idx_0;
      for (perm_0 = 0; perm_0 < vwork_size_idx_0; perm_0++) {
        b_x_data[perm_0] = vwork_data[perm_0];
        b_idx_data[perm_0] = 0;
      }

      if (vwork_size_idx_0 != 0) {
        memset(&b_idx_data[0], 0, (uint32_T)vwork_size_idx_0 * sizeof(int32_T));
        x4[0] = 0;
        idx4[0] = 0;
        x4[1] = 0;
        idx4[1] = 0;
        x4[2] = 0;
        idx4[2] = 0;
        x4[3] = 0;
        idx4[3] = 0;
        nQuartets_tmp = vwork_size_idx_0 >> 2;
        for (b_j = 0; b_j < nQuartets_tmp; b_j++) {
          b_i = b_j << 2;
          idx4[0] = (int8_T)(b_i + 1);
          idx4[1] = (int8_T)(b_i + 2);
          idx4[2] = (int8_T)(b_i + 3);
          idx4[3] = (int8_T)(b_i + 4);
          i1 = b_x_data[b_i];
          x4[0] = i1;
          i2 = b_x_data[b_i + 1];
          x4[1] = i2;
          i3 = b_x_data[b_i + 2];
          x4[2] = i3;
          i4 = b_x_data[b_i + 3];
          x4[3] = i4;
          if (i1 <= i2) {
            i1 = 1;
            i2 = 2;
          } else {
            i1 = 2;
            i2 = 1;
          }

          if (i3 <= i4) {
            i3 = 3;
            i4 = 4;
          } else {
            i3 = 4;
            i4 = 3;
          }

          perm_0 = x4[i3 - 1];
          nDone = x4[i1 - 1];
          if (nDone <= perm_0) {
            nDone = x4[i2 - 1];
            if (nDone <= perm_0) {
              perm_0 = i1;
              nDone = i2;
              nLeft = i3;
              vwork = i4;
            } else if (nDone <= x4[i4 - 1]) {
              perm_0 = i1;
              nDone = i3;
              nLeft = i2;
              vwork = i4;
            } else {
              perm_0 = i1;
              nDone = i3;
              nLeft = i4;
              vwork = i2;
            }
          } else {
            perm_0 = x4[i4 - 1];
            if (nDone <= perm_0) {
              if (x4[i2 - 1] <= perm_0) {
                perm_0 = i3;
                nDone = i1;
                nLeft = i2;
                vwork = i4;
              } else {
                perm_0 = i3;
                nDone = i1;
                nLeft = i4;
                vwork = i2;
              }
            } else {
              perm_0 = i3;
              nDone = i4;
              nLeft = i1;
              vwork = i2;
            }
          }

          b_idx_data[b_i] = idx4[perm_0 - 1];
          b_idx_data[b_i + 1] = idx4[nDone - 1];
          b_idx_data[b_i + 2] = idx4[nLeft - 1];
          b_idx_data[b_i + 3] = idx4[vwork - 1];
          b_x_data[b_i] = x4[perm_0 - 1];
          b_x_data[b_i + 1] = x4[nDone - 1];
          b_x_data[b_i + 2] = x4[nLeft - 1];
          b_x_data[b_i + 3] = x4[vwork - 1];
        }

        nDone = nQuartets_tmp << 2;
        nLeft = vwork_size_idx_0 - nDone;
        if (nLeft > 0) {
          for (d_k = 0; d_k < nLeft; d_k++) {
            idx4_tmp = nDone + d_k;
            idx4[d_k] = (int8_T)(idx4_tmp + 1);
            x4[d_k] = b_x_data[idx4_tmp];
          }

          perm[1] = 0;
          perm[2] = 0;
          perm[3] = 0;
          if (nLeft == 1) {
            perm[0] = 1;
          } else if (nLeft == 2) {
            if (x4[0] <= x4[1]) {
              perm[0] = 1;
              perm[1] = 2;
            } else {
              perm[0] = 2;
              perm[1] = 1;
            }
          } else if (x4[0] <= x4[1]) {
            if (x4[1] <= x4[2]) {
              perm[0] = 1;
              perm[1] = 2;
              perm[2] = 3;
            } else if (x4[0] <= x4[2]) {
              perm[0] = 1;
              perm[1] = 3;
              perm[2] = 2;
            } else {
              perm[0] = 3;
              perm[1] = 1;
              perm[2] = 2;
            }
          } else if (x4[0] <= x4[2]) {
            perm[0] = 2;
            perm[1] = 1;
            perm[2] = 3;
          } else if (x4[1] <= x4[2]) {
            perm[0] = 2;
            perm[1] = 3;
            perm[2] = 1;
          } else {
            perm[0] = 3;
            perm[1] = 2;
            perm[2] = 1;
          }

          for (idx4_tmp = 0; idx4_tmp < nLeft; idx4_tmp++) {
            perm_0 = perm[idx4_tmp];
            vwork = nDone + idx4_tmp;
            b_idx_data[vwork] = idx4[perm_0 - 1];
            b_x_data[vwork] = x4[perm_0 - 1];
          }
        }

        if (vwork_size_idx_0 > 1) {
          memset(&iwork_data[0], 0, (uint32_T)vwork_size_idx_0 * sizeof(int32_T));
          memset(&xwork_data[0], 0, (uint32_T)vwork_size_idx_0 * sizeof(int32_T));
          nBlocks = nQuartets_tmp;
          bLen = 4;
          while (nBlocks > 1) {
            if (((uint32_T)nBlocks & 1U) != 0U) {
              nBlocks--;
              tailOffset = bLen * nBlocks;
              nTail = vwork_size_idx_0 - tailOffset;
              if (nTail > bLen) {
                MATLAB0_merge(b_idx_data, b_x_data, tailOffset, bLen, nTail -
                              bLen, iwork_data, xwork_data);
              }
            }

            bLen2 = bLen << 1;
            nPairs = nBlocks >> 1;
            for (g_k = 0; g_k < nPairs; g_k++) {
              MATLAB0_merge(b_idx_data, b_x_data, g_k * bLen2, bLen, bLen,
                            iwork_data, xwork_data);
            }

            bLen = bLen2;
            nBlocks = nPairs;
          }

          if (vwork_size_idx_0 > bLen) {
            MATLAB0_merge(b_idx_data, b_x_data, 0, bLen, vwork_size_idx_0 - bLen,
                          iwork_data, xwork_data);
          }
        }
      }

      for (perm_0 = 0; perm_0 < b_x_size; perm_0++) {
        vwork_data[perm_0] = b_x_data[perm_0];
      }

      for (nQuartets_tmp = 0; nQuartets_tmp <= b; nQuartets_tmp++) {
        perm_0 = nQuartets_tmp * vstride + j;
        x_data[perm_0] = b_x_data[nQuartets_tmp];
        idx_data[perm_0] = b_idx_data[nQuartets_tmp];
      }
    }
  }
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static void MATLAB0_do_vectors(const real_T a[44], const
  emxArray_real_T_MATLAB0_T *b, real_T c_data[], int32_T c_size[2], int32_T
  ia_data[], int32_T *ia_size, int32_T *ib_size)
{
  static emxArray_int32_T_44_MATLAB0_T ia_data_0;
  static emxArray_int32_T_MATLAB0_T *bperm;
  static real_T a_0;
  static real_T ak;
  static real_T bk;
  static int32_T aperm[44];
  static int32_T iwork[44];
  static int32_T b_ialast;
  static int32_T b_iblast;
  static int32_T b_k;
  static int32_T b_p;
  static int32_T c_ialast;
  static int32_T c_k;
  static int32_T d_k;
  static int32_T e;
  static int32_T i;
  static int32_T i2;
  static int32_T iafirst;
  static int32_T ialast;
  static int32_T iblast;
  static int32_T j;
  static int32_T k;
  static int32_T kEnd;
  static int32_T nc;
  static int32_T nia;
  static int32_T pEnd;
  static int32_T q;
  static int32_T qEnd;
  boolean_T tmp;
  c_size[0] = 1;
  *ib_size = 0;
  for (c_k = 0; c_k <= 42; c_k += 2) {
    a_0 = a[c_k + 1];
    if ((a[c_k] <= a_0) || rtIsNaN(a_0)) {
      aperm[c_k] = c_k + 1;
      aperm[c_k + 1] = c_k + 2;
    } else {
      aperm[c_k] = c_k + 2;
      aperm[c_k + 1] = c_k + 1;
    }
  }

  i = 2;
  while (i < 44) {
    i2 = i << 1;
    j = 1;
    for (pEnd = i + 1; pEnd < 45; pEnd = qEnd + i) {
      b_p = j;
      q = pEnd - 1;
      qEnd = j + i2;
      if (qEnd > 45) {
        qEnd = 45;
      }

      b_k = 0;
      kEnd = qEnd - j;
      while (b_k + 1 <= kEnd) {
        a_0 = a[aperm[q] - 1];
        k = aperm[b_p - 1];
        if ((a[k - 1] <= a_0) || rtIsNaN(a_0)) {
          iwork[b_k] = k;
          b_p++;
          if (b_p == pEnd) {
            while (q + 1 < qEnd) {
              b_k++;
              iwork[b_k] = aperm[q];
              q++;
            }
          }
        } else {
          iwork[b_k] = aperm[q];
          q++;
          if (q + 1 == qEnd) {
            while (b_p < pEnd) {
              b_k++;
              iwork[b_k] = aperm[b_p - 1];
              b_p++;
            }
          }
        }

        b_k++;
      }

      for (d_k = 0; d_k < kEnd; d_k++) {
        aperm[(j + d_k) - 1] = iwork[d_k];
      }

      j = qEnd;
    }

    i = i2;
  }

  MATLAB0_emxInit_int32_T(&bperm, 2);
  MATLAB0_sortIdx(b, bperm);
  nc = 0;
  nia = -1;
  iafirst = 0;
  ialast = 1;
  iblast = 1;
  while ((ialast <= 44) && (iblast <= b->size[1])) {
    b_ialast = ialast;
    ak = a[aperm[ialast - 1] - 1];
    while ((b_ialast < 44) && (a[aperm[b_ialast] - 1] == ak)) {
      b_ialast++;
    }

    ialast = b_ialast;
    b_iblast = iblast;
    bk = b->data[bperm->data[iblast - 1] - 1];
    while ((b_iblast < b->size[1]) && (b->data[bperm->data[b_iblast] - 1] == bk))
    {
      b_iblast++;
    }

    iblast = b_iblast;
    if (ak == bk) {
      ialast = b_ialast + 1;
      iafirst = b_ialast;
      iblast = b_iblast + 1;
    } else {
      if (rtIsNaN(bk)) {
        tmp = !rtIsNaN(ak);
      } else {
        tmp = ((!rtIsNaN(ak)) && (ak < bk));
      }

      if (tmp) {
        nc++;
        nia++;
        ia_data[nia] = aperm[iafirst];
        ialast = b_ialast + 1;
        iafirst = b_ialast;
      } else {
        iblast = b_iblast + 1;
      }
    }
  }

  MATLAB0_emxFree_int32_T(&bperm);
  while (ialast <= 44) {
    c_ialast = ialast;
    while ((c_ialast < 44) && (a[aperm[ialast - 1] - 1] == a[aperm[c_ialast] - 1]))
    {
      c_ialast++;
    }

    nc++;
    nia++;
    ia_data[nia] = aperm[iafirst];
    ialast = c_ialast + 1;
    iafirst = c_ialast;
  }

  if (nia + 1 < 1) {
    e = -1;
  } else {
    e = nia;
  }

  *ia_size = e + 1;
  MATLAB0_sort(ia_data, ia_size, ia_data_0.data, &ia_data_0.size);
  e = (uint8_T)(nia + 1);
  for (k = 0; k < e; k++) {
    c_data[k] = a[ia_data[k] - 1];
  }

  if (nc < 1) {
    c_size[1] = 0;
  } else {
    c_size[1] = nc;
  }
}

static void MATLAB0_emxFree_uint64_T(emxArray_uint64_T_MATLAB0_T **pEmxArray)
{
  if (*pEmxArray != (emxArray_uint64_T_MATLAB0_T *)NULL) {
    if (((*pEmxArray)->data != (uint64_T *)NULL) && (*pEmxArray)->canFreeData) {
      free((*pEmxArray)->data);
    }

    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = (emxArray_uint64_T_MATLAB0_T *)NULL;
  }
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static void MATLAB0_merge_n(int32_T idx_data[], real_T x_data[], int32_T offset,
  int32_T np, int32_T nq, int32_T iwork_data[], real_T xwork_data[])
{
  static int32_T n;
  static int32_T p;
  static int32_T q;
  int32_T exitg1;
  int32_T iout;
  int32_T offset1;
  int32_T tmp;
  if (nq != 0) {
    n = np + nq;
    for (iout = 0; iout < n; iout++) {
      tmp = offset + iout;
      iwork_data[iout] = idx_data[tmp];
      xwork_data[iout] = x_data[tmp];
    }

    p = 0;
    q = np;
    iout = offset - 1;
    do {
      exitg1 = 0;
      iout++;
      if (xwork_data[p] <= xwork_data[q]) {
        idx_data[iout] = iwork_data[p];
        x_data[iout] = xwork_data[p];
        if (p + 1 < np) {
          p++;
        } else {
          exitg1 = 1;
        }
      } else {
        idx_data[iout] = iwork_data[q];
        x_data[iout] = xwork_data[q];
        if (q + 1 < n) {
          q++;
        } else {
          offset1 = iout - p;
          for (iout = p + 1; iout <= np; iout++) {
            tmp = offset1 + iout;
            idx_data[tmp] = iwork_data[iout - 1];
            x_data[tmp] = xwork_data[iout - 1];
          }

          exitg1 = 1;
        }
      }
    } while (exitg1 == 0);
  }
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static void MATLAB0_sort_o(real_T x_data[], const int32_T *x_size)
{
  static real_T b_x_data[44];
  static real_T vwork_data[44];
  static real_T xwork_data[44];
  static real_T x4[4];
  static real_T tmp;
  static real_T tmp_0;
  static int32_T idx_data[44];
  static int32_T iwork_data[44];
  static int32_T perm[4];
  static int32_T b;
  static int32_T bLen;
  static int32_T bLen2;
  static int32_T b_x_size;
  static int32_T c_k;
  static int32_T d_k;
  static int32_T dim;
  static int32_T e_k;
  static int32_T g_k;
  static int32_T h;
  static int32_T i;
  static int32_T i1;
  static int32_T i2;
  static int32_T i3;
  static int32_T i4;
  static int32_T ib;
  static int32_T idx_size;
  static int32_T itmp;
  static int32_T j;
  static int32_T k;
  static int32_T m;
  static int32_T nBlocks;
  static int32_T nNaNs;
  static int32_T nPairs;
  static int32_T nTail;
  static int32_T quartetOffset;
  static int32_T tailOffset;
  static int32_T vstride;
  static int32_T vwork_size_idx_0;
  static int8_T idx4[4];
  int32_T idx_tmp;
  int32_T perm_0;
  int32_T perm_1;
  int32_T vwork;
  dim = 0;
  if (*x_size != 1) {
    dim = -1;
    idx_tmp = *x_size;
  } else {
    idx_tmp = 1;
  }

  b = idx_tmp - 1;
  vwork_size_idx_0 = idx_tmp;
  vstride = 1;
  for (c_k = 0; c_k <= dim; c_k++) {
    vstride *= *x_size;
  }

  for (i = 0; i < 1; i++) {
    for (j = 0; j < vstride; j++) {
      for (k = 0; k <= b; k++) {
        vwork_data[k] = x_data[k * vstride + j];
      }

      b_x_size = vwork_size_idx_0;
      for (idx_tmp = 0; idx_tmp < vwork_size_idx_0; idx_tmp++) {
        b_x_data[idx_tmp] = vwork_data[idx_tmp];
      }

      if (vwork_size_idx_0 != 0) {
        idx_size = vwork_size_idx_0;
        x4[0] = 0.0;
        idx4[0] = 0;
        x4[1] = 0.0;
        idx4[1] = 0;
        x4[2] = 0.0;
        idx4[2] = 0;
        x4[3] = 0.0;
        idx4[3] = 0;
        memset(&idx_data[0], 0, (uint32_T)vwork_size_idx_0 * sizeof(int32_T));
        memset(&xwork_data[0], 0, (uint32_T)vwork_size_idx_0 * sizeof(real_T));
        nNaNs = 0;
        ib = 0;
        for (d_k = 0; d_k < vwork_size_idx_0; d_k++) {
          if (rtIsNaN(b_x_data[d_k])) {
            idx_tmp = (vwork_size_idx_0 - nNaNs) - 1;
            idx_data[idx_tmp] = d_k + 1;
            xwork_data[idx_tmp] = b_x_data[d_k];
            nNaNs++;
          } else {
            ib++;
            idx4[ib - 1] = (int8_T)(d_k + 1);
            x4[ib - 1] = b_x_data[d_k];
            if (ib == 4) {
              quartetOffset = d_k - nNaNs;
              if (x4[0] <= x4[1]) {
                i1 = 1;
                i2 = 2;
              } else {
                i1 = 2;
                i2 = 1;
              }

              if (x4[2] <= x4[3]) {
                i3 = 3;
                i4 = 4;
              } else {
                i3 = 4;
                i4 = 3;
              }

              tmp = x4[i3 - 1];
              tmp_0 = x4[i1 - 1];
              if (tmp_0 <= tmp) {
                tmp_0 = x4[i2 - 1];
                if (tmp_0 <= tmp) {
                  perm_0 = i1;
                  ib = i2;
                  m = i3;
                  perm_1 = i4;
                } else if (tmp_0 <= x4[i4 - 1]) {
                  perm_0 = i1;
                  ib = i3;
                  m = i2;
                  perm_1 = i4;
                } else {
                  perm_0 = i1;
                  ib = i3;
                  m = i4;
                  perm_1 = i2;
                }
              } else {
                tmp = x4[i4 - 1];
                if (tmp_0 <= tmp) {
                  if (x4[i2 - 1] <= tmp) {
                    perm_0 = i3;
                    ib = i1;
                    m = i2;
                    perm_1 = i4;
                  } else {
                    perm_0 = i3;
                    ib = i1;
                    m = i4;
                    perm_1 = i2;
                  }
                } else {
                  perm_0 = i3;
                  ib = i4;
                  m = i1;
                  perm_1 = i2;
                }
              }

              idx_data[quartetOffset - 3] = idx4[perm_0 - 1];
              idx_data[quartetOffset - 2] = idx4[ib - 1];
              idx_data[quartetOffset - 1] = idx4[m - 1];
              idx_data[quartetOffset] = idx4[perm_1 - 1];
              b_x_data[quartetOffset - 3] = x4[perm_0 - 1];
              b_x_data[quartetOffset - 2] = x4[ib - 1];
              b_x_data[quartetOffset - 1] = x4[m - 1];
              b_x_data[quartetOffset] = x4[perm_1 - 1];
              ib = 0;
            }
          }
        }

        perm_1 = vwork_size_idx_0 - nNaNs;
        if (ib > 0) {
          perm[1] = 0;
          perm[2] = 0;
          perm[3] = 0;
          if (ib == 1) {
            perm[0] = 1;
          } else if (ib == 2) {
            if (x4[0] <= x4[1]) {
              perm[0] = 1;
              perm[1] = 2;
            } else {
              perm[0] = 2;
              perm[1] = 1;
            }
          } else if (x4[0] <= x4[1]) {
            if (x4[1] <= x4[2]) {
              perm[0] = 1;
              perm[1] = 2;
              perm[2] = 3;
            } else if (x4[0] <= x4[2]) {
              perm[0] = 1;
              perm[1] = 3;
              perm[2] = 2;
            } else {
              perm[0] = 3;
              perm[1] = 1;
              perm[2] = 2;
            }
          } else if (x4[0] <= x4[2]) {
            perm[0] = 2;
            perm[1] = 1;
            perm[2] = 3;
          } else if (x4[1] <= x4[2]) {
            perm[0] = 2;
            perm[1] = 3;
            perm[2] = 1;
          } else {
            perm[0] = 3;
            perm[1] = 2;
            perm[2] = 1;
          }

          h = (uint8_T)ib;
          for (e_k = 0; e_k < h; e_k++) {
            perm_0 = perm[e_k];
            idx_tmp = (perm_1 - ib) + e_k;
            idx_data[idx_tmp] = idx4[perm_0 - 1];
            b_x_data[idx_tmp] = x4[perm_0 - 1];
          }
        }

        m = nNaNs >> 1;
        for (perm_0 = 0; perm_0 < m; perm_0++) {
          vwork = perm_1 + perm_0;
          itmp = idx_data[vwork];
          idx_tmp = (vwork_size_idx_0 - perm_0) - 1;
          idx_data[vwork] = idx_data[idx_tmp];
          idx_data[idx_tmp] = itmp;
          b_x_data[vwork] = xwork_data[idx_tmp];
          b_x_data[idx_tmp] = xwork_data[vwork];
        }

        if (((uint32_T)nNaNs & 1U) != 0U) {
          vwork = perm_1 + m;
          b_x_data[vwork] = xwork_data[vwork];
        }

        if (perm_1 > 1) {
          if (vwork_size_idx_0 - 1 >= 0) {
            memset(&iwork_data[0], 0, (uint32_T)vwork_size_idx_0 * sizeof
                   (int32_T));
          }

          nBlocks = perm_1 >> 2;
          bLen = 4;
          while (nBlocks > 1) {
            if (((uint32_T)nBlocks & 1U) != 0U) {
              nBlocks--;
              tailOffset = bLen * nBlocks;
              nTail = perm_1 - tailOffset;
              if (nTail > bLen) {
                MATLAB0_merge_n(idx_data, b_x_data, tailOffset, bLen, nTail -
                                bLen, iwork_data, xwork_data);
              }
            }

            bLen2 = bLen << 1;
            nPairs = nBlocks >> 1;
            for (g_k = 0; g_k < nPairs; g_k++) {
              MATLAB0_merge_n(idx_data, b_x_data, g_k * bLen2, bLen, bLen,
                              iwork_data, xwork_data);
            }

            bLen = bLen2;
            nBlocks = nPairs;
          }

          if (perm_1 > bLen) {
            MATLAB0_merge_n(idx_data, b_x_data, 0, bLen, perm_1 - bLen,
                            iwork_data, xwork_data);
          }
        }
      }

      for (idx_tmp = 0; idx_tmp < b_x_size; idx_tmp++) {
        vwork_data[idx_tmp] = b_x_data[idx_tmp];
      }

      for (vwork = 0; vwork <= b; vwork++) {
        x_data[j + vwork * vstride] = b_x_data[vwork];
      }
    }
  }
}

static void MATLAB_emxEnsureCapacity_char_T(emxArray_char_T_MATLAB0_T *emxArray,
  int32_T oldNumel)
{
  int32_T i;
  int32_T newNumel;
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  newNumel = 1;
  for (i = 0; i < emxArray->numDimensions; i++) {
    newNumel *= emxArray->size[i];
  }

  if (newNumel > emxArray->allocatedSize) {
    i = emxArray->allocatedSize;
    if (i < 16) {
      i = 16;
    }

    while (i < newNumel) {
      if (i > 1073741823) {
        i = MAX_int32_T;
      } else {
        i <<= 1;
      }
    }

    newData = malloc((uint32_T)i * sizeof(char_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(char_T) * (uint32_T)oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = (char_T *)newData;
    emxArray->allocatedSize = i;
    emxArray->canFreeData = true;
  }
}

static void MATL_emxEnsureCapacity_uint64_T(emxArray_uint64_T_MATLAB0_T
  *emxArray, int32_T oldNumel)
{
  int32_T i;
  int32_T newNumel;
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  newNumel = 1;
  for (i = 0; i < emxArray->numDimensions; i++) {
    newNumel *= emxArray->size[i];
  }

  if (newNumel > emxArray->allocatedSize) {
    i = emxArray->allocatedSize;
    if (i < 16) {
      i = 16;
    }

    while (i < newNumel) {
      if (i > 1073741823) {
        i = MAX_int32_T;
      } else {
        i <<= 1;
      }
    }

    newData = malloc((uint32_T)i * sizeof(uint64_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(uint64_T) * (uint32_T)oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = (uint64_T *)newData;
    emxArray->allocatedSize = i;
    emxArray->canFreeData = true;
  }
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static void ReedsSheppConnection_connectInt(const reedsSheppConnection_MATLAB0_T
  *this, const real_T startPose[3], const real_T goalPose[3], real_T
  motionLengths_data[], int32_T motionLengths_size[2], cell_wrap_53_MATLAB0_T
  motionTypes_data[], int32_T motionTypes_size[2], real_T motionCosts_data[],
  int32_T *motionCosts_size, real_T motionDirections_data[], int32_T
  motionDirections_size[2])
{
  static b_cell_wrap_6_MATLAB0_T f[44];
  static b_cell_wrap_6_MATLAB0_T ab;
  static b_cell_wrap_6_MATLAB0_T bb;
  static b_cell_wrap_6_MATLAB0_T cb;
  static b_cell_wrap_6_MATLAB0_T db;
  static b_cell_wrap_6_MATLAB0_T eb;
  static b_cell_wrap_6_MATLAB0_T fb;
  static b_cell_wrap_6_MATLAB0_T g;
  static b_cell_wrap_6_MATLAB0_T gb;
  static b_cell_wrap_6_MATLAB0_T h;
  static b_cell_wrap_6_MATLAB0_T hb;
  static b_cell_wrap_6_MATLAB0_T i;
  static b_cell_wrap_6_MATLAB0_T ib;
  static b_cell_wrap_6_MATLAB0_T j;
  static b_cell_wrap_6_MATLAB0_T jb;
  static b_cell_wrap_6_MATLAB0_T k;
  static b_cell_wrap_6_MATLAB0_T kb;
  static b_cell_wrap_6_MATLAB0_T l;
  static b_cell_wrap_6_MATLAB0_T lb;
  static b_cell_wrap_6_MATLAB0_T m;
  static b_cell_wrap_6_MATLAB0_T mb;
  static b_cell_wrap_6_MATLAB0_T n;
  static b_cell_wrap_6_MATLAB0_T nb;
  static b_cell_wrap_6_MATLAB0_T o;
  static b_cell_wrap_6_MATLAB0_T ob;
  static b_cell_wrap_6_MATLAB0_T p;
  static b_cell_wrap_6_MATLAB0_T pb;
  static b_cell_wrap_6_MATLAB0_T q;
  static b_cell_wrap_6_MATLAB0_T qb;
  static b_cell_wrap_6_MATLAB0_T r;
  static b_cell_wrap_6_MATLAB0_T rb;
  static b_cell_wrap_6_MATLAB0_T s;
  static b_cell_wrap_6_MATLAB0_T sb;
  static b_cell_wrap_6_MATLAB0_T t;
  static b_cell_wrap_6_MATLAB0_T tb;
  static b_cell_wrap_6_MATLAB0_T u;
  static b_cell_wrap_6_MATLAB0_T ub;
  static b_cell_wrap_6_MATLAB0_T v;
  static b_cell_wrap_6_MATLAB0_T vb;
  static b_cell_wrap_6_MATLAB0_T w;
  static b_cell_wrap_6_MATLAB0_T wb;
  static b_cell_wrap_6_MATLAB0_T x;
  static b_cell_wrap_6_MATLAB0_T xb;
  static b_cell_wrap_6_MATLAB0_T y;
  static b_cell_wrap_6_MATLAB0_T yb;
  static emxArray_real_T_MATLAB0_T *hdisabledTypes;
  static real_T c_data[220];
  static real_T tempMotionLengths_data[220];
  static real_T tempMotionTypes_data[220];
  static real_T a__1_data[44];
  static real_T cost_data[44];
  static real_T enabledIdx_data[44];
  static real_T hAllPathTypes[44];
  static real_T enabledPathInd;
  static int32_T iia_data[44];
  static int32_T e;
  static int32_T idAll;
  static int32_T idDis;
  static int32_T idy;
  static int32_T nRows;
  static int32_T ret;
  char_T pathSegments[3];
  static const char_T b_b[3] = { 'a', 'l', 'l' };

  static b_cell_wrap_6_MATLAB0_T g_0[44];
  static emxArray_uint64_T_MATLAB0_T *charVal;
  static real_T u_0;
  static uint64_T b_charVal_data[10];
  static uint64_T b_out;
  static uint64_T out;
  static uint64_T q0;
  static uint64_T qY;
  static int32_T b_idx;
  static int32_T b_j1;
  static int32_T b_j2;
  static int32_T c_idx;
  static int32_T c_j1;
  static int32_T cc;
  static int32_T d_k;
  static int32_T end;
  static int32_T fc;
  static int32_T gc;
  static int32_T hc;
  static int32_T i_0;
  static int32_T ic;
  static int32_T j2;
  static int8_T jc_data[44];
  static boolean_T idy_data[44];
  int32_T b_tmp;
  int32_T loop_ub;
  static const char_T kc[3] = { 'a', 'l', 'l' };

  static const char_T lc[128] = { '\x00', '\x01', '\x02', '\x03', '\x04', '\x05',
    '\x06', '\a', '\b', '\t', '\n', '\v', '\f', '\r', '\x0e', '\x0f', '\x10',
    '\x11', '\x12', '\x13', '\x14', '\x15', '\x16', '\x17', '\x18', '\x19',
    '\x1a', '\x1b', '\x1c', '\x1d', '\x1e', '\x1f', ' ', '!', '\"', '#', '$',
    '%', '&', '\'', '(', ')', '*', '+', ',', '-', '.', '/', '0', '1', '2', '3',
    '4', '5', '6', '7', '8', '9', ':', ';', '<', '=', '>', '?', '@', 'a', 'b',
    'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n', 'o', 'p', 'q',
    'r', 's', 't', 'u', 'v', 'w', 'x', 'y', 'z', '[', '\\', ']', '^', '_', '`',
    'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n', 'o',
    'p', 'q', 'r', 's', 't', 'u', 'v', 'w', 'x', 'y', 'z', '{', '|', '}', '~',
    '\x7f' };

  static const boolean_T wc[128] = { false, false, false, false, false, false,
    false, false, false, true, true, true, true, true, false, false, false,
    false, false, false, false, false, false, false, false, false, false, false,
    true, true, true, true, true, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false, false,
    false };

  static const char_T be[4] = { 'L', 'R', 'S', 'N' };

  static const char_T mc[10] = { 'L', 'p', 'S', 'p', 'L', 'p', ' ', ' ', ' ',
    ' ' };

  static const char_T nc[10] = { 'L', 'n', 'S', 'n', 'L', 'n', ' ', ' ', ' ',
    ' ' };

  static const char_T oc[10] = { 'R', 'p', 'S', 'p', 'R', 'p', ' ', ' ', ' ',
    ' ' };

  static const char_T pc[10] = { 'R', 'n', 'S', 'n', 'R', 'n', ' ', ' ', ' ',
    ' ' };

  static const char_T qc[10] = { 'L', 'p', 'S', 'p', 'R', 'p', ' ', ' ', ' ',
    ' ' };

  static const char_T rc[10] = { 'L', 'n', 'S', 'n', 'R', 'n', ' ', ' ', ' ',
    ' ' };

  static const char_T sc[10] = { 'R', 'p', 'S', 'p', 'L', 'p', ' ', ' ', ' ',
    ' ' };

  static const char_T tc[10] = { 'R', 'n', 'S', 'n', 'L', 'n', ' ', ' ', ' ',
    ' ' };

  static const char_T uc[10] = { 'L', 'p', 'R', 'n', 'L', 'p', ' ', ' ', ' ',
    ' ' };

  static const char_T vc[10] = { 'L', 'n', 'R', 'p', 'L', 'n', ' ', ' ', ' ',
    ' ' };

  static const char_T xc[10] = { 'R', 'p', 'L', 'n', 'R', 'p', ' ', ' ', ' ',
    ' ' };

  static const char_T yc[10] = { 'R', 'n', 'L', 'p', 'R', 'n', ' ', ' ', ' ',
    ' ' };

  static const char_T ad[10] = { 'L', 'p', 'R', 'p', 'L', 'n', ' ', ' ', ' ',
    ' ' };

  static const char_T bd[10] = { 'L', 'n', 'R', 'n', 'L', 'p', ' ', ' ', ' ',
    ' ' };

  static const char_T cd[10] = { 'R', 'p', 'L', 'p', 'R', 'n', ' ', ' ', ' ',
    ' ' };

  static const char_T dd[10] = { 'R', 'n', 'L', 'n', 'R', 'p', ' ', ' ', ' ',
    ' ' };

  static const char_T ed[10] = { 'L', 'p', 'R', 'p', 'L', 'n', 'R', 'n', ' ',
    ' ' };

  static const char_T fd[10] = { 'L', 'n', 'R', 'n', 'L', 'p', 'R', 'p', ' ',
    ' ' };

  static const char_T gd[10] = { 'R', 'p', 'L', 'p', 'R', 'n', 'L', 'n', ' ',
    ' ' };

  static const char_T hd[10] = { 'R', 'n', 'L', 'n', 'R', 'p', 'L', 'p', ' ',
    ' ' };

  static const char_T id[10] = { 'L', 'p', 'R', 'n', 'L', 'n', 'R', 'p', ' ',
    ' ' };

  static const char_T jd[10] = { 'L', 'n', 'R', 'p', 'L', 'p', 'R', 'n', ' ',
    ' ' };

  static const char_T kd[10] = { 'R', 'p', 'L', 'n', 'R', 'n', 'L', 'p', ' ',
    ' ' };

  static const char_T ld[10] = { 'R', 'n', 'L', 'p', 'R', 'p', 'L', 'n', ' ',
    ' ' };

  static const char_T md[10] = { 'L', 'p', 'R', 'n', 'S', 'n', 'L', 'n', ' ',
    ' ' };

  static const char_T nd[10] = { 'L', 'n', 'R', 'p', 'S', 'p', 'L', 'p', ' ',
    ' ' };

  static const char_T od[10] = { 'R', 'p', 'L', 'n', 'S', 'n', 'R', 'n', ' ',
    ' ' };

  static const char_T pd[10] = { 'R', 'n', 'L', 'p', 'S', 'p', 'R', 'p', ' ',
    ' ' };

  static const char_T qd[10] = { 'L', 'p', 'R', 'n', 'S', 'n', 'R', 'n', ' ',
    ' ' };

  static const char_T rd[10] = { 'L', 'n', 'R', 'p', 'S', 'p', 'R', 'p', ' ',
    ' ' };

  static const char_T sd[10] = { 'R', 'p', 'L', 'n', 'S', 'n', 'L', 'n', ' ',
    ' ' };

  static const char_T td[10] = { 'R', 'n', 'L', 'p', 'S', 'p', 'L', 'p', ' ',
    ' ' };

  static const char_T ud[10] = { 'L', 'p', 'S', 'p', 'R', 'p', 'L', 'n', ' ',
    ' ' };

  static const char_T vd[10] = { 'L', 'n', 'S', 'n', 'R', 'n', 'L', 'p', ' ',
    ' ' };

  static const char_T wd[10] = { 'R', 'p', 'S', 'p', 'L', 'p', 'R', 'n', ' ',
    ' ' };

  static const char_T xd[10] = { 'R', 'n', 'S', 'n', 'L', 'n', 'R', 'p', ' ',
    ' ' };

  static const char_T yd[10] = { 'R', 'n', 'S', 'n', 'R', 'n', 'L', 'p', ' ',
    ' ' };

  static const char_T ce[10] = { 'R', 'p', 'S', 'p', 'R', 'p', 'L', 'n', ' ',
    ' ' };

  static const char_T de[10] = { 'L', 'n', 'S', 'n', 'L', 'n', 'R', 'p', ' ',
    ' ' };

  static const char_T ee[10] = { 'L', 'p', 'S', 'p', 'L', 'p', 'R', 'n', ' ',
    ' ' };

  static const char_T fe[10] = { 'L', 'p', 'R', 'n', 'S', 'n', 'L', 'n', 'R',
    'p' };

  static const char_T ge[10] = { 'L', 'n', 'R', 'p', 'S', 'p', 'L', 'p', 'R',
    'n' };

  static const char_T he[10] = { 'R', 'p', 'L', 'n', 'S', 'n', 'R', 'n', 'L',
    'p' };

  static const char_T ie[10] = { 'R', 'n', 'L', 'p', 'S', 'p', 'R', 'p', 'L',
    'n' };

  static emxArray_boolean_T_MATLAB0_T idy_data_0;
  static int32_T a__1_size[2];
  static int32_T tempMotionLengths_size[2];
  static int32_T tempMotionTypes_size[2];
  static int32_T b_charVal_size_idx_1;
  static int32_T b_ib_size;
  static int32_T c_size_idx_0;
  static int32_T c_size_idx_1;
  static int32_T cost_size;
  static int32_T enabledIdx_size;
  static int32_T i_1;
  static int32_T iia_size;
  static int32_T jc_size_idx_0;
  for (d_k = 0; d_k < 3; d_k++) {
    pathSegments[d_k] = lc[(int32_T)kc[d_k]];
  }

  ReedsSheppBuiltins_autonomousRe(startPose, goalPose, this->MinTurningRadius,
    this->ForwardCostInternal, this->ReverseCostInternal, pathSegments,
    this->DisabledPathTypesInternal, cost_data, &cost_size,
    tempMotionLengths_data, tempMotionLengths_size, tempMotionTypes_data,
    tempMotionTypes_size);
  nRows = 1;
  enabledIdx_size = 1;
  enabledIdx_data[0] = 1.0;
  ret = memcmp(&pathSegments[0], &b_b[0], 3);
  if (ret == 0) {
    nRows = 44;
    for (idAll = 0; idAll < 44; idAll++) {
      for (i_0 = 0; i_0 < 10; i_0++) {
        g.f1[i_0] = mc[i_0];
        h.f1[i_0] = nc[i_0];
        i.f1[i_0] = oc[i_0];
        j.f1[i_0] = pc[i_0];
        k.f1[i_0] = qc[i_0];
        l.f1[i_0] = rc[i_0];
        m.f1[i_0] = sc[i_0];
        n.f1[i_0] = tc[i_0];
        o.f1[i_0] = uc[i_0];
        p.f1[i_0] = vc[i_0];
        q.f1[i_0] = xc[i_0];
        r.f1[i_0] = yc[i_0];
        s.f1[i_0] = ad[i_0];
        t.f1[i_0] = bd[i_0];
        u.f1[i_0] = cd[i_0];
        v.f1[i_0] = dd[i_0];
        w.f1[i_0] = ed[i_0];
        x.f1[i_0] = fd[i_0];
        y.f1[i_0] = gd[i_0];
        ab.f1[i_0] = hd[i_0];
        bb.f1[i_0] = id[i_0];
        cb.f1[i_0] = jd[i_0];
        db.f1[i_0] = kd[i_0];
        eb.f1[i_0] = ld[i_0];
        fb.f1[i_0] = md[i_0];
        gb.f1[i_0] = nd[i_0];
        hb.f1[i_0] = od[i_0];
        ib.f1[i_0] = pd[i_0];
        jb.f1[i_0] = qd[i_0];
        kb.f1[i_0] = rd[i_0];
        lb.f1[i_0] = sd[i_0];
        mb.f1[i_0] = td[i_0];
        nb.f1[i_0] = ud[i_0];
        ob.f1[i_0] = vd[i_0];
        pb.f1[i_0] = wd[i_0];
        qb.f1[i_0] = xd[i_0];
        rb.f1[i_0] = yd[i_0];
        sb.f1[i_0] = ce[i_0];
        tb.f1[i_0] = de[i_0];
        ub.f1[i_0] = ee[i_0];
        vb.f1[i_0] = fe[i_0];
        wb.f1[i_0] = ge[i_0];
        xb.f1[i_0] = he[i_0];
        yb.f1[i_0] = ie[i_0];
      }

      f[0] = g;
      f[1] = h;
      f[2] = i;
      f[3] = j;
      f[4] = k;
      f[5] = l;
      f[6] = m;
      f[7] = n;
      f[8] = o;
      f[9] = p;
      f[10] = q;
      f[11] = r;
      f[12] = s;
      f[13] = t;
      f[14] = u;
      f[15] = v;
      f[16] = w;
      f[17] = x;
      f[18] = y;
      f[19] = ab;
      f[20] = bb;
      f[21] = cb;
      f[22] = db;
      f[23] = eb;
      f[24] = fb;
      f[25] = gb;
      f[26] = hb;
      f[27] = ib;
      f[28] = jb;
      f[29] = kb;
      f[30] = lb;
      f[31] = mb;
      f[32] = nb;
      f[33] = ob;
      f[34] = pb;
      f[35] = qb;
      f[36] = rb;
      f[37] = sb;
      f[38] = tb;
      f[39] = ub;
      f[40] = vb;
      f[41] = wb;
      f[42] = xb;
      f[43] = yb;
      c_j1 = 0;
      while ((c_j1 + 1 <= 10) && wc[(int32_T)f[idAll].f1[c_j1]]) {
        c_j1++;
      }

      b_j2 = 9;
      while ((b_j2 + 1 > 0) && wc[(int32_T)f[idAll].f1[b_j2]]) {
        b_j2--;
      }

      if (c_j1 + 1 > b_j2 + 1) {
        ic = 0;
        hc = -1;
      } else {
        ic = c_j1;
        hc = b_j2;
      }

      g_0[0] = g;
      g_0[1] = h;
      g_0[2] = i;
      g_0[3] = j;
      g_0[4] = k;
      g_0[5] = l;
      g_0[6] = m;
      g_0[7] = n;
      g_0[8] = o;
      g_0[9] = p;
      g_0[10] = q;
      g_0[11] = r;
      g_0[12] = s;
      g_0[13] = t;
      g_0[14] = u;
      g_0[15] = v;
      g_0[16] = w;
      g_0[17] = x;
      g_0[18] = y;
      g_0[19] = ab;
      g_0[20] = bb;
      g_0[21] = cb;
      g_0[22] = db;
      g_0[23] = eb;
      g_0[24] = fb;
      g_0[25] = gb;
      g_0[26] = hb;
      g_0[27] = ib;
      g_0[28] = jb;
      g_0[29] = kb;
      g_0[30] = lb;
      g_0[31] = mb;
      g_0[32] = nb;
      g_0[33] = ob;
      g_0[34] = pb;
      g_0[35] = qb;
      g_0[36] = rb;
      g_0[37] = sb;
      g_0[38] = tb;
      g_0[39] = ub;
      g_0[40] = vb;
      g_0[41] = wb;
      g_0[42] = xb;
      g_0[43] = yb;
      loop_ub = hc - ic;
      b_charVal_size_idx_1 = loop_ub + 1;
      for (i_0 = 0; i_0 <= loop_ub; i_0++) {
        b_charVal_data[i_0] = (uint64_T)g_0[idAll].f1[ic + i_0];
      }

      b_out = 5381UL;
      for (c_idx = 0; c_idx < b_charVal_size_idx_1; c_idx++) {
        q0 = b_out << 5;
        qY = q0 + b_out;
        if (qY < q0) {
          qY = MAX_uint64_T;
        }

        b_out = qY + b_charVal_data[c_idx];
        if (b_out < qY) {
          b_out = MAX_uint64_T;
        }
      }

      hAllPathTypes[idAll] = (real_T)b_out;
    }

    MATLAB0_emxInit_real_T(&hdisabledTypes, 2);
    i_0 = hdisabledTypes->size[0] * hdisabledTypes->size[1];
    hdisabledTypes->size[0] = 1;
    hdisabledTypes->size[1] = this->DisabledPathTypesInternal->size[1];
    MATLAB_emxEnsureCapacity_real_T(hdisabledTypes, i_0);
    loop_ub = this->DisabledPathTypesInternal->size[1];
    if (loop_ub - 1 >= 0) {
      memset(&hdisabledTypes->data[0], 0, (uint32_T)loop_ub * sizeof(real_T));
    }

    if ((this->DisabledPathTypesInternal->size[0] != 0) &&
        (this->DisabledPathTypesInternal->size[1] != 0)) {
      b_tmp = this->DisabledPathTypesInternal->size[0] *
        this->DisabledPathTypesInternal->size[1];
      MATLAB0_emxInit_uint64_T(&charVal, 2);
      for (idDis = 0; idDis < b_tmp; idDis++) {
        b_j1 = 0;
        while ((b_j1 + 1 <= this->DisabledPathTypesInternal->data[idDis]
                .f1->size[1]) && wc[(uint8_T)this->
               DisabledPathTypesInternal->data[idDis].f1->data[b_j1] & 127] && (
                !(this->DisabledPathTypesInternal->data[idDis].f1->data[b_j1] ==
                  '\x00'))) {
          b_j1++;
        }

        j2 = this->DisabledPathTypesInternal->data[idDis].f1->size[1] - 1;
        while ((j2 + 1 > 0) && wc[(uint8_T)this->DisabledPathTypesInternal->
               data[idDis].f1->data[j2] & 127] &&
               (!(this->DisabledPathTypesInternal->data[idDis].f1->data[j2] ==
                  '\x00'))) {
          j2--;
        }

        if (b_j1 + 1 > j2 + 1) {
          gc = 0;
          fc = -1;
        } else {
          gc = b_j1;
          fc = j2;
        }

        i_0 = charVal->size[0] * charVal->size[1];
        charVal->size[0] = 1;
        loop_ub = fc - gc;
        charVal->size[1] = loop_ub + 1;
        MATL_emxEnsureCapacity_uint64_T(charVal, i_0);
        for (i_0 = 0; i_0 <= loop_ub; i_0++) {
          charVal->data[i_0] = (uint8_T)this->DisabledPathTypesInternal->
            data[idDis].f1->data[gc + i_0];
        }

        out = 5381UL;
        cc = charVal->size[1];
        for (b_idx = 0; b_idx < cc; b_idx++) {
          q0 = out << 5;
          qY = q0 + out;
          if (qY < q0) {
            qY = MAX_uint64_T;
          }

          out = qY + charVal->data[b_idx];
          if (out < qY) {
            out = MAX_uint64_T;
          }
        }

        hdisabledTypes->data[idDis] = (real_T)out;
      }

      MATLAB0_emxFree_uint64_T(&charVal);
    }

    MATLAB0_do_vectors(hAllPathTypes, hdisabledTypes, a__1_data, a__1_size,
                       iia_data, &iia_size, &b_ib_size);
    MATLAB0_emxFree_real_T(&hdisabledTypes);
    enabledIdx_size = iia_size;
    for (i_0 = 0; i_0 < iia_size; i_0++) {
      enabledIdx_data[i_0] = iia_data[i_0];
    }

    MATLAB0_sort_o(enabledIdx_data, &iia_size);
  }

  *motionCosts_size = nRows;
  for (i_0 = 0; i_0 < nRows; i_0++) {
    motionCosts_data[i_0] = (rtNaN);
  }

  end = cost_size - 1;
  b_tmp = 0;
  for (i_0 = 0; i_0 <= end; i_0++) {
    if (cost_data[i_0] == (rtInf)) {
      b_tmp++;
    }
  }

  jc_size_idx_0 = b_tmp;
  b_tmp = 0;
  for (i_0 = 0; i_0 <= end; i_0++) {
    if (cost_data[i_0] == (rtInf)) {
      jc_data[b_tmp] = (int8_T)i_0;
      b_tmp++;
    }
  }

  for (i_0 = 0; i_0 < jc_size_idx_0; i_0++) {
    cost_data[jc_data[i_0]] = (rtNaN);
  }

  for (i_0 = 0; i_0 < cost_size; i_0++) {
    motionCosts_data[(int32_T)enabledIdx_data[i_0] - 1] = cost_data[i_0];
  }

  motionDirections_size[0] = 5;
  motionDirections_size[1] = nRows;
  loop_ub = 5 * nRows;
  for (i_0 = 0; i_0 < loop_ub; i_0++) {
    motionDirections_data[i_0] = 1.0;
  }

  c_size_idx_1 = tempMotionLengths_size[1];
  b_tmp = 5 * tempMotionLengths_size[1];
  for (i_0 = 0; i_0 < b_tmp; i_0++) {
    c_data[i_0] = tempMotionLengths_data[i_0];
  }

  for (end = 0; end < b_tmp; end++) {
    u_0 = c_data[end];
    if (rtIsNaN(u_0)) {
      c_data[end] = (rtNaN);
    } else if (u_0 < 0.0) {
      c_data[end] = -1.0;
    } else {
      c_data[end] = (u_0 > 0.0);
    }
  }

  for (i_0 = 0; i_0 < c_size_idx_1; i_0++) {
    for (end = 0; end < 5; end++) {
      motionDirections_data[end + 5 * ((int32_T)enabledIdx_data[i_0] - 1)] =
        c_data[5 * i_0 + end];
    }
  }

  end = 5 * motionDirections_size[1] - 1;
  for (i_0 = 0; i_0 <= end; i_0++) {
    if (motionDirections_data[i_0] == 0.0) {
      motionDirections_data[i_0] = 1.0;
    }
  }

  for (i_0 = 0; i_0 < jc_size_idx_0; i_0++) {
    for (end = 0; end < 5; end++) {
      motionDirections_data[end + 5 * jc_data[i_0]] = 1.0;
    }
  }

  motionLengths_size[0] = 5;
  motionLengths_size[1] = nRows;
  for (i_0 = 0; i_0 < loop_ub; i_0++) {
    motionLengths_data[i_0] = (rtNaN);
  }

  c_size_idx_0 = 5;
  c_size_idx_1 = tempMotionLengths_size[1];
  for (loop_ub = 0; loop_ub < b_tmp; loop_ub++) {
    c_data[loop_ub] = fabs(tempMotionLengths_data[loop_ub]);
  }

  for (i_0 = 0; i_0 < c_size_idx_1; i_0++) {
    for (end = 0; end < 5; end++) {
      motionLengths_data[end + 5 * ((int32_T)enabledIdx_data[i_0] - 1)] =
        c_data[5 * i_0 + end];
    }
  }

  for (i_0 = 0; i_0 < jc_size_idx_0; i_0++) {
    for (end = 0; end < 5; end++) {
      motionLengths_data[end + 5 * jc_data[i_0]] = (rtNaN);
    }
  }

  i_0 = motionTypes_size[0] * motionTypes_size[1];
  motionTypes_size[0] = 5;
  motionTypes_size[1] = nRows;
  emxEnsureCapacity_cell_wrap_53(motionTypes_data, motionTypes_size, i_0);
  for (end = 0; end < 5; end++) {
    enabledPathInd = 1.0;
    e = nRows;
    for (idy = 0; idy < nRows; idy++) {
      loop_ub = enabledIdx_size;
      for (i_0 = 0; i_0 < enabledIdx_size; i_0++) {
        idy_data[i_0] = ((real_T)idy + 1.0 == enabledIdx_data[i_0]);
      }

      idy_data_0.data = &idy_data[0];
      idy_data_0.size = &loop_ub;
      idy_data_0.allocatedSize = 44;
      idy_data_0.numDimensions = 1;
      idy_data_0.canFreeData = false;
      if (MATLAB0_any(&idy_data_0)) {
        i_0 = 5 * idy + end;
        i_1 = motionTypes_data[i_0].f1->size[0] * motionTypes_data[i_0].f1->
          size[1];
        motionTypes_data[end + 5 * idy].f1->size[0] = 1;
        motionTypes_data[end + 5 * idy].f1->size[1] = 1;
        MATLAB_emxEnsureCapacity_char_T(motionTypes_data[end + 5 * idy].f1, i_1);
        motionTypes_data[end + 5 * idy].f1->data[0] = be[(int32_T)
          (tempMotionTypes_data[((int32_T)enabledPathInd - 1) * 5 + end] + 1.0)
          - 1];
        enabledPathInd++;
      } else {
        motionTypes_data[end + 5 * idy].f1->size[0] = 1;
        motionTypes_data[end + 5 * idy].f1->size[1] = 0;
      }
    }
  }
}

static void emxInit_reedsSheppPathSegment_4(emxArray_reedsSheppPathSegmen_T
  *pEmxArray)
{
  pEmxArray->size = 0;
}

static void MATL_emxFreeMatrix_cell_wrap_53(cell_wrap_53_MATLAB0_T pMatrix[5])
{
  int32_T i;
  for (i = 0; i < 5; i++) {
    MATL_emxFreeStruct_cell_wrap_53(&pMatrix[i]);
  }
}

static void emxFreeStruct_reedsSheppPathSeg(reedsSheppPathSegment_MATLAB0_T
  *pStruct)
{
  MATL_emxFreeMatrix_cell_wrap_53(pStruct->MotionTypes);
}

static void emxTrim_reedsSheppPathSegment_4(reedsSheppPathSegment_MATLAB0_T
  data[44], int32_T fromIndex, int32_T toIndex)
{
  int32_T i;
  for (i = fromIndex; i < toIndex; i++) {
    emxFreeStruct_reedsSheppPathSeg(&data[i]);
  }
}

static void MATL_emxInitMatrix_cell_wrap_53(cell_wrap_53_MATLAB0_T pMatrix[5])
{
  int32_T i;
  for (i = 0; i < 5; i++) {
    MATL_emxInitStruct_cell_wrap_53(&pMatrix[i]);
  }
}

static void emxInitStruct_reedsSheppPathSeg(reedsSheppPathSegment_MATLAB0_T
  *pStruct)
{
  MATL_emxInitMatrix_cell_wrap_53(pStruct->MotionTypes);
}

static void emxExpand_reedsSheppPathSegment(reedsSheppPathSegment_MATLAB0_T
  data[44], int32_T fromIndex, int32_T toIndex)
{
  int32_T i;
  for (i = fromIndex; i < toIndex; i++) {
    emxInitStruct_reedsSheppPathSeg(&data[i]);
  }
}

static void emxEnsureCapacity_reedsSheppPat(reedsSheppPathSegment_MATLAB0_T
  data[44], const int32_T *size, int32_T oldNumel)
{
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  if (oldNumel > *size) {
    emxTrim_reedsSheppPathSegment_4(data, *size, oldNumel);
  } else if (oldNumel < *size) {
    emxExpand_reedsSheppPathSegment(data, oldNumel, *size);
  }
}

static void MATLA_emxFree_cell_wrap_53_5x44(emxArray_cell_wrap_53_5x44_MA_T
  *pEmxArray)
{
  int32_T i;
  int32_T numEl;
  numEl = 1;
  for (i = 0; i < 2; i++) {
    numEl *= pEmxArray->size[i];
  }

  for (i = 0; i < numEl; i++) {
    MATL_emxFreeStruct_cell_wrap_53(&pEmxArray->data[i]);
  }
}

static void MATLAB0_emxCopy_char_T(emxArray_char_T_MATLAB0_T **dst,
  emxArray_char_T_MATLAB0_T * const *src)
{
  int32_T i;
  int32_T numElDst;
  int32_T numElSrc;
  numElDst = 1;
  numElSrc = 1;
  for (i = 0; i < (*dst)->numDimensions; i++) {
    numElDst *= (*dst)->size[i];
    numElSrc *= (*src)->size[i];
  }

  for (i = 0; i < (*dst)->numDimensions; i++) {
    (*dst)->size[i] = (*src)->size[i];
  }

  MATLAB_emxEnsureCapacity_char_T(*dst, numElDst);
  for (i = 0; i < numElSrc; i++) {
    (*dst)->data[i] = (*src)->data[i];
  }
}

static void MATL_emxCopyStruct_cell_wrap_53(cell_wrap_53_MATLAB0_T *dst, const
  cell_wrap_53_MATLAB0_T *src)
{
  MATLAB0_emxCopy_char_T(&dst->f1, &src->f1);
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static real_T MATLAB0_mod_p(real_T x)
{
  real_T q;
  real_T r;
  boolean_T rEQ0;
  if (rtIsNaN(x) || rtIsInf(x)) {
    r = (rtNaN);
  } else if (x == 0.0) {
    r = 0.0;
  } else {
    r = fmod(x, 6.2831853071795862);
    rEQ0 = (r == 0.0);
    if (!rEQ0) {
      q = fabs(x / 6.2831853071795862);
      rEQ0 = !(fabs(q - floor(q + 0.5)) > 2.2204460492503131E-16 * q);
    }

    if (rEQ0) {
      r = 0.0;
    } else if (x < 0.0) {
      r += 6.2831853071795862;
    }
  }

  return r;
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static void MATLAB0_linspace_j(real_T d1, real_T d2, real_T n,
  emxArray_real_T_MATLAB0_T *y)
{
  static real_T d2scaled;
  static real_T delta1;
  static real_T delta2;
  real_T tmp;
  int32_T c_k;
  int32_T y_tmp_tmp;
  if (!(n >= 0.0)) {
    y->size[0] = 1;
    y->size[1] = 0;
  } else {
    tmp = floor(n);
    c_k = y->size[0] * y->size[1];
    y->size[0] = 1;
    y->size[1] = (int32_T)tmp;
    MATLAB_emxEnsureCapacity_real_T(y, c_k);
    if ((int32_T)tmp >= 1) {
      y_tmp_tmp = (int32_T)tmp - 1;
      y->data[(int32_T)floor(n) - 1] = d2;
      if (y->size[1] >= 2) {
        y->data[0] = d1;
        if (y->size[1] >= 3) {
          if (d1 == -d2) {
            d2scaled = d2 / ((real_T)y->size[1] - 1.0);
            for (c_k = 2; c_k <= y_tmp_tmp; c_k++) {
              y->data[c_k - 1] = (real_T)(((c_k << 1) - y->size[1]) - 1) *
                d2scaled;
            }

            if ((y->size[1] & 1) == 1) {
              y->data[y->size[1] >> 1] = 0.0;
            }
          } else if (((d1 < 0.0) != (d2 < 0.0)) && ((fabs(d1) >
                       8.9884656743115785E+307) || (fabs(d2) >
                       8.9884656743115785E+307))) {
            delta1 = d1 / ((real_T)y->size[1] - 1.0);
            delta2 = d2 / ((real_T)y->size[1] - 1.0);
            y_tmp_tmp = y->size[1];
            for (c_k = 0; c_k <= y_tmp_tmp - 3; c_k++) {
              y->data[c_k + 1] = (((real_T)c_k + 1.0) * delta2 + d1) - ((real_T)
                c_k + 1.0) * delta1;
            }
          } else {
            delta1 = (d2 - d1) / ((real_T)y->size[1] - 1.0);
            y_tmp_tmp = y->size[1];
            for (c_k = 0; c_k <= y_tmp_tmp - 3; c_k++) {
              y->data[c_k + 1] = ((real_T)c_k + 1.0) * delta1 + d1;
            }
          }
        }
      }
    }
  }
}

static void emxFree_reedsSheppPathSegment_4(emxArray_reedsSheppPathSegmen_T
  *pEmxArray)
{
  int32_T i;
  int32_T numEl;
  numEl = 1;
  for (i = 0; i < 1; i++) {
    numEl *= pEmxArray->size;
  }

  for (i = 0; i < numEl; i++) {
    emxFreeStruct_reedsSheppPathSeg(&pEmxArray->data[i]);
  }
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static void M_MapInterface_world2gridImpl_d(const binaryOccupancyMap_MATLAB0_T
  *obj, const emxArray_real_T_MATLAB0_T *worldXY, emxArray_real_T_MATLAB0_T
  *gridInd)
{
  static emxArray_boolean_T_MATLAB0_T originIdx_0;
  static emxArray_boolean_T_MATLAB0_T *originIdx;
  static emxArray_real_T_MATLAB0_T *gridXY;
  static emxArray_real_T_MATLAB0_T *localXY;
  static real_T a[4];
  static real_T x[4];
  static real_T absx;
  static real_T r;
  static real_T r_0;
  static real_T xlimit_idx_0;
  static real_T ylimit_idx_0;
  static int32_T c_k;
  static int32_T nx;
  int32_T exponent;
  int32_T i;
  int32_T loop_ub;
  int32_T originIdx_1;
  MATLAB0_emxInit_real_T(&localXY, 2);
  i = localXY->size[0] * localXY->size[1];
  localXY->size[0] = worldXY->size[0];
  localXY->size[1] = 2;
  MATLAB_emxEnsureCapacity_real_T(localXY, i);
  loop_ub = worldXY->size[0];
  for (i = 0; i < 2; i++) {
    for (nx = 0; nx < loop_ub; nx++) {
      localXY->data[nx + localXY->size[0] * i] = worldXY->data[worldXY->size[0] *
        i + nx] - obj->SharedProperties.LocalOriginInWorld[i];
    }
  }

  xlimit_idx_0 = obj->SharedProperties.GridOriginInLocal[0];
  ylimit_idx_0 = obj->SharedProperties.GridOriginInLocal[1];
  MATLAB0_emxInit_real_T(&gridXY, 2);
  i = gridXY->size[0] * gridXY->size[1];
  gridXY->size[0] = localXY->size[0];
  gridXY->size[1] = 2;
  MATLAB_emxEnsureCapacity_real_T(gridXY, i);
  loop_ub = localXY->size[0];
  for (i = 0; i < loop_ub; i++) {
    gridXY->data[i] = localXY->data[i + localXY->size[0]] + -ylimit_idx_0;
    gridXY->data[i + gridXY->size[0]] = -xlimit_idx_0 + localXY->data[i];
  }

  i = gridInd->size[0] * gridInd->size[1];
  gridInd->size[0] = gridXY->size[0];
  gridInd->size[1] = 2;
  MATLAB_emxEnsureCapacity_real_T(gridInd, i);
  nx = gridXY->size[0] << 1;
  for (i = 0; i < nx; i++) {
    gridInd->data[i] = gridXY->data[i];
  }

  for (i = 0; i < nx; i++) {
    gridInd->data[i] = ceil(gridInd->data[i]);
  }

  x[0] = obj->SharedProperties.GridOriginInLocal[0];
  x[2] = obj->SharedProperties.GridOriginInLocal[1];
  x[1] = obj->SharedProperties.GridOriginInLocal[0] + 650.0;
  x[3] = obj->SharedProperties.GridOriginInLocal[1] + 650.0;
  for (i = 0; i < 4; i++) {
    a[i] = fabs(x[i]);
  }

  i = localXY->size[0] * localXY->size[1];
  localXY->size[0] = gridXY->size[0];
  localXY->size[1] = 2;
  MATLAB_emxEnsureCapacity_real_T(localXY, i);
  for (c_k = 0; c_k < nx; c_k++) {
    localXY->data[c_k] = fabs(gridXY->data[c_k]);
  }

  MATLAB0_emxFree_real_T(&gridXY);
  absx = fabs(MATLAB0_maximum(a));
  if (rtIsInf(absx) || rtIsNaN(absx)) {
    r = (rtNaN);
  } else if (absx < 4.4501477170144028E-308) {
    r = 4.94065645841247E-324;
  } else {
    frexp(absx, &exponent);
    r = ldexp(1.0, exponent - 53);
  }

  MATLAB0_emxInit_boolean_T(&originIdx, 2);
  i = originIdx->size[0] * originIdx->size[1];
  originIdx->size[0] = localXY->size[0];
  originIdx->size[1] = 2;
  MAT_emxEnsureCapacity_boolean_T(originIdx, i);
  r_0 = r * 2.0;
  for (i = 0; i < nx; i++) {
    originIdx->data[i] = (localXY->data[i] < r_0);
  }

  MATLAB0_emxFree_real_T(&localXY);
  originIdx_0 = *originIdx;
  originIdx_1 = nx;
  originIdx_0.size = &originIdx_1;
  originIdx_0.numDimensions = 1;
  if (MATLAB0_any(&originIdx_0)) {
    exponent = nx - 1;
    for (i = 0; i <= exponent; i++) {
      if (originIdx->data[i]) {
        gridInd->data[i] = 1.0;
      }
    }
  }

  MATLAB0_emxFree_boolean_T(&originIdx);
  loop_ub = gridInd->size[0];
  for (i = 0; i < loop_ub; i++) {
    gridInd->data[i] = 651.0 - gridInd->data[i];
  }
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static void binaryOccupancyMap_checkOccup_f(binaryOccupancyMap_MATLAB0_T *obj,
  const emxArray_real_T_MATLAB0_T *varargin_1, emxArray_real_T_MATLAB0_T
  *occupied, emxArray_boolean_T_MATLAB0_T *validIds)
{
  static d_matlabshared_autonomous_int_T *b_obj;
  static emxArray_boolean_T_MATLAB0_T *validInd;
  static emxArray_int32_T_MATLAB0_T *b;
  static emxArray_real_T_MATLAB0_T *ind;
  static emxArray_real_T_MATLAB0_T *mapStart;
  static real_T obj_0;
  static real_T obj_1;
  static real_T varargin_1_0;
  static real_T varargin_1_1;
  int32_T i;
  int32_T loop_ub;
  int32_T trueCount;
  obj_0 = obj->SharedProperties.LocalOriginInWorld[0] +
    obj->SharedProperties.GridOriginInLocal[0];
  obj_1 = obj->SharedProperties.LocalOriginInWorld[1] +
    obj->SharedProperties.GridOriginInLocal[1];
  i = validIds->size[0];
  validIds->size[0] = varargin_1->size[0];
  MAT_emxEnsureCapacity_boolean_T(validIds, i);
  loop_ub = varargin_1->size[0];
  for (i = 0; i < loop_ub; i++) {
    varargin_1_0 = varargin_1->data[i];
    varargin_1_1 = varargin_1->data[i + varargin_1->size[0]];
    validIds->data[i] = ((varargin_1_0 >= obj_0) && (varargin_1_0 <= obj_0 +
      650.0) && (varargin_1_1 >= obj_1) && (varargin_1_1 <= obj_1 + 650.0));
  }

  MATLAB0_emxInit_real_T(&ind, 2);
  M_MapInterface_world2gridImpl_d(obj, varargin_1, ind);
  MATLAB0_emxInit_boolean_T(&validInd, 1);
  i = validInd->size[0];
  validInd->size[0] = ind->size[0];
  MAT_emxEnsureCapacity_boolean_T(validInd, i);
  loop_ub = ind->size[0];
  i = occupied->size[0];
  occupied->size[0] = ind->size[0];
  MATLAB_emxEnsureCapacity_real_T(occupied, i);
  for (i = 0; i < loop_ub; i++) {
    obj_1 = ind->data[i];
    varargin_1_0 = ind->data[i + ind->size[0]];
    validInd->data[i] = ((obj_1 > 0.0) && (obj_1 < 651.0) && (varargin_1_0 > 0.0)
                         && (varargin_1_0 < 651.0));
    occupied->data[i] = -1.0;
  }

  if (MATLAB0_any(validInd)) {
    loop_ub = validInd->size[0] - 1;
    trueCount = 0;
    for (i = 0; i <= loop_ub; i++) {
      if (validInd->data[i]) {
        trueCount++;
      }
    }

    MATLAB0_emxInit_int32_T(&b, 1);
    i = b->size[0];
    b->size[0] = trueCount;
    MATLA_emxEnsureCapacity_int32_T(b, i);
    trueCount = 0;
    for (i = 0; i <= loop_ub; i++) {
      if (validInd->data[i]) {
        b->data[trueCount] = i;
        trueCount++;
      }
    }

    b_obj = obj->Buffer.Index;
    MATLAB0_emxInit_real_T(&mapStart, 2);
    i = mapStart->size[0] * mapStart->size[1];
    mapStart->size[0] = b->size[0];
    mapStart->size[1] = 2;
    MATLAB_emxEnsureCapacity_real_T(mapStart, i);
    loop_ub = b->size[0];
    for (i = 0; i < 2; i++) {
      for (trueCount = 0; trueCount < loop_ub; trueCount++) {
        mapStart->data[trueCount + mapStart->size[0] * i] = (ind->data[ind->
          size[0] * i + b->data[trueCount]] + b_obj->Head[i]) - 1.0;
      }
    }

    loop_ub = mapStart->size[0] << 1;
    i = mapStart->size[0] * mapStart->size[1];
    mapStart->size[1] = 2;
    MATLAB_emxEnsureCapacity_real_T(mapStart, i);
    for (i = 0; i < loop_ub; i++) {
      mapStart->data[i]--;
    }

    if (mapStart->size[0] == 1) {
      i = ind->size[0] * ind->size[1];
      ind->size[0] = 1;
      ind->size[1] = 2;
      MATLAB_emxEnsureCapacity_real_T(ind, i);
      ind->data[0] = MATLAB0_mod(mapStart->data[0], 650.0);
      ind->data[1] = MATLAB0_mod(mapStart->data[1], 650.0);
    } else {
      MATLAB0_expand_mod(mapStart, ind);
    }

    MATLAB0_emxFree_real_T(&mapStart);
    loop_ub = ind->size[0] << 1;
    i = ind->size[0] * ind->size[1];
    ind->size[1] = 2;
    MATLAB_emxEnsureCapacity_real_T(ind, i);
    for (i = 0; i < loop_ub; i++) {
      ind->data[i]++;
    }

    i = validInd->size[0];
    validInd->size[0] = ind->size[0];
    MAT_emxEnsureCapacity_boolean_T(validInd, i);
    loop_ub = ind->size[0];
    for (i = 0; i < loop_ub; i++) {
      validInd->data[i] = obj->Buffer.Buffer[(int32_T)((ind->data[i + ind->size
        [0]] - 1.0) * 650.0 + ind->data[i]) - 1];
    }

    loop_ub = validInd->size[0];
    for (i = 0; i < loop_ub; i++) {
      occupied->data[b->data[i]] = validInd->data[i];
    }

    MATLAB0_emxFree_int32_T(&b);
  }

  MATLAB0_emxFree_boolean_T(&validInd);
  MATLAB0_emxFree_real_T(&ind);
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static void validatorOccupancyMap_checkMapO(const
  validatorOccupancyMap_MATLAB0_T *obj, const emxArray_real_T_MATLAB0_T *stateXY,
  emxArray_boolean_T_MATLAB0_T *validPos)
{
  static emxArray_boolean_T_MATLAB0_T originIdx_0;
  static emxArray_boolean_T_MATLAB0_T *originIdx;
  static emxArray_real_T_MATLAB0_T *gridInd;
  static emxArray_real_T_MATLAB0_T *linIdx;
  static emxArray_real_T_MATLAB0_T *y;
  static real_T obj_0;
  static real_T obj_1;
  static real_T obj_2;
  static real_T obj_3;
  static real_T obj_4;
  static real_T obj_5;
  static real_T stateXY_0;
  static real_T stateXY_1;
  int32_T k;
  int32_T loop_ub;
  int32_T originIdx_1;
  boolean_T b_y;
  boolean_T exitg1;
  obj_0 = obj->MapBounds[0];
  obj_1 = obj->MapBounds[2];
  obj_2 = obj->MapBounds[1];
  obj_3 = obj->MapBounds[3];
  k = validPos->size[0];
  validPos->size[0] = stateXY->size[0];
  MAT_emxEnsureCapacity_boolean_T(validPos, k);
  loop_ub = stateXY->size[0];
  obj_4 = -obj->MapBounds[1];
  obj_5 = -obj->MapBounds[0];
  MATLAB0_emxInit_real_T(&gridInd, 2);
  k = gridInd->size[0] * gridInd->size[1];
  gridInd->size[0] = stateXY->size[0];
  gridInd->size[1] = 2;
  MATLAB_emxEnsureCapacity_real_T(gridInd, k);
  for (k = 0; k < loop_ub; k++) {
    stateXY_0 = stateXY->data[k];
    stateXY_1 = stateXY->data[k + stateXY->size[0]];
    validPos->data[k] = ((stateXY_0 >= obj_0) && (stateXY_0 <= obj_1) &&
                         (stateXY_1 >= obj_2) && (stateXY_1 <= obj_3));
    gridInd->data[k] = obj_4 + stateXY_1;
    gridInd->data[k + gridInd->size[0]] = obj_5 + stateXY_0;
  }

  loop_ub = gridInd->size[0] << 1;
  for (k = 0; k < loop_ub; k++) {
    gridInd->data[k] = ceil(gridInd->data[k]);
  }

  MATLAB0_emxInit_real_T(&y, 2);
  k = y->size[0] * y->size[1];
  y->size[0] = gridInd->size[0];
  y->size[1] = 2;
  MATLAB_emxEnsureCapacity_real_T(y, k);
  for (k = 0; k < loop_ub; k++) {
    y->data[k] = fabs(gridInd->data[k]);
  }

  MATLAB0_emxInit_boolean_T(&originIdx, 2);
  k = originIdx->size[0] * originIdx->size[1];
  originIdx->size[0] = y->size[0];
  originIdx->size[1] = 2;
  MAT_emxEnsureCapacity_boolean_T(originIdx, k);
  for (k = 0; k < loop_ub; k++) {
    originIdx->data[k] = (y->data[k] < 2.2204460492503131E-16);
  }

  MATLAB0_emxFree_real_T(&y);
  originIdx_0 = *originIdx;
  originIdx_1 = loop_ub;
  originIdx_0.size = &originIdx_1;
  originIdx_0.numDimensions = 1;
  if (MATLAB0_any(&originIdx_0)) {
    loop_ub--;
    for (k = 0; k <= loop_ub; k++) {
      if (originIdx->data[k]) {
        gridInd->data[k] = 1.0;
      }
    }
  }

  MATLAB0_emxFree_boolean_T(&originIdx);
  loop_ub = gridInd->size[0];
  for (k = 0; k < loop_ub; k++) {
    gridInd->data[k] = 651.0 - gridInd->data[k];
  }

  MATLAB0_emxInit_real_T(&linIdx, 1);
  k = linIdx->size[0];
  linIdx->size[0] = gridInd->size[0];
  MATLAB_emxEnsureCapacity_real_T(linIdx, k);
  loop_ub = gridInd->size[0];
  for (k = 0; k < loop_ub; k++) {
    linIdx->data[k] = (gridInd->data[k + gridInd->size[0]] - 1.0) * 650.0 +
      gridInd->data[k];
  }

  MATLAB0_emxFree_real_T(&gridInd);
  b_y = true;
  k = 1;
  exitg1 = false;
  while ((!exitg1) && (k <= validPos->size[0])) {
    if (!validPos->data[k - 1]) {
      b_y = false;
      exitg1 = true;
    } else {
      k++;
    }
  }

  if (b_y) {
    k = validPos->size[0];
    validPos->size[0] = linIdx->size[0];
    MAT_emxEnsureCapacity_boolean_T(validPos, k);
    loop_ub = linIdx->size[0];
    for (k = 0; k < loop_ub; k++) {
      validPos->data[k] = !obj->ValidMatrix[(int32_T)linIdx->data[k] - 1];
    }
  } else {
    loop_ub = validPos->size[0] - 1;
    for (k = 0; k <= loop_ub; k++) {
      if (validPos->data[k]) {
        validPos->data[k] = !obj->ValidMatrix[(int32_T)linIdx->data[k] - 1];
      }
    }
  }

  MATLAB0_emxFree_real_T(&linIdx);
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static void MATLAB0_all(const emxArray_boolean_T_MATLAB0_T *x,
  emxArray_boolean_T_MATLAB0_T *y)
{
  int32_T i1;
  int32_T i2;
  int32_T ix;
  int32_T j;
  int32_T loop_ub;
  int32_T vstride;
  boolean_T exitg1;
  j = y->size[0];
  y->size[0] = x->size[0];
  MAT_emxEnsureCapacity_boolean_T(y, j);
  loop_ub = x->size[0];
  vstride = x->size[0];
  i1 = 0;
  i2 = x->size[0];
  for (j = 0; j < loop_ub; j++) {
    y->data[j] = true;
    i1++;
    i2++;
    ix = i1;
    exitg1 = false;
    while ((!exitg1) && ((vstride > 0) && (ix <= i2))) {
      if (!x->data[ix - 1]) {
        y->data[j] = false;
        exitg1 = true;
      } else {
        ix += vstride;
      }
    }
  }
}

static void MATLAB0_binary_expand_op_1(emxArray_boolean_T_MATLAB0_T *in1, const
  emxArray_real_T_MATLAB0_T *in2, const real_T in3[2], const
  emxArray_real_T_MATLAB0_T *in4, const emxArray_real_T_MATLAB0_T *in5)
{
  static emxArray_boolean_T_MATLAB0_T *in2_0;
  static real_T in2_tmp;
  static int32_T i;
  static int32_T stride_1_0;
  static int32_T stride_3_0;
  int32_T i_0;
  int32_T loop_ub;
  int32_T stride_0_0_tmp;
  MATLAB0_emxInit_boolean_T(&in2_0, 2);
  i_0 = in5->size[0] == 1 ? in2->size[0] : in5->size[0];
  loop_ub = i_0 == 1 ? in4->size[0] == 1 ? in2->size[0] : in4->size[0] : i_0;
  i_0 = in2_0->size[0] * in2_0->size[1];
  in2_0->size[0] = loop_ub;
  in2_0->size[1] = 2;
  MAT_emxEnsureCapacity_boolean_T(in2_0, i_0);
  stride_0_0_tmp = (in2->size[0] != 1);
  stride_1_0 = (in4->size[0] != 1);
  stride_3_0 = (in5->size[0] != 1);
  for (i_0 = 0; i_0 < 2; i_0++) {
    for (i = 0; i < loop_ub; i++) {
      in2_tmp = in2->data[((int32_T)in3[i_0] - 1) * in2->size[0] + i *
        stride_0_0_tmp];
      in2_0->data[i + in2_0->size[0] * i_0] = ((in2_tmp >= in4->data[i *
        stride_1_0 + in4->size[0] * i_0]) && (in2_tmp <= in5->data[i *
        stride_3_0 + in5->size[0] * i_0]));
    }
  }

  MATLAB0_all(in2_0, in1);
  MATLAB0_emxFree_boolean_T(&in2_0);
}

static void MATLAB0_binary_expand_op_2(emxArray_boolean_T_MATLAB0_T *in1, const
  emxArray_real_T_MATLAB0_T *in2, const real_T in3[2], const
  emxArray_real_T_MATLAB0_T *in4, const emxArray_real_T_MATLAB0_T *in5, const
  emxArray_boolean_T_MATLAB0_T *in6)
{
  static emxArray_boolean_T_MATLAB0_T *in2_0;
  static emxArray_boolean_T_MATLAB0_T *tmp;
  static real_T in2_tmp;
  static int32_T stride_0_0;
  static int32_T stride_1_0;
  static int32_T stride_3_0;
  int32_T i;
  int32_T loop_ub;
  int32_T stride_0_0_tmp;
  MATLAB0_emxInit_boolean_T(&in2_0, 2);
  i = in5->size[0] == 1 ? in2->size[0] : in5->size[0];
  loop_ub = i == 1 ? in4->size[0] == 1 ? in2->size[0] : in4->size[0] : i;
  i = in2_0->size[0] * in2_0->size[1];
  in2_0->size[0] = loop_ub;
  in2_0->size[1] = 2;
  MAT_emxEnsureCapacity_boolean_T(in2_0, i);
  stride_0_0_tmp = (in2->size[0] != 1);
  stride_1_0 = (in4->size[0] != 1);
  stride_3_0 = (in5->size[0] != 1);
  for (i = 0; i < 2; i++) {
    for (stride_0_0 = 0; stride_0_0 < loop_ub; stride_0_0++) {
      in2_tmp = in2->data[((int32_T)in3[i] - 1) * in2->size[0] + stride_0_0 *
        stride_0_0_tmp];
      in2_0->data[stride_0_0 + in2_0->size[0] * i] = ((in2_tmp >= in4->
        data[stride_0_0 * stride_1_0 + in4->size[0] * i]) && (in2_tmp <=
        in5->data[stride_0_0 * stride_3_0 + in5->size[0] * i]));
    }
  }

  MATLAB0_emxInit_boolean_T(&tmp, 1);
  MATLAB0_all(in2_0, tmp);
  MATLAB0_emxFree_boolean_T(&in2_0);
  loop_ub = in6->size[0] == 1 ? tmp->size[0] : in6->size[0];
  i = in1->size[0];
  in1->size[0] = loop_ub;
  MAT_emxEnsureCapacity_boolean_T(in1, i);
  stride_0_0 = (tmp->size[0] != 1);
  stride_1_0 = (in6->size[0] != 1);
  for (i = 0; i < loop_ub; i++) {
    in1->data[i] = (tmp->data[i * stride_0_0] && in6->data[i * stride_1_0]);
  }

  MATLAB0_emxFree_boolean_T(&tmp);
}

static void MATLAB0_binary_expand_op_5(emxArray_boolean_T_MATLAB0_T *in1, const
  emxArray_real_T_MATLAB0_T *in2, const real_T in3[2], const
  emxArray_real_T_MATLAB0_T *in4, const emxArray_real_T_MATLAB0_T *in5, const
  emxArray_boolean_T_MATLAB0_T *in6, const emxArray_real_T_MATLAB0_T *in7)
{
  static emxArray_boolean_T_MATLAB0_T *in2_0;
  static emxArray_boolean_T_MATLAB0_T *tmp;
  static real_T in2_tmp;
  static int32_T stride_0_0;
  static int32_T stride_1_0;
  static int32_T stride_3_0;
  int32_T i;
  int32_T loop_ub;
  int32_T stride_0_0_tmp;
  MATLAB0_emxInit_boolean_T(&in2_0, 2);
  i = in5->size[0] == 1 ? in2->size[0] : in5->size[0];
  loop_ub = i == 1 ? in4->size[0] == 1 ? in2->size[0] : in4->size[0] : i;
  i = in2_0->size[0] * in2_0->size[1];
  in2_0->size[0] = loop_ub;
  in2_0->size[1] = 2;
  MAT_emxEnsureCapacity_boolean_T(in2_0, i);
  stride_0_0_tmp = (in2->size[0] != 1);
  stride_1_0 = (in4->size[0] != 1);
  stride_3_0 = (in5->size[0] != 1);
  for (i = 0; i < 2; i++) {
    for (stride_0_0 = 0; stride_0_0 < loop_ub; stride_0_0++) {
      in2_tmp = in2->data[((int32_T)in3[i] - 1) * in2->size[0] + stride_0_0 *
        stride_0_0_tmp];
      in2_0->data[stride_0_0 + in2_0->size[0] * i] = ((in2_tmp >= in4->
        data[stride_0_0 * stride_1_0 + in4->size[0] * i]) && (in2_tmp <=
        in5->data[stride_0_0 * stride_3_0 + in5->size[0] * i]));
    }
  }

  MATLAB0_emxInit_boolean_T(&tmp, 1);
  MATLAB0_all(in2_0, tmp);
  MATLAB0_emxFree_boolean_T(&in2_0);
  loop_ub = in7->size[0] == 1 ? in6->size[0] == 1 ? tmp->size[0] : in6->size[0] :
    in7->size[0];
  i = in1->size[0];
  in1->size[0] = loop_ub;
  MAT_emxEnsureCapacity_boolean_T(in1, i);
  stride_0_0 = (tmp->size[0] != 1);
  stride_1_0 = (in6->size[0] != 1);
  stride_0_0_tmp = (in7->size[0] != 1);
  for (i = 0; i < loop_ub; i++) {
    in1->data[i] = (tmp->data[i * stride_0_0] && in6->data[i * stride_1_0] &&
                    (!(in7->data[i * stride_0_0_tmp] != 0.0)));
  }

  MATLAB0_emxFree_boolean_T(&tmp);
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static boolean_T plannerHybridAStar_checkAnalyti(plannerHybridAStar_MATLAB0_T
  *obj, const real_T initialPose[3], const real_T finalPose[3], real_T stepSize,
  const reedsSheppConnection_MATLAB0_T *rsPathObj)
{
  static cell_wrap_53_MATLAB0_T subsetMotionTypes[5];
  static emxArray_boolean_T_MATLAB0_T *idxInBounds;
  static emxArray_boolean_T_MATLAB0_T *isInBounds;
  static emxArray_boolean_T_MATLAB0_T *poses_1;
  static emxArray_boolean_T_MATLAB0_T *tmp;
  static emxArray_boolean_T_MATLAB0_T *tmp_0;
  static emxArray_cell_wrap_53_5x44_MA_T motionTypes;
  static emxArray_real_T_MATLAB0_T *directions;
  static emxArray_real_T_MATLAB0_T *poses;
  static emxArray_real_T_MATLAB0_T *poses_0;
  static emxArray_real_T_MATLAB0_T *samples;
  static emxArray_real_T_MATLAB0_T *ssLowerBounds;
  static emxArray_real_T_MATLAB0_T *ssUpperBounds;
  static emxArray_reedsSheppPathSegmen_T rsPathSegObjs;
  static stateSpaceSE2_MATLAB0_T *c_obj;
  static validatorOccupancyMap_MATLAB0_T *b_obj;
  static real_T motionDirections_data[220];
  static real_T motionLengths_data[220];
  static real_T pathCosts_data[44];
  static real_T ssBounds[6];
  static real_T b_x_data[5];
  static real_T x[5];
  static real_T y[5];
  static real_T xyInd[2];
  static real_T a__3;
  static real_T b_len;
  static real_T b_theta;
  static real_T b_turningRadius;
  static real_T b_y;
  static real_T c_y;
  static real_T cost;
  static real_T costTemp;
  static real_T forwardCost;
  static real_T len;
  static real_T poses_tmp;
  static real_T reverseCost;
  static real_T sz;
  static real_T theta;
  static real_T turningRadius;
  static int32_T tmp_data[5];
  static int32_T tmp_data_0[5];
  static int32_T tmp_data_1[5];
  static int32_T motionDirections_size[2];
  static int32_T motionLengths_size[2];
  static int32_T b;
  static int32_T b_i;
  static int32_T b_itilerow;
  static int32_T b_ntilerows;
  static int32_T b_x_size_idx_0;
  static int32_T b_x_size_idx_1;
  static int32_T e_k;
  static int32_T f_k;
  static int32_T h;
  static int32_T h_k;
  static int32_T i;
  static int32_T i_0;
  static int32_T i_k;
  static int32_T ibmat;
  static int32_T itilerow;
  static int32_T ix;
  static int32_T j_k;
  static int32_T jcol;
  static int32_T k;
  static int32_T minCostIdx;
  static int32_T ntilerows;
  static int32_T partialTrueCount;
  static int32_T pathCosts_size;
  static int32_T trueCount;
  static int32_T trueCount_tmp;
  static uint32_T segTypes[5];
  static int8_T f_data[5];
  static int8_T segmentDirections[5];
  static boolean_T allPathTypes[44];
  static boolean_T forwardPathIdx[5];
  boolean_T exitg1;
  boolean_T forwardPathIdx_0;
  boolean_T result;
  obj->ExpansionPoint[0] = initialPose[0];
  obj->ExpansionPoint[1] = initialPose[1];
  obj->ExpansionPoint[2] = initialPose[2];
  if (obj->CustomAECostFlag == 0.0) {
    turningRadius = obj->MinTurningRadius;
    forwardCost = obj->ForwardCost;
    reverseCost = obj->ReverseCost;
    for (i_0 = 0; i_0 < 44; i_0++) {
      allPathTypes[i_0] = true;
    }

    autonomousReedsSheppSegmentsCodegen_real64(&initialPose[0], 1U, &finalPose[0],
      1U, turningRadius, forwardCost, reverseCost, &allPathTypes[0], 0U, 1U,
      true, 3U, &a__3, &x[0], &y[0]);
    for (i_0 = 0; i_0 < 5; i_0++) {
      obj->AnalyticPathSegments[i_0] = x[i_0];
    }

    for (i_0 = 0; i_0 < 5; i_0++) {
      obj->AnalyticPathTypes[i_0] = y[i_0];
    }

    for (i_0 = 0; i_0 < 5; i_0++) {
      x[i_0] = obj->AnalyticPathSegments[i_0];
    }

    for (k = 0; k < 5; k++) {
      y[k] = fabs(x[k]);
    }

    b_y = y[0];
    for (f_k = 0; f_k < 4; f_k++) {
      b_y += y[f_k + 1];
    }

    obj->AnalyticPathLength = b_y;
  } else {
    MATLA_emxInit_cell_wrap_53_5x44(&motionTypes);
    ReedsSheppConnection_connectInt(rsPathObj, initialPose, finalPose,
      motionLengths_data, motionLengths_size, motionTypes.data, motionTypes.size,
      pathCosts_data, &pathCosts_size, motionDirections_data,
      motionDirections_size);
    emxInit_reedsSheppPathSegment_4(&rsPathSegObjs);
    trueCount = rsPathSegObjs.size;
    rsPathSegObjs.size = pathCosts_size;
    emxEnsureCapacity_reedsSheppPat(rsPathSegObjs.data, &pathCosts_size,
      trueCount);
    MATL_emxInitMatrix_cell_wrap_53(subsetMotionTypes);
    for (b_i = 0; b_i < pathCosts_size; b_i++) {
      MATL_emxCopyStruct_cell_wrap_53(&subsetMotionTypes[0], &motionTypes.data[5
        * b_i]);
      MATL_emxCopyStruct_cell_wrap_53(&subsetMotionTypes[1], &motionTypes.data[1
        + 5 * b_i]);
      MATL_emxCopyStruct_cell_wrap_53(&subsetMotionTypes[2], &motionTypes.data[2
        + 5 * b_i]);
      MATL_emxCopyStruct_cell_wrap_53(&subsetMotionTypes[3], &motionTypes.data[3
        + 5 * b_i]);
      MATL_emxCopyStruct_cell_wrap_53(&subsetMotionTypes[4], &motionTypes.data[4
        + 5 * b_i]);
      rsPathSegObjs.data[b_i].StartPoseInternal[0] = initialPose[0];
      rsPathSegObjs.data[b_i].GoalPoseInternal[0] = finalPose[0];
      rsPathSegObjs.data[b_i].StartPoseInternal[1] = initialPose[1];
      rsPathSegObjs.data[b_i].GoalPoseInternal[1] = finalPose[1];
      theta = MATLAB0_mod_p(initialPose[2]);
      rsPathSegObjs.data[b_i].StartPoseInternal[2] = (real_T)((theta == 0.0) &&
        (initialPose[2] > 0.0)) * 6.2831853071795862 + theta;
      b_theta = MATLAB0_mod_p(finalPose[2]);
      rsPathSegObjs.data[b_i].GoalPoseInternal[2] = (real_T)((b_theta == 0.0) &&
        (finalPose[2] > 0.0)) * 6.2831853071795862 + b_theta;
      for (h = 0; h < 5; h++) {
        rsPathSegObjs.data[b_i].MotionTypes[h].f1->size[0] = 1;
        rsPathSegObjs.data[b_i].MotionTypes[h].f1->size[1] = 0;
        trueCount = 5 * b_i + h;
        rsPathSegObjs.data[b_i].MotionLengths[h] = motionLengths_data[trueCount];
        MATL_emxCopyStruct_cell_wrap_53(&rsPathSegObjs.data[b_i].MotionTypes[h],
          &subsetMotionTypes[h]);
        rsPathSegObjs.data[b_i].MotionDirections[h] =
          motionDirections_data[trueCount];
      }
    }

    MATLA_emxFree_cell_wrap_53_5x44(&motionTypes);
    MATL_emxFreeMatrix_cell_wrap_53(subsetMotionTypes);
    cost = (rtInf);
    minCostIdx = 0;
    b = rsPathSegObjs.size;
    for (i = 0; i < b; i++) {
      b_len = rsPathSegObjs.data[i].MotionLengths[0];
      for (h_k = 0; h_k < 4; h_k++) {
        b_len += rsPathSegObjs.data[i].MotionLengths[h_k + 1];
      }

      if (!rtIsNaN(b_len)) {
        trueCount = 0;
        trueCount_tmp = 0;
        for (i_0 = 0; i_0 < 5; i_0++) {
          forwardPathIdx_0 = (rsPathSegObjs.data[i].MotionDirections[i_0] == 1.0);
          forwardPathIdx[i_0] = forwardPathIdx_0;
          if (forwardPathIdx_0) {
            trueCount_tmp = trueCount + 1;
            trueCount++;
          }
        }

        if (trueCount_tmp == 0) {
          costTemp = 0.0;
        } else {
          trueCount_tmp = 0;
          for (i_0 = 0; i_0 < 5; i_0++) {
            if (forwardPathIdx[i_0]) {
              trueCount_tmp++;
            }
          }

          if (trueCount_tmp == 0) {
            costTemp = 0.0;
          } else {
            partialTrueCount = 0;
            for (i_0 = 0; i_0 < 5; i_0++) {
              if (forwardPathIdx[i_0]) {
                tmp_data_0[partialTrueCount] = i_0;
                partialTrueCount++;
              }
            }

            costTemp = rsPathSegObjs.data[i].MotionLengths[(int8_T)tmp_data_0[0]];
            for (i_k = 2; i_k <= trueCount; i_k++) {
              partialTrueCount = 0;
              for (i_0 = 0; i_0 < 5; i_0++) {
                if (forwardPathIdx[i_0]) {
                  tmp_data_1[partialTrueCount] = i_0;
                  partialTrueCount++;
                }
              }

              costTemp += rsPathSegObjs.data[i].MotionLengths[(int8_T)
                tmp_data_1[i_k - 1]];
            }
          }
        }

        trueCount = 0;
        for (i_0 = 0; i_0 < 5; i_0++) {
          if (!forwardPathIdx[i_0]) {
            trueCount++;
          }
        }

        partialTrueCount = 0;
        for (i_0 = 0; i_0 < 5; i_0++) {
          if (!forwardPathIdx[i_0]) {
            tmp_data[partialTrueCount] = i_0;
            partialTrueCount++;
          }
        }

        b_x_size_idx_0 = 1;
        b_x_size_idx_1 = trueCount;
        for (i_0 = 0; i_0 < trueCount; i_0++) {
          b_x_data[i_0] = rsPathSegObjs.data[i].MotionLengths[tmp_data[i_0]] *
            3.0;
        }

        if (trueCount == 0) {
          c_y = 0.0;
        } else {
          c_y = b_x_data[0];
          for (j_k = 2; j_k <= trueCount; j_k++) {
            c_y += b_x_data[j_k - 1];
          }
        }

        costTemp += c_y;
        if (costTemp < cost) {
          cost = costTemp;
          minCostIdx = i;
        }
      }
    }

    for (i_0 = 0; i_0 < 5; i_0++) {
      obj->AnalyticPathSegments[i_0] = rsPathSegObjs.data[minCostIdx].
        MotionLengths[i_0] * rsPathSegObjs.data[minCostIdx].MotionDirections[i_0];
    }

    len = rsPathSegObjs.data[minCostIdx].MotionLengths[0];
    for (trueCount_tmp = 0; trueCount_tmp < 4; trueCount_tmp++) {
      len += rsPathSegObjs.data[minCostIdx].MotionLengths[trueCount_tmp + 1];
    }

    obj->AnalyticPathLength = len;
    for (i_0 = 0; i_0 < 5; i_0++) {
      obj->AnalyticPathTypes[i_0] = 0.0;
    }

    for (trueCount_tmp = 0; trueCount_tmp < 5; trueCount_tmp++) {
      forwardPathIdx[trueCount_tmp] = false;
      if ((rsPathSegObjs.data[minCostIdx].MotionTypes[trueCount_tmp].f1->size[1]
           == 1) && (rsPathSegObjs.data[minCostIdx].MotionTypes[trueCount_tmp].
                     f1->data[0] == 'R')) {
        forwardPathIdx[trueCount_tmp] = true;
      }
    }

    for (i_0 = 0; i_0 < 5; i_0++) {
      if (forwardPathIdx[i_0]) {
        obj->AnalyticPathTypes[i_0] = 1.0;
      }
    }

    for (trueCount_tmp = 0; trueCount_tmp < 5; trueCount_tmp++) {
      forwardPathIdx[trueCount_tmp] = false;
      if ((rsPathSegObjs.data[minCostIdx].MotionTypes[trueCount_tmp].f1->size[1]
           == 1) && (rsPathSegObjs.data[minCostIdx].MotionTypes[trueCount_tmp].
                     f1->data[0] == 'S')) {
        forwardPathIdx[trueCount_tmp] = true;
      }
    }

    for (i_0 = 0; i_0 < 5; i_0++) {
      if (forwardPathIdx[i_0]) {
        obj->AnalyticPathTypes[i_0] = 2.0;
      }
    }

    for (e_k = 0; e_k < 5; e_k++) {
      forwardPathIdx[e_k] = false;
      if ((rsPathSegObjs.data[minCostIdx].MotionTypes[e_k].f1->size[1] == 1) &&
          (rsPathSegObjs.data[minCostIdx].MotionTypes[e_k].f1->data[0] == 'N'))
      {
        forwardPathIdx[e_k] = true;
      }
    }

    emxFree_reedsSheppPathSegment_4(&rsPathSegObjs);
    for (i_0 = 0; i_0 < 5; i_0++) {
      if (forwardPathIdx[i_0]) {
        obj->AnalyticPathTypes[i_0] = 3.0;
      }
    }
  }

  sz = obj->AnalyticPathLength / stepSize;
  MATLAB0_emxInit_real_T(&samples, 2);
  MATLAB0_linspace_j(stepSize, obj->AnalyticPathLength, sz, samples);
  for (i_0 = 0; i_0 < 5; i_0++) {
    segmentDirections[i_0] = 1;
  }

  trueCount = 0;
  for (i_0 = 0; i_0 < 5; i_0++) {
    if (obj->AnalyticPathSegments[i_0] < 0.0) {
      trueCount++;
    }
  }

  partialTrueCount = 0;
  for (i_0 = 0; i_0 < 5; i_0++) {
    if (obj->AnalyticPathSegments[i_0] < 0.0) {
      f_data[partialTrueCount] = (int8_T)i_0;
      partialTrueCount++;
    }
  }

  for (i_0 = 0; i_0 < trueCount; i_0++) {
    segmentDirections[f_data[i_0]] = -1;
  }

  for (i_0 = 0; i_0 < 5; i_0++) {
    x[i_0] = obj->AnalyticPathSegments[i_0];
  }

  for (trueCount = 0; trueCount < 5; trueCount++) {
    y[trueCount] = fabs(x[trueCount]);
  }

  b_turningRadius = obj->MinTurningRadius;
  for (i_0 = 0; i_0 < 5; i_0++) {
    tmp_data_0[i_0] = segmentDirections[i_0];
  }

  for (i_0 = 0; i_0 < 5; i_0++) {
    poses_tmp = MATLAB0_rt_roundd_snf(obj->AnalyticPathTypes[i_0]);
    if (poses_tmp < 4.294967296E+9) {
      if (poses_tmp >= 0.0) {
        segTypes[i_0] = (uint32_T)poses_tmp;
      } else {
        segTypes[i_0] = 0U;
      }
    } else {
      segTypes[i_0] = MAX_uint32_T;
    }
  }

  MATLAB0_emxInit_real_T(&poses, 2);
  trueCount = poses->size[0] * poses->size[1];
  poses->size[0] = samples->size[1];
  poses->size[1] = 3;
  MATLAB_emxEnsureCapacity_real_T(poses, trueCount);
  MATLAB0_emxInit_real_T(&directions, 1);
  trueCount = directions->size[0];
  directions->size[0] = samples->size[1];
  MATLAB_emxEnsureCapacity_real_T(directions, trueCount);
  autonomousReedsSheppInterpolateSegmentsCodegen_real64(&initialPose[0],
    &finalPose[0], &samples->data[0], (uint32_T)samples->size[1],
    b_turningRadius, &y[0], &tmp_data_0[0], &segTypes[0], &poses->data[0],
    &directions->data[0]);
  MATLAB0_emxFree_real_T(&samples);
  if (poses->size[0] == 0) {
    result = true;
  } else {
    b_obj = obj->StateValidator;
    xyInd[0] = b_obj->XYIndices[0];
    xyInd[1] = b_obj->XYIndices[1];
    c_obj = b_obj->StateSpace;
    for (i_0 = 0; i_0 < 6; i_0++) {
      ssBounds[i_0] = c_obj->StateBoundsInternal[i_0];
    }

    MATLAB0_emxInit_real_T(&ssLowerBounds, 2);
    trueCount = ssLowerBounds->size[0] * ssLowerBounds->size[1];
    ssLowerBounds->size[0] = poses->size[0];
    ssLowerBounds->size[1] = 2;
    MATLAB_emxEnsureCapacity_real_T(ssLowerBounds, trueCount);
    ntilerows = poses->size[0];
    MATLAB0_emxInit_real_T(&ssUpperBounds, 2);
    trueCount = ssUpperBounds->size[0] * ssUpperBounds->size[1];
    ssUpperBounds->size[0] = poses->size[0];
    ssUpperBounds->size[1] = 2;
    MATLAB_emxEnsureCapacity_real_T(ssUpperBounds, trueCount);
    b_ntilerows = poses->size[0];
    for (jcol = 0; jcol < 2; jcol++) {
      ibmat = jcol * ntilerows;
      for (itilerow = 0; itilerow < ntilerows; itilerow++) {
        ssLowerBounds->data[ibmat + itilerow] = ssBounds[jcol];
      }

      for (b_itilerow = 0; b_itilerow < b_ntilerows; b_itilerow++) {
        ssUpperBounds->data[ibmat + b_itilerow] = ssBounds[jcol + 3];
      }
    }

    MATLAB0_emxInit_boolean_T(&isInBounds, 1);
    MATLAB0_emxInit_boolean_T(&idxInBounds, 1);
    MATLAB0_emxInit_boolean_T(&tmp, 1);
    MATLAB0_emxInit_real_T(&poses_0, 2);
    MATLAB0_emxInit_boolean_T(&poses_1, 2);
    if (b_obj->SkipStateValidation) {
      trueCount = poses_0->size[0] * poses_0->size[1];
      poses_0->size[0] = poses->size[0];
      poses_0->size[1] = 2;
      MATLAB_emxEnsureCapacity_real_T(poses_0, trueCount);
      partialTrueCount = poses->size[0];
      for (i_0 = 0; i_0 < 2; i_0++) {
        for (ix = 0; ix < partialTrueCount; ix++) {
          poses_0->data[ix + poses_0->size[0] * i_0] = poses->data[((int32_T)
            xyInd[i_0] - 1) * poses->size[0] + ix];
        }
      }

      validatorOccupancyMap_checkMapO(b_obj, poses_0, idxInBounds);
      MATLAB0_binary_expand_op_1(tmp, poses, xyInd, ssLowerBounds, ssUpperBounds);
      if ((poses->size[0] == ssLowerBounds->size[0]) && (poses->size[0] ==
           ssUpperBounds->size[0]) && ((poses->size[0] == 1 ?
            ssLowerBounds->size[0] : poses->size[0]) == (poses->size[0] == 1 ?
            ssUpperBounds->size[0] : poses->size[0])) && (tmp->size[0] ==
           idxInBounds->size[0])) {
        trueCount = poses_1->size[0] * poses_1->size[1];
        poses_1->size[0] = poses->size[0];
        poses_1->size[1] = 2;
        MAT_emxEnsureCapacity_boolean_T(poses_1, trueCount);
        partialTrueCount = poses->size[0];
        for (i_0 = 0; i_0 < 2; i_0++) {
          for (ix = 0; ix < partialTrueCount; ix++) {
            poses_tmp = poses->data[((int32_T)xyInd[i_0] - 1) * poses->size[0] +
              ix];
            poses_1->data[ix + poses_1->size[0] * i_0] = ((poses_tmp >=
              ssLowerBounds->data[ssLowerBounds->size[0] * i_0 + ix]) &&
              (poses_tmp <= ssUpperBounds->data[ssUpperBounds->size[0] * i_0 +
               ix]));
          }
        }

        MATLAB0_all(poses_1, tmp);
        trueCount = isInBounds->size[0];
        isInBounds->size[0] = tmp->size[0];
        MAT_emxEnsureCapacity_boolean_T(isInBounds, trueCount);
        partialTrueCount = tmp->size[0];
        for (i_0 = 0; i_0 < partialTrueCount; i_0++) {
          isInBounds->data[i_0] = (tmp->data[i_0] && idxInBounds->data[i_0]);
        }
      } else {
        MATLAB0_binary_expand_op_2(isInBounds, poses, xyInd, ssLowerBounds,
          ssUpperBounds, idxInBounds);
      }
    } else {
      trueCount = poses_0->size[0] * poses_0->size[1];
      poses_0->size[0] = poses->size[0];
      poses_0->size[1] = 2;
      MATLAB_emxEnsureCapacity_real_T(poses_0, trueCount);
      partialTrueCount = poses->size[0];
      for (i_0 = 0; i_0 < 2; i_0++) {
        for (ix = 0; ix < partialTrueCount; ix++) {
          poses_0->data[ix + poses_0->size[0] * i_0] = poses->data[((int32_T)
            xyInd[i_0] - 1) * poses->size[0] + ix];
        }
      }

      binaryOccupancyMap_checkOccup_f(b_obj->Map, poses_0, directions,
        idxInBounds);
      MATLAB0_binary_expand_op_1(tmp, poses, xyInd, ssLowerBounds, ssUpperBounds);
      MATLAB0_emxInit_boolean_T(&tmp_0, 1);
      MATLAB0_binary_expand_op_1(tmp_0, poses, xyInd, ssLowerBounds,
        ssUpperBounds);
      if ((poses->size[0] == ssLowerBounds->size[0]) && (poses->size[0] ==
           ssUpperBounds->size[0]) && ((poses->size[0] == 1 ?
            ssLowerBounds->size[0] : poses->size[0]) == (poses->size[0] == 1 ?
            ssUpperBounds->size[0] : poses->size[0])) && (tmp->size[0] ==
           idxInBounds->size[0]) && ((tmp_0->size[0] == 1 ? idxInBounds->size[0]
            : tmp_0->size[0]) == directions->size[0])) {
        trueCount = poses_1->size[0] * poses_1->size[1];
        poses_1->size[0] = poses->size[0];
        poses_1->size[1] = 2;
        MAT_emxEnsureCapacity_boolean_T(poses_1, trueCount);
        partialTrueCount = poses->size[0];
        for (i_0 = 0; i_0 < 2; i_0++) {
          for (ix = 0; ix < partialTrueCount; ix++) {
            poses_tmp = poses->data[((int32_T)xyInd[i_0] - 1) * poses->size[0] +
              ix];
            poses_1->data[ix + poses_1->size[0] * i_0] = ((poses_tmp >=
              ssLowerBounds->data[ssLowerBounds->size[0] * i_0 + ix]) &&
              (poses_tmp <= ssUpperBounds->data[ssUpperBounds->size[0] * i_0 +
               ix]));
          }
        }

        MATLAB0_all(poses_1, tmp);
        trueCount = isInBounds->size[0];
        isInBounds->size[0] = tmp->size[0];
        MAT_emxEnsureCapacity_boolean_T(isInBounds, trueCount);
        partialTrueCount = tmp->size[0];
        for (i_0 = 0; i_0 < partialTrueCount; i_0++) {
          isInBounds->data[i_0] = (tmp->data[i_0] && idxInBounds->data[i_0] && (
            !(directions->data[i_0] != 0.0)));
        }
      } else {
        MATLAB0_binary_expand_op_5(isInBounds, poses, xyInd, ssLowerBounds,
          ssUpperBounds, idxInBounds, directions);
      }

      MATLAB0_emxFree_boolean_T(&tmp_0);
    }

    MATLAB0_emxFree_boolean_T(&poses_1);
    MATLAB0_emxFree_real_T(&poses_0);
    MATLAB0_emxFree_boolean_T(&tmp);
    MATLAB0_emxFree_boolean_T(&idxInBounds);
    MATLAB0_emxFree_real_T(&ssUpperBounds);
    MATLAB0_emxFree_real_T(&ssLowerBounds);
    result = true;
    ix = 1;
    exitg1 = false;
    while ((!exitg1) && (ix <= isInBounds->size[0])) {
      if (!isInBounds->data[ix - 1]) {
        result = false;
        exitg1 = true;
      } else {
        ix++;
      }
    }

    MATLAB0_emxFree_boolean_T(&isInBounds);
  }

  MATLAB0_emxFree_real_T(&directions);
  MATLAB0_emxFree_real_T(&poses);
  return result;
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static void plannerHybridAStar_getStraightP(real_T length, const real_T
  initialNodePose[3], const real_T direction[2], real_T newNodePose[6])
{
  real_T b_x;
  real_T x;
  real_T xNew_tmp;
  x = cos(initialNodePose[2]);
  b_x = sin(initialNodePose[2]);
  xNew_tmp = direction[0] * length;
  newNodePose[0] = xNew_tmp * x + initialNodePose[0];
  newNodePose[2] = xNew_tmp * b_x + initialNodePose[1];
  newNodePose[4] = initialNodePose[2];
  xNew_tmp = direction[1] * length;
  newNodePose[1] = xNew_tmp * x + initialNodePose[0];
  newNodePose[3] = xNew_tmp * b_x + initialNodePose[1];
  newNodePose[5] = initialNodePose[2];
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static void MATLAB0_MapInterface_world2grid(const binaryOccupancyMap_MATLAB0_T
  *obj, const real_T pos[20], real_T idx[20])
{
  static real_T gridXY[20];
  static real_T localXY[20];
  static real_T a[4];
  static real_T absx;
  static real_T xlimit_idx_0;
  static real_T xlimit_idx_1;
  static real_T ylimit_idx_0;
  static real_T ylimit_idx_1;
  static const int32_T tmp = 20;
  static boolean_T originIdx[20];
  real_T gridXY_0;
  int32_T exponent;
  int32_T i;
  int32_T k;
  int32_T localXY_tmp;
  for (k = 0; k < 2; k++) {
    for (i = 0; i < 10; i++) {
      localXY_tmp = 10 * k + i;
      localXY[localXY_tmp] = pos[localXY_tmp] -
        obj->SharedProperties.LocalOriginInWorld[k];
    }
  }

  xlimit_idx_0 = obj->SharedProperties.GridOriginInLocal[0];
  xlimit_idx_1 = obj->SharedProperties.GridOriginInLocal[0] + 650.0;
  ylimit_idx_0 = obj->SharedProperties.GridOriginInLocal[1];
  ylimit_idx_1 = obj->SharedProperties.GridOriginInLocal[1] + 650.0;
  for (k = 0; k < 10; k++) {
    gridXY[k] = localXY[k + 10] - ylimit_idx_0;
    gridXY[k + 10] = -xlimit_idx_0 + localXY[k];
  }

  a[0] = fabs(obj->SharedProperties.GridOriginInLocal[0]);
  a[1] = fabs(obj->SharedProperties.GridOriginInLocal[0] + 650.0);
  a[2] = fabs(obj->SharedProperties.GridOriginInLocal[1]);
  a[3] = fabs(obj->SharedProperties.GridOriginInLocal[1] + 650.0);
  for (k = 0; k < 20; k++) {
    gridXY_0 = gridXY[k];
    idx[k] = ceil(gridXY_0);
    localXY[k] = fabs(gridXY_0);
  }

  absx = fabs(MATLAB0_maximum(a));
  if (rtIsInf(absx) || rtIsNaN(absx)) {
    gridXY_0 = (rtNaN);
  } else if (absx < 4.4501477170144028E-308) {
    gridXY_0 = 4.94065645841247E-324;
  } else {
    frexp(absx, &exponent);
    gridXY_0 = ldexp(1.0, exponent - 53);
  }

  gridXY_0 *= 2.0;
  for (k = 0; k < 20; k++) {
    originIdx[k] = (localXY[k] < gridXY_0);
  }

  if (MATLAB0_vectorAny(originIdx, &tmp)) {
    for (k = 0; k < 20; k++) {
      if (originIdx[k]) {
        idx[k] = 1.0;
      }
    }
  }

  for (k = 0; k < 10; k++) {
    idx[k] = 651.0 - idx[k];
  }
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static void plannerHybridAStar_checkNodeVal(const plannerHybridAStar_MATLAB0_T
  *obj, const real_T PointsGrid[20], const real_T direction[10], real_T
  nodeValidity[10])
{
  real_T PointsGrid_0;
  real_T PointsGrid_1;
  real_T obj_0;
  int32_T i;
  obj_0 = obj->Dimensions[0];
  for (i = 0; i < 10; i++) {
    PointsGrid_0 = PointsGrid[i + 10];
    PointsGrid_1 = PointsGrid[i];
    if ((PointsGrid_1 < 1.0) || (PointsGrid_0 < 1.0) || (PointsGrid_1 >
         obj->Dimensions[0]) || (PointsGrid_0 > obj->Dimensions[1])) {
      nodeValidity[i] = 0.0;
    } else if (direction[i] == 1.0) {
      nodeValidity[i] = !obj->visitedCellsFront[(int32_T)((PointsGrid_0 - 1.0) *
        obj_0 + PointsGrid_1) - 1];
    } else {
      nodeValidity[i] = !obj->visitedCellsBack[(int32_T)((PointsGrid_0 - 1.0) *
        obj_0 + PointsGrid_1) - 1];
    }
  }
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static void MATLAB0_linspace_jf(real_T n, emxArray_real_T_MATLAB0_T *y)
{
  real_T delta1;
  int32_T b;
  int32_T k;
  if (!(n >= 0.0)) {
    y->size[0] = 1;
    y->size[1] = 0;
  } else {
    delta1 = floor(n);
    k = y->size[0] * y->size[1];
    y->size[0] = 1;
    y->size[1] = (int32_T)delta1;
    MATLAB_emxEnsureCapacity_real_T(y, k);
    if ((int32_T)delta1 >= 1) {
      y->data[(int32_T)delta1 - 1] = 1.0;
      if (y->size[1] >= 2) {
        y->data[0] = 0.0;
        if (y->size[1] >= 3) {
          delta1 = 1.0 / ((real_T)y->size[1] - 1.0);
          b = y->size[1];
          for (k = 0; k <= b - 3; k++) {
            y->data[k + 1] = ((real_T)k + 1.0) * delta1;
          }
        }
      }
    }
  }
}

static void MATLAB0_binary_expand_op_11(emxArray_real_T_MATLAB0_T *in1, const
  real_T in2_data[], const int32_T in2_size[2], const real_T in3_data[], const
  int32_T in3_size[2], real_T in4, real_T in5)
{
  static emxArray_real_T_MATLAB0_T *tmp;
  static real_T in2_data_0[8];
  int32_T i;
  int32_T in2_size_idx_0;
  int32_T loop_ub;
  int32_T stride_0_0;
  int32_T stride_1_0;
  MATLAB0_emxInit_real_T(&tmp, 2);
  MATLAB0_linspace_jf(in4 / in5 + 2.0, tmp);
  loop_ub = in3_size[0] == 1 ? in2_size[0] : in3_size[0];
  in2_size_idx_0 = loop_ub;
  stride_0_0 = (in2_size[0] != 1);
  stride_1_0 = (in3_size[0] != 1);
  for (i = 0; i < loop_ub; i++) {
    in2_data_0[i] = in2_data[i * stride_0_0 + (in2_size[0] << 1)] - in3_data[i *
      stride_1_0 + (in3_size[0] << 1)];
  }

  i = in1->size[0] * in1->size[1];
  in1->size[0] = loop_ub;
  in1->size[1] = tmp->size[1];
  MATLAB_emxEnsureCapacity_real_T(in1, i);
  loop_ub = tmp->size[1];
  for (i = 0; i < loop_ub; i++) {
    for (stride_0_0 = 0; stride_0_0 < in2_size_idx_0; stride_0_0++) {
      in1->data[stride_0_0 + in1->size[0] * i] = in2_data_0[stride_0_0] *
        tmp->data[i];
    }
  }

  MATLAB0_emxFree_real_T(&tmp);
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static void MATLAB0_repmat_k(const real_T a_data[], const int32_T *a_size,
  real_T varargin_2, emxArray_real_T_MATLAB0_T *b)
{
  int32_T ibtile;
  int32_T jtilecol;
  int32_T k;
  int32_T nrows;
  int32_T tmp;
  jtilecol = b->size[0] * b->size[1];
  b->size[0] = *a_size;
  tmp = (int32_T)varargin_2;
  b->size[1] = (int32_T)varargin_2;
  MATLAB_emxEnsureCapacity_real_T(b, jtilecol);
  nrows = *a_size;
  for (jtilecol = 0; jtilecol < tmp; jtilecol++) {
    ibtile = jtilecol * *a_size;
    for (k = 0; k < nrows; k++) {
      b->data[ibtile + k] = a_data[k];
    }
  }
}

static void MATLAB0_binary_expand_op_10(emxArray_real_T_MATLAB0_T *in1, const
  real_T in2_data[], const int32_T in2_size[2], const emxArray_real_T_MATLAB0_T *
  in3)
{
  static emxArray_real_T_MATLAB0_T *tmp;
  static real_T in2_data_0[8];
  static int32_T aux_0_1;
  static int32_T aux_1_1;
  static int32_T i;
  static int32_T i_0;
  static int32_T loop_ub;
  static int32_T stride_0_0;
  static int32_T stride_1_0;
  int32_T in2_size_0;
  int32_T stride_0_1;
  int32_T stride_1_1;
  in2_size_0 = in2_size[0];
  loop_ub = in2_size[0];
  for (i = 0; i < loop_ub; i++) {
    in2_data_0[i] = in2_data[(in2_size[0] << 1) + i];
  }

  MATLAB0_emxInit_real_T(&tmp, 2);
  MATLAB0_repmat_k(in2_data_0, &in2_size_0, (real_T)in3->size[1], tmp);
  loop_ub = in3->size[0] == 1 ? tmp->size[0] : in3->size[0];
  i = in1->size[0] * in1->size[1];
  in1->size[0] = loop_ub;
  MATLAB_emxEnsureCapacity_real_T(in1, i);
  in2_size_0 = in3->size[1] == 1 ? tmp->size[1] : in3->size[1];
  i = in1->size[0] * in1->size[1];
  in1->size[1] = in2_size_0;
  MATLAB_emxEnsureCapacity_real_T(in1, i);
  stride_0_0 = (tmp->size[0] != 1);
  stride_0_1 = (tmp->size[1] != 1);
  stride_1_0 = (in3->size[0] != 1);
  stride_1_1 = (in3->size[1] != 1);
  aux_0_1 = 0;
  aux_1_1 = 0;
  for (i = 0; i < in2_size_0; i++) {
    for (i_0 = 0; i_0 < loop_ub; i_0++) {
      in1->data[i_0 + in1->size[0] * i] = tmp->data[i_0 * stride_0_0 + tmp->
        size[0] * aux_0_1] + in3->data[i_0 * stride_1_0 + in3->size[0] * aux_1_1];
    }

    aux_1_1 += stride_1_1;
    aux_0_1 += stride_0_1;
  }

  MATLAB0_emxFree_real_T(&tmp);
}

static void MATLAB0_binary_expand_op_8(emxArray_real_T_MATLAB0_T *in1, const
  real_T in2_data[], const int32_T *in2_size, const emxArray_real_T_MATLAB0_T
  *in3)
{
  static emxArray_real_T_MATLAB0_T *in1_0;
  static emxArray_real_T_MATLAB0_T *tmp;
  static int32_T aux_0_1;
  static int32_T aux_1_1;
  static int32_T i;
  static int32_T stride_0_0;
  static int32_T stride_0_1;
  static int32_T stride_1_0;
  int32_T i_0;
  int32_T loop_ub;
  int32_T loop_ub_0;
  int32_T stride_1_1;
  MATLAB0_emxInit_real_T(&tmp, 2);
  MATLAB0_repmat_k(in2_data, in2_size, (real_T)in3->size[1], tmp);
  MATLAB0_emxInit_real_T(&in1_0, 2);
  loop_ub_0 = tmp->size[0] == 1 ? in1->size[0] : tmp->size[0];
  i_0 = in1_0->size[0] * in1_0->size[1];
  in1_0->size[0] = loop_ub_0;
  loop_ub = tmp->size[1] == 1 ? in1->size[1] : tmp->size[1];
  in1_0->size[1] = loop_ub;
  MATLAB_emxEnsureCapacity_real_T(in1_0, i_0);
  stride_0_0 = (in1->size[0] != 1);
  stride_0_1 = (in1->size[1] != 1);
  stride_1_0 = (tmp->size[0] != 1);
  stride_1_1 = (tmp->size[1] != 1);
  aux_0_1 = 0;
  aux_1_1 = 0;
  for (i_0 = 0; i_0 < loop_ub; i_0++) {
    for (i = 0; i < loop_ub_0; i++) {
      in1_0->data[i + in1_0->size[0] * i_0] = in1->data[i * stride_0_0 +
        in1->size[0] * aux_0_1] * tmp->data[i * stride_1_0 + tmp->size[0] *
        aux_1_1];
    }

    aux_1_1 += stride_1_1;
    aux_0_1 += stride_0_1;
  }

  MATLAB0_emxFree_real_T(&tmp);
  i_0 = in1->size[0] * in1->size[1];
  in1->size[0] = in1_0->size[0];
  in1->size[1] = in1_0->size[1];
  MATLAB_emxEnsureCapacity_real_T(in1, i_0);
  loop_ub_0 = in1_0->size[1];
  for (i_0 = 0; i_0 < loop_ub_0; i_0++) {
    loop_ub = in1_0->size[0];
    for (i = 0; i < loop_ub; i++) {
      in1->data[i + in1->size[0] * i_0] = in1_0->data[in1_0->size[0] * i_0 + i];
    }
  }

  MATLAB0_emxFree_real_T(&in1_0);
}

static void MATLAB0_binary_expand_op_7(emxArray_real_T_MATLAB0_T *in1, const
  real_T in2_data[], const int32_T in2_size[2], const emxArray_real_T_MATLAB0_T *
  in3)
{
  static emxArray_real_T_MATLAB0_T *tmp;
  static real_T in2_data_0[8];
  static int32_T aux_0_1;
  static int32_T aux_1_1;
  static int32_T i;
  static int32_T i_0;
  static int32_T loop_ub;
  static int32_T stride_0_0;
  static int32_T stride_1_0;
  int32_T in2_size_0;
  int32_T stride_0_1;
  int32_T stride_1_1;
  in2_size_0 = in2_size[0];
  loop_ub = in2_size[0];
  for (i = 0; i < loop_ub; i++) {
    in2_data_0[i] = in2_data[i];
  }

  MATLAB0_emxInit_real_T(&tmp, 2);
  MATLAB0_repmat_k(in2_data_0, &in2_size_0, (real_T)in3->size[1], tmp);
  loop_ub = in3->size[0] == 1 ? tmp->size[0] : in3->size[0];
  i = in1->size[0] * in1->size[1];
  in1->size[0] = loop_ub;
  MATLAB_emxEnsureCapacity_real_T(in1, i);
  in2_size_0 = in3->size[1] == 1 ? tmp->size[1] : in3->size[1];
  i = in1->size[0] * in1->size[1];
  in1->size[1] = in2_size_0;
  MATLAB_emxEnsureCapacity_real_T(in1, i);
  stride_0_0 = (tmp->size[0] != 1);
  stride_0_1 = (tmp->size[1] != 1);
  stride_1_0 = (in3->size[0] != 1);
  stride_1_1 = (in3->size[1] != 1);
  aux_0_1 = 0;
  aux_1_1 = 0;
  for (i = 0; i < in2_size_0; i++) {
    for (i_0 = 0; i_0 < loop_ub; i_0++) {
      in1->data[i_0 + in1->size[0] * i] = tmp->data[i_0 * stride_0_0 + tmp->
        size[0] * aux_0_1] + in3->data[i_0 * stride_1_0 + in3->size[0] * aux_1_1];
    }

    aux_1_1 += stride_1_1;
    aux_0_1 += stride_0_1;
  }

  MATLAB0_emxFree_real_T(&tmp);
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static void MATLAB0_nullAssignment(emxArray_real_T_MATLAB0_T *x)
{
  int32_T b;
  int32_T i;
  int32_T j;
  int32_T ncols;
  ncols = x->size[1];
  for (j = 0; j <= ncols - 2; j++) {
    b = x->size[0];
    for (i = 0; i < b; i++) {
      x->data[i + x->size[0] * j] = x->data[(j + 1) * x->size[0] + i];
    }
  }

  ncols = x->size[0];
  if (x->size[1] - 1 < 1) {
    b = -1;
  } else {
    b = x->size[1] - 2;
  }

  for (j = 0; j <= b; j++) {
    for (i = 0; i < ncols; i++) {
      x->data[i + ncols * j] = x->data[x->size[0] * j + i];
    }
  }

  j = x->size[0] * x->size[1];
  x->size[1] = b + 1;
  MATLAB_emxEnsureCapacity_real_T(x, j);
}

static void MATLAB0_binary_expand_op_6(emxArray_real_T_MATLAB0_T *in1, const
  real_T in2_data[], const int32_T in2_size[2], const emxArray_real_T_MATLAB0_T *
  in3)
{
  static emxArray_real_T_MATLAB0_T *tmp;
  static real_T in2_data_0[8];
  static int32_T aux_0_1;
  static int32_T aux_1_1;
  static int32_T i;
  static int32_T i_0;
  static int32_T loop_ub;
  static int32_T stride_0_0;
  static int32_T stride_1_0;
  int32_T in2_size_0;
  int32_T stride_0_1;
  int32_T stride_1_1;
  in2_size_0 = in2_size[0];
  loop_ub = in2_size[0];
  for (i = 0; i < loop_ub; i++) {
    in2_data_0[i] = in2_data[i + in2_size[0]];
  }

  MATLAB0_emxInit_real_T(&tmp, 2);
  MATLAB0_repmat_k(in2_data_0, &in2_size_0, (real_T)in3->size[1], tmp);
  loop_ub = in3->size[0] == 1 ? tmp->size[0] : in3->size[0];
  i = in1->size[0] * in1->size[1];
  in1->size[0] = loop_ub;
  MATLAB_emxEnsureCapacity_real_T(in1, i);
  in2_size_0 = in3->size[1] == 1 ? tmp->size[1] : in3->size[1];
  i = in1->size[0] * in1->size[1];
  in1->size[1] = in2_size_0;
  MATLAB_emxEnsureCapacity_real_T(in1, i);
  stride_0_0 = (tmp->size[0] != 1);
  stride_0_1 = (tmp->size[1] != 1);
  stride_1_0 = (in3->size[0] != 1);
  stride_1_1 = (in3->size[1] != 1);
  aux_0_1 = 0;
  aux_1_1 = 0;
  for (i = 0; i < in2_size_0; i++) {
    for (i_0 = 0; i_0 < loop_ub; i_0++) {
      in1->data[i_0 + in1->size[0] * i] = tmp->data[i_0 * stride_0_0 + tmp->
        size[0] * aux_0_1] - in3->data[i_0 * stride_1_0 + in3->size[0] * aux_1_1];
    }

    aux_1_1 += stride_1_1;
    aux_0_1 += stride_0_1;
  }

  MATLAB0_emxFree_real_T(&tmp);
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static void plannerHybridAStar_getPosesCirc(const real_T initialPose_data[],
  const int32_T initialPose_size[2], const real_T finalPoses_data[], const
  int32_T finalPoses_size[2], const real_T ICRData_data[], const int32_T
  ICRData_size[2], const real_T radius_data[], const int32_T radius_size[2],
  real_T length, real_T stepSize, emxArray_real_T_MATLAB0_T *poses)
{
  static emxArray_real_T_MATLAB0_T *angles;
  static emxArray_real_T_MATLAB0_T *angles_0;
  static emxArray_real_T_MATLAB0_T *c;
  static emxArray_real_T_MATLAB0_T *deltaX;
  static emxArray_real_T_MATLAB0_T *deltaY;
  static emxArray_real_T_MATLAB0_T *expandedIntervals;
  static emxArray_real_T_MATLAB0_T *expandedIntervals_0;
  static emxArray_real_T_MATLAB0_T *tmp;
  static emxArray_real_T_MATLAB0_T *tmp_0;
  static emxArray_real_T_MATLAB0_T *tmp_1;
  static emxArray_real_T_MATLAB0_T *tmp_2;
  static emxArray_real_T_MATLAB0_T *tmp_3;
  static emxArray_real_T_MATLAB0_T *tmp_4;
  static emxArray_real_T_MATLAB0_T *tmp_5;
  static emxArray_real_T_MATLAB0_T *tmp_6;
  static emxArray_real_T_MATLAB0_T *tmp_7;
  static emxArray_real_T_MATLAB0_T *tmp_8;
  static emxArray_real_T_MATLAB0_T *tmp_9;
  static emxArray_real_T_MATLAB0_T *tmp_a;
  static emxArray_real_T_MATLAB0_T *xPoints;
  static emxArray_real_T_MATLAB0_T *xPoints_0;
  static real_T finalPoses_data_0[8];
  static real_T initialPose_data_0[8];
  static real_T initialPose_data_1[8];
  static real_T finalPoses_data_tmp;
  static int32_T angles_1;
  static int32_T deltaY_0;
  static int32_T finalPoses_size_0;
  static int32_T initialPose_size_0;
  static int32_T initialPose_size_1;
  static int32_T loop_ub;
  static int32_T loop_ub_0;
  static int32_T loop_ub_tmp;
  static int32_T xPoints_1;
  MATLAB0_emxInit_real_T(&expandedIntervals, 2);
  if (finalPoses_size[0] == initialPose_size[0]) {
    MATLAB0_emxInit_real_T(&tmp, 2);
    MATLAB0_linspace_jf(length / stepSize + 2.0, tmp);
    finalPoses_size_0 = finalPoses_size[0];
    loop_ub = finalPoses_size[0];
    for (angles_1 = 0; angles_1 < loop_ub; angles_1++) {
      finalPoses_data_0[angles_1] = finalPoses_data[(finalPoses_size[0] << 1) +
        angles_1] - initialPose_data[(initialPose_size[0] << 1) + angles_1];
    }

    angles_1 = expandedIntervals->size[0] * expandedIntervals->size[1];
    expandedIntervals->size[0] = finalPoses_size[0];
    expandedIntervals->size[1] = tmp->size[1];
    MATLAB_emxEnsureCapacity_real_T(expandedIntervals, angles_1);
    loop_ub = tmp->size[1];
    for (angles_1 = 0; angles_1 < loop_ub; angles_1++) {
      loop_ub_0 = finalPoses_size_0;
      for (xPoints_1 = 0; xPoints_1 < finalPoses_size_0; xPoints_1++) {
        expandedIntervals->data[xPoints_1 + expandedIntervals->size[0] *
          angles_1] = finalPoses_data_0[xPoints_1] * tmp->data[angles_1];
      }
    }

    MATLAB0_emxFree_real_T(&tmp);
  } else {
    MATLAB0_binary_expand_op_11(expandedIntervals, finalPoses_data,
      finalPoses_size, initialPose_data, initialPose_size, length, stepSize);
  }

  finalPoses_size_0 = initialPose_size[0];
  loop_ub = initialPose_size[0];
  initialPose_size_0 = initialPose_size[0];
  initialPose_size_1 = initialPose_size[0];
  for (angles_1 = 0; angles_1 < loop_ub; angles_1++) {
    finalPoses_data_tmp = initialPose_data[(initialPose_size[0] << 1) + angles_1];
    finalPoses_data_0[angles_1] = finalPoses_data_tmp;
    initialPose_data_0[angles_1] = finalPoses_data_tmp;
    initialPose_data_1[angles_1] = finalPoses_data_tmp;
  }

  MATLAB0_emxInit_real_T(&c, 2);
  MATLAB0_repmat_k(finalPoses_data_0, &finalPoses_size_0, (real_T)
                   expandedIntervals->size[1], c);
  MATLAB0_emxFree_real_T(&c);
  MATLAB0_emxInit_real_T(&tmp_9, 2);
  MATLAB0_repmat_k(initialPose_data_0, &initialPose_size_0, (real_T)
                   expandedIntervals->size[1], tmp_9);
  MATLAB0_emxInit_real_T(&tmp_a, 2);
  MATLAB0_repmat_k(initialPose_data_1, &initialPose_size_1, (real_T)
                   expandedIntervals->size[1], tmp_a);
  MATLAB0_emxInit_real_T(&angles, 2);
  MATLAB0_emxInit_real_T(&tmp_0, 2);
  if ((tmp_9->size[0] == expandedIntervals->size[0]) && (tmp_a->size[1] ==
       expandedIntervals->size[1])) {
    finalPoses_size_0 = initialPose_size[0];
    loop_ub = initialPose_size[0];
    for (angles_1 = 0; angles_1 < loop_ub; angles_1++) {
      finalPoses_data_0[angles_1] = initialPose_data[(initialPose_size[0] << 1)
        + angles_1];
    }

    MATLAB0_repmat_k(finalPoses_data_0, &finalPoses_size_0, (real_T)
                     expandedIntervals->size[1], tmp_0);
    angles_1 = angles->size[0] * angles->size[1];
    angles->size[0] = tmp_0->size[0];
    angles->size[1] = tmp_0->size[1];
    MATLAB_emxEnsureCapacity_real_T(angles, angles_1);
    loop_ub = tmp_0->size[0] * tmp_0->size[1];
    for (angles_1 = 0; angles_1 < loop_ub; angles_1++) {
      angles->data[angles_1] = tmp_0->data[angles_1] + expandedIntervals->
        data[angles_1];
    }
  } else {
    MATLAB0_binary_expand_op_10(angles, initialPose_data, initialPose_size,
      expandedIntervals);
  }

  MATLAB0_emxFree_real_T(&tmp_a);
  MATLAB0_emxFree_real_T(&tmp_9);
  MATLAB0_emxInit_real_T(&deltaX, 2);
  angles_1 = deltaX->size[0] * deltaX->size[1];
  deltaX->size[0] = angles->size[0];
  deltaX->size[1] = angles->size[1];
  MATLAB_emxEnsureCapacity_real_T(deltaX, angles_1);
  loop_ub_tmp = angles->size[0] * angles->size[1];
  for (angles_1 = 0; angles_1 < loop_ub_tmp; angles_1++) {
    deltaX->data[angles_1] = angles->data[angles_1];
  }

  for (angles_1 = 0; angles_1 < loop_ub_tmp; angles_1++) {
    deltaX->data[angles_1] = sin(deltaX->data[angles_1]);
  }

  finalPoses_size_0 = radius_size[1];
  loop_ub = radius_size[1];
  for (angles_1 = 0; angles_1 < loop_ub; angles_1++) {
    finalPoses_data_0[angles_1] = radius_data[angles_1];
  }

  MATLAB0_emxInit_real_T(&tmp_7, 2);
  MATLAB0_repmat_k(finalPoses_data_0, &finalPoses_size_0, (real_T)angles->size[1],
                   tmp_7);
  MATLAB0_emxInit_real_T(&tmp_8, 2);
  MATLAB0_repmat_k(finalPoses_data_0, &finalPoses_size_0, (real_T)angles->size[1],
                   tmp_8);
  if ((deltaX->size[0] == tmp_7->size[0]) && (deltaX->size[1] == tmp_8->size[1]))
  {
    MATLAB0_repmat_k(finalPoses_data_0, &finalPoses_size_0, (real_T)angles->
                     size[1], tmp_0);
    for (xPoints_1 = 0; xPoints_1 < loop_ub_tmp; xPoints_1++) {
      deltaX->data[xPoints_1] *= tmp_0->data[xPoints_1];
    }
  } else {
    MATLAB0_binary_expand_op_8(deltaX, finalPoses_data_0, &finalPoses_size_0,
      angles);
  }

  MATLAB0_emxFree_real_T(&tmp_8);
  MATLAB0_emxFree_real_T(&tmp_7);
  MATLAB0_emxInit_real_T(&deltaY, 2);
  angles_1 = deltaY->size[0] * deltaY->size[1];
  deltaY->size[0] = angles->size[0];
  deltaY->size[1] = angles->size[1];
  MATLAB_emxEnsureCapacity_real_T(deltaY, angles_1);
  loop_ub_tmp = angles->size[0] * angles->size[1];
  for (angles_1 = 0; angles_1 < loop_ub_tmp; angles_1++) {
    deltaY->data[angles_1] = angles->data[angles_1];
  }

  for (angles_1 = 0; angles_1 < loop_ub_tmp; angles_1++) {
    deltaY->data[angles_1] = cos(deltaY->data[angles_1]);
  }

  MATLAB0_emxInit_real_T(&tmp_5, 2);
  MATLAB0_repmat_k(finalPoses_data_0, &finalPoses_size_0, (real_T)angles->size[1],
                   tmp_5);
  MATLAB0_emxInit_real_T(&tmp_6, 2);
  MATLAB0_repmat_k(finalPoses_data_0, &finalPoses_size_0, (real_T)angles->size[1],
                   tmp_6);
  if ((deltaY->size[0] == tmp_5->size[0]) && (deltaY->size[1] == tmp_6->size[1]))
  {
    MATLAB0_repmat_k(finalPoses_data_0, &finalPoses_size_0, (real_T)angles->
                     size[1], tmp_0);
    for (xPoints_1 = 0; xPoints_1 < loop_ub_tmp; xPoints_1++) {
      deltaY->data[xPoints_1] *= tmp_0->data[xPoints_1];
    }
  } else {
    MATLAB0_binary_expand_op_8(deltaY, finalPoses_data_0, &finalPoses_size_0,
      angles);
  }

  MATLAB0_emxFree_real_T(&tmp_6);
  MATLAB0_emxFree_real_T(&tmp_5);
  initialPose_size_0 = ICRData_size[0];
  loop_ub = ICRData_size[0];
  initialPose_size_1 = ICRData_size[0];
  for (angles_1 = 0; angles_1 < loop_ub; angles_1++) {
    finalPoses_data_tmp = ICRData_data[angles_1];
    initialPose_data_0[angles_1] = finalPoses_data_tmp;
    initialPose_data_1[angles_1] = finalPoses_data_tmp;
  }

  MATLAB0_emxInit_real_T(&tmp_3, 2);
  MATLAB0_repmat_k(initialPose_data_0, &initialPose_size_0, (real_T)deltaX->
                   size[1], tmp_3);
  MATLAB0_emxInit_real_T(&tmp_4, 2);
  MATLAB0_repmat_k(initialPose_data_1, &initialPose_size_1, (real_T)deltaX->
                   size[1], tmp_4);
  MATLAB0_emxInit_real_T(&xPoints, 2);
  if ((tmp_3->size[0] == deltaX->size[0]) && (tmp_4->size[1] == deltaX->size[1]))
  {
    initialPose_size_0 = ICRData_size[0];
    loop_ub = ICRData_size[0];
    for (angles_1 = 0; angles_1 < loop_ub; angles_1++) {
      initialPose_data_0[angles_1] = ICRData_data[angles_1];
    }

    MATLAB0_repmat_k(initialPose_data_0, &initialPose_size_0, (real_T)
                     deltaX->size[1], tmp_0);
    angles_1 = xPoints->size[0] * xPoints->size[1];
    xPoints->size[0] = tmp_0->size[0];
    xPoints->size[1] = tmp_0->size[1];
    MATLAB_emxEnsureCapacity_real_T(xPoints, angles_1);
    loop_ub = tmp_0->size[0] * tmp_0->size[1];
    for (angles_1 = 0; angles_1 < loop_ub; angles_1++) {
      xPoints->data[angles_1] = tmp_0->data[angles_1] + deltaX->data[angles_1];
    }
  } else {
    MATLAB0_binary_expand_op_7(xPoints, ICRData_data, ICRData_size, deltaX);
  }

  MATLAB0_emxFree_real_T(&tmp_4);
  MATLAB0_emxFree_real_T(&tmp_3);
  MATLAB0_emxFree_real_T(&deltaX);
  MATLAB0_nullAssignment(xPoints);
  initialPose_size_0 = ICRData_size[0];
  loop_ub = ICRData_size[0];
  initialPose_size_1 = ICRData_size[0];
  for (angles_1 = 0; angles_1 < loop_ub; angles_1++) {
    finalPoses_data_tmp = ICRData_data[angles_1 + ICRData_size[0]];
    initialPose_data_0[angles_1] = finalPoses_data_tmp;
    initialPose_data_1[angles_1] = finalPoses_data_tmp;
  }

  MATLAB0_emxInit_real_T(&tmp_1, 2);
  MATLAB0_repmat_k(initialPose_data_0, &initialPose_size_0, (real_T)deltaY->
                   size[1], tmp_1);
  MATLAB0_emxInit_real_T(&tmp_2, 2);
  MATLAB0_repmat_k(initialPose_data_1, &initialPose_size_1, (real_T)deltaY->
                   size[1], tmp_2);
  if ((tmp_1->size[0] == deltaY->size[0]) && (tmp_2->size[1] == deltaY->size[1]))
  {
    initialPose_size_0 = ICRData_size[0];
    loop_ub = ICRData_size[0];
    for (angles_1 = 0; angles_1 < loop_ub; angles_1++) {
      initialPose_data_0[angles_1] = ICRData_data[angles_1 + ICRData_size[0]];
    }

    MATLAB0_repmat_k(initialPose_data_0, &initialPose_size_0, (real_T)
                     deltaY->size[1], tmp_0);
    angles_1 = expandedIntervals->size[0] * expandedIntervals->size[1];
    expandedIntervals->size[0] = tmp_0->size[0];
    expandedIntervals->size[1] = tmp_0->size[1];
    MATLAB_emxEnsureCapacity_real_T(expandedIntervals, angles_1);
    loop_ub = tmp_0->size[0] * tmp_0->size[1];
    for (angles_1 = 0; angles_1 < loop_ub; angles_1++) {
      expandedIntervals->data[angles_1] = tmp_0->data[angles_1] - deltaY->
        data[angles_1];
    }
  } else {
    MATLAB0_binary_expand_op_6(expandedIntervals, ICRData_data, ICRData_size,
      deltaY);
  }

  MATLAB0_emxFree_real_T(&tmp_2);
  MATLAB0_emxFree_real_T(&tmp_1);
  MATLAB0_emxFree_real_T(&tmp_0);
  MATLAB0_emxFree_real_T(&deltaY);
  MATLAB0_nullAssignment(expandedIntervals);
  MATLAB0_nullAssignment(angles);
  MATLAB0_emxInit_real_T(&xPoints_0, 2);
  angles_1 = xPoints_0->size[0] * xPoints_0->size[1];
  xPoints_0->size[0] = xPoints->size[1];
  xPoints_0->size[1] = xPoints->size[0];
  MATLAB_emxEnsureCapacity_real_T(xPoints_0, angles_1);
  loop_ub = xPoints->size[0];
  for (angles_1 = 0; angles_1 < loop_ub; angles_1++) {
    loop_ub_0 = xPoints->size[1];
    for (xPoints_1 = 0; xPoints_1 < loop_ub_0; xPoints_1++) {
      xPoints_0->data[xPoints_1 + xPoints_0->size[0] * angles_1] = xPoints->
        data[xPoints->size[0] * xPoints_1 + angles_1];
    }
  }

  loop_ub_tmp = xPoints->size[0] * xPoints->size[1];
  MATLAB0_emxFree_real_T(&xPoints);
  MATLAB0_emxInit_real_T(&expandedIntervals_0, 2);
  angles_1 = expandedIntervals_0->size[0] * expandedIntervals_0->size[1];
  expandedIntervals_0->size[0] = expandedIntervals->size[1];
  expandedIntervals_0->size[1] = expandedIntervals->size[0];
  MATLAB_emxEnsureCapacity_real_T(expandedIntervals_0, angles_1);
  loop_ub = expandedIntervals->size[0];
  for (angles_1 = 0; angles_1 < loop_ub; angles_1++) {
    loop_ub_0 = expandedIntervals->size[1];
    for (xPoints_1 = 0; xPoints_1 < loop_ub_0; xPoints_1++) {
      expandedIntervals_0->data[xPoints_1 + expandedIntervals_0->size[0] *
        angles_1] = expandedIntervals->data[expandedIntervals->size[0] *
        xPoints_1 + angles_1];
    }
  }

  deltaY_0 = expandedIntervals->size[0] * expandedIntervals->size[1];
  MATLAB0_emxFree_real_T(&expandedIntervals);
  MATLAB0_emxInit_real_T(&angles_0, 2);
  angles_1 = angles_0->size[0] * angles_0->size[1];
  angles_0->size[0] = angles->size[1];
  angles_0->size[1] = angles->size[0];
  MATLAB_emxEnsureCapacity_real_T(angles_0, angles_1);
  loop_ub = angles->size[0];
  for (angles_1 = 0; angles_1 < loop_ub; angles_1++) {
    loop_ub_0 = angles->size[1];
    for (xPoints_1 = 0; xPoints_1 < loop_ub_0; xPoints_1++) {
      angles_0->data[xPoints_1 + angles_0->size[0] * angles_1] = angles->
        data[angles->size[0] * xPoints_1 + angles_1];
    }
  }

  loop_ub = angles->size[0] * angles->size[1];
  MATLAB0_emxFree_real_T(&angles);
  angles_1 = poses->size[0] * poses->size[1];
  poses->size[0] = loop_ub_tmp;
  poses->size[1] = 3;
  MATLAB_emxEnsureCapacity_real_T(poses, angles_1);
  for (angles_1 = 0; angles_1 < loop_ub_tmp; angles_1++) {
    poses->data[angles_1] = xPoints_0->data[angles_1];
  }

  MATLAB0_emxFree_real_T(&xPoints_0);
  for (angles_1 = 0; angles_1 < deltaY_0; angles_1++) {
    poses->data[angles_1 + loop_ub_tmp] = expandedIntervals_0->data[angles_1];
  }

  MATLAB0_emxFree_real_T(&expandedIntervals_0);
  for (angles_1 = 0; angles_1 < loop_ub; angles_1++) {
    poses->data[(angles_1 + loop_ub_tmp) + deltaY_0] = angles_0->data[angles_1];
  }

  MATLAB0_emxFree_real_T(&angles_0);
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static void MATLAB0_wrapToPi(real_T *theta)
{
  static real_T b_idx_0;
  static real_T thetaWrap;
  int32_T i;
  if (fabs(*theta) > 3.1415926535897931) {
    thetaWrap = MATLAB0_mod_p(*theta + 3.1415926535897931);
    b_idx_0 = thetaWrap;
    if ((thetaWrap == 0.0) && (*theta + 3.1415926535897931 > 0.0)) {
      for (i = 0; i < 1; i++) {
        b_idx_0 = 6.2831853071795862;
      }
    }

    *theta = b_idx_0 - 3.1415926535897931;
  }
}

static void MATLAB0_binary_expand_op_12(emxArray_real_T_MATLAB0_T *in1, const
  emxArray_real_T_MATLAB0_T *in2, const emxArray_real_T_MATLAB0_T *in3, const
  real_T in4[3])
{
  static emxArray_real_T_MATLAB0_T *in3_0;
  int32_T i;
  int32_T i_0;
  int32_T in3_idx_0;
  int32_T stride_0_0;
  int32_T stride_1_0;
  in3_idx_0 = in3->size[1];
  MATLAB0_emxInit_real_T(&in3_0, 2);
  i_0 = in3_0->size[0] * in3_0->size[1];
  in3_0->size[0] = in3->size[1];
  in3_0->size[1] = 3;
  MATLAB_emxEnsureCapacity_real_T(in3_0, i_0);
  for (i_0 = 0; i_0 < 3; i_0++) {
    for (i = 0; i < in3_idx_0; i++) {
      in3_0->data[i + in3_0->size[0] * i_0] = in3->data[i] * in4[i_0];
    }
  }

  in3_idx_0 = in3_0->size[0] == 1 ? in2->size[0] : in3_0->size[0];
  i_0 = in1->size[0] * in1->size[1];
  in1->size[0] = in3_idx_0;
  in1->size[1] = 3;
  MATLAB_emxEnsureCapacity_real_T(in1, i_0);
  stride_0_0 = (in2->size[0] != 1);
  stride_1_0 = (in3_0->size[0] != 1);
  for (i_0 = 0; i_0 < 3; i_0++) {
    for (i = 0; i < in3_idx_0; i++) {
      in1->data[i + in1->size[0] * i_0] = in2->data[i * stride_0_0 + in2->size[0]
        * i_0] + in3_0->data[i * stride_1_0 + in3_0->size[0] * i_0];
    }
  }

  MATLAB0_emxFree_real_T(&in3_0);
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static void MATLA_stateSpaceSE2_interpolate(const stateSpaceSE2_MATLAB0_T *obj,
  const real_T state1[3], const real_T state2[3], emxArray_real_T_MATLAB0_T
  *ratios, emxArray_real_T_MATLAB0_T *interpState)
{
  static emxArray_real_T_MATLAB0_T *b;
  static emxArray_real_T_MATLAB0_T *c;
  static emxArray_real_T_MATLAB0_T *ratios_0;
  static real_T stateDiff[3];
  static int32_T colIdx;
  static int32_T rowreps;
  static int32_T sz_idx_0;
  int32_T j;
  int32_T loop_ub;
  int32_T ratios_idx_0;
  if (!obj->SkipStateValidation) {
    sz_idx_0 = ratios->size[1];
    MATLAB0_emxInit_real_T(&ratios_0, 2);
    j = ratios_0->size[0] * ratios_0->size[1];
    ratios_0->size[0] = 1;
    ratios_0->size[1] = ratios->size[1];
    MATLAB_emxEnsureCapacity_real_T(ratios_0, j);
    for (j = 0; j < sz_idx_0; j++) {
      ratios_0->data[j] = ratios->data[j];
    }

    j = ratios->size[0] * ratios->size[1];
    ratios->size[0] = 1;
    ratios->size[1] = ratios_0->size[1];
    MATLAB_emxEnsureCapacity_real_T(ratios, j);
    loop_ub = ratios_0->size[1];
    for (j = 0; j < loop_ub; j++) {
      ratios->data[j] = ratios_0->data[j];
    }

    MATLAB0_emxFree_real_T(&ratios_0);
  }

  stateDiff[0] = state2[0] - state1[0];
  stateDiff[1] = state2[1] - state1[1];
  stateDiff[2] = state2[2] - state1[2];
  MATLAB0_wrapToPi(&stateDiff[2]);
  MATLAB0_emxInit_real_T(&b, 2);
  j = b->size[0] * b->size[1];
  b->size[0] = ratios->size[1];
  b->size[1] = 3;
  MATLAB_emxEnsureCapacity_real_T(b, j);
  if (ratios->size[1] != 0) {
    colIdx = -1;
    for (j = 0; j < 3; j++) {
      colIdx++;
      rowreps = ratios->size[1];
      for (loop_ub = 0; loop_ub < rowreps; loop_ub++) {
        b->data[loop_ub + b->size[0] * colIdx] = state1[j];
      }
    }
  }

  if (b->size[0] == ratios->size[1]) {
    ratios_idx_0 = ratios->size[1];
    j = interpState->size[0] * interpState->size[1];
    interpState->size[0] = ratios->size[1];
    interpState->size[1] = 3;
    MATLAB_emxEnsureCapacity_real_T(interpState, j);
    for (j = 0; j < 3; j++) {
      for (loop_ub = 0; loop_ub < ratios_idx_0; loop_ub++) {
        interpState->data[loop_ub + interpState->size[0] * j] = b->data[b->size
          [0] * j + loop_ub] + ratios->data[loop_ub] * stateDiff[j];
      }
    }
  } else {
    MATLAB0_binary_expand_op_12(interpState, b, ratios, stateDiff);
  }

  MATLAB0_emxFree_real_T(&b);
  MATLAB0_emxInit_real_T(&c, 1);
  j = c->size[0];
  c->size[0] = interpState->size[0];
  MATLAB_emxEnsureCapacity_real_T(c, j);
  loop_ub = interpState->size[0];
  for (j = 0; j < loop_ub; j++) {
    c->data[j] = interpState->data[(interpState->size[0] << 1) + j];
  }

  MATLAB0_wrapToPi_o(c);
  loop_ub = c->size[0];
  for (j = 0; j < loop_ub; j++) {
    interpState->data[j + (interpState->size[0] << 1)] = c->data[j];
  }

  MATLAB0_emxFree_real_T(&c);
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static void validatorOccupancyMap_isState_c(validatorOccupancyMap_MATLAB0_T *obj,
  const emxArray_real_T_MATLAB0_T *state, emxArray_boolean_T_MATLAB0_T *isValid)
{
  static emxArray_boolean_T_MATLAB0_T *idxInBounds;
  static emxArray_boolean_T_MATLAB0_T *state_1;
  static emxArray_boolean_T_MATLAB0_T *tmp;
  static emxArray_boolean_T_MATLAB0_T *tmp_0;
  static emxArray_real_T_MATLAB0_T *isOccupied;
  static emxArray_real_T_MATLAB0_T *ssLowerBounds;
  static emxArray_real_T_MATLAB0_T *ssUpperBounds;
  static emxArray_real_T_MATLAB0_T *state_0;
  static stateSpaceSE2_MATLAB0_T *b_obj;
  static real_T ssBounds[6];
  static real_T xyInd[2];
  static real_T state_tmp;
  static int32_T i;
  static int32_T i_0;
  static int32_T ibmat;
  static int32_T itilerow;
  static int32_T jcol;
  static int32_T loop_ub;
  static int32_T ntilerows;
  xyInd[0] = obj->XYIndices[0];
  xyInd[1] = obj->XYIndices[1];
  b_obj = obj->StateSpace;
  for (i = 0; i < 6; i++) {
    ssBounds[i] = b_obj->StateBoundsInternal[i];
  }

  MATLAB0_emxInit_real_T(&ssLowerBounds, 2);
  i = ssLowerBounds->size[0] * ssLowerBounds->size[1];
  ssLowerBounds->size[0] = state->size[0];
  ssLowerBounds->size[1] = 2;
  MATLAB_emxEnsureCapacity_real_T(ssLowerBounds, i);
  ntilerows = state->size[0];
  MATLAB0_emxInit_real_T(&ssUpperBounds, 2);
  i = ssUpperBounds->size[0] * ssUpperBounds->size[1];
  ssUpperBounds->size[0] = state->size[0];
  ssUpperBounds->size[1] = 2;
  MATLAB_emxEnsureCapacity_real_T(ssUpperBounds, i);
  for (jcol = 0; jcol < 2; jcol++) {
    ibmat = jcol * ntilerows;
    for (itilerow = 0; itilerow < ntilerows; itilerow++) {
      i = ibmat + itilerow;
      ssLowerBounds->data[i] = ssBounds[jcol];
      ssUpperBounds->data[i] = ssBounds[jcol + 3];
    }
  }

  MATLAB0_emxInit_boolean_T(&idxInBounds, 1);
  MATLAB0_emxInit_boolean_T(&tmp, 1);
  MATLAB0_emxInit_real_T(&state_0, 2);
  MATLAB0_emxInit_boolean_T(&state_1, 2);
  if (obj->SkipStateValidation) {
    i = state_0->size[0] * state_0->size[1];
    state_0->size[0] = state->size[0];
    state_0->size[1] = 2;
    MATLAB_emxEnsureCapacity_real_T(state_0, i);
    loop_ub = state->size[0];
    for (i = 0; i < 2; i++) {
      for (i_0 = 0; i_0 < loop_ub; i_0++) {
        state_0->data[i_0 + state_0->size[0] * i] = state->data[((int32_T)
          xyInd[i] - 1) * state->size[0] + i_0];
      }
    }

    validatorOccupancyMap_checkMapO(obj, state_0, idxInBounds);
    if ((state->size[0] != ssLowerBounds->size[0]) || (state->size[0] !=
         ssUpperBounds->size[0])) {
      MATLAB0_binary_expand_op_1(tmp, state, xyInd, ssLowerBounds, ssUpperBounds);
    }

    MATLAB0_binary_expand_op_1(tmp, state, xyInd, ssLowerBounds, ssUpperBounds);
    if ((state->size[0] == ssLowerBounds->size[0]) && (state->size[0] ==
         ssUpperBounds->size[0]) && ((state->size[0] == 1 ? ssLowerBounds->size
          [0] : state->size[0]) == (state->size[0] == 1 ? ssUpperBounds->size[0]
          : state->size[0])) && (tmp->size[0] == idxInBounds->size[0])) {
      i = state_1->size[0] * state_1->size[1];
      state_1->size[0] = state->size[0];
      state_1->size[1] = 2;
      MAT_emxEnsureCapacity_boolean_T(state_1, i);
      loop_ub = state->size[0];
      for (i = 0; i < 2; i++) {
        for (i_0 = 0; i_0 < loop_ub; i_0++) {
          state_tmp = state->data[((int32_T)xyInd[i] - 1) * state->size[0] + i_0];
          state_1->data[i_0 + state_1->size[0] * i] = ((state_tmp >=
            ssLowerBounds->data[ssLowerBounds->size[0] * i + i_0]) && (state_tmp
            <= ssUpperBounds->data[ssUpperBounds->size[0] * i + i_0]));
        }
      }

      MATLAB0_all(state_1, tmp);
      i = isValid->size[0];
      isValid->size[0] = tmp->size[0];
      MAT_emxEnsureCapacity_boolean_T(isValid, i);
      loop_ub = tmp->size[0];
      for (i = 0; i < loop_ub; i++) {
        isValid->data[i] = (tmp->data[i] && idxInBounds->data[i]);
      }
    } else {
      MATLAB0_binary_expand_op_2(isValid, state, xyInd, ssLowerBounds,
        ssUpperBounds, idxInBounds);
    }
  } else {
    i = state_0->size[0] * state_0->size[1];
    state_0->size[0] = state->size[0];
    state_0->size[1] = 2;
    MATLAB_emxEnsureCapacity_real_T(state_0, i);
    loop_ub = state->size[0];
    for (i = 0; i < 2; i++) {
      for (i_0 = 0; i_0 < loop_ub; i_0++) {
        state_0->data[i_0 + state_0->size[0] * i] = state->data[((int32_T)
          xyInd[i] - 1) * state->size[0] + i_0];
      }
    }

    MATLAB0_emxInit_real_T(&isOccupied, 1);
    binaryOccupancyMap_checkOccup_f(obj->Map, state_0, isOccupied, idxInBounds);
    if ((state->size[0] != ssLowerBounds->size[0]) || (state->size[0] !=
         ssUpperBounds->size[0])) {
      MATLAB0_binary_expand_op_1(tmp, state, xyInd, ssLowerBounds, ssUpperBounds);
    }

    if ((state->size[0] != ssLowerBounds->size[0]) || (state->size[0] !=
         ssUpperBounds->size[0]) || ((state->size[0] == 1 ? ssLowerBounds->size
          [0] : state->size[0]) != (state->size[0] == 1 ? ssUpperBounds->size[0]
          : state->size[0]))) {
      MATLAB0_binary_expand_op_1(tmp, state, xyInd, ssLowerBounds, ssUpperBounds);
    }

    MATLAB0_binary_expand_op_1(tmp, state, xyInd, ssLowerBounds, ssUpperBounds);
    MATLAB0_emxInit_boolean_T(&tmp_0, 1);
    MATLAB0_binary_expand_op_1(tmp_0, state, xyInd, ssLowerBounds, ssUpperBounds);
    if ((state->size[0] == ssLowerBounds->size[0]) && (state->size[0] ==
         ssUpperBounds->size[0]) && ((state->size[0] == 1 ? ssLowerBounds->size
          [0] : state->size[0]) == (state->size[0] == 1 ? ssUpperBounds->size[0]
          : state->size[0])) && (tmp->size[0] == idxInBounds->size[0]) &&
        ((tmp_0->size[0] == 1 ? idxInBounds->size[0] : tmp_0->size[0]) ==
         isOccupied->size[0])) {
      i = state_1->size[0] * state_1->size[1];
      state_1->size[0] = state->size[0];
      state_1->size[1] = 2;
      MAT_emxEnsureCapacity_boolean_T(state_1, i);
      loop_ub = state->size[0];
      for (i = 0; i < 2; i++) {
        for (i_0 = 0; i_0 < loop_ub; i_0++) {
          state_tmp = state->data[((int32_T)xyInd[i] - 1) * state->size[0] + i_0];
          state_1->data[i_0 + state_1->size[0] * i] = ((state_tmp >=
            ssLowerBounds->data[ssLowerBounds->size[0] * i + i_0]) && (state_tmp
            <= ssUpperBounds->data[ssUpperBounds->size[0] * i + i_0]));
        }
      }

      MATLAB0_all(state_1, tmp);
      i = isValid->size[0];
      isValid->size[0] = tmp->size[0];
      MAT_emxEnsureCapacity_boolean_T(isValid, i);
      loop_ub = tmp->size[0];
      for (i = 0; i < loop_ub; i++) {
        isValid->data[i] = (tmp->data[i] && idxInBounds->data[i] &&
                            (!(isOccupied->data[i] != 0.0)));
      }
    } else {
      MATLAB0_binary_expand_op_5(isValid, state, xyInd, ssLowerBounds,
        ssUpperBounds, idxInBounds, isOccupied);
    }

    MATLAB0_emxFree_boolean_T(&tmp_0);
    MATLAB0_emxFree_real_T(&isOccupied);
  }

  MATLAB0_emxFree_boolean_T(&state_1);
  MATLAB0_emxFree_real_T(&state_0);
  MATLAB0_emxFree_boolean_T(&tmp);
  MATLAB0_emxFree_boolean_T(&idxInBounds);
  MATLAB0_emxFree_real_T(&ssUpperBounds);
  MATLAB0_emxFree_real_T(&ssLowerBounds);
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static void plannerHybridAStar_isPrimitiveV(plannerHybridAStar_MATLAB0_T *obj,
  const real_T initialPose[3], const real_T finalPoses[30], const real_T
  ICRsData[24], const real_T radius_data[], real_T length, real_T stepSize,
  boolean_T result[10], real_T finalPosesGridIndices_data[], int32_T
  finalPosesGridIndices_size[2])
{
  static binaryOccupancyMap_MATLAB0_T *b_obj;
  static emxArray_boolean_T_MATLAB0_T *validMotionPrimitives;
  static emxArray_real_T_MATLAB0_T *m;
  static emxArray_real_T_MATLAB0_T *n;
  static emxArray_real_T_MATLAB0_T *samples;
  static emxArray_real_T_MATLAB0_T *varargin_1;
  static emxArray_real_T_MATLAB0_T *varargin_1_0;
  static emxArray_real_T_MATLAB0_T *varargin_2;
  static emxArray_real_T_MATLAB0_T *varargin_3;
  static real_T b_initialPose[30];
  static real_T ICRsData_data[24];
  static real_T b_initialPose_data[24];
  static real_T finalPoses_data[24];
  static real_T finalPointsGrid[20];
  static real_T finalPoses_data_0[20];
  static real_T validFinalPoseGrids[10];
  static real_T validIndex_data[10];
  static real_T radius_data_0[8];
  static real_T b_initialPose_0[3];
  static real_T finalPoses_0[3];
  static real_T b_b;
  static real_T c_tmp;
  static real_T k;
  static real_T numPoints;
  static int32_T a;
  static int32_T b_ii;
  static int32_T b_k;
  static int32_T c_k;
  static int32_T d;
  static int32_T e;
  static int32_T i;
  static int32_T ibmat;
  static int32_T idx;
  static int32_T itilerow;
  static int32_T ix;
  static int32_T jcol;
  static int32_T loop_ub;
  static int32_T nxout;
  static int32_T varargin_2_0;
  static int8_T ii_data[10];
  static int8_T tmp_data_0[10];
  static int8_T tmp_data[8];
  int8_T sizes_idx_1;
  boolean_T y;
  static const real_T j[10] = { 1.0, 1.0, 1.0, 1.0, -1.0, -1.0, -1.0, -1.0, 1.0,
    -1.0 };

  static emxArray_real_T_MATLAB0_T finalPoses_data_1;
  static emxArray_real_T_MATLAB0_T *samples_0;
  static emxArray_real_T_MATLAB0_T *samples_1;
  static int32_T ICRsData_size[2];
  static int32_T b_initialPose_size[2];
  static int32_T finalPoses_size[2];
  static int32_T finalPoses_size_0[2];
  static int32_T radius_size[2];
  static const int32_T tmp = 10;
  static int32_T ii_size_idx_0;
  static int32_T tmp_size_idx_0;
  static int32_T validIndex_size_idx_0;
  static int32_T varargin_2_idx_0;
  static int32_T varargin_3_idx_0;
  boolean_T exitg1;
  for (jcol = 0; jcol < 3; jcol++) {
    ibmat = jcol * 10;
    for (itilerow = 0; itilerow < 10; itilerow++) {
      b_initialPose[ibmat + itilerow] = initialPose[jcol];
    }
  }

  MATLAB0_MapInterface_world2grid(obj->Map, &finalPoses[0], finalPointsGrid);
  plannerHybridAStar_checkNodeVal(obj, finalPointsGrid, j, validFinalPoseGrids);
  finalPosesGridIndices_size[0] = 0;
  finalPosesGridIndices_size[1] = 0;
  y = false;
  b_k = 0;
  exitg1 = false;
  while ((!exitg1) && (b_k < 10)) {
    if ((validFinalPoseGrids[b_k] == 0.0) || rtIsNaN(validFinalPoseGrids[b_k]))
    {
      b_k++;
    } else {
      y = true;
      exitg1 = true;
    }
  }

  if (y) {
    idx = 0;
    b_ii = 0;
    exitg1 = false;
    while ((!exitg1) && (b_ii < 10)) {
      if (validFinalPoseGrids[b_ii] != 0.0) {
        idx++;
        ii_data[idx - 1] = (int8_T)(b_ii + 1);
        if (idx >= 10) {
          exitg1 = true;
        } else {
          b_ii++;
        }
      } else {
        b_ii++;
      }
    }

    if (idx < 1) {
      ii_size_idx_0 = 0;
    } else {
      ii_size_idx_0 = idx;
    }

    for (i = 0; i < ii_size_idx_0; i++) {
      validIndex_data[i] = ii_data[i];
    }

    loop_ub = 0;
    for (i = 0; i < 8; i++) {
      if (validFinalPoseGrids[i] == 1.0) {
        loop_ub++;
      }
    }

    tmp_size_idx_0 = loop_ub;
    loop_ub = 0;
    for (i = 0; i < 8; i++) {
      if (validFinalPoseGrids[i] == 1.0) {
        tmp_data[loop_ub] = (int8_T)i;
        loop_ub++;
      }
    }

    b_initialPose_size[0] = tmp_size_idx_0;
    b_initialPose_size[1] = 3;
    finalPoses_size[0] = tmp_size_idx_0;
    finalPoses_size[1] = 3;
    ICRsData_size[0] = tmp_size_idx_0;
    ICRsData_size[1] = 3;
    for (i = 0; i < 3; i++) {
      for (varargin_2_0 = 0; varargin_2_0 < tmp_size_idx_0; varargin_2_0++) {
        c_k = tmp_data[varargin_2_0];
        nxout = 10 * i + c_k;
        b_initialPose_data[varargin_2_0 + tmp_size_idx_0 * i] =
          b_initialPose[nxout];
        finalPoses_data[varargin_2_0 + tmp_size_idx_0 * i] = finalPoses[nxout];
        ICRsData_data[varargin_2_0 + tmp_size_idx_0 * i] = ICRsData[(i << 3) +
          c_k];
      }
    }

    radius_size[0] = 1;
    radius_size[1] = tmp_size_idx_0;
    for (i = 0; i < tmp_size_idx_0; i++) {
      radius_data_0[i] = radius_data[tmp_data[i]];
    }

    MATLAB0_emxInit_real_T(&varargin_1, 2);
    plannerHybridAStar_getPosesCirc(b_initialPose_data, b_initialPose_size,
      finalPoses_data, finalPoses_size, ICRsData_data, ICRsData_size,
      radius_data_0, radius_size, length, stepSize, varargin_1);
    MATLAB0_emxInit_real_T(&samples, 2);
    MATLAB0_linspace_jf(length / stepSize + 2.0, samples);
    nxout = samples->size[1];
    for (c_k = 0; c_k <= nxout - 2; c_k++) {
      samples->data[c_k] = samples->data[c_k + 1];
    }

    if (samples->size[1] - 1 < 1) {
      i = 1;
    } else {
      i = samples->size[1];
    }

    varargin_2_0 = samples->size[0] * samples->size[1];
    samples->size[1] = i - 1;
    MATLAB_emxEnsureCapacity_real_T(samples, varargin_2_0);
    MATLAB0_emxInit_real_T(&varargin_2, 2);
    varargin_2->size[0] = 0;
    varargin_2->size[1] = 0;
    MATLAB0_emxInit_real_T(&varargin_3, 2);
    varargin_3->size[0] = 0;
    varargin_3->size[1] = 0;
    MATLAB0_emxInit_real_T(&m, 2);
    if (validFinalPoseGrids[8] == 1.0) {
      b_initialPose_0[0] = b_initialPose[0];
      finalPoses_0[0] = finalPoses[8];
      b_initialPose_0[1] = b_initialPose[10];
      finalPoses_0[1] = finalPoses[18];
      b_initialPose_0[2] = b_initialPose[20];
      finalPoses_0[2] = finalPoses[28];
      MATLAB0_emxInit_real_T(&samples_1, 2);
      varargin_2_0 = samples_1->size[0] * samples_1->size[1];
      samples_1->size[0] = 1;
      samples_1->size[1] = samples->size[1];
      MATLAB_emxEnsureCapacity_real_T(samples_1, varargin_2_0);
      loop_ub = samples->size[0] * samples->size[1] - 1;
      for (varargin_2_0 = 0; varargin_2_0 <= loop_ub; varargin_2_0++) {
        samples_1->data[varargin_2_0] = samples->data[varargin_2_0];
      }

      MATLA_stateSpaceSE2_interpolate(obj->StateValidator->StateSpace,
        b_initialPose_0, finalPoses_0, samples_1, m);
      MATLAB0_emxFree_real_T(&samples_1);
      varargin_2_0 = varargin_2->size[0] * varargin_2->size[1];
      varargin_2->size[0] = m->size[0];
      varargin_2->size[1] = 3;
      MATLAB_emxEnsureCapacity_real_T(varargin_2, varargin_2_0);
      loop_ub = m->size[0] * 3;
      for (i = 0; i < loop_ub; i++) {
        varargin_2->data[i] = m->data[i];
      }
    }

    if (validFinalPoseGrids[9] == 1.0) {
      b_initialPose_0[0] = b_initialPose[0];
      finalPoses_0[0] = finalPoses[9];
      b_initialPose_0[1] = b_initialPose[10];
      finalPoses_0[1] = finalPoses[19];
      b_initialPose_0[2] = b_initialPose[20];
      finalPoses_0[2] = finalPoses[29];
      MATLAB0_emxInit_real_T(&samples_0, 2);
      varargin_2_0 = samples_0->size[0] * samples_0->size[1];
      samples_0->size[0] = 1;
      samples_0->size[1] = samples->size[1];
      MATLAB_emxEnsureCapacity_real_T(samples_0, varargin_2_0);
      loop_ub = samples->size[0] * samples->size[1] - 1;
      for (varargin_2_0 = 0; varargin_2_0 <= loop_ub; varargin_2_0++) {
        samples_0->data[varargin_2_0] = samples->data[varargin_2_0];
      }

      MATLA_stateSpaceSE2_interpolate(obj->StateValidator->StateSpace,
        b_initialPose_0, finalPoses_0, samples_0, m);
      MATLAB0_emxFree_real_T(&samples_0);
      varargin_2_0 = varargin_3->size[0] * varargin_3->size[1];
      varargin_3->size[0] = m->size[0];
      varargin_3->size[1] = 3;
      MATLAB_emxEnsureCapacity_real_T(varargin_3, varargin_2_0);
      loop_ub = m->size[0] * 3;
      for (i = 0; i < loop_ub; i++) {
        varargin_3->data[i] = m->data[i];
      }
    }

    MATLAB0_emxFree_real_T(&m);
    MATLAB0_emxFree_real_T(&samples);
    numPoints = obj->NumPointsMotionPrimitive - 1.0;
    if (varargin_1->size[0] != 0) {
      sizes_idx_1 = 3;
      loop_ub = varargin_1->size[0];
    } else {
      if ((varargin_2->size[0] != 0) && (varargin_2->size[1] != 0)) {
        sizes_idx_1 = (int8_T)varargin_2->size[1];
      } else if ((varargin_3->size[0] != 0) && (varargin_3->size[1] != 0)) {
        sizes_idx_1 = (int8_T)varargin_3->size[1];
      } else {
        sizes_idx_1 = 3;
      }

      loop_ub = 0;
    }

    tmp_size_idx_0 = sizes_idx_1;
    if ((varargin_2->size[0] != 0) && (varargin_2->size[1] != 0)) {
      varargin_2_idx_0 = varargin_2->size[0];
    } else {
      varargin_2_idx_0 = 0;
    }

    if ((varargin_3->size[0] != 0) && (varargin_3->size[1] != 0)) {
      varargin_3_idx_0 = varargin_3->size[0];
    } else {
      varargin_3_idx_0 = 0;
    }

    MATLAB0_emxInit_real_T(&varargin_1_0, 2);
    varargin_2_0 = varargin_1_0->size[0] * varargin_1_0->size[1];
    varargin_1_0->size[0] = (loop_ub + varargin_2_idx_0) + varargin_3_idx_0;
    varargin_1_0->size[1] = sizes_idx_1;
    MATLAB_emxEnsureCapacity_real_T(varargin_1_0, varargin_2_0);
    for (i = 0; i < tmp_size_idx_0; i++) {
      for (varargin_2_0 = 0; varargin_2_0 < loop_ub; varargin_2_0++) {
        varargin_1_0->data[varargin_2_0 + varargin_1_0->size[0] * i] =
          varargin_1->data[loop_ub * i + varargin_2_0];
      }

      for (varargin_2_0 = 0; varargin_2_0 < varargin_2_idx_0; varargin_2_0++) {
        varargin_1_0->data[(varargin_2_0 + loop_ub) + varargin_1_0->size[0] * i]
          = varargin_2->data[varargin_2_idx_0 * i + varargin_2_0];
      }

      for (varargin_2_0 = 0; varargin_2_0 < varargin_3_idx_0; varargin_2_0++) {
        varargin_1_0->data[((varargin_2_0 + loop_ub) + varargin_2_idx_0) +
          varargin_1_0->size[0] * i] = varargin_3->data[varargin_3_idx_0 * i +
          varargin_2_0];
      }
    }

    MATLAB0_emxFree_real_T(&varargin_3);
    MATLAB0_emxFree_real_T(&varargin_2);
    MATLAB0_emxFree_real_T(&varargin_1);
    MATLAB0_emxInit_boolean_T(&validMotionPrimitives, 1);
    validatorOccupancyMap_isState_c(obj->StateValidator, varargin_1_0,
      validMotionPrimitives);
    MATLAB0_emxFree_real_T(&varargin_1_0);
    k = 1.0;
    for (i = 0; i < ii_size_idx_0; i++) {
      c_tmp = k + numPoints;
      if (k > c_tmp - 1.0) {
        e = 1;
        d = 1;
      } else {
        e = (int32_T)k;
        d = (int32_T)(c_tmp - 1.0) + 1;
      }

      y = true;
      a = d - e;
      ix = 1;
      exitg1 = false;
      while ((!exitg1) && (ix <= a)) {
        if (!validMotionPrimitives->data[(e + ix) - 2]) {
          y = false;
          exitg1 = true;
        } else {
          ix++;
        }
      }

      validFinalPoseGrids[(int32_T)validIndex_data[i] - 1] = y;
      k = c_tmp;
    }

    MATLAB0_emxFree_boolean_T(&validMotionPrimitives);
    for (i = 0; i < 10; i++) {
      result[i] = (validFinalPoseGrids[i] == 1.0);
    }

    if (MATLAB0_vectorAny(result, &tmp)) {
      b_obj = obj->Map;
      loop_ub = 0;
      for (i = 0; i < 10; i++) {
        if (result[i]) {
          loop_ub++;
        }
      }

      tmp_size_idx_0 = loop_ub;
      loop_ub = 0;
      for (i = 0; i < 10; i++) {
        if (result[i]) {
          tmp_data_0[loop_ub] = (int8_T)i;
          loop_ub++;
        }
      }

      finalPoses_size_0[0] = tmp_size_idx_0;
      finalPoses_size_0[1] = 2;
      for (i = 0; i < 2; i++) {
        for (varargin_2_0 = 0; varargin_2_0 < tmp_size_idx_0; varargin_2_0++) {
          finalPoses_data_0[varargin_2_0 + finalPoses_size_0[0] * i] =
            finalPoses[10 * i + tmp_data_0[varargin_2_0]];
        }
      }

      finalPoses_data_1.data = &finalPoses_data_0[0];
      finalPoses_data_1.size = &finalPoses_size_0[0];
      finalPoses_data_1.allocatedSize = 20;
      finalPoses_data_1.numDimensions = 2;
      finalPoses_data_1.canFreeData = false;
      MATLAB0_emxInit_real_T(&n, 2);
      M_MapInterface_world2gridImpl_d(b_obj, &finalPoses_data_1, n);
      b_b = obj->Dimensions[0];
      validIndex_size_idx_0 = n->size[0];
      loop_ub = n->size[0];
      for (i = 0; i < loop_ub; i++) {
        validIndex_data[i] = (n->data[i + n->size[0]] - 1.0) * b_b;
      }

      finalPosesGridIndices_size[0] = validIndex_size_idx_0;
      finalPosesGridIndices_size[1] = 1;
      for (i = 0; i < validIndex_size_idx_0; i++) {
        finalPosesGridIndices_data[i] = validIndex_data[i] + n->data[i];
      }

      MATLAB0_emxFree_real_T(&n);
    }
  } else {
    for (i = 0; i < 10; i++) {
      result[i] = (validFinalPoseGrids[i] == 1.0);
    }
  }
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static int32_T MATLAB0_intnnz(const boolean_T s[10])
{
  int32_T k;
  int32_T n;
  n = 0;
  for (k = 0; k < 10; k++) {
    if (s[k]) {
      n++;
    }
  }

  return n;
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static void MATLAB0_NodeMap_traceBack(nav_algs_internal_NodeMap_MAT_T *obj,
  real_T idx, emxArray_real_T_MATLAB0_T *nodeDataVec)
{
  static real_T dim;
  static real_T numNodes;
  emxArray_real_T_MATLAB0_T *data;
  real_T maxNumNodes;
  int32_T i;
  int32_T i_0;
  int32_T loop_ub;
  int32_T loop_ub_0;
  dim = priorityqueuecodegen_nodemap_getDataDim(obj->NodeMapInternal);
  maxNumNodes = priorityqueuecodegen_nodemap_getNumNodes(obj->NodeMapInternal);
  MATLAB0_emxInit_real_T(&data, 2);
  i_0 = data->size[0] * data->size[1];
  data->size[0] = (int32_T)dim;
  data->size[1] = (int32_T)maxNumNodes;
  MATLAB_emxEnsureCapacity_real_T(data, i_0);
  priorityqueuecodegen_nodemap_traceBack(obj->NodeMapInternal, idx, &data->data
    [0], &numNodes);
  if (numNodes < 1.0) {
    loop_ub = -1;
  } else {
    loop_ub = (int32_T)numNodes - 1;
  }

  if (dim < 1.0) {
    loop_ub_0 = -1;
  } else {
    loop_ub_0 = (int32_T)dim - 1;
  }

  i_0 = nodeDataVec->size[0] * nodeDataVec->size[1];
  nodeDataVec->size[0] = loop_ub + 1;
  nodeDataVec->size[1] = loop_ub_0 + 1;
  MATLAB_emxEnsureCapacity_real_T(nodeDataVec, i_0);
  for (i_0 = 0; i_0 <= loop_ub_0; i_0++) {
    for (i = 0; i <= loop_ub; i++) {
      nodeDataVec->data[i + nodeDataVec->size[0] * i_0] = data->data[data->size
        [0] * i + i_0];
    }
  }

  MATLAB0_emxFree_real_T(&data);
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static void MATLAB0_flipud(emxArray_real_T_MATLAB0_T *x)
{
  real_T xtmp;
  int32_T b;
  int32_T i;
  int32_T j;
  int32_T m;
  int32_T md2;
  int32_T tmp;
  m = x->size[0] - 1;
  md2 = x->size[0] >> 1;
  b = x->size[1];
  for (j = 0; j < b; j++) {
    for (i = 0; i < md2; i++) {
      xtmp = x->data[x->size[0] * j + i];
      tmp = m - i;
      x->data[i + x->size[0] * j] = x->data[x->size[0] * j + tmp];
      x->data[tmp + x->size[0] * j] = xtmp;
    }
  }
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static boolean_T MATLAB0_strcmp_c3(const char_T a_data[], const int32_T a_size[2])
{
  int32_T kstr;
  boolean_T b_bool;
  static const char_T c[6] = { 'g', 'r', 'e', 'e', 'd', 'y' };

  int32_T exitg1;
  b_bool = false;
  if (a_size[1] == 6) {
    kstr = 0;
    do {
      exitg1 = 0;
      if (kstr < 6) {
        if (a_data[kstr] != c[kstr]) {
          exitg1 = 1;
        } else {
          kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  return b_bool;
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static void plannerHybridAStar_getFinalPath(const plannerHybridAStar_MATLAB0_T
  *obj, const emxArray_real_T_MATLAB0_T *pathData, emxArray_real_T_MATLAB0_T
  *finalPathData)
{
  static emxArray_int32_T_MATLAB0_T *tmp;
  static emxArray_int32_T_MATLAB0_T *tmp_0;
  static real_T b_finalPathData[23237500];
  static real_T pathData_0;
  static real_T pathData_1;
  static boolean_T d[2112500];
  static boolean_T e[2112500];
  static boolean_T f[2112500];
  static boolean_T g[2112500];
  real_T pathData_2;
  real_T pathData_3;
  int32_T PathDataRow;
  int32_T b;
  int32_T i;
  int32_T trueCount;
  boolean_T d_0;
  boolean_T e_0;
  boolean_T f_0;
  boolean_T g_0;
  memset(&b_finalPathData[0], 0, 23237500U * sizeof(real_T));
  b = pathData->size[0] - 2;
  MATLAB0_emxInit_int32_T(&tmp, 1);
  MATLAB0_emxInit_int32_T(&tmp_0, 1);
  for (PathDataRow = 0; PathDataRow <= b; PathDataRow++) {
    if (pathData->data[((pathData->size[0] << 1) + PathDataRow) + 1] !=
        pathData->data[(pathData->size[0] << 1) + PathDataRow]) {
      pathData_0 = pathData->data[PathDataRow];
      pathData_1 = pathData->data[PathDataRow + pathData->size[0]];
      pathData_2 = pathData->data[PathDataRow + 1];
      pathData_3 = pathData->data[(PathDataRow + pathData->size[0]) + 1];
      trueCount = 0;
      for (i = 0; i < 2112500; i++) {
        f_0 = (pathData_0 == obj->PrimitivesData[i]);
        f[i] = f_0;
        g_0 = (obj->PrimitivesData[i + 2112500] == pathData_1);
        g[i] = g_0;
        e_0 = (obj->PrimitivesData[i + 6337500] == pathData_2);
        e[i] = e_0;
        d_0 = (obj->PrimitivesData[i + 8450000] == pathData_3);
        d[i] = d_0;
        if (f_0 && g_0 && e_0 && d_0) {
          trueCount++;
        }
      }

      i = tmp->size[0];
      tmp->size[0] = trueCount;
      MATLA_emxEnsureCapacity_int32_T(tmp, i);
      trueCount = 0;
      for (i = 0; i < 2112500; i++) {
        if (f[i] && g[i] && e[i] && d[i]) {
          tmp->data[trueCount] = i;
          trueCount++;
        }
      }

      for (i = 0; i < 11; i++) {
        b_finalPathData[PathDataRow + 2112500 * i] = obj->PrimitivesData[2112500
          * i + tmp->data[0]];
      }
    } else {
      pathData_0 = pathData->data[PathDataRow];
      pathData_1 = pathData->data[PathDataRow + pathData->size[0]];
      pathData_2 = pathData->data[PathDataRow + 1];
      pathData_3 = pathData->data[(PathDataRow + pathData->size[0]) + 1];
      trueCount = 0;
      for (i = 0; i < 2112500; i++) {
        f_0 = (pathData_0 == obj->LinesData[i]);
        f[i] = f_0;
        g_0 = (obj->LinesData[i + 2112500] == pathData_1);
        g[i] = g_0;
        e_0 = (obj->LinesData[i + 6337500] == pathData_2);
        e[i] = e_0;
        d_0 = (obj->LinesData[i + 8450000] == pathData_3);
        d[i] = d_0;
        if (f_0 && g_0 && e_0 && d_0) {
          trueCount++;
        }
      }

      i = tmp_0->size[0];
      tmp_0->size[0] = trueCount;
      MATLA_emxEnsureCapacity_int32_T(tmp_0, i);
      trueCount = 0;
      for (i = 0; i < 2112500; i++) {
        if (f[i] && g[i] && e[i] && d[i]) {
          tmp_0->data[trueCount] = i;
          trueCount++;
        }
      }

      for (i = 0; i < 6; i++) {
        b_finalPathData[PathDataRow + 2112500 * i] = obj->LinesData[2112500 * i
          + tmp_0->data[0]];
      }

      b_finalPathData[PathDataRow + 12675000] = (rtNaN);
      b_finalPathData[PathDataRow + 14787500] = (rtNaN);
      b_finalPathData[PathDataRow + 16900000] = (rtNaN);
      b_finalPathData[PathDataRow + 19012500] = (rtInf);
      b_finalPathData[PathDataRow + 21125000] = obj->LinesData[tmp_0->data[0] +
        12675000];
    }
  }

  MATLAB0_emxFree_int32_T(&tmp_0);
  MATLAB0_emxFree_int32_T(&tmp);
  PathDataRow = pathData->size[0] - 2 < 0 ? 0 : pathData->size[0] - 1;
  if (PathDataRow < 1) {
    b = -1;
  } else {
    b = PathDataRow - 1;
  }

  i = finalPathData->size[0] * finalPathData->size[1];
  finalPathData->size[0] = b + 1;
  finalPathData->size[1] = 11;
  MATLAB_emxEnsureCapacity_real_T(finalPathData, i);
  for (i = 0; i < 11; i++) {
    for (PathDataRow = 0; PathDataRow <= b; PathDataRow++) {
      finalPathData->data[PathDataRow + finalPathData->size[0] * i] =
        b_finalPathData[2112500 * i + PathDataRow];
    }
  }
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static void MATLAB0_nonzeros(const real_T s[5], real_T v_data[], int32_T *v_size)
{
  real_T s_0;
  int32_T b_k;
  int32_T i;
  i = 0;
  for (b_k = 0; b_k < 5; b_k++) {
    if (s[b_k] != 0.0) {
      i++;
    }
  }

  *v_size = i;
  i = -1;
  for (b_k = 0; b_k < 5; b_k++) {
    s_0 = s[b_k];
    if (s_0 != 0.0) {
      i++;
      v_data[i] = s_0;
    }
  }
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static void MATLAB0_diff(const real_T x_data[], const int32_T *x_size, real_T
  y_data[], int32_T *y_size)
{
  static real_T tmp1;
  static real_T work_data_idx_0;
  int32_T dimSize;
  int32_T m;
  dimSize = *x_size;
  if (*x_size == 0) {
    *y_size = 0;
  } else {
    if (*x_size - 1 <= 1) {
      m = *x_size - 1;
    } else {
      m = 1;
    }

    if (m < 1) {
      *y_size = 0;
    } else {
      *y_size = *x_size - 1;
      if (*x_size - 1 != 0) {
        work_data_idx_0 = x_data[0];
        for (m = 2; m <= dimSize; m++) {
          tmp1 = x_data[m - 1];
          y_data[m - 2] = tmp1 - work_data_idx_0;
          work_data_idx_0 = tmp1;
        }
      }
    }
  }
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static void ReedsSheppBuiltins_autonomous_n(const real_T startPose[3], const
  real_T goalPose[3], const emxArray_real_T_MATLAB0_T *samples, real_T
  turningRadius, const real_T segmentsLengths[5], const int32_T
  segmentsDirections[5], const uint32_T segmentsTypes[5],
  emxArray_real_T_MATLAB0_T *poses, emxArray_real_T_MATLAB0_T *directions)
{
  static emxArray_real_T_MATLAB0_T *b_poses;
  emxArray_real_T_MATLAB0_T *b_directions;
  int32_T i;
  int32_T loop_ub;
  MATLAB0_emxInit_real_T(&b_poses, 2);
  i = b_poses->size[0] * b_poses->size[1];
  b_poses->size[0] = samples->size[1];
  b_poses->size[1] = 3;
  MATLAB_emxEnsureCapacity_real_T(b_poses, i);
  MATLAB0_emxInit_real_T(&b_directions, 1);
  i = b_directions->size[0];
  b_directions->size[0] = samples->size[1];
  MATLAB_emxEnsureCapacity_real_T(b_directions, i);
  autonomousReedsSheppInterpolateSegmentsCodegen_real64(&startPose[0],
    &goalPose[0], &samples->data[0], (uint32_T)samples->size[1], turningRadius,
    &segmentsLengths[0], &segmentsDirections[0], &segmentsTypes[0],
    &b_poses->data[0], &b_directions->data[0]);
  i = poses->size[0] * poses->size[1];
  poses->size[0] = b_poses->size[0];
  poses->size[1] = 3;
  MATLAB_emxEnsureCapacity_real_T(poses, i);
  loop_ub = b_poses->size[0] * 3;
  for (i = 0; i < loop_ub; i++) {
    poses->data[i] = b_poses->data[i];
  }

  MATLAB0_emxFree_real_T(&b_poses);
  i = directions->size[0];
  directions->size[0] = b_directions->size[0];
  MATLAB_emxEnsureCapacity_real_T(directions, i);
  loop_ub = b_directions->size[0];
  for (i = 0; i < loop_ub; i++) {
    directions->data[i] = b_directions->data[i];
  }

  MATLAB0_emxFree_real_T(&b_directions);
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static void MATLAB0_unique_vector(const emxArray_real_T_MATLAB0_T *a,
  emxArray_real_T_MATLAB0_T *b)
{
  static emxArray_int32_T_MATLAB0_T *d;
  static real_T tmp;
  static real_T x;
  static int32_T b_k;
  static int32_T c;
  static int32_T c_k;
  static int32_T j;
  static int32_T k;
  static int32_T nInf;
  static int32_T nMInf;
  static int32_T nNaN;
  static int32_T na;
  static int32_T nb;
  int32_T tmp_0;
  boolean_T exitg1;
  na = a->size[1];
  MATLAB0_emxInit_int32_T(&d, 2);
  MATLAB0_sortIdx(a, d);
  tmp_0 = b->size[0] * b->size[1];
  b->size[0] = 1;
  b->size[1] = a->size[1];
  MATLAB_emxEnsureCapacity_real_T(b, tmp_0);
  for (b_k = 0; b_k < na; b_k++) {
    b->data[b_k] = a->data[d->data[b_k] - 1];
  }

  MATLAB0_emxFree_int32_T(&d);
  c_k = 0;
  while ((c_k + 1 <= na) && rtIsInf(b->data[c_k]) && (b->data[c_k] < 0.0)) {
    c_k++;
  }

  nMInf = c_k;
  c_k = a->size[1];
  while ((c_k >= 1) && rtIsNaN(b->data[c_k - 1])) {
    c_k--;
  }

  nNaN = a->size[1] - c_k;
  exitg1 = false;
  while ((!exitg1) && (c_k >= 1)) {
    tmp = b->data[c_k - 1];
    if (rtIsInf(tmp) && (tmp > 0.0)) {
      c_k--;
    } else {
      exitg1 = true;
    }
  }

  nInf = (a->size[1] - c_k) - nNaN;
  nb = -1;
  if (nMInf > 0) {
    nb = 0;
  }

  k = nMInf;
  while (k + 1 <= c_k) {
    x = b->data[k];
    do {
      k++;
    } while (!((k + 1 > c_k) || (b->data[k] != x)));

    nb++;
    b->data[nb] = x;
  }

  if (nInf > 0) {
    nb++;
    b->data[nb] = b->data[c_k];
  }

  k = c_k + nInf;
  for (j = 0; j < nNaN; j++) {
    b->data[(nb + j) + 1] = b->data[k + j];
  }

  nb = (nNaN - 1 < 0 ? nb : nb + nNaN) + 1;
  if (nb < 1) {
    c = 0;
  } else {
    c = nb;
  }

  tmp_0 = b->size[0] * b->size[1];
  b->size[1] = c;
  MATLAB_emxEnsureCapacity_real_T(b, tmp_0);
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static void plannerHybridAStar_getInterpola(const plannerHybridAStar_MATLAB0_T
  *obj, const emxArray_real_T_MATLAB0_T *pathData, emxArray_real_T_MATLAB0_T
  *path, emxArray_real_T_MATLAB0_T *dir)
{
  static emxArray_real_T_MATLAB0_T *directions;
  static emxArray_real_T_MATLAB0_T *expansionDirs;
  static emxArray_real_T_MATLAB0_T *expansionPoints;
  static emxArray_real_T_MATLAB0_T *g;
  static emxArray_real_T_MATLAB0_T *g_0;
  static emxArray_real_T_MATLAB0_T *samples;
  static emxArray_real_T_MATLAB0_T *states;
  static emxArray_real_T_MATLAB0_T *tmp;
  static real_T b_y[5];
  static real_T segmentDirections[5];
  static real_T x_data[5];
  static real_T getSwitchingMotion_data[4];
  static real_T curvature;
  static real_T direction_tmp;
  static real_T lengthOnPrimitive;
  static real_T primitivePathLength;
  static real_T turningRadius;
  static real_T u;
  static real_T u_tmp;
  static real_T val;
  static real_T val_tmp;
  static real_T y;
  static int32_T segmentDirections_0[5];
  static int32_T b_i;
  static int32_T b_idx;
  static int32_T b_k;
  static int32_T d_k;
  static int32_T f_k;
  static int32_T getSwitchingMotion_size;
  static int32_T i;
  static int32_T idx;
  static int32_T j;
  static int32_T k;
  static int32_T x_size;
  static uint32_T obj_0[5];
  int32_T c_k;
  int32_T loop_ub;
  int32_T loop_ub_0;
  int32_T trueCount;
  int8_T tmp_data[4];
  boolean_T b_p;
  boolean_T exitg1;
  boolean_T p;
  primitivePathLength = obj->MotionPrimitiveLength * (real_T)pathData->size[0];
  val_tmp = primitivePathLength / obj->InterpolationDistance;
  val = floor(val_tmp);
  MATLAB0_emxInit_real_T(&states, 2);
  trueCount = states->size[0] * states->size[1];
  states->size[0] = (int32_T)val;
  states->size[1] = 3;
  MATLAB_emxEnsureCapacity_real_T(states, trueCount);
  loop_ub = (int32_T)val * 3;
  if (loop_ub - 1 >= 0) {
    memset(&states->data[0], 0, (uint32_T)loop_ub * sizeof(real_T));
  }

  MATLAB0_emxInit_real_T(&directions, 1);
  trueCount = directions->size[0];
  directions->size[0] = (int32_T)val;
  MATLAB_emxEnsureCapacity_real_T(directions, trueCount);
  if ((int32_T)val - 1 >= 0) {
    memset(&directions->data[0], 0, (uint32_T)(int32_T)val * sizeof(real_T));
  }

  i = 0;
  exitg1 = false;
  while ((!exitg1) && (i <= (int32_T)val_tmp - 1)) {
    u_tmp = ((real_T)i + 1.0) * obj->InterpolationDistance;
    u = u_tmp / obj->MotionPrimitiveLength;
    if (u < 0.0) {
      y = ceil(u);
    } else {
      y = floor(u);
    }

    lengthOnPrimitive = MATLAB0_rt_remd_snf(u_tmp, obj->MotionPrimitiveLength);
    if (u_tmp >= primitivePathLength) {
      idx = states->size[0];
      loop_ub = states->size[0] - 1;
      for (j = 0; j < 3; j++) {
        for (b_i = idx; b_i <= loop_ub; b_i++) {
          states->data[(b_i + states->size[0] * j) - 1] = states->data
            [states->size[0] * j + b_i];
        }
      }

      if (states->size[0] - 1 < 1) {
        c_k = -1;
        trueCount = -1;
      } else {
        c_k = states->size[0] - 2;
        trueCount = states->size[0] - 2;
      }

      loop_ub_0 = trueCount + 1;
      for (trueCount = 0; trueCount < 3; trueCount++) {
        for (f_k = 0; f_k < loop_ub_0; f_k++) {
          states->data[f_k + (c_k + 1) * trueCount] = states->data[states->size
            [0] * trueCount + f_k];
        }
      }

      trueCount = states->size[0] * states->size[1];
      states->size[0] = c_k + 1;
      states->size[1] = 3;
      MATLAB_emxEnsureCapacity_real_T(states, trueCount);
      b_idx = directions->size[0];
      for (f_k = b_idx; f_k <= loop_ub; f_k++) {
        directions->data[f_k - 1] = directions->data[f_k];
      }

      if (directions->size[0] - 1 < 1) {
        c_k = 1;
      } else {
        c_k = directions->size[0];
      }

      trueCount = directions->size[0];
      directions->size[0] = c_k - 1;
      MATLAB_emxEnsureCapacity_real_T(directions, trueCount);
      exitg1 = true;
    } else {
      u_tmp = pathData->data[((int32_T)(y + 1.0) + pathData->size[0] * 9) - 1];
      if (rtIsInf(u_tmp)) {
        direction_tmp = pathData->data[((int32_T)(y + 1.0) + pathData->size[0] *
          10) - 1];
        u_tmp = pathData->data[((int32_T)(y + 1.0) + (pathData->size[0] << 1)) -
          1];
        u = direction_tmp * lengthOnPrimitive;
        states->data[i] = pathData->data[(int32_T)(y + 1.0) - 1] + cos(u_tmp) *
          u;
        states->data[i + states->size[0]] = pathData->data[((int32_T)(y + 1.0) +
          pathData->size[0]) - 1] + sin(u_tmp) * u;
        states->data[i + (states->size[0] << 1)] = u_tmp;
        directions->data[i] = direction_tmp;
      } else {
        curvature = 1.0 / u_tmp;
        direction_tmp = pathData->data[((int32_T)(y + 1.0) + pathData->size[0] *
          10) - 1];
        turningRadius = 1.0 / curvature;
        u_tmp = pathData->data[((int32_T)(y + 1.0) + (pathData->size[0] << 1)) -
          1];
        u = lengthOnPrimitive * curvature * direction_tmp + u_tmp;
        states->data[i] = (pathData->data[(int32_T)(y + 1.0) - 1] - sin(u_tmp) *
                           turningRadius) + sin(u) * turningRadius;
        states->data[i + states->size[0]] = (pathData->data[((int32_T)(y + 1.0)
          + pathData->size[0]) - 1] + cos(u_tmp) * turningRadius) - cos(u) *
          turningRadius;
        states->data[i + (states->size[0] << 1)] = u;
        directions->data[i] = direction_tmp;
      }

      i++;
    }
  }

  MATLAB0_emxInit_real_T(&g, 2);
  MATLAB0_linspace_j(obj->InterpolationDistance, obj->AnalyticPathLength,
                     obj->AnalyticPathLength / obj->InterpolationDistance, g);
  MATLAB0_emxInit_real_T(&samples, 2);
  trueCount = samples->size[0] * samples->size[1];
  samples->size[0] = 1;
  samples->size[1] = g->size[1];
  MATLAB_emxEnsureCapacity_real_T(samples, trueCount);
  loop_ub_0 = g->size[1];
  for (c_k = 0; c_k < loop_ub_0; c_k++) {
    samples->data[c_k] = g->data[c_k];
  }

  if (g->size[1] != 0) {
    MATLAB0_nonzeros(obj->AnalyticPathSegments, x_data, &x_size);
    for (k = 0; k < x_size; k++) {
      u = x_data[k];
      if (rtIsNaN(u)) {
        x_data[k] = (rtNaN);
      } else if (u < 0.0) {
        x_data[k] = -1.0;
      } else {
        x_data[k] = (u > 0.0);
      }
    }

    MATLAB0_diff(x_data, &x_size, getSwitchingMotion_data,
                 &getSwitchingMotion_size);
    for (b_k = 0; b_k < 5; b_k++) {
      segmentDirections[b_k] = fabs(obj->AnalyticPathSegments[b_k]);
    }

    for (d_k = 0; d_k < 4; d_k++) {
      segmentDirections[d_k + 1] += segmentDirections[d_k];
    }

    loop_ub_0 = getSwitchingMotion_size - 1;
    trueCount = 0;
    for (c_k = 0; c_k <= loop_ub_0; c_k++) {
      u_tmp = getSwitchingMotion_data[c_k];
      if ((u_tmp == 2.0) || (u_tmp == -2.0)) {
        trueCount++;
      }
    }

    loop_ub = trueCount;
    trueCount = 0;
    for (c_k = 0; c_k <= loop_ub_0; c_k++) {
      u_tmp = getSwitchingMotion_data[c_k];
      if ((u_tmp == 2.0) || (u_tmp == -2.0)) {
        tmp_data[trueCount] = (int8_T)c_k;
        trueCount++;
      }
    }

    MATLAB0_emxInit_real_T(&g_0, 2);
    trueCount = g_0->size[0] * g_0->size[1];
    g_0->size[0] = 1;
    g_0->size[1] = loop_ub + g->size[1];
    MATLAB_emxEnsureCapacity_real_T(g_0, trueCount);
    loop_ub_0 = g->size[1];
    for (c_k = 0; c_k < loop_ub_0; c_k++) {
      g_0->data[c_k] = g->data[c_k];
    }

    for (c_k = 0; c_k < loop_ub; c_k++) {
      g_0->data[c_k + g->size[1]] = segmentDirections[tmp_data[c_k]];
    }

    MATLAB0_unique_vector(g_0, samples);
    MATLAB0_emxFree_real_T(&g_0);
  }

  MATLAB0_emxFree_real_T(&g);
  for (c_k = 0; c_k < 5; c_k++) {
    segmentDirections[c_k] = 1.0;
    u_tmp = obj->AnalyticPathSegments[c_k];
    if (u_tmp < 0.0) {
      segmentDirections[c_k] = -1.0;
    }

    b_y[c_k] = fabs(u_tmp);
  }

  MATLAB0_emxInit_real_T(&tmp, 2);
  trueCount = tmp->size[0] * tmp->size[1];
  tmp->size[0] = 1;
  tmp->size[1] = samples->size[1] + 1;
  MATLAB_emxEnsureCapacity_real_T(tmp, trueCount);
  tmp->data[0] = 0.0;
  loop_ub_0 = samples->size[1];
  for (c_k = 0; c_k < loop_ub_0; c_k++) {
    tmp->data[c_k + 1] = samples->data[c_k];
  }

  MATLAB0_emxFree_real_T(&samples);
  for (c_k = 0; c_k < 5; c_k++) {
    segmentDirections_0[c_k] = (int32_T)segmentDirections[c_k];
    u_tmp = MATLAB0_rt_roundd_snf(obj->AnalyticPathTypes[c_k]);
    if (u_tmp < 4.294967296E+9) {
      if (u_tmp >= 0.0) {
        obj_0[c_k] = (uint32_T)u_tmp;
      } else {
        obj_0[c_k] = 0U;
      }
    } else {
      obj_0[c_k] = MAX_uint32_T;
    }
  }

  MATLAB0_emxInit_real_T(&expansionPoints, 2);
  MATLAB0_emxInit_real_T(&expansionDirs, 1);
  ReedsSheppBuiltins_autonomous_n(obj->ExpansionPoint, obj->GoalPose, tmp,
    obj->MinTurningRadius, b_y, segmentDirections_0, obj_0, expansionPoints,
    expansionDirs);
  MATLAB0_emxFree_real_T(&tmp);
  p = false;
  b_p = true;
  loop_ub = 0;
  exitg1 = false;
  while ((!exitg1) && (loop_ub < 3)) {
    if (!(obj->StartPose[loop_ub] == obj->ExpansionPoint[loop_ub])) {
      b_p = false;
      exitg1 = true;
    } else {
      loop_ub++;
    }
  }

  if (b_p) {
    p = true;
  }

  if (!p) {
    trueCount = path->size[0] * path->size[1];
    path->size[0] = (states->size[0] + expansionPoints->size[0]) + 1;
    path->size[1] = 3;
    MATLAB_emxEnsureCapacity_real_T(path, trueCount);
    loop_ub_0 = states->size[0];
    loop_ub = expansionPoints->size[0];
    for (c_k = 0; c_k < 3; c_k++) {
      path->data[path->size[0] * c_k] = obj->StartPose[c_k];
      for (trueCount = 0; trueCount < loop_ub_0; trueCount++) {
        path->data[(trueCount + path->size[0] * c_k) + 1] = states->data
          [states->size[0] * c_k + trueCount];
      }

      for (trueCount = 0; trueCount < loop_ub; trueCount++) {
        path->data[((trueCount + states->size[0]) + path->size[0] * c_k) + 1] =
          expansionPoints->data[expansionPoints->size[0] * c_k + trueCount];
      }
    }

    trueCount = dir->size[0];
    dir->size[0] = (directions->size[0] + expansionDirs->size[0]) + 1;
    MATLAB_emxEnsureCapacity_real_T(dir, trueCount);
    dir->data[0] = pathData->data[pathData->size[0] * 10];
    loop_ub_0 = directions->size[0];
    for (c_k = 0; c_k < loop_ub_0; c_k++) {
      dir->data[c_k + 1] = directions->data[c_k];
    }

    loop_ub_0 = expansionDirs->size[0];
    for (c_k = 0; c_k < loop_ub_0; c_k++) {
      dir->data[(c_k + directions->size[0]) + 1] = expansionDirs->data[c_k];
    }
  } else {
    trueCount = path->size[0] * path->size[1];
    path->size[0] = expansionPoints->size[0];
    path->size[1] = 3;
    MATLAB_emxEnsureCapacity_real_T(path, trueCount);
    loop_ub_0 = expansionPoints->size[0] * 3;
    for (c_k = 0; c_k < loop_ub_0; c_k++) {
      path->data[c_k] = expansionPoints->data[c_k];
    }

    trueCount = dir->size[0];
    dir->size[0] = expansionDirs->size[0];
    MATLAB_emxEnsureCapacity_real_T(dir, trueCount);
    loop_ub_0 = expansionDirs->size[0];
    for (c_k = 0; c_k < loop_ub_0; c_k++) {
      dir->data[c_k] = expansionDirs->data[c_k];
    }
  }

  MATLAB0_emxFree_real_T(&expansionDirs);
  MATLAB0_emxFree_real_T(&expansionPoints);
  MATLAB0_emxFree_real_T(&directions);
  MATLAB0_emxFree_real_T(&states);
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static void plannerHybridAStar_closeCell_i(plannerHybridAStar_MATLAB0_T *obj,
  const real_T direction_data[], const int32_T *direction_size, const real_T
  Indice_data[])
{
  static int32_T b_data[10];
  int32_T b_size_idx_0;
  int32_T end_tmp;
  int32_T i;
  int32_T trueCount;
  end_tmp = *direction_size - 1;
  trueCount = 0;
  for (i = 0; i <= end_tmp; i++) {
    if (direction_data[i] == 1.0) {
      trueCount++;
    }
  }

  b_size_idx_0 = trueCount;
  trueCount = 0;
  for (i = 0; i <= end_tmp; i++) {
    if (direction_data[i] == 1.0) {
      b_data[trueCount] = (int32_T)Indice_data[i];
      trueCount++;
    }
  }

  for (i = 0; i < b_size_idx_0; i++) {
    obj->visitedCellsFront[b_data[i] - 1] = true;
  }

  trueCount = 0;
  for (i = 0; i <= end_tmp; i++) {
    if (!(direction_data[i] == 1.0)) {
      trueCount++;
    }
  }

  b_size_idx_0 = trueCount;
  trueCount = 0;
  for (i = 0; i <= end_tmp; i++) {
    if (!(direction_data[i] == 1.0)) {
      b_data[trueCount] = (int32_T)Indice_data[i];
      trueCount++;
    }
  }

  for (i = 0; i < b_size_idx_0; i++) {
    obj->visitedCellsBack[b_data[i] - 1] = true;
  }
}

static void MATLAB0_binary_expand_op_21(real_T in1_data[], int32_T *in1_size,
  const real_T in2_data[], const int8_T in3[2], const
  plannerHybridAStar_MATLAB0_T *in4, const emxArray_real_T_MATLAB0_T *in5, const
  real_T in6_data[], const int32_T *in6_size)
{
  static real_T in2_data_0[10];
  int32_T i;
  int32_T loop_ub;
  int32_T stride_1_0;
  int32_T stride_2_0;
  int32_T stride_3_0;
  i = *in6_size == 1 ? in5->size[0] == 1 ? *in1_size : in5->size[0] : *in6_size;
  loop_ub = i == 1 ? (int32_T)in3[0] : i;
  stride_1_0 = (*in1_size != 1);
  stride_2_0 = (in5->size[0] != 1);
  stride_3_0 = (*in6_size != 1);
  for (i = 0; i < loop_ub; i++) {
    in2_data_0[i] = ((in5->data[i * stride_2_0] + in4->MotionPrimitiveLength) *
                     in1_data[i * stride_1_0] + in6_data[i * stride_3_0]) +
      in2_data[1];
  }

  *in1_size = loop_ub;
  for (i = 0; i < loop_ub; i++) {
    in1_data[i] = in2_data_0[i];
  }
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static void plannerHybridAStar_get2DHeuri_o(plannerHybridAStar_MATLAB0_T *obj,
  c_nav_algs_internal_plannerAS_T *heuristic2DObjInternal, const real_T
  point_data[], const int32_T point_size[2], real_T cost_data[], int32_T
  *cost_size)
{
  static binaryOccupancyMap_MATLAB0_T *b_obj;
  static emxArray_real_T_MATLAB0_T point_data_1;
  static emxArray_real_T_MATLAB0_T *h;
  static real_T b_x[422500];
  static real_T minval[422500];
  static real_T y[422500];
  static real_T point_data_0[20];
  static real_T h_data[18];
  static real_T neighborPoints[18];
  static real_T tmp_data_0[18];
  static real_T indices_data[10];
  static real_T c_data[9];
  static real_T c_data_0[9];
  static real_T d_data[9];
  static real_T h_0[2];
  static real_T minCost;
  static real_T neighborPoints_0;
  static real_T u1;
  static real_T x;
  static int32_T point_size_0[2];
  static int32_T tmp_size[2];
  static int32_T b;
  static int32_T b_k;
  static int32_T d_size;
  static int32_T i;
  static int32_T i_0;
  static int32_T indices_size_idx_0;
  static int32_T loop_ub;
  static int32_T tmp_size_idx_0;
  static int32_T trueCount;
  static int32_T x_tmp;
  static int8_T tmp_data[9];
  static boolean_T tmp[9];
  boolean_T tmp_0;
  b_obj = obj->Map;
  point_size_0[0] = point_size[0];
  point_size_0[1] = 2;
  loop_ub = point_size[0];
  for (i_0 = 0; i_0 < 2; i_0++) {
    for (trueCount = 0; trueCount < loop_ub; trueCount++) {
      point_data_0[trueCount + point_size_0[0] * i_0] = point_data[point_size[0]
        * i_0 + trueCount];
    }
  }

  point_data_1.data = &point_data_0[0];
  point_data_1.size = &point_size_0[0];
  point_data_1.allocatedSize = 20;
  point_data_1.numDimensions = 2;
  point_data_1.canFreeData = false;
  MATLAB0_emxInit_real_T(&h, 2);
  M_MapInterface_world2gridImpl_d(b_obj, &point_data_1, h);
  indices_size_idx_0 = h->size[0];
  loop_ub = h->size[0];
  for (i_0 = 0; i_0 < loop_ub; i_0++) {
    indices_data[i_0] = (h->data[i_0 + h->size[0]] - 1.0) * 650.0 + h->data[i_0];
  }

  *cost_size = indices_size_idx_0;
  for (i_0 = 0; i_0 < indices_size_idx_0; i_0++) {
    cost_data[i_0] = obj->Heuristic2DMat[(int32_T)indices_data[i_0] - 1];
  }

  b = h->size[0];
  for (i = 0; i < b; i++) {
    x_tmp = (int32_T)indices_data[i] - 1;
    x = obj->Heuristic2DMat[x_tmp];
    if (rtIsInf(x)) {
      for (i_0 = 0; i_0 < 2; i_0++) {
        for (trueCount = 0; trueCount < 9; trueCount++) {
          loop_ub = 9 * i_0 + trueCount;
          neighborPoints[loop_ub] = h->data[h->size[0] * i_0 + i] +
            obj->Neighbors[loop_ub];
        }
      }

      trueCount = 0;
      for (i_0 = 0; i_0 < 9; i_0++) {
        minCost = neighborPoints[i_0];
        neighborPoints_0 = neighborPoints[i_0 + 9];
        tmp_0 = ((minCost > 0.0) && (neighborPoints_0 > 0.0) && (minCost <=
                  650.0) && (neighborPoints_0 <= 650.0));
        tmp[i_0] = tmp_0;
        if (tmp_0) {
          trueCount++;
        }
      }

      tmp_size_idx_0 = trueCount;
      trueCount = 0;
      for (i_0 = 0; i_0 < 9; i_0++) {
        if (tmp[i_0]) {
          tmp_data[trueCount] = (int8_T)i_0;
          trueCount++;
        }
      }

      for (i_0 = 0; i_0 < tmp_size_idx_0; i_0++) {
        trueCount = tmp_data[i_0];
        c_data[i_0] = obj->Heuristic2DMat[(int32_T)((neighborPoints[trueCount +
          9] - 1.0) * 650.0 + neighborPoints[trueCount]) - 1];
      }

      for (i_0 = 0; i_0 < 2; i_0++) {
        for (trueCount = 0; trueCount < tmp_size_idx_0; trueCount++) {
          h_data[trueCount + tmp_size_idx_0 * i_0] = h->data[h->size[0] * i_0 +
            i] - neighborPoints[9 * i_0 + tmp_data[trueCount]];
        }
      }

      tmp_size[0] = tmp_size_idx_0;
      tmp_size[1] = 2;
      loop_ub = tmp_size_idx_0 << 1;
      for (trueCount = 0; trueCount < loop_ub; trueCount++) {
        minCost = h_data[trueCount];
        tmp_data_0[trueCount] = minCost * minCost;
      }

      MATLAB0_sum(tmp_data_0, tmp_size, d_data, &d_size);
      for (loop_ub = 0; loop_ub < d_size; loop_ub++) {
        d_data[loop_ub] = sqrt(d_data[loop_ub]);
      }

      if (tmp_size_idx_0 == d_size) {
        for (i_0 = 0; i_0 < tmp_size_idx_0; i_0++) {
          c_data_0[i_0] = c_data[i_0] + d_data[i_0];
        }

        minCost = MATLAB0_minimum(c_data_0, &tmp_size_idx_0);
      } else {
        minCost = MATLAB0_binary_expand_op(c_data, &tmp_size_idx_0, d_data,
          &d_size);
      }

      if (!rtIsInf(minCost)) {
        obj->Heuristic2DMat[x_tmp] = minCost;
      } else {
        h_0[0] = h->data[i];
        h_0[1] = h->data[i + h->size[0]];
        MATLAB_plannerAStarGrid_runPlan(heuristic2DObjInternal, obj->GoalPoint,
          h_0);
        plannerAStarGrid_getGCostMatrix(heuristic2DObjInternal, y);
        for (i_0 = 0; i_0 < 422500; i_0++) {
          b_x[i_0] = obj->Heuristic2DMat[i_0];
        }

        for (b_k = 0; b_k < 422500; b_k++) {
          neighborPoints_0 = b_x[b_k];
          u1 = y[b_k];
          if ((neighborPoints_0 <= u1) || rtIsNaN(u1)) {
            minval[b_k] = neighborPoints_0;
          } else {
            minval[b_k] = u1;
          }
        }

        for (i_0 = 0; i_0 < 422500; i_0++) {
          obj->Heuristic2DMat[i_0] = minval[i_0];
        }
      }
    }

    for (i_0 = 0; i_0 < indices_size_idx_0; i_0++) {
      cost_data[i_0] = obj->Heuristic2DMat[(int32_T)indices_data[i_0] - 1];
    }
  }

  MATLAB0_emxFree_real_T(&h);
}

static void MATLAB0_plus(real_T in1_data[], int32_T *in1_size, const real_T
  in2_data[], const int32_T *in2_size, const real_T in3_data[], const int32_T
  *in3_size)
{
  int32_T i;
  int32_T loop_ub;
  int32_T stride_0_0;
  int32_T stride_1_0;
  loop_ub = *in3_size == 1 ? *in2_size : *in3_size;
  *in1_size = loop_ub;
  stride_0_0 = (*in2_size != 1);
  stride_1_0 = (*in3_size != 1);
  for (i = 0; i < loop_ub; i++) {
    in1_data[i] = in2_data[i * stride_0_0] + in3_data[i * stride_1_0];
  }
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static void plannerHybridAStar_calculateCos(plannerHybridAStar_MATLAB0_T *obj,
  c_nav_algs_internal_plannerAS_T *heuristic2DObjInternal, const real_T
  newNodeData_data[], const int32_T newNodeData_size[2], const real_T
  currentNode_data[], const real_T curvature_data[], const int32_T
  *curvature_size, const real_T direction_data[], const int32_T *direction_size,
  real_T fScore_data[], int32_T *fScore_size, real_T gScore_data[], int32_T
  *gScore_size, real_T hScore_data[], int32_T *hScore_size)
{
  static emxArray_real_T_MATLAB0_T *y;
  static real_T motionLengths_data[50];
  static real_T motionTypes_data[50];
  static real_T x_data[20];
  static real_T cost_data[10];
  static real_T dirSwitchingCosts_data[10];
  static real_T pathLength_data[10];
  static real_T hScore;
  static real_T pathLength;
  static int32_T b_i;
  static int32_T c_k;
  static int32_T dirSwitchingCosts_size;
  static int32_T end_tmp;
  static int32_T f;
  static int32_T loop_ub;
  static int32_T maxNumPoses;
  static int32_T nx;
  static int32_T x_size_idx_0_tmp;
  static int32_T x_size_idx_1;
  static int32_T xi;
  static int32_T xpageoffset;
  static boolean_T allPathTypes[44];
  int8_T outsize[2];
  int8_T sizes_idx_0;
  boolean_T p;
  outsize[0] = (int8_T)*curvature_size;
  dirSwitchingCosts_size = *curvature_size;
  if (*curvature_size - 1 >= 0) {
    memset(&dirSwitchingCosts_data[0], 0, (uint32_T)*curvature_size * sizeof
           (real_T));
  }

  pathLength = currentNode_data[6];
  if (pathLength != 0.0) {
    end_tmp = *direction_size - 1;
    for (loop_ub = 0; loop_ub <= end_tmp; loop_ub++) {
      if (direction_data[loop_ub] != pathLength) {
        dirSwitchingCosts_data[loop_ub] = obj->DirectionSwitchingCost;
      }
    }
  }

  *gScore_size = *curvature_size;
  loop_ub = *curvature_size;
  for (end_tmp = 0; end_tmp < loop_ub; end_tmp++) {
    gScore_data[end_tmp] = obj->ForwardCost;
  }

  end_tmp = *direction_size - 1;
  for (loop_ub = 0; loop_ub <= end_tmp; loop_ub++) {
    if (direction_data[loop_ub] == -1.0) {
      gScore_data[loop_ub] = obj->ReverseCost;
    }
  }

  nx = *curvature_size;
  MATLAB0_emxInit_real_T(&y, 1);
  loop_ub = y->size[0];
  y->size[0] = *curvature_size;
  MATLAB_emxEnsureCapacity_real_T(y, loop_ub);
  for (loop_ub = 0; loop_ub < nx; loop_ub++) {
    y->data[loop_ub] = fabs(curvature_data[loop_ub]);
  }

  end_tmp = *curvature_size == 1 ? y->size[0] : *curvature_size;
  if ((*curvature_size == y->size[0]) && (end_tmp == *curvature_size) &&
      ((end_tmp == 1 ? *curvature_size : end_tmp) == *curvature_size)) {
    loop_ub = *curvature_size;
    *gScore_size = *curvature_size;
    for (end_tmp = 0; end_tmp < loop_ub; end_tmp++) {
      gScore_data[end_tmp] = ((obj->MotionPrimitiveLength + y->data[end_tmp]) *
        gScore_data[end_tmp] + dirSwitchingCosts_data[end_tmp]) +
        currentNode_data[1];
    }
  } else {
    MATLAB0_binary_expand_op_21(gScore_data, gScore_size, currentNode_data,
      outsize, obj, y, dirSwitchingCosts_data, curvature_size);
  }

  MATLAB0_emxFree_real_T(&y);
  plannerHybridAStar_get2DHeuri_o(obj, heuristic2DObjInternal, newNodeData_data,
    newNodeData_size, dirSwitchingCosts_data, &dirSwitchingCosts_size);
  if (newNodeData_size[0] >= 1) {
    maxNumPoses = newNodeData_size[0];
  } else {
    maxNumPoses = 1;
  }

  for (end_tmp = 0; end_tmp < 44; end_tmp++) {
    allPathTypes[end_tmp] = true;
  }

  autonomousReedsSheppSegmentsCodegen_real64(&newNodeData_data[0], (uint32_T)
    newNodeData_size[0], &obj->GoalPose[0], 1U, obj->MinTurningRadius,
    obj->ForwardCost, obj->ReverseCost, &allPathTypes[0], 0U, 1U, true, 3U,
    &cost_data[0], &motionLengths_data[0], &motionTypes_data[0]);
  f = 5 * maxNumPoses;
  for (loop_ub = 0; loop_ub < f; loop_ub++) {
    motionTypes_data[loop_ub] = fabs(motionLengths_data[loop_ub]);
  }

  for (xi = 0; xi < maxNumPoses; xi++) {
    xpageoffset = xi * 5;
    pathLength = motionTypes_data[xpageoffset];
    for (c_k = 0; c_k < 4; c_k++) {
      pathLength += motionTypes_data[(xpageoffset + c_k) + 1];
    }

    pathLength_data[xi] = pathLength;
  }

  outsize[0] = 1;
  if (maxNumPoses != 1) {
    outsize[0] = (int8_T)maxNumPoses;
  }

  if (dirSwitchingCosts_size != 0) {
    sizes_idx_0 = (int8_T)dirSwitchingCosts_size;
  } else {
    sizes_idx_0 = outsize[0];
  }

  x_size_idx_0_tmp = sizes_idx_0;
  end_tmp = (dirSwitchingCosts_size != 0);
  x_size_idx_1 = end_tmp + 1;
  end_tmp *= sizes_idx_0;
  for (loop_ub = 0; loop_ub < end_tmp; loop_ub++) {
    x_data[loop_ub] = dirSwitchingCosts_data[loop_ub];
  }

  for (loop_ub = 0; loop_ub < x_size_idx_0_tmp; loop_ub++) {
    x_data[loop_ub + end_tmp] = pathLength_data[loop_ub];
  }

  *hScore_size = sizes_idx_0;
  for (loop_ub = 0; loop_ub < x_size_idx_0_tmp; loop_ub++) {
    hScore_data[loop_ub] = x_data[loop_ub];
  }

  for (end_tmp = 2; end_tmp <= x_size_idx_1; end_tmp++) {
    for (b_i = 0; b_i < x_size_idx_0_tmp; b_i++) {
      pathLength = x_data[b_i + sizes_idx_0];
      if (rtIsNaN(pathLength)) {
        p = false;
      } else {
        hScore = hScore_data[b_i];
        p = (rtIsNaN(hScore) || (hScore < pathLength));
      }

      if (p) {
        hScore_data[b_i] = pathLength;
      }
    }
  }

  if (*gScore_size == sizes_idx_0) {
    *fScore_size = *gScore_size;
    loop_ub = *gScore_size;
    for (end_tmp = 0; end_tmp < loop_ub; end_tmp++) {
      fScore_data[end_tmp] = gScore_data[end_tmp] + hScore_data[end_tmp];
    }
  } else {
    MATLAB0_plus(fScore_data, fScore_size, gScore_data, gScore_size, hScore_data,
                 hScore_size);
  }
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static void MATLAB0_eml_find(const boolean_T x[10], int32_T i_data[], int32_T
  *i_size)
{
  int32_T idx;
  int32_T ii;
  boolean_T exitg1;
  idx = 0;
  ii = 0;
  exitg1 = false;
  while ((!exitg1) && (ii < 10)) {
    if (x[ii]) {
      idx++;
      i_data[idx - 1] = ii + 1;
      if (idx >= 10) {
        exitg1 = true;
      } else {
        ii++;
      }
    } else {
      ii++;
    }
  }

  if (idx < 1) {
    *i_size = 0;
  } else {
    *i_size = idx;
  }
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static int32_T MATLAB0_intnnz_l(const boolean_T s[8])
{
  int32_T k;
  int32_T n;
  n = 0;
  for (k = 0; k < 8; k++) {
    if (s[k]) {
      n++;
    }
  }

  return n;
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static int32_T MATLAB0_intnnz_li(const boolean_T s[2])
{
  int32_T n;
  n = 0;
  if (s[0]) {
    n = 1;
  }

  if (s[1]) {
    n++;
  }

  return n;
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static void MATLAB0_repmat_kk(const real_T a[3], real_T varargin_1,
  emxArray_real_T_MATLAB0_T *b)
{
  int32_T ibmat;
  int32_T itilerow;
  int32_T jcol;
  int32_T ntilerows;
  jcol = b->size[0] * b->size[1];
  b->size[0] = (int32_T)varargin_1;
  b->size[1] = 3;
  MATLAB_emxEnsureCapacity_real_T(b, jcol);
  ntilerows = (int32_T)varargin_1 - 1;
  for (jcol = 0; jcol < 3; jcol++) {
    ibmat = jcol * (int32_T)varargin_1;
    for (itilerow = 0; itilerow <= ntilerows; itilerow++) {
      b->data[ibmat + itilerow] = a[jcol];
    }
  }
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static navPath_MATLAB0_T *MATLAB0_plannerHybridAStar_plan
  (plannerHybridAStar_MATLAB0_T *obj, const real_T start[3], const real_T goal[3],
   plannerAStarGrid_MATLAB0_T *iobj_0, navPath_MATLAB0_T *iobj_1)
{
  static b_cell_wrap_19_MATLAB0_T parsedResults_idx_0;
  static binaryOccupancyMap_MATLAB0_T *c_obj;
  static binaryOccupancyMap_MATLAB0_T *e_obj;
  static binaryOccupancyMap_MATLAB0_T *f_obj;
  static binaryOccupancyMap_MATLAB0_T *g_obj;
  static binaryOccupancyMap_MATLAB0_T *h_obj;
  static c_nav_algs_internal_plannerAS_T heuristic2DObjInternal;
  static c_robotics_core_internal_Name_T showParser;
  static emxArray_real_T_MATLAB0_T *directions;
  static emxArray_real_T_MATLAB0_T *pathData;
  static emxArray_real_T_MATLAB0_T *pathPoses1;
  static emxArray_real_T_MATLAB0_T *pathStates;
  static emxArray_real_T_MATLAB0_T *varargin_1;
  static nav_algs_internal_NodeMap_MAT_T nodeMap;
  static nav_algs_internal_PriorityQue_T openSet;
  static reedsSheppConnection_MATLAB0_T rsPathObj;
  static validatorOccupancyMap_MATLAB0_T *b_obj;
  static validatorOccupancyMap_MATLAB0_T *d_obj;
  static real_T e_result_data[110];
  static real_T newNodesPoses[30];
  static real_T newNodesPoses_data[30];
  static real_T ICRsData[24];
  static real_T ICRsData_data[24];
  static real_T circularNewNodesPoses[24];
  static real_T newNodesPoses_data_0[24];
  static real_T b_hScore_data[10];
  static real_T b_hScore_data_0[10];
  static real_T curvature_data[10];
  static real_T fScore_data[10];
  static real_T gScore_data[10];
  static real_T newNodesPosesGridIndices_data[10];
  static real_T p_data[10];
  static real_T tmp_data_5[10];
  static real_T tmp_data_4[8];
  static real_T varargin_4_data[8];
  static real_T currentNode_data[7];
  static real_T nodeData[7];
  static real_T straightNewNodePose[6];
  static real_T straightNewNodePose_data[6];
  static real_T r[5];
  static real_T x_data[5];
  static real_T currentNodePose[3];
  static real_T currentNodeGrid[2];
  static real_T ylims[2];
  static real_T b_x;
  static real_T b_y;
  static real_T currentNodeGridIndices;
  static real_T currentNodeId;
  static real_T flag;
  static real_T g_obj_0;
  static real_T hScore;
  static real_T h_obj_0;
  static real_T linesDataRow;
  static real_T numIterations;
  static real_T primitivesDataRow;
  static real_T stepSize;
  static real_T y;
  static int32_T s_data[10];
  static int32_T b_k;
  static int32_T c;
  static int32_T d;
  static int32_T e;
  static int32_T i;
  static int32_T i_0;
  static int32_T loop_ub;
  static int32_T newNodesPoses_tmp;
  static int32_T numValidCirc;
  static int32_T numValidLine;
  static int32_T numValidPrimitives;
  static int32_T sizes_idx_0;
  static int32_T trueCount;
  static char_T searchMode_data[10];
  static char_T b_value[6];
  static int8_T p_data_1[10];
  static int8_T tmp_data_0[10];
  static int8_T tmp_data_1[10];
  static int8_T tmp_data_3[10];
  static int8_T varargin_1_tmp_data[10];
  static int8_T p_data_0[8];
  static int8_T tmp_data[8];
  static boolean_T occupied[422500];
  static boolean_T validPrimitives[10];
  static boolean_T validCirc[8];
  navPath_MATLAB0_T *pathObj;
  int8_T tmp_data_2[2];
  int8_T tmp_data_6[2];
  int8_T b_input_sizes_idx_1;
  int8_T c_input_sizes_idx_1;
  int8_T d_input_sizes_idx_1;
  int8_T e_input_sizes_idx_1;
  int8_T f_input_sizes_idx_1;
  int8_T input_sizes_idx_1;
  boolean_T start_0[3];
  boolean_T validLine[2];
  boolean_T result;
  static const real_T o[8] = { 1.0, 1.0, 1.0, 1.0, -1.0, -1.0, -1.0, -1.0 };

  static const int8_T p[10] = { 1, 1, 1, 1, -1, -1, -1, -1, 1, -1 };

  static real_T obj_0[3];
  static real_T obj_1[2];
  static int32_T currentNode_size[2];
  static int32_T newNodesPosesGridIndices_size[2];
  static int32_T newNodesPoses_size[2];
  static int32_T searchMode_size[2];
  static int32_T tmp_size[2];
  static int32_T b_hScore_size;
  static int32_T curvature_size;
  static int32_T curvature_size_idx_1;
  static int32_T fScore_size;
  static int32_T gScore_size;
  static int32_T i_1;
  static int32_T i_2;
  static int32_T s_size;
  static int32_T sizes_idx_1_tmp;
  static int32_T sizes_idx_1_tmp_0;
  static int32_T tmp_size_idx_0;
  static int32_T x_data_tmp;
  int32_T exitg1;
  boolean_T guard1;
  emxInitStruct_reedsSheppConnect(&rsPathObj);
  openSet.matlabCodegenIsDeleted = true;
  nodeMap.matlabCodegenIsDeleted = true;
  plannerHybridAStar_validateStar(obj, start, goal);
  for (i_0 = 0; i_0 < 23237500; i_0++) {
    obj->PrimitivesData[i_0] = (rtNaN);
  }

  for (i_0 = 0; i_0 < 14787500; i_0++) {
    obj->LinesData[i_0] = (rtNaN);
  }

  NameValueParser_NameValueParser(&showParser);
  parsedResults_idx_0 = showParser.Defaults;
  showParser.ParsedResults = parsedResults_idx_0;
  for (i_0 = 0; i_0 < 6; i_0++) {
    b_value[i_0] = showParser.ParsedResults.f1[i_0];
  }

  MATLAB0_validatestring(b_value, searchMode_data, searchMode_size);
  obj->Map = obj->StateValidator->Map;
  obj->Dimensions[0] = 650.0;
  obj->Dimensions[1] = 650.0;
  obj->StateValidator->SkipStateValidation = true;
  b_obj = obj->StateValidator;
  f_obj = b_obj->Map;
  MATLAB_MapLayer_getValueAllImpl(f_obj, occupied);
  for (i_0 = 0; i_0 < 422500; i_0++) {
    b_obj->ValidMatrix[i_0] = occupied[i_0];
  }

  g_obj = b_obj->Map;
  g_obj_0 = g_obj->SharedProperties.LocalOriginInWorld[0] +
    g_obj->SharedProperties.GridOriginInLocal[0];
  h_obj = b_obj->Map;
  h_obj_0 = h_obj->SharedProperties.LocalOriginInWorld[1] +
    h_obj->SharedProperties.GridOriginInLocal[1];
  b_obj->MapBounds[0] = g_obj_0;
  b_obj->MapBounds[2] = g_obj_0 + 650.0;
  b_obj->MapBounds[1] = h_obj_0;
  b_obj->MapBounds[3] = h_obj_0 + 650.0;
  obj->PathFound = false;
  MATLAB0_emxInit_real_T(&pathStates, 2);
  pathStates->size[0] = 0;
  pathStates->size[1] = 3;
  start_0[0] = (start[0] == goal[0]);
  start_0[1] = (start[1] == goal[1]);
  start_0[2] = (start[2] == goal[2]);
  if (MATLAB0_ifWhileCond(start_0)) {
    i_0 = pathStates->size[0] * pathStates->size[1];
    pathStates->size[0] = 1;
    pathStates->size[1] = 3;
    MATLAB_emxEnsureCapacity_real_T(pathStates, i_0);
    pathStates->data[0] = start[0];
    pathStates->data[1] = start[1];
    pathStates->data[2] = start[2];
  } else {
    primitivesDataRow = 1.0;
    linesDataRow = 1.0;
    for (i_0 = 0; i_0 < 422500; i_0++) {
      obj->visitedCellsFront[i_0] = false;
    }

    for (i_0 = 0; i_0 < 422500; i_0++) {
      obj->visitedCellsBack[i_0] = false;
    }

    c_obj = obj->Map;
    currentNodeGrid[0] = obj->GoalPose[0];
    currentNodeGrid[1] = obj->GoalPose[1];
    MAT_MapInterface_world2gridImpl(c_obj, currentNodeGrid, ylims);
    obj->GoalPoint[0] = ylims[0];
    obj->GoalPoint[1] = ylims[1];
    obj->Heuristic2DObj = plannerAStarGrid_plannerAStarGr(iobj_0, obj->Map);
    plannerAStarGrid_initializeInte(obj->Heuristic2DObj, &heuristic2DObjInternal);
    for (i_0 = 0; i_0 < 422500; i_0++) {
      obj->Heuristic2DMat[i_0] = (rtInf);
    }

    openSet.PQInternal = priorityqueuecodegen_constructPQ(7.0, 0.0);
    openSet.matlabCodegenIsDeleted = false;
    nodeMap.NodeMapInternal = priorityqueuecodegen_constructNodeMap(3.0);
    nodeMap.matlabCodegenIsDeleted = false;
    for (i_1 = 0; i_1 < 2; i_1++) {
      obj_1[i_1] = obj->StartPose[i_1];
    }

    currentNodeGrid[0] = plannerHybridAStar_get2DHeurist(obj,
      &heuristic2DObjInternal, obj_1);
    currentNodeGrid[1] = plannerHybridAStar_get3DHeurist(obj, obj->StartPose,
      obj->GoalPose);
    hScore = MATLAB0_maximum_b(currentNodeGrid);
    if (!(hScore == (rtInf))) {
      nodeData[0] = hScore;
      nodeData[1] = 0.0;
      nodeData[2] = hScore;
      nodeData[3] = obj->StartPose[0];
      nodeData[4] = obj->StartPose[1];
      nodeData[5] = obj->StartPose[2];
      nodeData[6] = 0.0;
      priorityqueuecodegen_push(openSet.PQInternal, &nodeData[0]);
      currentNodePose[0] = obj->StartPose[0];
      currentNodePose[1] = obj->StartPose[1];
      currentNodePose[2] = obj->StartPose[2];
      priorityqueuecodegen_nodemap_getNumNodes(nodeMap.NodeMapInternal);
      priorityqueuecodegen_nodemap_insertNode(nodeMap.NodeMapInternal,
        &currentNodePose[0], 0.0);
      MATLAB0_linspace(-1.0 / obj->MinTurningRadius, 1.0 / obj->MinTurningRadius,
                       r);
      for (i_0 = 0; i_0 < 5; i_0++) {
        x_data[i_0] = r[i_0];
      }

      for (b_k = 0; b_k < 2; b_k++) {
        x_data[b_k + 2] = x_data[b_k + 3];
      }

      x_data_tmp = 3;
      x_data[3] = x_data[3];
      MATLAB0_repmat(x_data, tmp_data_4, tmp_size);
      curvature_size_idx_1 = 10;
      for (i_0 = 0; i_0 < 8; i_0++) {
        curvature_data[i_0] = tmp_data_4[i_0];
      }

      curvature_data[8] = 0.0;
      curvature_data[9] = 0.0;
      b_x = obj->StateValidator->ValidationDistance;
      if (rtIsInf(b_x)) {
        stepSize = 1.0;
        d_obj = obj->StateValidator;
        d_obj->ValidationDistance = 1.0;
      } else {
        stepSize = obj->StateValidator->ValidationDistance;
      }

      y = obj->MotionPrimitiveLength / stepSize;
      obj->NumPointsMotionPrimitive = floor(y) + 2.0;
      numIterations = 0.0;
      reedsSheppConnection_reedsShepp(&rsPathObj, obj->MinTurningRadius,
        obj->ForwardCost, obj->ReverseCost);
      MATLAB0_emxInit_real_T(&pathPoses1, 2);
      MATLAB0_emxInit_real_T(&pathData, 2);
      MATLAB0_emxInit_real_T(&directions, 1);
      MATLAB0_emxInit_real_T(&varargin_1, 2);
      do {
        exitg1 = 0;
        flag = priorityqueuecodegen_isEmpty(openSet.PQInternal);
        result = ((flag != 0.0) || obj->PathFound);
        if (!result) {
          MATLAB0_PriorityQueue_top(&openSet, currentNode_data, currentNode_size,
            &currentNodeId);
          priorityqueuecodegen_pop(openSet.PQInternal);
          numIterations++;
          if (MATLAB0_strcmp(searchMode_data, searchMode_size)) {
            e_obj = obj->Map;
            MAT_MapInterface_world2gridImpl(e_obj, &currentNode_data[3],
              currentNodeGrid);
            currentNodeGridIndices = (currentNodeGrid[1] - 1.0) *
              obj->Dimensions[0] + currentNodeGrid[0];
            MA_plannerHybridAStar_closeCell(obj, currentNode_data[6],
              currentNodeGridIndices);
          }

          b_y = obj->AnalyticExpansionInterval;
          guard1 = false;
          if (MATLAB0_rt_remd_snf(numIterations, b_y) == 0.0) {
            for (i_2 = 0; i_2 < 3; i_2++) {
              obj_0[i_2] = obj->GoalPose[i_2];
            }

            result = plannerHybridAStar_checkAnalyti(obj, &currentNode_data[3],
              obj_0, stepSize, &rsPathObj);
            if (result) {
              if (primitivesDataRow > 2.1125E+6) {
                d = -1;
                c = -1;
              } else {
                d = (int32_T)primitivesDataRow - 2;
                c = 2112499;
              }

              i = c - d;
              for (i_0 = 0; i_0 < 11; i_0++) {
                for (newNodesPoses_tmp = 0; newNodesPoses_tmp < i;
                     newNodesPoses_tmp++) {
                  obj->PrimitivesData[((d + newNodesPoses_tmp) + 2112500 * i_0)
                    + 1] = (rtNaN);
                }
              }

              if (linesDataRow > 2.1125E+6) {
                i = -1;
                e = -1;
              } else {
                i = (int32_T)linesDataRow - 2;
                e = 2112499;
              }

              loop_ub = e - i;
              for (i_0 = 0; i_0 < 7; i_0++) {
                for (newNodesPoses_tmp = 0; newNodesPoses_tmp < loop_ub;
                     newNodesPoses_tmp++) {
                  obj->LinesData[((i + newNodesPoses_tmp) + 2112500 * i_0) + 1] =
                    (rtNaN);
                }
              }

              MATLAB0_NodeMap_traceBack(&nodeMap, currentNodeId, pathPoses1);
              MATLAB0_flipud(pathPoses1);
              plannerHybridAStar_getFinalPath(obj, pathPoses1, pathData);
              plannerHybridAStar_getInterpola(obj, pathData, varargin_1,
                directions);
              i_0 = pathStates->size[0] * pathStates->size[1];
              pathStates->size[0] = varargin_1->size[0];
              pathStates->size[1] = 3;
              MATLAB_emxEnsureCapacity_real_T(pathStates, i_0);
              loop_ub = varargin_1->size[0] * 3;
              for (i_0 = 0; i_0 < loop_ub; i_0++) {
                pathStates->data[i_0] = varargin_1->data[i_0];
              }

              obj->PathFound = true;
            } else {
              guard1 = true;
            }
          } else {
            guard1 = true;
          }

          if (guard1) {
            plannerHybridAStar_getCircularP(obj->MotionPrimitiveLength,
              &curvature_data[0], &currentNode_data[3], o, circularNewNodesPoses,
              ICRsData);
            obj_1[0] = 1.0;
            obj_1[1] = -1.0;
            plannerHybridAStar_getStraightP(obj->MotionPrimitiveLength,
              &currentNode_data[3], obj_1, straightNewNodePose);
            for (i_0 = 0; i_0 < 3; i_0++) {
              for (newNodesPoses_tmp = 0; newNodesPoses_tmp < 8;
                   newNodesPoses_tmp++) {
                newNodesPoses[newNodesPoses_tmp + 10 * i_0] =
                  circularNewNodesPoses[(i_0 << 3) + newNodesPoses_tmp];
              }

              newNodesPoses_tmp = i_0 << 1;
              newNodesPoses[10 * i_0 + 8] =
                straightNewNodePose[newNodesPoses_tmp];
              newNodesPoses[10 * i_0 + 9] =
                straightNewNodePose[newNodesPoses_tmp + 1];
            }

            for (i_0 = 0; i_0 < 10; i_0++) {
              tmp_data_5[i_0] = 1.0 / curvature_data[i_0];
            }

            plannerHybridAStar_isPrimitiveV(obj, &currentNode_data[3],
              newNodesPoses, ICRsData, tmp_data_5, obj->MotionPrimitiveLength,
              stepSize, validPrimitives, newNodesPosesGridIndices_data,
              newNodesPosesGridIndices_size);
            numValidPrimitives = MATLAB0_intnnz(validPrimitives);
            if (numValidPrimitives != 0) {
              newNodesPoses_tmp = 0;
              for (i = 0; i < 10; i++) {
                if (validPrimitives[i]) {
                  newNodesPoses_tmp++;
                }
              }

              tmp_size_idx_0 = newNodesPoses_tmp;
              newNodesPoses_tmp = 0;
              for (i = 0; i < 10; i++) {
                if (validPrimitives[i]) {
                  tmp_data_0[newNodesPoses_tmp] = (int8_T)i;
                  newNodesPoses_tmp++;
                }
              }

              newNodesPoses_size[0] = tmp_size_idx_0;
              newNodesPoses_size[1] = 3;
              for (i_0 = 0; i_0 < 3; i_0++) {
                for (newNodesPoses_tmp = 0; newNodesPoses_tmp < tmp_size_idx_0;
                     newNodesPoses_tmp++) {
                  newNodesPoses_data[newNodesPoses_tmp + tmp_size_idx_0 * i_0] =
                    newNodesPoses[10 * i_0 + tmp_data_0[newNodesPoses_tmp]];
                }
              }

              curvature_size = tmp_size_idx_0;
              for (i_0 = 0; i_0 < tmp_size_idx_0; i_0++) {
                f_input_sizes_idx_1 = tmp_data_0[i_0];
                tmp_data_5[i_0] = curvature_data[f_input_sizes_idx_1];
                p_data[i_0] = p[f_input_sizes_idx_1];
              }

              plannerHybridAStar_calculateCos(obj, &heuristic2DObjInternal,
                newNodesPoses_data, newNodesPoses_size, currentNode_data,
                tmp_data_5, &tmp_size_idx_0, p_data, &tmp_size_idx_0,
                fScore_data, &fScore_size, gScore_data, &gScore_size,
                b_hScore_data, &b_hScore_size);
              MATLAB0_eml_find(validPrimitives, s_data, &s_size);
              for (i_0 = 0; i_0 < fScore_size; i_0++) {
                validPrimitives[s_data[i_0] - 1] = (fScore_data[i_0] != (rtInf));
              }

              numValidPrimitives = s_size;
              for (i = 0; i < 8; i++) {
                validCirc[i] = validPrimitives[i];
              }

              numValidCirc = MATLAB0_intnnz_l(&validPrimitives[0]);
              validLine[0] = validPrimitives[8];
              validLine[1] = validPrimitives[9];
              numValidLine = MATLAB0_intnnz_li(&validPrimitives[8]);
              g_obj_0 = primitivesDataRow + (real_T)numValidCirc;
              if (primitivesDataRow > g_obj_0 - 1.0) {
                i = 0;
              } else {
                i = (int32_T)primitivesDataRow - 1;
              }

              newNodesPoses_tmp = 0;
              for (i_0 = 0; i_0 < 8; i_0++) {
                if (validCirc[i_0]) {
                  newNodesPoses_tmp++;
                }
              }

              tmp_size_idx_0 = newNodesPoses_tmp;
              newNodesPoses_tmp = 0;
              for (i_0 = 0; i_0 < 8; i_0++) {
                if (validCirc[i_0]) {
                  tmp_data[newNodesPoses_tmp] = (int8_T)i_0;
                  newNodesPoses_tmp++;
                }
              }

              for (i_0 = 0; i_0 < tmp_size_idx_0; i_0++) {
                varargin_4_data[i_0] = 1.0 / curvature_data[tmp_data[i_0]];
              }

              MATLAB0_repmat_kk(&currentNode_data[3], (real_T)numValidCirc,
                                varargin_1);
              if (varargin_1->size[0] != 0) {
                sizes_idx_0 = varargin_1->size[0];
              } else {
                newNodesPoses_tmp = 0;
                for (i_0 = 0; i_0 < 8; i_0++) {
                  if (validCirc[i_0]) {
                    newNodesPoses_tmp++;
                  }
                }

                if (newNodesPoses_tmp != 0) {
                  sizes_idx_0 = 0;
                  for (i_0 = 0; i_0 < 8; i_0++) {
                    if (validCirc[i_0]) {
                      sizes_idx_0++;
                    }
                  }
                } else {
                  newNodesPoses_tmp = 0;
                  for (i_0 = 0; i_0 < 8; i_0++) {
                    if (validCirc[i_0]) {
                      newNodesPoses_tmp++;
                    }
                  }

                  if (newNodesPoses_tmp != 0) {
                    sizes_idx_0 = 0;
                    for (i_0 = 0; i_0 < 8; i_0++) {
                      if (validCirc[i_0]) {
                        sizes_idx_0++;
                      }
                    }
                  } else if (tmp_size_idx_0 != 0) {
                    sizes_idx_0 = tmp_size_idx_0;
                  } else {
                    newNodesPoses_tmp = 0;
                    for (i_0 = 0; i_0 < 8; i_0++) {
                      if (validCirc[i_0]) {
                        newNodesPoses_tmp++;
                      }
                    }

                    if (newNodesPoses_tmp != 0) {
                      sizes_idx_0 = 0;
                      for (i_0 = 0; i_0 < 8; i_0++) {
                        if (validCirc[i_0]) {
                          sizes_idx_0++;
                        }
                      }
                    } else {
                      trueCount = 0;
                      newNodesPoses_tmp = 0;
                      for (i_0 = 0; i_0 < 8; i_0++) {
                        if (validCirc[i_0]) {
                          newNodesPoses_tmp++;
                        }
                      }

                      if (newNodesPoses_tmp > 0) {
                        trueCount = 0;
                        for (i_0 = 0; i_0 < 8; i_0++) {
                          if (validCirc[i_0]) {
                            trueCount++;
                          }
                        }
                      }

                      newNodesPoses_tmp = 0;
                      for (i_0 = 0; i_0 < 8; i_0++) {
                        if (validCirc[i_0]) {
                          newNodesPoses_tmp++;
                        }
                      }

                      if (newNodesPoses_tmp > trueCount) {
                        trueCount = 0;
                        for (i_0 = 0; i_0 < 8; i_0++) {
                          if (validCirc[i_0]) {
                            trueCount++;
                          }
                        }
                      }

                      sizes_idx_0 = 0;
                      for (i_0 = 0; i_0 < 8; i_0++) {
                        if (validCirc[i_0]) {
                          sizes_idx_0++;
                        }
                      }

                      if (sizes_idx_0 < trueCount) {
                        sizes_idx_0 = trueCount;
                      }
                    }
                  }
                }
              }

              result = (sizes_idx_0 == 0);
              if (result || (varargin_1->size[0] != 0)) {
                input_sizes_idx_1 = 3;
              } else {
                input_sizes_idx_1 = 0;
              }

              if (result) {
                b_input_sizes_idx_1 = 3;
              } else {
                newNodesPoses_tmp = 0;
                for (i_0 = 0; i_0 < 8; i_0++) {
                  if (validCirc[i_0]) {
                    newNodesPoses_tmp++;
                  }
                }

                if (newNodesPoses_tmp != 0) {
                  b_input_sizes_idx_1 = 3;
                } else {
                  b_input_sizes_idx_1 = 0;
                }
              }

              if (result) {
                c_input_sizes_idx_1 = 3;
              } else {
                newNodesPoses_tmp = 0;
                for (i_0 = 0; i_0 < 8; i_0++) {
                  if (validCirc[i_0]) {
                    newNodesPoses_tmp++;
                  }
                }

                if (newNodesPoses_tmp != 0) {
                  c_input_sizes_idx_1 = 3;
                } else {
                  c_input_sizes_idx_1 = 0;
                }
              }

              if (result || (tmp_size_idx_0 != 0)) {
                d_input_sizes_idx_1 = 1;
              } else {
                d_input_sizes_idx_1 = 0;
              }

              if (result) {
                e_input_sizes_idx_1 = 1;
              } else {
                newNodesPoses_tmp = 0;
                for (i_0 = 0; i_0 < 8; i_0++) {
                  if (validCirc[i_0]) {
                    newNodesPoses_tmp++;
                  }
                }

                if (newNodesPoses_tmp != 0) {
                  e_input_sizes_idx_1 = 1;
                } else {
                  e_input_sizes_idx_1 = 0;
                }
              }

              sizes_idx_1_tmp = input_sizes_idx_1;
              trueCount = b_input_sizes_idx_1;
              for (i_0 = 0; i_0 < 3; i_0++) {
                for (newNodesPoses_tmp = 0; newNodesPoses_tmp < tmp_size_idx_0;
                     newNodesPoses_tmp++) {
                  f_input_sizes_idx_1 = tmp_data[newNodesPoses_tmp];
                  newNodesPoses_data_0[newNodesPoses_tmp + tmp_size_idx_0 * i_0]
                    = newNodesPoses[10 * i_0 + f_input_sizes_idx_1];
                  ICRsData_data[newNodesPoses_tmp + tmp_size_idx_0 * i_0] =
                    ICRsData[(i_0 << 3) + f_input_sizes_idx_1];
                }
              }

              loop_ub = c_input_sizes_idx_1;
              sizes_idx_1_tmp_0 = d_input_sizes_idx_1;
              for (i_0 = 0; i_0 < tmp_size_idx_0; i_0++) {
                p_data_0[i_0] = p[tmp_data[i_0]];
              }

              tmp_size_idx_0 = e_input_sizes_idx_1;
              for (i_0 = 0; i_0 < sizes_idx_1_tmp; i_0++) {
                for (newNodesPoses_tmp = 0; newNodesPoses_tmp < sizes_idx_0;
                     newNodesPoses_tmp++) {
                  obj->PrimitivesData[(i + newNodesPoses_tmp) + 2112500 * i_0] =
                    varargin_1->data[sizes_idx_0 * i_0 + newNodesPoses_tmp];
                }
              }

              for (i_0 = 0; i_0 < trueCount; i_0++) {
                for (newNodesPoses_tmp = 0; newNodesPoses_tmp < sizes_idx_0;
                     newNodesPoses_tmp++) {
                  obj->PrimitivesData[(i + newNodesPoses_tmp) + 2112500 * (i_0 +
                    input_sizes_idx_1)] = newNodesPoses_data_0[sizes_idx_0 * i_0
                    + newNodesPoses_tmp];
                }
              }

              for (i_0 = 0; i_0 < loop_ub; i_0++) {
                for (newNodesPoses_tmp = 0; newNodesPoses_tmp < sizes_idx_0;
                     newNodesPoses_tmp++) {
                  obj->PrimitivesData[(i + newNodesPoses_tmp) + 2112500 * ((i_0
                    + input_sizes_idx_1) + b_input_sizes_idx_1)] =
                    ICRsData_data[sizes_idx_0 * i_0 + newNodesPoses_tmp];
                }
              }

              for (i_0 = 0; i_0 < sizes_idx_1_tmp_0; i_0++) {
                for (newNodesPoses_tmp = 0; newNodesPoses_tmp < sizes_idx_0;
                     newNodesPoses_tmp++) {
                  obj->PrimitivesData[(i + newNodesPoses_tmp) + 2112500 *
                    ((input_sizes_idx_1 + b_input_sizes_idx_1) +
                     c_input_sizes_idx_1)] = varargin_4_data[newNodesPoses_tmp];
                }
              }

              for (i_0 = 0; i_0 < tmp_size_idx_0; i_0++) {
                for (newNodesPoses_tmp = 0; newNodesPoses_tmp < sizes_idx_0;
                     newNodesPoses_tmp++) {
                  obj->PrimitivesData[(i + newNodesPoses_tmp) + 2112500 *
                    (((input_sizes_idx_1 + b_input_sizes_idx_1) +
                      c_input_sizes_idx_1) + d_input_sizes_idx_1)] =
                    p_data_0[newNodesPoses_tmp];
                }
              }

              primitivesDataRow = g_obj_0;
              g_obj_0 = linesDataRow + (real_T)numValidLine;
              if (linesDataRow > g_obj_0 - 1.0) {
                sizes_idx_1_tmp_0 = 0;
              } else {
                sizes_idx_1_tmp_0 = (int32_T)linesDataRow - 1;
              }

              MATLAB0_repmat_kk(&currentNode_data[3], (real_T)numValidLine,
                                varargin_1);
              if (varargin_1->size[0] != 0) {
                sizes_idx_0 = varargin_1->size[0];
              } else {
                newNodesPoses_tmp = 0;
                for (i_0 = 0; i_0 < 2; i_0++) {
                  if (validLine[i_0]) {
                    newNodesPoses_tmp++;
                  }
                }

                if (newNodesPoses_tmp != 0) {
                  sizes_idx_0 = 0;
                  for (i_0 = 0; i_0 < 2; i_0++) {
                    if (validLine[i_0]) {
                      sizes_idx_0++;
                    }
                  }
                } else {
                  newNodesPoses_tmp = 0;
                  for (i_0 = 0; i_0 < 2; i_0++) {
                    if (validLine[i_0]) {
                      newNodesPoses_tmp++;
                    }
                  }

                  if (newNodesPoses_tmp != 0) {
                    sizes_idx_0 = 0;
                    for (i_0 = 0; i_0 < 2; i_0++) {
                      if (validLine[i_0]) {
                        sizes_idx_0++;
                      }
                    }
                  } else {
                    trueCount = 0;
                    newNodesPoses_tmp = 0;
                    for (i_0 = 0; i_0 < 2; i_0++) {
                      if (validLine[i_0]) {
                        newNodesPoses_tmp++;
                      }
                    }

                    if (newNodesPoses_tmp > 0) {
                      trueCount = 0;
                      for (i_0 = 0; i_0 < 2; i_0++) {
                        if (validLine[i_0]) {
                          trueCount++;
                        }
                      }
                    }

                    sizes_idx_0 = 0;
                    for (i_0 = 0; i_0 < 2; i_0++) {
                      if (validLine[i_0]) {
                        sizes_idx_0++;
                      }
                    }

                    if (sizes_idx_0 < trueCount) {
                      sizes_idx_0 = trueCount;
                    }
                  }
                }
              }

              result = (sizes_idx_0 == 0);
              if (result || (varargin_1->size[0] != 0)) {
                f_input_sizes_idx_1 = 3;
              } else {
                f_input_sizes_idx_1 = 0;
              }

              if (result) {
                input_sizes_idx_1 = 3;
              } else {
                newNodesPoses_tmp = 0;
                for (i_0 = 0; i_0 < 2; i_0++) {
                  if (validLine[i_0]) {
                    newNodesPoses_tmp++;
                  }
                }

                if (newNodesPoses_tmp != 0) {
                  input_sizes_idx_1 = 3;
                } else {
                  input_sizes_idx_1 = 0;
                }
              }

              if (result) {
                b_input_sizes_idx_1 = 1;
              } else {
                newNodesPoses_tmp = 0;
                for (i_0 = 0; i_0 < 2; i_0++) {
                  if (validLine[i_0]) {
                    newNodesPoses_tmp++;
                  }
                }

                if (newNodesPoses_tmp != 0) {
                  b_input_sizes_idx_1 = 1;
                } else {
                  b_input_sizes_idx_1 = 0;
                }
              }

              newNodesPoses_tmp = 0;
              for (i_0 = 0; i_0 < 2; i_0++) {
                if (validLine[i_0]) {
                  newNodesPoses_tmp++;
                }
              }

              i = newNodesPoses_tmp;
              newNodesPoses_tmp = 0;
              for (i_0 = 0; i_0 < 2; i_0++) {
                if (validLine[i_0]) {
                  tmp_data_2[newNodesPoses_tmp] = (int8_T)i_0;
                  newNodesPoses_tmp++;
                }
              }

              sizes_idx_1_tmp = f_input_sizes_idx_1;
              for (i_0 = 0; i_0 < 3; i_0++) {
                for (newNodesPoses_tmp = 0; newNodesPoses_tmp < i;
                     newNodesPoses_tmp++) {
                  straightNewNodePose_data[newNodesPoses_tmp + i * i_0] =
                    straightNewNodePose[(i_0 << 1) +
                    tmp_data_2[newNodesPoses_tmp]];
                }
              }

              trueCount = input_sizes_idx_1;
              for (i_0 = 0; i_0 < i; i_0++) {
                tmp_data_6[i_0] = (int8_T)(-2 * tmp_data_2[i_0] + 1);
              }

              loop_ub = b_input_sizes_idx_1;
              for (i_0 = 0; i_0 < sizes_idx_1_tmp; i_0++) {
                for (newNodesPoses_tmp = 0; newNodesPoses_tmp < sizes_idx_0;
                     newNodesPoses_tmp++) {
                  obj->LinesData[(sizes_idx_1_tmp_0 + newNodesPoses_tmp) +
                    2112500 * i_0] = varargin_1->data[sizes_idx_0 * i_0 +
                    newNodesPoses_tmp];
                }
              }

              for (i_0 = 0; i_0 < trueCount; i_0++) {
                for (newNodesPoses_tmp = 0; newNodesPoses_tmp < sizes_idx_0;
                     newNodesPoses_tmp++) {
                  obj->LinesData[(sizes_idx_1_tmp_0 + newNodesPoses_tmp) +
                    2112500 * (i_0 + f_input_sizes_idx_1)] =
                    straightNewNodePose_data[sizes_idx_0 * i_0 +
                    newNodesPoses_tmp];
                }
              }

              for (i_0 = 0; i_0 < loop_ub; i_0++) {
                for (newNodesPoses_tmp = 0; newNodesPoses_tmp < sizes_idx_0;
                     newNodesPoses_tmp++) {
                  obj->LinesData[(sizes_idx_1_tmp_0 + newNodesPoses_tmp) +
                    2112500 * (f_input_sizes_idx_1 + input_sizes_idx_1)] =
                    tmp_data_6[newNodesPoses_tmp];
                }
              }

              linesDataRow = g_obj_0;
              i = fScore_size - 1;
              newNodesPoses_tmp = 0;
              for (i_0 = 0; i_0 <= i; i_0++) {
                if (fScore_data[i_0] != (rtInf)) {
                  newNodesPoses_tmp++;
                }
              }

              sizes_idx_1_tmp = newNodesPoses_tmp;
              newNodesPoses_tmp = 0;
              for (i_0 = 0; i_0 <= i; i_0++) {
                if (fScore_data[i_0] != (rtInf)) {
                  varargin_1_tmp_data[newNodesPoses_tmp] = (int8_T)i_0;
                  newNodesPoses_tmp++;
                }
              }

              if (sizes_idx_1_tmp != 0) {
                sizes_idx_0 = sizes_idx_1_tmp;
              } else {
                newNodesPoses_tmp = 0;
                for (i_0 = 0; i_0 < 10; i_0++) {
                  if (validPrimitives[i_0]) {
                    newNodesPoses_tmp++;
                  }
                }

                if (newNodesPoses_tmp != 0) {
                  sizes_idx_0 = 0;
                  for (i_0 = 0; i_0 < 10; i_0++) {
                    if (validPrimitives[i_0]) {
                      sizes_idx_0++;
                    }
                  }
                } else {
                  newNodesPoses_tmp = 0;
                  for (i_0 = 0; i_0 < 10; i_0++) {
                    if (validPrimitives[i_0]) {
                      newNodesPoses_tmp++;
                    }
                  }

                  if (newNodesPoses_tmp != 0) {
                    sizes_idx_0 = 0;
                    for (i_0 = 0; i_0 < 10; i_0++) {
                      if (validPrimitives[i_0]) {
                        sizes_idx_0++;
                      }
                    }
                  } else {
                    trueCount = 0;
                    newNodesPoses_tmp = 0;
                    for (i_0 = 0; i_0 < 10; i_0++) {
                      if (validPrimitives[i_0]) {
                        newNodesPoses_tmp++;
                      }
                    }

                    if (newNodesPoses_tmp > 0) {
                      trueCount = 0;
                      for (i_0 = 0; i_0 < 10; i_0++) {
                        if (validPrimitives[i_0]) {
                          trueCount++;
                        }
                      }
                    }

                    sizes_idx_0 = 0;
                    for (i_0 = 0; i_0 < 10; i_0++) {
                      if (validPrimitives[i_0]) {
                        sizes_idx_0++;
                      }
                    }

                    if (sizes_idx_0 < trueCount) {
                      sizes_idx_0 = trueCount;
                    }
                  }
                }
              }

              result = (sizes_idx_0 == 0);
              if (result || (sizes_idx_1_tmp != 0)) {
                f_input_sizes_idx_1 = 1;
              } else {
                f_input_sizes_idx_1 = 0;
              }

              if (result || (sizes_idx_1_tmp != 0)) {
                input_sizes_idx_1 = 1;
              } else {
                input_sizes_idx_1 = 0;
              }

              if (result || (sizes_idx_1_tmp != 0)) {
                b_input_sizes_idx_1 = 1;
              } else {
                b_input_sizes_idx_1 = 0;
              }

              if (result) {
                c_input_sizes_idx_1 = 3;
              } else {
                newNodesPoses_tmp = 0;
                for (i_0 = 0; i_0 < 10; i_0++) {
                  if (validPrimitives[i_0]) {
                    newNodesPoses_tmp++;
                  }
                }

                if (newNodesPoses_tmp != 0) {
                  c_input_sizes_idx_1 = 3;
                } else {
                  c_input_sizes_idx_1 = 0;
                }
              }

              if (result) {
                d_input_sizes_idx_1 = 1;
              } else {
                newNodesPoses_tmp = 0;
                for (i_0 = 0; i_0 < 10; i_0++) {
                  if (validPrimitives[i_0]) {
                    newNodesPoses_tmp++;
                  }
                }

                if (newNodesPoses_tmp != 0) {
                  d_input_sizes_idx_1 = 1;
                } else {
                  d_input_sizes_idx_1 = 0;
                }
              }

              newNodesPoses_tmp = 0;
              for (i_0 = 0; i_0 < 10; i_0++) {
                if (validPrimitives[i_0]) {
                  newNodesPoses_tmp++;
                }
              }

              i = newNodesPoses_tmp;
              newNodesPoses_tmp = 0;
              for (i_0 = 0; i_0 < 10; i_0++) {
                if (validPrimitives[i_0]) {
                  tmp_data_3[newNodesPoses_tmp] = (int8_T)i_0;
                  newNodesPoses_tmp++;
                }
              }

              for (i_0 = 0; i_0 < sizes_idx_1_tmp; i_0++) {
                e_input_sizes_idx_1 = varargin_1_tmp_data[i_0];
                tmp_data_5[i_0] = fScore_data[e_input_sizes_idx_1];
                p_data[i_0] = gScore_data[e_input_sizes_idx_1];
                b_hScore_data_0[i_0] = b_hScore_data[e_input_sizes_idx_1];
              }

              for (i_0 = 0; i_0 < 3; i_0++) {
                for (newNodesPoses_tmp = 0; newNodesPoses_tmp < i;
                     newNodesPoses_tmp++) {
                  newNodesPoses_data[newNodesPoses_tmp + i * i_0] =
                    newNodesPoses[10 * i_0 + tmp_data_3[newNodesPoses_tmp]];
                }
              }

              for (i_0 = 0; i_0 < i; i_0++) {
                p_data_1[i_0] = p[tmp_data_3[i_0]];
              }

              trueCount = (((f_input_sizes_idx_1 + input_sizes_idx_1) +
                            b_input_sizes_idx_1) + c_input_sizes_idx_1) +
                d_input_sizes_idx_1;
              i = sizes_idx_0 * f_input_sizes_idx_1;
              for (i_0 = 0; i_0 < i; i_0++) {
                e_result_data[i_0] = tmp_data_5[i_0];
              }

              newNodesPoses_tmp = sizes_idx_0 * input_sizes_idx_1;
              for (i_0 = 0; i_0 < newNodesPoses_tmp; i_0++) {
                e_result_data[i_0 + i] = p_data[i_0];
              }

              tmp_size_idx_0 = sizes_idx_0 * b_input_sizes_idx_1;
              for (i_0 = 0; i_0 < tmp_size_idx_0; i_0++) {
                e_result_data[(i_0 + i) + newNodesPoses_tmp] =
                  b_hScore_data_0[i_0];
              }

              sizes_idx_1_tmp = sizes_idx_0 * c_input_sizes_idx_1;
              for (i_0 = 0; i_0 < sizes_idx_1_tmp; i_0++) {
                e_result_data[((i_0 + i) + newNodesPoses_tmp) + tmp_size_idx_0] =
                  newNodesPoses_data[i_0];
              }

              loop_ub = sizes_idx_0 * d_input_sizes_idx_1;
              for (i_0 = 0; i_0 < loop_ub; i_0++) {
                e_result_data[(((i_0 + i) + newNodesPoses_tmp) + tmp_size_idx_0)
                  + sizes_idx_1_tmp] = p_data_1[i_0];
              }

              newNodesPoses_tmp = s_size;
              for (i = 0; i < s_size; i++) {
                for (i_0 = 0; i_0 < trueCount; i_0++) {
                  currentNode_data[i_0] = e_result_data[sizes_idx_0 * i_0 + i];
                }

                priorityqueuecodegen_push(openSet.PQInternal, &currentNode_data
                  [0]);
                currentNodePose[0] = e_result_data[sizes_idx_0 * 3 + i];
                currentNodePose[1] = e_result_data[(sizes_idx_0 << 2) + i];
                currentNodePose[2] = e_result_data[sizes_idx_0 * 5 + i];
                priorityqueuecodegen_nodemap_getNumNodes(nodeMap.NodeMapInternal);
                priorityqueuecodegen_nodemap_insertNode(nodeMap.NodeMapInternal,
                  &currentNodePose[0], currentNodeId);
              }
            }

            if (MATLAB0_strcmp_c3(searchMode_data, searchMode_size) &&
                (numValidPrimitives != 0)) {
              newNodesPoses_tmp = 0;
              for (i_0 = 0; i_0 < 10; i_0++) {
                if (validPrimitives[i_0]) {
                  newNodesPoses_tmp++;
                }
              }

              i = newNodesPoses_tmp;
              newNodesPoses_tmp = 0;
              for (i_0 = 0; i_0 < 10; i_0++) {
                if (validPrimitives[i_0]) {
                  tmp_data_1[newNodesPoses_tmp] = (int8_T)i_0;
                  newNodesPoses_tmp++;
                }
              }

              for (i_0 = 0; i_0 < i; i_0++) {
                p_data[i_0] = p[tmp_data_1[i_0]];
              }

              plannerHybridAStar_closeCell_i(obj, p_data, &i,
                newNodesPosesGridIndices_data);
            }
          }
        } else {
          exitg1 = 1;
        }
      } while (exitg1 == 0);

      MATLAB0_emxFree_real_T(&varargin_1);
      MATLAB0_emxFree_real_T(&directions);
      MATLAB0_emxFree_real_T(&pathData);
      MATLAB0_emxFree_real_T(&pathPoses1);
    }
  }

  pathObj = MATLAB0_navPath_navPath(iobj_1, obj->StateValidator->StateSpace,
    pathStates);
  MATLAB0_emxFree_real_T(&pathStates);
  if (!nodeMap.matlabCodegenIsDeleted) {
    priorityqueuecodegen_destructNodeMap(nodeMap.NodeMapInternal);
  }

  if (!openSet.matlabCodegenIsDeleted) {
    priorityqueuecodegen_destructPQ(openSet.PQInternal);
  }

  emxFreeStruct_reedsSheppConnect(&rsPathObj);
  return pathObj;
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static boolean_T validatorOccupancyMap_isMotionV(validatorOccupancyMap_MATLAB0_T
  *obj, const real_T state1[3], real_T state2[3])
{
  static stateSpaceSE2_MATLAB0_T *b_obj;
  static real_T iState[3];
  static real_T stateDiff[3];
  static real_T d;
  static real_T delta;
  static real_T dist;
  static real_T dtheta;
  static real_T n;
  static real_T ratios;
  static int32_T j;
  boolean_T exitg1;
  boolean_T isValid;
  if (!validatorOccupancyMap_isStateVa(obj, state1)) {
    isValid = false;
    state2[0] = (rtNaN);
    state2[1] = (rtNaN);
    state2[2] = (rtNaN);
  } else {
    b_obj = obj->StateSpace;
    dtheta = state1[0] - state2[0];
    iState[0] = dtheta;
    stateDiff[0] = fabs(dtheta);
    dtheta = state1[1] - state2[1];
    iState[1] = dtheta;
    stateDiff[1] = fabs(dtheta);
    dtheta = state1[2] - state2[2];
    iState[2] = dtheta;
    dtheta = fabs(dtheta);
    MATLAB0_wrapToPi(&dtheta);
    dist = (stateDiff[0] * stateDiff[0] + stateDiff[1] * stateDiff[1]) *
      b_obj->WeightXY + dtheta * dtheta * b_obj->WeightTheta;
    dist = sqrt(dist);
    if (rtIsNaN(dist)) {
      isValid = false;
      state2[0] = state1[0];
      state2[1] = state1[1];
      state2[2] = state1[2];
    } else {
      delta = obj->ValidationDistance;
      d = delta;
      n = 1.0;
      isValid = true;
      iState[0] = state1[0];
      iState[1] = state1[1];
      iState[2] = state1[2];
      exitg1 = false;
      while ((!exitg1) && (d < dist)) {
        ratios = d / dist;
        stateDiff[0] = state2[0] - state1[0];
        stateDiff[1] = state2[1] - state1[1];
        stateDiff[2] = state2[2] - state1[2];
        MATLAB0_wrapToPi(&stateDiff[2]);
        for (j = 0; j < 3; j++) {
          iState[j] = ratios * stateDiff[j] + state1[j];
        }

        MATLAB0_wrapToPi(&iState[2]);
        if (!validatorOccupancyMap_isStateVa(obj, iState)) {
          isValid = false;
          exitg1 = true;
        } else {
          n++;
          d = delta * n;
        }
      }

      if (isValid) {
        isValid = validatorOccupancyMap_isStateVa(obj, state2);
      }

      if (!isValid) {
        state2[0] = iState[0];
        state2[1] = iState[1];
        state2[2] = iState[2];
      }
    }
  }

  return isValid;
}

static void MATLAB0_emxFreeStruct_navPath(navPath_MATLAB0_T *pStruct)
{
  MATLAB0_emxFree_real_T(&pStruct->StateInternal);
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static void MATLAB0_sf_gateway_c6_MATLAB0(SimStruct *S)
{
  static binaryOccupancyMap_MATLAB0_T map;
  static emxArray_real_T_MATLAB0_T *d;
  static navPath_MATLAB0_T refpath;
  static plannerAStarGrid_MATLAB0_T lobj_5;
  static plannerHybridAStar_MATLAB0_T planner;
  static stateSpaceSE2_MATLAB0_T ss;
  static validatorOccupancyMap_MATLAB0_T sv;
  static real_T goalPose[3];
  static real_T lastValid[3];
  static real_T map_0;
  static real_T map_1;
  static int32_T i;
  static int32_T i_0;
  static int32_T i_1;
  B_MATLAB0_T *_rtB;
  _rtB = ((B_MATLAB0_T *) ssGetLocalBlockIO(S));
  MATLAB0_emxInitStruct_navPath(&refpath);
  binaryOccupancyMap_binaryOccupa(&map, *(const boolean_T **)
    ssGetInputPortSignalPtrs(S, 0));
  map_0 = map.SharedProperties.LocalOriginInWorld[0] +
    map.SharedProperties.GridOriginInLocal[0];
  map_1 = map.SharedProperties.LocalOriginInWorld[1] +
    map.SharedProperties.GridOriginInLocal[1];
  map.SharedProperties.GridOriginInLocal[0] = -((map_0 + 650.0) - map_0) / 2.0;
  map.SharedProperties.GridOriginInLocal[1] = -((map_1 + 650.0) - map_1) / 2.0;
  MAT_stateSpaceSE2_stateSpaceSE2(&ss);
  map_0 = map.SharedProperties.LocalOriginInWorld[0] +
    map.SharedProperties.GridOriginInLocal[0];
  map_1 = map.SharedProperties.LocalOriginInWorld[1] +
    map.SharedProperties.GridOriginInLocal[1];
  ss.StateBoundsInternal[0] = map_0;
  ss.StateBoundsInternal[1] = map_1;
  ss.StateBoundsInternal[2] = -3.1415926535897931;
  ss.StateBoundsInternal[3] = map_0 + 650.0;
  ss.StateBoundsInternal[4] = map_1 + 650.0;
  ss.StateBoundsInternal[5] = 3.1415926535897931;
  validatorOccupancyMap_validator(&sv, &ss, &map);
  plannerHybridAStar_plannerHybri(&planner, &sv, *((const real_T **)
    ssGetInputPortSignalPtrs(S, 3))[0], *((const real_T **)
    ssGetInputPortSignalPtrs(S, 4))[0], *((const real_T **)
    ssGetInputPortSignalPtrs(S, 5))[0]);
  goalPose[0] = *((const real_T **)ssGetInputPortSignalPtrs(S, 2))[0];
  goalPose[1] = *((const real_T **)ssGetInputPortSignalPtrs(S, 2))[1];
  map_0 = MATLAB0_rt_atan2d_snf((real_T)(*((const real_T **)
    ssGetInputPortSignalPtrs(S, 2))[1] - *((const real_T **)
    ssGetInputPortSignalPtrs(S, 1))[1]), (real_T)(*((const real_T **)
    ssGetInputPortSignalPtrs(S, 2))[0] - *((const real_T **)
    ssGetInputPortSignalPtrs(S, 1))[0]));
  goalPose[2] = map_0;
  if (!validatorOccupancyMap_isStateVa(&sv, goalPose)) {
    lastValid[0] = *((const real_T **)ssGetInputPortSignalPtrs(S, 2))[0];
    lastValid[1] = *((const real_T **)ssGetInputPortSignalPtrs(S, 2))[1];
    lastValid[2] = map_0;
    validatorOccupancyMap_isMotionV(&sv, *(const real_T **)
      ssGetInputPortSignalPtrs(S, 1), lastValid);
    goalPose[0] = lastValid[0];
    goalPose[1] = lastValid[1];
    goalPose[2] = MATLAB0_rt_atan2d_snf(lastValid[1] - *((const real_T **)
      ssGetInputPortSignalPtrs(S, 1))[1], lastValid[0] - *((const real_T **)
      ssGetInputPortSignalPtrs(S, 1))[0]);
  }

  MATLAB0_plannerHybridAStar_plan(&planner, *(const real_T **)
    ssGetInputPortSignalPtrs(S, 1), goalPose, &lobj_5, &refpath);
  memset(&_rtB->path[0], 0, 300U * sizeof(real_T));
  map_0 = refpath.NumStates;
  if (map_0 < 1.0) {
    i = -1;
  } else {
    i = (int32_T)map_0 - 1;
  }

  MATLAB0_emxInit_real_T(&d, 2);
  i_1 = d->size[0] * d->size[1];
  d->size[0] = i + 1;
  d->size[1] = 3;
  MATLAB_emxEnsureCapacity_real_T(d, i_1);
  for (i_1 = 0; i_1 < 3; i_1++) {
    for (i_0 = 0; i_0 <= i; i_0++) {
      d->data[i_0 + d->size[0] * i_1] = refpath.StateInternal->
        data[refpath.StateInternal->size[0] * i_1 + i_0];
    }
  }

  i = d->size[0];
  for (i_1 = 0; i_1 < 3; i_1++) {
    for (i_0 = 0; i_0 < i; i_0++) {
      _rtB->path[i_0 + 100 * i_1] = d->data[d->size[0] * i_1 + i_0];
    }
  }

  MATLAB0_emxFree_real_T(&d);
  MATLAB0_emxFreeStruct_navPath(&refpath);
}

/* Start for root system: '<Root>' */
#define MDL_START

static void mdlStart(SimStruct *S)
{
  /* instance underlying S-Function data */
#if defined(RT_MALLOC) || defined(MATLAB_MEX_FILE)
#if defined(MATLAB_MEX_FILE)

  /* non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

#endif

  MATLAB0_malloc(S);
  if (ssGetErrorStatus(S) != (NULL) ) {
    return;
  }

#endif

  {
  }
}

/* Outputs for root system: '<Root>' */
static void mdlOutputs(SimStruct *S, int_T tid)
{
  static int32_T i;
  B_MATLAB0_T *_rtB;
  _rtB = ((B_MATLAB0_T *) ssGetLocalBlockIO(S));

  /* MATLAB Function: '<Root>/MATLAB Function' */
  MATLAB0_sf_gateway_c6_MATLAB0(S);

  /* Outport: '<Root>/path' */
  for (i = 0; i < 300; i++) {
    ((real_T *)ssGetOutputPortSignal(S, 0))[i] = _rtB->path[i];
  }

  /* End of Outport: '<Root>/path' */
  UNUSED_PARAMETER(tid);
}

/* Update for root system: '<Root>' */
#define MDL_UPDATE

static void mdlUpdate(SimStruct *S, int_T tid)
{
  UNUSED_PARAMETER(tid);
}

/* Termination for root system: '<Root>' */
static void mdlTerminate(SimStruct *S)
{

#if defined(RT_MALLOC) || defined(MATLAB_MEX_FILE)

  if (ssGetUserData(S) != (NULL) ) {
    rt_FREE(ssGetLocalBlockIO(S));
  }

  rt_FREE(ssGetUserData(S));

#endif

}

#if defined(RT_MALLOC) || defined(MATLAB_MEX_FILE)
#include "MATLAB0_mid.h"
#endif

/* Function to initialize sizes. */
static void mdlInitializeSizes(SimStruct *S)
{
  ssSetNumSampleTimes(S, 1);           /* Number of sample times */
  ssSetNumContStates(S, 0);            /* Number of continuous states */
  ssSetNumNonsampledZCs(S, 0);         /* Number of nonsampled ZCs */

  /* Number of output ports */
  if (!ssSetNumOutputPorts(S, 2))
    return;

  /* outport number: 0 */
  if (!ssSetOutputPortMatrixDimensions(S, 0, 100, 3))
    return;
  if (ssGetSimMode(S) != SS_SIMMODE_SIZES_CALL_ONLY) {
    ssSetOutputPortDataType(S, 0, SS_DOUBLE);
  }

  ssSetOutputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);

  /* outport number: 1 */
  if (!ssSetOutputPortVectorDimension(S, 1, 1))
    return;
  if (ssGetSimMode(S) != SS_SIMMODE_SIZES_CALL_ONLY) {
    ssSetOutputPortDataType(S, 1, SS_DOUBLE);
  }

  ssSetOutputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);

  /* Number of input ports */
  if (!ssSetNumInputPorts(S, 6))
    return;

  /* inport number: 0 */
  {
    if (!ssSetInputPortMatrixDimensions(S, 0, 650, 650))
      return;
    if (ssGetSimMode(S) != SS_SIMMODE_SIZES_CALL_ONLY) {
      ssSetInputPortDataType(S, 0, SS_BOOLEAN);
    }

    ssSetInputPortDirectFeedThrough(S, 0, 1);
    ssSetInputPortOverWritable(S, 0, 0);
    ssSetInputPortOptimOpts(S, 0, SS_NOT_REUSABLE_AND_GLOBAL);
  }

  /* inport number: 1 */
  {
    if (!ssSetInputPortVectorDimension(S, 1, 3))
      return;
    if (ssGetSimMode(S) != SS_SIMMODE_SIZES_CALL_ONLY) {
      ssSetInputPortDataType(S, 1, SS_DOUBLE);
    }

    ssSetInputPortDirectFeedThrough(S, 1, 1);
    ssSetInputPortOverWritable(S, 1, 0);
    ssSetInputPortOptimOpts(S, 1, SS_NOT_REUSABLE_AND_GLOBAL);
  }

  /* inport number: 2 */
  {
    if (!ssSetInputPortVectorDimension(S, 2, 2))
      return;
    if (ssGetSimMode(S) != SS_SIMMODE_SIZES_CALL_ONLY) {
      ssSetInputPortDataType(S, 2, SS_DOUBLE);
    }

    ssSetInputPortDirectFeedThrough(S, 2, 1);
    ssSetInputPortOverWritable(S, 2, 0);
    ssSetInputPortOptimOpts(S, 2, SS_NOT_REUSABLE_AND_GLOBAL);
  }

  /* inport number: 3 */
  {
    if (!ssSetInputPortVectorDimension(S, 3, 1))
      return;
    if (ssGetSimMode(S) != SS_SIMMODE_SIZES_CALL_ONLY) {
      ssSetInputPortDataType(S, 3, SS_DOUBLE);
    }

    ssSetInputPortDirectFeedThrough(S, 3, 1);
    ssSetInputPortOverWritable(S, 3, 0);
    ssSetInputPortOptimOpts(S, 3, SS_NOT_REUSABLE_AND_GLOBAL);
  }

  /* inport number: 4 */
  {
    if (!ssSetInputPortVectorDimension(S, 4, 1))
      return;
    if (ssGetSimMode(S) != SS_SIMMODE_SIZES_CALL_ONLY) {
      ssSetInputPortDataType(S, 4, SS_DOUBLE);
    }

    ssSetInputPortDirectFeedThrough(S, 4, 1);
    ssSetInputPortOverWritable(S, 4, 0);
    ssSetInputPortOptimOpts(S, 4, SS_NOT_REUSABLE_AND_GLOBAL);
  }

  /* inport number: 5 */
  {
    if (!ssSetInputPortVectorDimension(S, 5, 1))
      return;
    if (ssGetSimMode(S) != SS_SIMMODE_SIZES_CALL_ONLY) {
      ssSetInputPortDataType(S, 5, SS_DOUBLE);
    }

    ssSetInputPortDirectFeedThrough(S, 5, 1);
    ssSetInputPortOverWritable(S, 5, 0);
    ssSetInputPortOptimOpts(S, 5, SS_NOT_REUSABLE_AND_GLOBAL);
  }

  ssSetRTWGeneratedSFcn(S, 1);         /* Generated S-function */

  /* Tunable Parameters */
  ssSetNumSFcnParams(S, 0);

  /* Number of expected parameters */
#if defined(MATLAB_MEX_FILE)

  if (ssGetNumSFcnParams(S) == ssGetSFcnParamsCount(S)) {

#if defined(MDL_CHECK_PARAMETERS)

    mdlCheckParameters(S);

#endif                                 /* MDL_CHECK_PARAMETERS */

    if (ssGetErrorStatus(S) != (NULL) ) {
      return;
    }
  } else {
    return;                /* Parameter mismatch will be reported by Simulink */
  }

#endif                                 /* MATLAB_MEX_FILE */

  /* Options */
  ssSetOptions(S, (SS_OPTION_RUNTIME_EXCEPTION_FREE_CODE ));

#if SS_SFCN_FOR_SIM

  {
    ssSupportsMultipleExecInstances(S, true);
    ssHasStateInsideForEachSS(S, false);
    ssSetModelReferenceSampleTimeInheritanceRule(S,
      USE_DEFAULT_FOR_DISCRETE_INHERITANCE);
  }

#endif

}

/* Function to initialize sample times. */
static void mdlInitializeSampleTimes(SimStruct *S)
{
  ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
  ssSetOffsetTime(S, 0, 0.0);
}

#if defined(MATLAB_MEX_FILE)
#include "fixedpoint.c"
#include "simulink.c"
#else
#undef S_FUNCTION_NAME
#define S_FUNCTION_NAME                MATLAB0_sf
#include "cg_sfun.h"
#endif                                 /* defined(MATLAB_MEX_FILE) */
