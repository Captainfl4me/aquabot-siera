/*
 * MATLAB0_sf.h
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

#ifndef RTW_HEADER_MATLAB0_sf_h_
#define RTW_HEADER_MATLAB0_sf_h_
#ifndef MATLAB0_sf_COMMON_INCLUDES_
#define MATLAB0_sf_COMMON_INCLUDES_
#include <stdlib.h>
#include <string.h>
#define S_FUNCTION_NAME                MATLAB0_sf
#define S_FUNCTION_LEVEL               2
#ifndef RTW_GENERATED_S_FUNCTION
#define RTW_GENERATED_S_FUNCTION
#endif

#include "rtwtypes.h"
#include "simstruc.h"
#include "fixedpoint.h"
#include "autonomouscodegen_reeds_shepp_api.hpp"
#include "priorityqueue_api.hpp"
#if !defined(MATLAB_MEX_FILE)
#include "rt_matrx.h"
#endif

#if !defined(RTW_SFUNCTION_DEFINES)
#define RTW_SFUNCTION_DEFINES

typedef struct {
  void *blockIO;
  void *defaultParam;
  void *nonContDerivSig;
} LocalS;

#define ssSetLocalBlockIO(S, io)       ((LocalS *)ssGetUserData(S))->blockIO = ((void *)(io))
#define ssGetLocalBlockIO(S)           ((LocalS *)ssGetUserData(S))->blockIO
#define ssSetLocalDefaultParam(S, paramVector) ((LocalS *)ssGetUserData(S))->defaultParam = (paramVector)
#define ssGetLocalDefaultParam(S)      ((LocalS *)ssGetUserData(S))->defaultParam
#define ssSetLocalNonContDerivSig(S, pSig) ((LocalS *)ssGetUserData(S))->nonContDerivSig = (pSig)
#define ssGetLocalNonContDerivSig(S)   ((LocalS *)ssGetUserData(S))->nonContDerivSig
#endif
#endif                                 /* MATLAB0_sf_COMMON_INCLUDES_ */

#include "MATLAB0_sf_types.h"
#include "rt_nonfinite.h"
#include "rtGetInf.h"
#include "rtGetNaN.h"
#include <stddef.h>
#include "rt_defines.h"

/* Block signals (default storage) */
typedef struct {
  real_T path[300];                    /* '<Root>/MATLAB Function' */
} B_MATLAB0_T;

/* External inputs (root inport signals with default storage) */
typedef struct {
  boolean_T *map_matrix[422500];       /* '<Root>/mapMatrix' */
  real_T *pose[3];                     /* '<Root>/pose' */
  real_T *goal[2];                     /* '<Root>/goal' */
  real_T *MinTurningRadius;            /* '<Root>/MinTurningRadius' */
  real_T *MotionPrimitiveLength;       /* '<Root>/MotionPrimitiveLength' */
  real_T *InterpolationDistance;       /* '<Root>/InterpolationDistance' */
} ExternalUPtrs_MATLAB0_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real_T *path[300];                   /* '<Root>/path' */
  real_T *numPath;                     /* '<Root>/numPath' */
} ExtY_MATLAB0_T;

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Note that this particular code originates from a subsystem build,
 * and has its own system numbers different from the parent model.
 * Refer to the system hierarchy for this subsystem below, and use the
 * MATLAB hilite_system command to trace the generated code back
 * to the parent model.  For example,
 *
 * hilite_system('aquabot_main/Trigger Planner/MATLAB Function')    - opens subsystem aquabot_main/Trigger Planner/MATLAB Function
 * hilite_system('aquabot_main/Trigger Planner/MATLAB Function/Kp') - opens and selects block Kp
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'aquabot_main/Trigger Planner'
 * '<S1>'   : 'aquabot_main/Trigger Planner/MATLAB Function'
 */
#endif                                 /* RTW_HEADER_MATLAB0_sf_h_ */
