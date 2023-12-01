/*
 * MATLAB0_sid.h
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
 *
 * SOURCES: MATLAB0_sf.c
 */

/* statically allocated instance data for model: MATLAB0 */
{
  {
    /* Local SimStruct for the generated S-Function */
    static LocalS slS;
    LocalS *lS = &slS;
    ssSetUserData(rts, lS);

    /* block I/O */
    {
      static B_MATLAB0_T sfcnB;
      void *b = (real_T *) &sfcnB;
      ssSetLocalBlockIO(rts, b);
      (void) memset(b, 0,
                    sizeof(B_MATLAB0_T));
    }

    /* model checksums */
    ssSetChecksumVal(rts, 0, 1537176858U);
    ssSetChecksumVal(rts, 1, 1877979224U);
    ssSetChecksumVal(rts, 2, 1126616928U);
    ssSetChecksumVal(rts, 3, 1185095475U);
  }
}
