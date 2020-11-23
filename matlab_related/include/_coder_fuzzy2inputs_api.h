/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * _coder_fuzzy2inputs_api.h
 *
 * Code generation for function '_coder_fuzzy2inputs_api'
 *
 */

#ifndef _CODER_FUZZY2INPUTS_API_H
#define _CODER_FUZZY2INPUTS_API_H

/* Include files */
#include "tmwtypes.h"
#include "mex.h"
#include "emlrt.h"
#include <stddef.h>
#include <stdlib.h>
#include "_coder_fuzzy2inputs_api.h"

/* Variable Declarations */
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

/* Function Declarations */
extern real_T fuzzy2inputs(real_T inputs[3]);
extern void fuzzy2inputs_api(const mxArray * const prhs[1], int32_T nlhs, const
  mxArray *plhs[1]);
extern void fuzzy2inputs_atexit(void);
extern void fuzzy2inputs_initialize(void);
extern void fuzzy2inputs_terminate(void);
extern void fuzzy2inputs_xil_terminate(void);

#endif

/* End of code generation (_coder_fuzzy2inputs_api.h) */
