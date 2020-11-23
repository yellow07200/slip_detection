/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * _coder_fuzzy2inputs_mex.cpp
 *
 * Code generation for function '_coder_fuzzy2inputs_mex'
 *
 */

/* Include files */
#include "_coder_fuzzy2inputs_api.h"
#include "_coder_fuzzy2inputs_mex.h"

/* Function Declarations */
static void fuzzy2inputs_mexFunction(int32_T nlhs, mxArray *plhs[1], int32_T
  nrhs, const mxArray *prhs[1]);

/* Function Definitions */
static void fuzzy2inputs_mexFunction(int32_T nlhs, mxArray *plhs[1], int32_T
  nrhs, const mxArray *prhs[1])
{
  const mxArray *outputs[1];
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  st.tls = emlrtRootTLSGlobal;

  /* Check for proper number of arguments. */
  if (nrhs != 1) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 1, 4,
                        12, "fuzzy2inputs");
  }

  if (nlhs > 1) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 12,
                        "fuzzy2inputs");
  }

  /* Call the function. */
  fuzzy2inputs_api(prhs, nlhs, outputs);

  /* Copy over outputs to the caller. */
  emlrtReturnArrays(1, plhs, outputs);
}

void mexFunction(int32_T nlhs, mxArray *plhs[], int32_T nrhs, const mxArray
                 *prhs[])
{
  mexAtExit(fuzzy2inputs_atexit);

  /* Module initialization. */
  fuzzy2inputs_initialize();

  /* Dispatch the entry-point. */
  fuzzy2inputs_mexFunction(nlhs, plhs, nrhs, prhs);

  /* Module termination. */
  fuzzy2inputs_terminate();
}

emlrtCTX mexFunctionCreateRootTLS(void)
{
  emlrtCreateRootTLS(&emlrtRootTLSGlobal, &emlrtContextGlobal, NULL, 1);
  return emlrtRootTLSGlobal;
}

/* End of code generation (_coder_fuzzy2inputs_mex.cpp) */
