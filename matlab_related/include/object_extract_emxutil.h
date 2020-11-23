//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: object_extract_emxutil.h
//
// MATLAB Coder version            : 4.1
// C/C++ source code generated on  : 29-May-2019 18:34:50
//
#ifndef OBJECT_EXTRACT_EMXUTIL_H
#define OBJECT_EXTRACT_EMXUTIL_H

// Include Files
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "object_extract_types.h"

// Function Declarations
extern void emxEnsureCapacity_int32_T(emxArray_int32_T *emxArray, int oldNumel);
extern void emxEnsureCapacity_real_T(emxArray_real_T *emxArray, int oldNumel);
extern void emxEnsureCapacity_uint16_T(emxArray_uint16_T *emxArray, int oldNumel);
extern void emxFree_int32_T(emxArray_int32_T **pEmxArray);
extern void emxFree_real_T(emxArray_real_T **pEmxArray);
extern void emxFree_uint16_T(emxArray_uint16_T **pEmxArray);
extern void emxInit_int32_T(emxArray_int32_T **pEmxArray, int numDimensions);
extern void emxInit_real_T(emxArray_real_T **pEmxArray, int numDimensions);
extern void emxInit_uint16_T(emxArray_uint16_T **pEmxArray, int numDimensions);

#endif

//
// File trailer for object_extract_emxutil.h
//
// [EOF]
//
