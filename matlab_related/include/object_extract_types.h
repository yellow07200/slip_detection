//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: object_extract_types.h
//
// MATLAB Coder version            : 4.1
// C/C++ source code generated on  : 29-May-2019 18:34:50
//
#ifndef OBJECT_EXTRACT_TYPES_H
#define OBJECT_EXTRACT_TYPES_H

// Include Files
#include "rtwtypes.h"

// Type Definitions
struct emxArray_int32_T
{
  int *data;
  int *size;
  int allocatedSize;
  int numDimensions;
  boolean_T canFreeData;
};

struct emxArray_real_T
{
  double *data;
  int *size;
  int allocatedSize;
  int numDimensions;
  boolean_T canFreeData;
};

struct emxArray_uint16_T
{
  unsigned short *data;
  int *size;
  int allocatedSize;
  int numDimensions;
  boolean_T canFreeData;
};

#endif

//
// File trailer for object_extract_types.h
//
// [EOF]
//
