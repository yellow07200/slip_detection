/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * applyOperators.cpp
 *
 * Code generation for function 'applyOperators'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "fuzzy2inputs.h"
#include "applyOperators.h"

/* Function Definitions */
void applyOperators(const double fuzzifiedInputs[18], double antecedentOutputs[9],
                    double *sumAntecedentOutputs, double irrOfOneRule[2])
{
  int ruleID;
  double d0;
  *sumAntecedentOutputs = 0.0;
  for (ruleID = 0; ruleID < 9; ruleID++) {
    irrOfOneRule[0] = fuzzifiedInputs[ruleID];
    irrOfOneRule[1] = fuzzifiedInputs[ruleID + 9];
    if ((fuzzifiedInputs[ruleID] > fuzzifiedInputs[ruleID + 9]) || (rtIsNaN
         (fuzzifiedInputs[ruleID]) && (!rtIsNaN(fuzzifiedInputs[ruleID + 9]))))
    {
      d0 = fuzzifiedInputs[ruleID + 9];
      antecedentOutputs[ruleID] = fuzzifiedInputs[ruleID + 9];
    } else {
      d0 = fuzzifiedInputs[ruleID];
      antecedentOutputs[ruleID] = fuzzifiedInputs[ruleID];
    }

    *sumAntecedentOutputs += d0;
  }
}

/* End of code generation (applyOperators.cpp) */
