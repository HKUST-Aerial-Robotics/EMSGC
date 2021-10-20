#pragma once

#include <gsl/gsl_vector.h>

double vs_gsl_Gradient_ForwardDiff (
  const gsl_vector * x, /**< [in] Point at which the gradient is to be evaluated */
  void * data,          /**< [in] Optional parameters passed directly to the function func_f */
  double (*func_f)(const gsl_vector * x, void *data), /**< [in] User-supplied routine that returns the value of the function at x */
  gsl_vector * J,       /**< [out] Gradient vector (same length as x) */
  double dh             /**< [in] Increment in variable for numerical differentiation */
);
