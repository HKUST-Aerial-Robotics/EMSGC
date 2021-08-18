#include <emsgc/core/numerical_deriv.h>

/**
 * \brief Computes the forward-difference approximation to the gradient of a function at a given point.
 *
 * \author Guillermo Gallego -- ggb@gatech.edu
 *
 * \date 2009-07
 *
 * \return Value of the function at x.
 */
double vs_gsl_Gradient_ForwardDiff (
  const gsl_vector * x, /**< [in] Point at which the gradient is to be evaluated */
  void * data,          /**< [in] Optional parameters passed directly to the function func_f */
  double (*func_f)(const gsl_vector * x, void *data), /**< [in] User-supplied routine that returns the value of the function at x */
  gsl_vector * J,       /**< [out] Gradient vector (same length as x) */
  double dh = 1e-6      /**< [in] Increment in variable for numerical differentiation */
)
{
  // Evaluate vector function at x
  double fx = func_f(x, data);

  // Clone the parameter vector x
  gsl_vector *xh = gsl_vector_alloc (J->size);
  gsl_vector_memcpy(xh, x);

  for (int j=0; j < J->size; j++)
  {
    gsl_vector_set(xh,j,gsl_vector_get(x,j)+dh); // Take a (forward) step in the current dimension
    double fh = func_f(xh, data); // Evaluate vector function at new x
    gsl_vector_set(J ,j,fh-fx); // Finite difference approximation (except for 1/dh factor)
    gsl_vector_set(xh,j,gsl_vector_get(x,j)); // restore original value of the current variable
  }
  gsl_vector_scale(J, 1.0/dh);

  gsl_vector_free(xh);

  return fx;
}
