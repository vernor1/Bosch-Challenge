#ifndef TRAJECTORYCOST_H
#define TRAJECTORYCOST_H

#include <vector>

namespace helpers {

// Evaluates a polynomial.
// @param[in] coeffs  Polynomial coefficients.
// @param[in] x       Argument to avaluate the polynomial for.
// @return            Value of the function.
double EvaluatePolynomial(const std::vector<double>& coeffs, double x);

// Calculates the derivative of a polynomial and returns the corresponding
// coefficients.
std::vector<double> GetDerivative(const std::vector<double>& coeffs);

} // helpers

#endif // TRAJECTORYCOST_H
