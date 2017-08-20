#ifndef HELPERS_H
#define HELPERS_H

#include <vector>

namespace helpers {

// Meters in mile per international agreement of 1959.
static const double kMetersInMile = 1609.344;

// Coefficient of conversion miles-per-hour to meters-per-second.
static const double kMphToMps = kMetersInMile / (60. * 60.);

// Evaluates a polynomial.
// @param[in] coeffs  Polynomial coefficients.
// @param[in] x       Argument to avaluate the polynomial for.
// @return            Value of the function.
double EvaluatePolynomial(const std::vector<double>& coeffs, double x);

// Calculates the derivative of a polynomial and returns the corresponding
// coefficients.
// @param[in] coeffs  Polynomial coefficients.
// @return  Coefficients of the derivative polynomial.
std::vector<double> GetDerivative(const std::vector<double>& coeffs);

} // helpers

#endif // HELPERS_H
