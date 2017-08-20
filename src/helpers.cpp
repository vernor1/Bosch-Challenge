#include "helpers.h"
#include <cassert>
#include <cmath>

double helpers::EvaluatePolynomial(
  const std::vector<double>& coeffs, double x) {
  auto result = 0.;
  for (std::size_t i = 0; i < coeffs.size(); ++i) {
    result += coeffs[i] * std::pow(x, i);
  }
  return result;
}

std::vector<double> helpers::GetDerivative(const std::vector<double>& coeffs) {
  assert(coeffs.size() > 0);
  std::vector<double> deriv_coeffs;
  for (std::size_t i = 1; i < coeffs.size(); ++i) {
    deriv_coeffs.push_back(static_cast<double>(i) * coeffs[i]);
  }
  return deriv_coeffs;
}
