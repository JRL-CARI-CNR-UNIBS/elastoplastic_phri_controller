#ifndef NOTCH_FILTER_HPP
#define NOTCH_FILTER_HPP

#include <cmath>

class NotchFilter {
public:
  NotchFilter(const double cut_freq, const double Q, const double sample_rate) : z1(0), z2(0) {
    configure(cut_freq, Q, sample_rate);
  };

  void configure(const double cut_freq, const double Q, const double sample_rate) {
    const double omega = 2.0 * M_PI * cut_freq / sample_rate;
    const double sn = std::sin(omega);
    const double cs = std::cos(omega);
    // alpha determines bandwidth: alpha = sin(ω0)/(2Q)
    const double alpha = sn / (2.0 * Q);

    // Notch filter (zeroes on unit circle at e^{±jω0}, poles at r·e^{±jω0})
    const double b0_un = 1.0;
    const double b1_un = -2.0 * cs;
    const double b2_un = 1.0;
    const double a0_un = 1.0 + alpha;
    const double a1_un = -2.0 * cs;
    const double a2_un = 1.0 - alpha;

    // Normalize so a0 = 1
    b0 = b0_un / a0_un;
    b1 = b1_un / a0_un;
    b2 = b2_un / a0_un;
    a1 = a1_un / a0_un;
    a2 = a2_un / a0_un;
  }

  double update(const double u) {
    // Direct Form II Transposed
    double y = b0 * u + z1;
    z1 = b1 * u + z2 - a1 * y;
    z2 = b2 * u - a2 * y;
    return y;
  }

private:
  double b0;
  double b1;
  double b2;
  double a1;
  double a2;

  double z1, z2;
};

#endif // NOTCH_FILTER_HPP
