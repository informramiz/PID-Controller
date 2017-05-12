/*
 * Twiddle.h
 *
 *  Created on: May 12, 2017
 *      Author: ramiz
 */

#ifndef TWIDDLE_H_
#define TWIDDLE_H_

#include <uWS/uWS.h>
#include "json.hpp"

// for convenience
using json = nlohmann::json;

class Twiddle {
public:
  Twiddle();
  int RunPID(double params[]);
  void FindParams(double tolerance = 0.001);
  bool TryIncreasing(double p[], double dp[], int i, double &best_error);
  bool TryDecreasing(double p[], double dp[], int i, double &best_error);
private:
  double last_avg_error_;
  const int N_;
  const int ksteps_before_error_sum_;
};



#endif /* TWIDDLE_H_ */
