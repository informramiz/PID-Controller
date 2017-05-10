#include <iostream>
#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {
  Kp = 0.2;
  Kd = 3.0;
  Ki = 0.004;

  p_error = 0.0;
  d_error = 0.0;
  i_error = 0.0;

  prev_cte_ = 0.0;
  error_sum_ = 0.0;
  total_error = 0.0;
}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
}

void PID::UpdateError(double cte) {
  if (prev_cte_ == 0.0) {
    prev_cte_ = cte;
  }

  //calculate P (proportional part) to reduce CTE
  p_error = -Kp * cte;

  //calculate D (derivative) part to avoid overshooting, assuming delta=1
  d_error = -Kd * (cte - prev_cte_);

  //add to to sum of all errors observed till now
  total_error += cte;

  //calculate I (integral) part to handle any system bias (wheel alignment, wind pressure, slippery road etc.)
  i_error = -Ki * total_error;
}

double PID::TotalError() {
  return p_error + d_error + i_error;
}

