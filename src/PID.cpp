#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {

  p_error = 0;
  i_error = 0;
  d_error = 0;
  Kp = 0;
  Ki = 0;
  Kd = 0;

}

PID::~PID() {}

void PID::Init(double K_p, double K_i, double K_d) {

  Kp = K_p;
  Ki = K_i;
  Kd = K_d;
}

void PID::UpdateError(double cte) {

  d_error = cte - p_error;
  p_error = cte;
  i_error = TotalError();

}

double PID::TotalError() {

  double total_error;
  total_error = i_error + p_error;

  return total_error;
}

double PID::CalculatePIDOut(double cte){

  UpdateError(cte);

  double steer_value;

  steer_value = - (Kp * p_error + Ki * i_error + Kd * d_error);

  if (steer_value <= -1.0)
  {
    steer_value = -1.0;
  }
  else if (steer_value >= 1.0)
  {
    steer_value = 1.0;
  }
  else
  {
    /*Do nothing*/
  }

  return steer_value;
}

