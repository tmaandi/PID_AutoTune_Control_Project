#include<iostream>

#include "PID.h"

using namespace std;

PID::PID()
{

  p_error = 0;
  i_error = 0;
  d_error = 0;
  Kp = 0;
  Ki = 0;
  Kd = 0;
  pid_optimized = false;
  tunetest_count = 0;
  reset_simulator = false;
  is_initialized = false;

}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_, bool pid_optimized_)
{

  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;
  pid_optimized = pid_optimized_;
  is_initialized = true;
}

void PID::UpdateError(double cte)
{

  d_error = cte - p_error;
  p_error = cte;
  i_error = i_error + cte;

}

double PID::TotalError(double last_total_error)
{

  double total_error = last_total_error + p_error * p_error;

  return total_error;
}

double PID::CalculatePIDOut(double cte)
{
  double steer_value;

  UpdateError(cte);

  steer_value = - (Kp * p_error + Ki * i_error + Kd * d_error);

  /*
   * Limiting steer_value between -1.0 and 1.0
   */
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

void PID::twiddle()
{
  double* p[] = {&Kp, &Ki, &Kd};

  /*
   * Deltas for P,I,D gain adjustment respectively
   */
  static double dp[] = {0.01, 0.001, 0.1};

  static double total_error, best_error, dp_sum = 100;

  /*
   * Three tuning states: P, I and D tuning
   */
  static TuningStageType tuning_stage;

  /*
   * Three tuing phases: Gain_Update, Gain_Evaluate_I and Gain_Evaluate_II
   */
  static TuningPhaseType tuning_phase;

  /*
   * Don't accumulate error for some initial frames of simulation
   * Also, as the tunetest_count is set to 0 after each tuning
   * simulation run, the total error is also reset to 0 below
   */
  if (tunetest_count < TUNELOOPS/2)
  {
    total_error = 0;
  }
  /*
   * Accumulate error for rest of the tuning simulation
   */
  else if (tunetest_count < TUNELOOPS)
  {
    total_error = TotalError(total_error);
  }
  /*
   * Once reached the end of tuning simulation time,
   * update gains based on accumulated error
   */
  else
  {
    if (tunetest_count == TUNELOOPS)
    {
      /*
       * Average the accumulated error
       */
      total_error = total_error/(TUNELOOPS/2);
    }
    /*
     * Don't update gains for the first run and assign
     * the first total error as the best error
     */
    if (best_error != 0)
    {

      if (dp_sum < GAIN_TOL)
      {
        /*
         * No more PID auto-tuning
         */
        pid_optimized = true;

        reset_simulator = true;
      }
      else
      {
        if (tuning_phase == Phase_Gain_Update)
        {
          *p[tuning_stage] += dp[tuning_stage];

          /*
           * Gain updated, move to evaluation phase
           */
          tuning_phase = Phase_Gain_Evaluate_I;

          reset_simulator = true;
        }
        else if ((tuning_phase == Phase_Gain_Evaluate_I) \
            || (tuning_phase == Phase_Gain_Evaluate_II))
        {
          if ((total_error < best_error) \
              && (tuning_phase == Phase_Gain_Evaluate_I))
          {
            /*
             * If the last gain update resulted in error
             * reduction, then set the deltas to update
             * the gain in the same direction for the
             * future loops
             */
            /*
             * Update best error
             */
            best_error = total_error;

            /*
             * Update dp for the corresponding gain
             */
            dp[tuning_stage] = 1.1 * dp[tuning_stage];

            /*
             * Reset the tuning phase to Phase_Gain_Update
             * at the end of evaluation loop
             */
            tuning_phase = Phase_Gain_Update;

            if (tuning_stage == D_Tuning)
            {
              /*
               * Sum the dp terms. If this sum gets smaller than
               * GAIN_TOL, then stop optimization and set the
               * pid_optimized flag to true
               */
              dp_sum = 0;
              for (unsigned int i = 0; i < sizeof(dp)/sizeof(dp[0]); ++i)
              {
                dp_sum += dp[i];
              }
            }

            /*
             * Set the tuning stage to move to next gain tuning
             */
            tuning_stage = static_cast<TuningStageType>((tuning_stage + 1) \
                                                        % NumofTuningStages);

            reset_simulator = true;
          }
          else
          {
            /*
             * If increasing the gain doesn't work, try
             * decreasing the gain
             */
            if (tuning_phase == Phase_Gain_Evaluate_I)
            {
              *p[tuning_stage] -= 2*dp[tuning_stage];
              tuning_phase = Phase_Gain_Evaluate_II;
              reset_simulator = true;
            }
            else if ( tuning_phase == Phase_Gain_Evaluate_II)
            {
              if (total_error < best_error)
              {
                best_error = total_error;
                dp[tuning_stage] = dp[tuning_stage] * 1.1;
              }
              else
              {
                *p[tuning_stage] += dp[tuning_stage];
                dp[tuning_stage] = dp[tuning_stage] * 0.9;
              }

              /*
               * Reset the tuning phase to Phase_Gain_Update
               * at the end of evaluation loop
               */
              tuning_phase = Phase_Gain_Update;

              if (tuning_stage == D_Tuning)
              {
                /*
                 * Sum the dp terms. If this sum gets smaller than
                 * GAIN_TOL, then stop optimization and set the
                 * pid_optimized flag to true
                 */
                dp_sum = 0;
                for (unsigned int i = 0; i < sizeof(dp)/sizeof(dp[0]); ++i)
                {
                  dp_sum += dp[i];
                }
              }
              /*
               * Set the tuning stage to move to next gain tuning
               */
              tuning_stage = static_cast<TuningStageType>((tuning_stage + 1) \
                                                          % NumofTuningStages);

              reset_simulator = true;
            }
            else
            {

            }

          }
        }
        else
        {

        }
      }
    }
    else
    {
      best_error = total_error;
      reset_simulator = true;
    }

  }

  tunetest_count ++;
}

