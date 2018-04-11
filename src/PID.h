#ifndef PID_H
#define PID_H

/*
 * Define the number of tuning loops
 */
#define TUNELOOPS 50

/*
 * Define enum representing tuning stages
 */
enum TuningStageType {P_Tuning, I_Tuning, D_Tuning, NumofTuningStages};

/*
 * Define enum representing tuning phases for each stage
 */
enum TuningPhaseType {Phase_Gain_Update, Phase_Gain_Evaluate_I,\
  Phase_Gain_Evaluate_II};

/*
 * Define gain tolerance to halt
 * optimization
 */
#define GAIN_TOL 0.0001

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  /*
   * Flag to trigger PID tuning
   */
  bool pid_optimized;

  /*
   * Flag to reset simulator
   */
  bool reset_simulator;

  /*
   * Counter to demark tuning time of track
   */
  int tunetest_count;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp_, double Ki_, double Kd_, bool pid_optimized_ = false);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError(double last_total_error);

  /*
   * Calculates and returns output of the PID
   */
  double CalculatePIDOut(double cte);

  /*
   * This function tunes the PID gains
   */
  void twiddle();
};

#endif /* PID_H */
