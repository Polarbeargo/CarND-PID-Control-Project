#ifndef PID_H
#define PID_H

class PID
{
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * cross track error components
  */
  double pre_cte;

  /*
  * Coefficients
  */
  double Kp;
  double Ki;
  double Kd;

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
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  /**
   * Computing the throttle by PID controller
   * @param max_throttle max throttle
   * @return the computed throttle
   */
  double OutputThrottle(double max_throttle);
};

#endif /* PID_H */
