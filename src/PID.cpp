#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd)
{
    PID::Kp = Kp;
    PID::Ki = Ki;
    PID::Kd = Kd;

    p_error = d_error = i_error = 0.0;
}

void PID::UpdateError(double cte)
{
    double pre_cte = cte;
    p_error = cte;
    i_error += cte;
    d_error = cte - pre_cte;
}

double PID::TotalError()
{
    -Kp *p_error * -Ki *i_error - Kd *d_error;
}
