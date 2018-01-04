#include "PID.h"
#include <iostream>
#include <math.h>

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

    pre_cte = 0.0;
    p_error = d_error = i_error = 0.0;
}

void PID::UpdateError(double cte)
{
    double pre_cte = cte;
    p_error = cte;
    i_error += cte;
    d_error = cte - pre_cte;

    // Add debug log
    cout << "Kp = "
         << "\t" << Kp << "\t"
         << "P_error= " << Kp * p_error << endl;
    cout << "Ki = "
         << "\t" << Ki << "\t"
         << "I_error= " << Ki * i_error << endl;
    cout << "Kd = "
         << "\t" << Kd << "\t"
         << "D_error= " << Kd * d_error << endl;
}

double PID::TotalError()
{
    return p_error * Kp + i_error * Ki + d_error * Kd;
}
