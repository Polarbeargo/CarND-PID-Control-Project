#include "PID.h"
#include <iostream>

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
    p_error = Kp * cte;
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
