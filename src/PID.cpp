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
    // Initial Kp = 0.2 keep car correct back to center of road
    Kp = 0.2;
    double pre_cte = cte;
    p_error = Kp * cte;
    i_error += Ki * cte;
    d_error = Kd * (cte - pre_cte);

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
