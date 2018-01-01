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

    p_error = d_error = i_error = 0.0;
}

void PID::UpdateError(double cte)
{
    // Initial Kp = 0.2 keep car correct back to center of road
    Kp = 0.2;
    Kd = 0.1;

    double pre_cte = cte;

    // TODO Sharp turn track correct back to center of road
    if (cte > 4)
    {
        // d_error = Kd * (cte - p_error);
        Ki = 0.1;
        Kd = 0;
    }
    else if (cte < -6)
    {
        Ki = -0.1;
        Kd = 0;
    }
    else
    {
        Ki = 0;
        Kd = 0;
    }

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

/**
 * Computing the throttle by PID controller
 * @param max_throttle max throttle
 * @return the computed throttle
 */
double PID::OutputThrottle(double max_throttle)
{
    return max_throttle - Kp * p_error - Ki * i_error - Kd * d_error;
}
