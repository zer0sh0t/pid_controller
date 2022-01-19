#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

class PIDController
{
    public:
    double Kp, Ki, Kd, set_point, sample_time, p_term, i_term, d_term, last_error, curr_time, last_time, last_output;
    double *clamp_vals;

    PIDController(double Kp_, double Ki_, double Kd_, double set_point_, double sample_time_, double *clamp_vals_);

    double update(double feedback_val);
};

#endif
